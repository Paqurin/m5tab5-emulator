#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <thread>

namespace m5tab5::emulator {

DECLARE_LOGGER("I2CController");

I2CController::I2CController(u8 controller_id)
    : controller_id_(controller_id),
      initialized_(false),
      mode_(I2CMode::MASTER),
      speed_(I2CSpeed::STANDARD),
      state_(I2CState::IDLE),
      timeout_ms_(DEFAULT_TIMEOUT_MS),
      interrupt_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("I2C controller {} created", controller_id_);
}

I2CController::~I2CController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("I2C controller {} destroyed", controller_id_);
}

Result<void> I2CController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "I2C controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing I2C controller {}", controller_id_);
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset registers to default values
    registers_ = {};
    registers_.timeout = timeout_ms_;
    registers_.clock_divider = static_cast<u32>(speed_);
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    // Reset state
    state_ = I2CState::IDLE;
    current_transaction_.reset();
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("I2C controller {} initialized successfully", controller_id_);
    
    return {};
}

Result<void> I2CController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down I2C controller {}", controller_id_);
    
    // Complete any pending transaction
    if (current_transaction_) {
        complete_transaction(unexpected(MAKE_ERROR(SYSTEM_SHUTDOWN,
            "I2C controller shutting down")));
    }
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("I2C controller {} shutdown completed", controller_id_);
    return {};
}

Result<void> I2CController::configure(I2CMode mode, I2CSpeed speed) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    if (state_ != I2CState::IDLE) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot configure I2C controller while busy"));
    }
    
    mode_ = mode;
    speed_ = speed;
    registers_.clock_divider = static_cast<u32>(speed_);
    
    COMPONENT_LOG_INFO("I2C controller {} configured: mode={}, speed={} Hz",
                      controller_id_, static_cast<u8>(mode_), static_cast<u32>(speed_));
    
    return {};
}

Result<void> I2CController::set_timeout(u32 timeout_ms) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    timeout_ms_ = timeout_ms;
    registers_.timeout = timeout_ms_;
    
    COMPONENT_LOG_DEBUG("I2C controller {} timeout set to {} ms", controller_id_, timeout_ms_);
    return {};
}

Result<void> I2CController::start_transaction(u8 slave_address, bool is_read, const std::vector<u8>& data) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    if (mode_ != I2CMode::MASTER) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Can only start transactions in master mode"));
    }
    
    if (state_ != I2CState::IDLE) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "I2C controller busy with another transaction"));
    }
    
    // Create new transaction
    current_transaction_ = std::make_unique<I2CTransaction>();
    current_transaction_->slave_address = slave_address;
    current_transaction_->is_read = is_read;
    current_transaction_->data = data;
    current_transaction_->is_complete = false;
    current_transaction_->start_time = std::chrono::steady_clock::now();
    
    transaction_start_ = std::chrono::steady_clock::now();
    state_ = I2CState::START_SENT;
    
    update_status_register();
    
    COMPONENT_LOG_DEBUG("I2C controller {} started transaction: address=0x{:02X}, read={}, {} bytes",
                       controller_id_, slave_address, is_read, data.size());
    
    return {};
}

Result<std::vector<u8>> I2CController::read_data(size_t num_bytes) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    std::vector<u8> result;
    result.reserve(num_bytes);
    
    while (result.size() < num_bytes && !rx_fifo_.empty()) {
        result.push_back(rx_fifo_.front());
        rx_fifo_.pop();
    }
    
    if (result.size() < num_bytes) {
        return unexpected(MAKE_ERROR(INSUFFICIENT_DATA,
            "Not enough data in RX FIFO"));
    }
    
    statistics_.bytes_received += result.size();
    return result;
}

Result<void> I2CController::write_data(const std::vector<u8>& data) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    if (tx_fifo_.size() + data.size() > TX_FIFO_SIZE) {
        statistics_.fifo_overflows++;
        return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
            "TX FIFO overflow"));
    }
    
    for (u8 byte : data) {
        tx_fifo_.push(byte);
    }
    
    statistics_.bytes_transmitted += data.size();
    return {};
}

Result<void> I2CController::stop_transaction() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    if (!current_transaction_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No active transaction to stop"));
    }
    
    state_ = I2CState::STOP_SENT;
    complete_transaction({});
    
    COMPONENT_LOG_DEBUG("I2C controller {} transaction stopped", controller_id_);
    return {};
}

Result<void> I2CController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    Address offset = address - (I2C0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register
            registers_.control = value;
            if (value & 0x01) { // Start bit
                // Trigger start condition
                state_ = I2CState::START_SENT;
            }
            if (value & 0x02) { // Stop bit
                // Trigger stop condition
                if (current_transaction_) {
                    stop_transaction();
                }
            }
            break;
            
        case 0x08: // Interrupt enable register
            registers_.interrupt_enable = value;
            break;
            
        case 0x0C: // Interrupt clear register
            registers_.interrupt_status &= ~value;
            break;
            
        case 0x10: // Clock divider register
            registers_.clock_divider = value;
            speed_ = static_cast<I2CSpeed>(value);
            break;
            
        case 0x14: // Slave address register
            registers_.slave_address = value & 0x7F;
            break;
            
        case 0x18: // Data register
            registers_.data = value & 0xFF;
            if (tx_fifo_.size() < TX_FIFO_SIZE) {
                tx_fifo_.push(static_cast<u8>(value));
            } else {
                statistics_.fifo_overflows++;
                trigger_interrupt(I2CInterruptType::FIFO_OVERFLOW);
            }
            break;
            
        case 0x1C: // Timeout register
            registers_.timeout = value;
            timeout_ms_ = value;
            break;
            
        case 0x20: // FIFO control register
            registers_.fifo_control = value;
            if (value & 0x01) { // Clear TX FIFO
                while (!tx_fifo_.empty()) tx_fifo_.pop();
            }
            if (value & 0x02) { // Clear RX FIFO
                while (!rx_fifo_.empty()) rx_fifo_.pop();
            }
            break;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid I2C register offset: 0x" + std::to_string(offset)));
    }
    
    return {};
}

Result<u32> I2CController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "I2C controller not initialized"));
    }
    
    Address offset = address - (I2C0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register
            return registers_.control;
            
        case 0x04: // Status register
            update_status_register();
            return registers_.status;
            
        case 0x08: // Interrupt enable register
            return registers_.interrupt_enable;
            
        case 0x0C: // Interrupt status register
            return registers_.interrupt_status;
            
        case 0x10: // Clock divider register
            return registers_.clock_divider;
            
        case 0x14: // Slave address register
            return registers_.slave_address;
            
        case 0x18: // Data register
            if (!rx_fifo_.empty()) {
                u8 data = rx_fifo_.front();
                rx_fifo_.pop();
                return data;
            } else {
                statistics_.fifo_underflows++;
                trigger_interrupt(I2CInterruptType::FIFO_UNDERFLOW);
                return 0;
            }
            
        case 0x1C: // Timeout register
            return registers_.timeout;
            
        case 0x20: // FIFO control register
            return registers_.fifo_control;
            
        case 0x24: // FIFO status register
            update_status_register();
            return registers_.fifo_status;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid I2C register offset: 0x" + std::to_string(offset)));
    }
}

void I2CController::update() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_ || !current_transaction_) {
        return;
    }
    
    update_transaction();
    process_tx_fifo();
    process_rx_fifo();
}

void I2CController::update_transaction() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - transaction_start_);
    
    // Check for timeout
    if (elapsed.count() > timeout_ms_) {
        statistics_.timeout_errors++;
        complete_transaction(unexpected(MAKE_ERROR(TIMEOUT_ERROR,
            "I2C transaction timeout")));
        trigger_interrupt(I2CInterruptType::TIMEOUT);
        return;
    }
    
    switch (state_) {
        case I2CState::START_SENT:
            // Simulate start condition delay
            if (elapsed.count() >= 1) {
                state_ = I2CState::ADDRESS_SENT;
            }
            break;
            
        case I2CState::ADDRESS_SENT:
            // Simulate address ACK/NACK
            if (elapsed.count() >= 2) {
                // Simple device simulation - always ACK for now
                if (current_transaction_->is_read) {
                    state_ = I2CState::DATA_RECEIVING;
                } else {
                    state_ = I2CState::DATA_SENDING;
                }
            }
            break;
            
        case I2CState::DATA_SENDING:
            // Process TX data
            if (tx_fifo_.empty()) {
                state_ = I2CState::STOP_SENT;
                complete_transaction({});
            }
            break;
            
        case I2CState::DATA_RECEIVING:
            // Simulate device response
            if (elapsed.count() >= 5) {
                std::vector<u8> response;
                if (simulate_device_response(current_transaction_->slave_address,
                                           true, {}, response)) {
                    for (u8 byte : response) {
                        if (rx_fifo_.size() < RX_FIFO_SIZE) {
                            rx_fifo_.push(byte);
                        }
                    }
                }
                state_ = I2CState::STOP_SENT;
                complete_transaction({});
            }
            break;
            
        case I2CState::STOP_SENT:
            complete_transaction({});
            break;
            
        default:
            break;
    }
}

void I2CController::complete_transaction(Result<void> result) {
    if (!current_transaction_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - current_transaction_->start_time);
    
    current_transaction_->result = result;
    current_transaction_->is_complete = true;
    
    // Update statistics
    statistics_.transactions_completed++;
    double transaction_time = duration.count();
    statistics_.average_transaction_time_us = 
        (statistics_.average_transaction_time_us * (statistics_.transactions_completed - 1) + transaction_time) /
        statistics_.transactions_completed;
    
    state_ = I2CState::IDLE;
    current_transaction_.reset();
    
    // Trigger completion interrupt
    trigger_interrupt(I2CInterruptType::TRANSACTION_COMPLETE);
    
    COMPONENT_LOG_DEBUG("I2C controller {} transaction completed in {:.2f} μs",
                       controller_id_, transaction_time);
}

void I2CController::trigger_interrupt(I2CInterruptType interrupt_type) {
    registers_.interrupt_status |= static_cast<u8>(interrupt_type);
    
    if (registers_.interrupt_enable & static_cast<u8>(interrupt_type)) {
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(
                static_cast<InterruptType>(static_cast<u8>(InterruptType::I2C0) + controller_id_));
        }
    }
}

void I2CController::update_status_register() {
    registers_.status = 0;
    
    // Bus busy
    if (state_ != I2CState::IDLE) {
        registers_.status |= 0x01;
    }
    
    // TX FIFO status
    if (tx_fifo_.empty()) {
        registers_.status |= 0x02; // TX FIFO empty
    }
    if (tx_fifo_.size() >= TX_FIFO_SIZE) {
        registers_.status |= 0x04; // TX FIFO full
    }
    
    // RX FIFO status
    if (rx_fifo_.empty()) {
        registers_.status |= 0x08; // RX FIFO empty
    }
    if (rx_fifo_.size() >= RX_FIFO_SIZE) {
        registers_.status |= 0x10; // RX FIFO full
    }
    
    // FIFO status register
    registers_.fifo_status = (tx_fifo_.size() & 0xFF) | ((rx_fifo_.size() & 0xFF) << 8);
}

void I2CController::process_tx_fifo() {
    if (state_ == I2CState::DATA_SENDING && !tx_fifo_.empty()) {
        // Simulate sending one byte at a time
        static auto last_send = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_send);
        
        // Calculate bit time based on speed
        u32 bit_time_us = 1000000 / static_cast<u32>(speed_);
        u32 byte_time_us = bit_time_us * 9; // 8 data bits + 1 ACK bit
        
        if (elapsed.count() >= byte_time_us) {
            tx_fifo_.pop();
            last_send = now;
        }
    }
}

void I2CController::process_rx_fifo() {
    // RX FIFO is populated during transaction simulation
}

bool I2CController::simulate_device_response(u8 slave_address, bool is_read,
                                           const std::vector<u8>& data, std::vector<u8>& response) {
    // Simple device simulation
    switch (slave_address) {
        case 0x68: // Common IMU address
            if (is_read) {
                response = {0x12, 0x34, 0x56, 0x78}; // Mock sensor data
                return true;
            }
            break;
            
        case 0x50: // Common EEPROM address
            if (is_read) {
                response = {0xAA, 0xBB, 0xCC, 0xDD}; // Mock EEPROM data
                return true;
            }
            break;
            
        default:
            // Unknown device - NACK
            statistics_.nack_errors++;
            trigger_interrupt(I2CInterruptType::ADDRESS_NACK);
            return false;
    }
    
    return true;
}

void I2CController::clear_statistics() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    statistics_ = {};
}

void I2CController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== I2C Controller {} Status ===", controller_id_);
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Mode: {}", static_cast<u8>(mode_));
        COMPONENT_LOG_INFO("Speed: {} Hz", static_cast<u32>(speed_));
        COMPONENT_LOG_INFO("State: {}", static_cast<u8>(state_));
        COMPONENT_LOG_INFO("Timeout: {} ms", timeout_ms_);
        
        COMPONENT_LOG_INFO("TX FIFO: {}/{}", tx_fifo_.size(), TX_FIFO_SIZE);
        COMPONENT_LOG_INFO("RX FIFO: {}/{}", rx_fifo_.size(), RX_FIFO_SIZE);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Transactions completed: {}", statistics_.transactions_completed);
        COMPONENT_LOG_INFO("  Bytes transmitted: {}", statistics_.bytes_transmitted);
        COMPONENT_LOG_INFO("  Bytes received: {}", statistics_.bytes_received);
        COMPONENT_LOG_INFO("  Average transaction time: {:.2f} μs", statistics_.average_transaction_time_us);
        COMPONENT_LOG_INFO("  Arbitration losses: {}", statistics_.arbitration_losses);
        COMPONENT_LOG_INFO("  Timeout errors: {}", statistics_.timeout_errors);
        COMPONENT_LOG_INFO("  NACK errors: {}", statistics_.nack_errors);
        COMPONENT_LOG_INFO("  FIFO overflows: {}", statistics_.fifo_overflows);
        COMPONENT_LOG_INFO("  FIFO underflows: {}", statistics_.fifo_underflows);
    }
}

}  // namespace m5tab5::emulator