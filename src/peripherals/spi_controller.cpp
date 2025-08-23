#include "emulator/peripherals/spi_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("SPIController");

SPIController::SPIController(u8 controller_id)
    : controller_id_(controller_id),
      initialized_(false),
      mode_(SPIMode::MASTER),
      clock_rate_(1000000), // 1 MHz default
      clock_polarity_(SPIClockPolarity::IDLE_LOW),
      clock_phase_(SPIClockPhase::FIRST_EDGE),
      bit_order_(SPIBitOrder::MSB_FIRST),
      data_size_(SPIDataSize::BITS_8),
      interrupt_controller_(nullptr),
      active_cs_pin_(0),
      cs_active_(false) {
    COMPONENT_LOG_DEBUG("SPI controller {} created", controller_id_);
}

SPIController::~SPIController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("SPI controller {} destroyed", controller_id_);
}

Result<void> SPIController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "SPI controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing SPI controller {}", controller_id_);
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset registers to default values
    registers_ = {};
    registers_.clock_control = clock_rate_;
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    // Reset transfer state
    current_transfer_.reset();
    cs_active_ = false;
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("SPI controller {} initialized successfully", controller_id_);
    
    return {};
}

Result<void> SPIController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down SPI controller {}", controller_id_);
    
    // Complete any pending transfer
    if (current_transfer_) {
        complete_transfer(unexpected(MAKE_ERROR(SYSTEM_SHUTDOWN,
            "SPI controller shutting down")));
    }
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    // Deactivate chip select
    cs_active_ = false;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("SPI controller {} shutdown completed", controller_id_);
    return {};
}

Result<void> SPIController::configure(SPIMode mode, u32 clock_rate, SPIClockPolarity cpol,
                                     SPIClockPhase cpha, SPIBitOrder bit_order, SPIDataSize data_size) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    if (current_transfer_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot configure SPI controller while transfer in progress"));
    }
    
    if (clock_rate < MIN_CLOCK_RATE || clock_rate > MAX_CLOCK_RATE) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Clock rate out of range"));
    }
    
    mode_ = mode;
    clock_rate_ = clock_rate;
    clock_polarity_ = cpol;
    clock_phase_ = cpha;
    bit_order_ = bit_order;
    data_size_ = data_size;
    
    // Update registers
    registers_.control1 = (static_cast<u32>(mode) << 0) |
                         (static_cast<u32>(cpol) << 1) |
                         (static_cast<u32>(cpha) << 2) |
                         (static_cast<u32>(bit_order) << 3) |
                         (static_cast<u32>(data_size) << 4);
    registers_.clock_control = clock_rate_;
    
    COMPONENT_LOG_INFO("SPI controller {} configured: mode={}, rate={} Hz, CPOL={}, CPHA={}, bits={}",
                      controller_id_, static_cast<u8>(mode_), clock_rate_,
                      static_cast<u8>(cpol), static_cast<u8>(cpha), static_cast<u8>(data_size_));
    
    return {};
}

Result<void> SPIController::set_chip_select(u8 cs_pin, bool active) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    if (cs_pin > 7) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid chip select pin"));
    }
    
    if (active && cs_active_ && active_cs_pin_ != cs_pin) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Another chip select already active"));
    }
    
    active_cs_pin_ = cs_pin;
    cs_active_ = active;
    
    if (active) {
        statistics_.cs_toggles++;
    }
    
    // Update chip select register
    registers_.chip_select = (cs_pin << 0) | (active ? 0x80 : 0x00);
    
    COMPONENT_LOG_DEBUG("SPI controller {} CS{} {}", controller_id_, cs_pin,
                       active ? "activated" : "deactivated");
    
    return {};
}

Result<void> SPIController::start_transfer(const std::vector<u8>& tx_data) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    if (mode_ != SPIMode::MASTER) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Can only start transfers in master mode"));
    }
    
    if (current_transfer_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "SPI controller busy with another transfer"));
    }
    
    if (!cs_active_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No chip select active"));
    }
    
    if (tx_data.empty()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Transfer data cannot be empty"));
    }
    
    // Create new transfer
    current_transfer_ = std::make_unique<SPITransfer>();
    current_transfer_->tx_data = tx_data;
    current_transfer_->rx_data.resize(tx_data.size());
    current_transfer_->bytes_transferred = 0;
    current_transfer_->is_complete = false;
    current_transfer_->start_time = std::chrono::steady_clock::now();
    
    transfer_start_ = std::chrono::steady_clock::now();
    
    // Fill TX FIFO
    for (size_t i = 0; i < std::min(tx_data.size(), TX_FIFO_SIZE); ++i) {
        tx_fifo_.push(tx_data[i]);
    }
    
    update_status_register();
    
    COMPONENT_LOG_DEBUG("SPI controller {} started transfer: {} bytes, CS{}",
                       controller_id_, tx_data.size(), active_cs_pin_);
    
    return {};
}

Result<std::vector<u8>> SPIController::get_received_data() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    if (!current_transfer_ || !current_transfer_->is_complete) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No completed transfer available"));
    }
    
    return current_transfer_->rx_data;
}

Result<void> SPIController::abort_transfer() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    if (!current_transfer_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No active transfer to abort"));
    }
    
    complete_transfer(unexpected(MAKE_ERROR(OPERATION_ABORTED,
        "Transfer aborted by user")));
    
    COMPONENT_LOG_DEBUG("SPI controller {} transfer aborted", controller_id_);
    return {};
}

Result<void> SPIController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    Address offset = address - (SPI0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register 1
            registers_.control1 = value;
            // Update configuration from register
            mode_ = static_cast<SPIMode>((value >> 0) & 0x01);
            clock_polarity_ = static_cast<SPIClockPolarity>((value >> 1) & 0x01);
            clock_phase_ = static_cast<SPIClockPhase>((value >> 2) & 0x01);
            bit_order_ = static_cast<SPIBitOrder>((value >> 3) & 0x01);
            data_size_ = static_cast<SPIDataSize>((value >> 4) & 0xFF);
            break;
            
        case 0x04: // Control register 2
            registers_.control2 = value;
            if (value & 0x01) { // Start transfer
                if (!tx_fifo_.empty() && cs_active_) {
                    std::vector<u8> tx_data;
                    while (!tx_fifo_.empty()) {
                        tx_data.push_back(tx_fifo_.front());
                        tx_fifo_.pop();
                    }
                    start_transfer(tx_data);
                }
            }
            break;
            
        case 0x0C: // Interrupt enable register
            registers_.interrupt_enable = value;
            break;
            
        case 0x10: // Interrupt clear register
            registers_.interrupt_status &= ~value;
            break;
            
        case 0x14: // Clock control register
            registers_.clock_control = value;
            clock_rate_ = std::clamp(value, MIN_CLOCK_RATE, MAX_CLOCK_RATE);
            break;
            
        case 0x18: // Chip select register
            registers_.chip_select = value;
            {
                u8 cs_pin = value & 0x07;
                bool active = (value & 0x80) != 0;
                set_chip_select(cs_pin, active);
            }
            break;
            
        case 0x1C: // Data register
            registers_.data = value & 0xFF;
            if (tx_fifo_.size() < TX_FIFO_SIZE) {
                tx_fifo_.push(static_cast<u8>(value));
            } else {
                statistics_.fifo_overflows++;
                trigger_interrupt(SPIInterruptType::RX_FIFO_OVERFLOW);
            }
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
            
        case 0x28: // DMA control register
            registers_.dma_control = value;
            break;
            
        case 0x2C: // Timing control register
            registers_.timing_control = value;
            break;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid SPI register offset: 0x" + std::to_string(offset)));
    }
    
    return {};
}

Result<u32> SPIController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SPI controller not initialized"));
    }
    
    Address offset = address - (SPI0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register 1
            return registers_.control1;
            
        case 0x04: // Control register 2
            return registers_.control2;
            
        case 0x08: // Status register
            update_status_register();
            return registers_.status;
            
        case 0x0C: // Interrupt enable register
            return registers_.interrupt_enable;
            
        case 0x10: // Interrupt status register
            return registers_.interrupt_status;
            
        case 0x14: // Clock control register
            return registers_.clock_control;
            
        case 0x18: // Chip select register
            return registers_.chip_select;
            
        case 0x1C: // Data register
            if (!rx_fifo_.empty()) {
                u8 data = rx_fifo_.front();
                rx_fifo_.pop();
                return data;
            } else {
                statistics_.fifo_underflows++;
                trigger_interrupt(SPIInterruptType::TX_FIFO_UNDERFLOW);
                return 0;
            }
            
        case 0x20: // FIFO control register
            return registers_.fifo_control;
            
        case 0x24: // FIFO status register
            update_status_register();
            return registers_.fifo_status;
            
        case 0x28: // DMA control register
            return registers_.dma_control;
            
        case 0x2C: // Timing control register
            return registers_.timing_control;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid SPI register offset: 0x" + std::to_string(offset)));
    }
}

void SPIController::update() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    if (current_transfer_) {
        update_transfer();
    }
    
    process_tx_fifo();
    process_rx_fifo();
}

void SPIController::update_transfer() {
    if (!current_transfer_) {
        return;
    }
    
    u32 bit_time_ns = calculate_bit_time_ns();
    u32 byte_time_ns = bit_time_ns * static_cast<u32>(data_size_);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - transfer_start_);
    
    // Calculate how many bytes should have been transferred by now
    size_t expected_bytes = elapsed.count() / byte_time_ns;
    expected_bytes = std::min(expected_bytes, current_transfer_->tx_data.size());
    
    // Transfer bytes up to the expected count
    while (current_transfer_->bytes_transferred < expected_bytes) {
        size_t idx = current_transfer_->bytes_transferred;
        
        // Simulate device response
        std::vector<u8> tx_byte = {current_transfer_->tx_data[idx]};
        std::vector<u8> rx_byte;
        simulate_device_response(tx_byte, rx_byte);
        
        if (!rx_byte.empty()) {
            current_transfer_->rx_data[idx] = rx_byte[0];
        } else {
            current_transfer_->rx_data[idx] = 0xFF; // Default response
        }
        
        current_transfer_->bytes_transferred++;
        statistics_.bytes_transmitted++;
        statistics_.bytes_received++;
    }
    
    // Check if transfer is complete
    if (current_transfer_->bytes_transferred >= current_transfer_->tx_data.size()) {
        complete_transfer({});
    }
}

void SPIController::complete_transfer(Result<void> result) {
    if (!current_transfer_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - current_transfer_->start_time);
    
    current_transfer_->result = result;
    current_transfer_->is_complete = true;
    
    // Update statistics
    statistics_.transfers_completed++;
    double transfer_time = duration.count();
    statistics_.average_transfer_time_us = 
        (statistics_.average_transfer_time_us * (statistics_.transfers_completed - 1) + transfer_time) /
        statistics_.transfers_completed;
    
    // Fill RX FIFO with received data
    for (u8 byte : current_transfer_->rx_data) {
        if (rx_fifo_.size() < RX_FIFO_SIZE) {
            rx_fifo_.push(byte);
        }
    }
    
    // Trigger completion interrupt
    trigger_interrupt(SPIInterruptType::TRANSFER_COMPLETE);
    
    COMPONENT_LOG_DEBUG("SPI controller {} transfer completed: {} bytes in {:.2f} μs",
                       controller_id_, current_transfer_->tx_data.size(), transfer_time);
    
    // Keep the transfer object until data is read
}

void SPIController::trigger_interrupt(SPIInterruptType interrupt_type) {
    registers_.interrupt_status |= static_cast<u8>(interrupt_type);
    
    if (registers_.interrupt_enable & static_cast<u8>(interrupt_type)) {
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(
                static_cast<InterruptType>(static_cast<u8>(InterruptType::SPI0) + controller_id_));
        }
    }
}

void SPIController::update_status_register() {
    registers_.status = 0;
    
    // Transfer busy
    if (current_transfer_ && !current_transfer_->is_complete) {
        registers_.status |= 0x01;
    }
    
    // TX FIFO status
    if (tx_fifo_.empty()) {
        registers_.status |= 0x02; // TX FIFO empty
        trigger_interrupt(SPIInterruptType::TX_FIFO_EMPTY);
    }
    if (tx_fifo_.size() >= TX_FIFO_SIZE) {
        registers_.status |= 0x04; // TX FIFO full
    }
    
    // RX FIFO status
    if (!rx_fifo_.empty()) {
        registers_.status |= 0x08; // RX FIFO not empty
        trigger_interrupt(SPIInterruptType::RX_FIFO_NOT_EMPTY);
    }
    if (rx_fifo_.size() >= RX_FIFO_SIZE) {
        registers_.status |= 0x10; // RX FIFO full
    }
    
    // Chip select status
    if (cs_active_) {
        registers_.status |= 0x20;
    }
    
    // FIFO status register
    registers_.fifo_status = (tx_fifo_.size() & 0xFF) | ((rx_fifo_.size() & 0xFF) << 8);
}

void SPIController::process_tx_fifo() {
    // TX FIFO is processed during transfer simulation
}

void SPIController::process_rx_fifo() {
    // RX FIFO is populated during transfer completion
}

void SPIController::simulate_device_response(const std::vector<u8>& tx_data, std::vector<u8>& rx_data) {
    rx_data.clear();
    
    // Simple device simulation based on chip select
    switch (active_cs_pin_) {
        case 0: // Flash memory simulation
            if (!tx_data.empty()) {
                u8 cmd = tx_data[0];
                switch (cmd) {
                    case 0x9F: // Read JEDEC ID
                        rx_data = {0x00, 0xEF, 0x40, 0x18}; // Mock flash ID
                        break;
                    case 0x03: // Read data
                        rx_data = {0x00, 0xAA, 0xBB, 0xCC, 0xDD}; // Mock data
                        break;
                    default:
                        rx_data = {0xFF}; // Default response
                        break;
                }
            }
            break;
            
        case 1: // ADC simulation
            if (!tx_data.empty()) {
                // Return mock ADC reading
                rx_data = {static_cast<u8>(rand() % 256), static_cast<u8>(rand() % 256)};
            }
            break;
            
        default:
            // Echo back transmitted data (loopback mode)
            rx_data = tx_data;
            break;
    }
}

u32 SPIController::calculate_bit_time_ns() const {
    return 1000000000 / clock_rate_; // nanoseconds per bit
}

void SPIController::clear_statistics() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    statistics_ = {};
}

void SPIController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== SPI Controller {} Status ===", controller_id_);
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Mode: {}", static_cast<u8>(mode_));
        COMPONENT_LOG_INFO("Clock rate: {} Hz", clock_rate_);
        COMPONENT_LOG_INFO("Clock polarity: {}", static_cast<u8>(clock_polarity_));
        COMPONENT_LOG_INFO("Clock phase: {}", static_cast<u8>(clock_phase_));
        COMPONENT_LOG_INFO("Bit order: {}", static_cast<u8>(bit_order_));
        COMPONENT_LOG_INFO("Data size: {} bits", static_cast<u8>(data_size_));
        
        COMPONENT_LOG_INFO("Chip select: CS{} {}", active_cs_pin_, cs_active_ ? "active" : "inactive");
        COMPONENT_LOG_INFO("Transfer active: {}", current_transfer_ != nullptr);
        
        COMPONENT_LOG_INFO("TX FIFO: {}/{}", tx_fifo_.size(), TX_FIFO_SIZE);
        COMPONENT_LOG_INFO("RX FIFO: {}/{}", rx_fifo_.size(), RX_FIFO_SIZE);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Transfers completed: {}", statistics_.transfers_completed);
        COMPONENT_LOG_INFO("  Bytes transmitted: {}", statistics_.bytes_transmitted);
        COMPONENT_LOG_INFO("  Bytes received: {}", statistics_.bytes_received);
        COMPONENT_LOG_INFO("  Average transfer time: {:.2f} μs", statistics_.average_transfer_time_us);
        COMPONENT_LOG_INFO("  Mode faults: {}", statistics_.mode_faults);
        COMPONENT_LOG_INFO("  FIFO overflows: {}", statistics_.fifo_overflows);
        COMPONENT_LOG_INFO("  FIFO underflows: {}", statistics_.fifo_underflows);
        COMPONENT_LOG_INFO("  CS toggles: {}", statistics_.cs_toggles);
    }
}

}  // namespace m5tab5::emulator