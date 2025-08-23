#include "emulator/peripherals/uart_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <sstream>

namespace m5tab5::emulator {

DECLARE_LOGGER("UARTController");

UARTController::UARTController(u8 controller_id)
    : controller_id_(controller_id),
      initialized_(false),
      baud_rate_(115200), // Default baud rate
      data_bits_(UARTDataBits::BITS_8),
      stop_bits_(UARTStopBits::STOP_1),
      parity_(UARTParity::NONE),
      flow_control_(UARTFlowControl::NONE),
      timeout_ms_(DEFAULT_TIMEOUT_MS),
      interrupt_controller_(nullptr),
      rts_state_(false),
      dtr_state_(false),
      cts_state_(true),  // Default to ready
      dsr_state_(true),  // Default to ready
      cd_state_(false),
      ri_state_(false) {
    COMPONENT_LOG_DEBUG("UART controller {} created", controller_id_);
}

UARTController::~UARTController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("UART controller {} destroyed", controller_id_);
}

Result<void> UARTController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "UART controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing UART controller {}", controller_id_);
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset registers to default values
    registers_ = {};
    registers_.line_status = static_cast<u32>(UARTLineStatus::TX_EMPTY | UARTLineStatus::TX_HOLDING_EMPTY);
    
    // Calculate divisor for default baud rate
    u32 divisor = 115200 / baud_rate_; // Assuming 115200 base clock
    registers_.divisor_latch_low = divisor & 0xFF;
    registers_.divisor_latch_high = (divisor >> 8) & 0xFF;
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    // Reset frame state
    current_tx_frame_.reset();
    current_rx_frame_.reset();
    
    // Initialize modem status
    registers_.modem_status = (cts_state_ ? 0x10 : 0) |
                             (dsr_state_ ? 0x20 : 0) |
                             (cd_state_ ? 0x80 : 0) |
                             (ri_state_ ? 0x40 : 0);
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("UART controller {} initialized successfully", controller_id_);
    
    return {};
}

Result<void> UARTController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down UART controller {}", controller_id_);
    
    // Clear FIFOs
    while (!tx_fifo_.empty()) tx_fifo_.pop();
    while (!rx_fifo_.empty()) rx_fifo_.pop();
    
    // Reset frame state
    current_tx_frame_.reset();
    current_rx_frame_.reset();
    
    // Reset modem signals
    rts_state_ = false;
    dtr_state_ = false;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("UART controller {} shutdown completed", controller_id_);
    return {};
}

Result<void> UARTController::configure(u32 baud_rate, UARTDataBits data_bits, UARTStopBits stop_bits,
                                      UARTParity parity, UARTFlowControl flow_control) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    if (baud_rate < MIN_BAUD_RATE || baud_rate > MAX_BAUD_RATE) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Baud rate out of range"));
    }
    
    baud_rate_ = baud_rate;
    data_bits_ = data_bits;
    stop_bits_ = stop_bits;
    parity_ = parity;
    flow_control_ = flow_control;
    
    // Update line control register
    registers_.line_control = (static_cast<u32>(data_bits) - 5) |
                             ((static_cast<u32>(stop_bits) - 1) << 2) |
                             (static_cast<u32>(parity) << 3);
    
    // Update divisor registers
    u32 divisor = 115200 / baud_rate_;
    registers_.divisor_latch_low = divisor & 0xFF;
    registers_.divisor_latch_high = (divisor >> 8) & 0xFF;
    
    COMPONENT_LOG_INFO("UART controller {} configured: {} bps, {}-{}-{}, flow={}",
                      controller_id_, baud_rate_, static_cast<u8>(data_bits_),
                      static_cast<u8>(parity_), static_cast<u8>(stop_bits_),
                      static_cast<u8>(flow_control_));
    
    return {};
}

Result<void> UARTController::set_timeout(u32 timeout_ms) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    timeout_ms_ = timeout_ms;
    COMPONENT_LOG_DEBUG("UART controller {} timeout set to {} ms", controller_id_, timeout_ms_);
    return {};
}

Result<void> UARTController::send_data(const std::vector<u8>& data) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    if (data.empty()) {
        return {};
    }
    
    // Check flow control
    if (flow_control_ == UARTFlowControl::RTS_CTS && !cts_state_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "CTS not asserted - cannot send data"));
    }
    
    // Add data to TX FIFO
    for (u8 byte : data) {
        if (tx_fifo_.size() >= TX_FIFO_SIZE) {
            statistics_.fifo_overflows++;
            return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
                "TX FIFO overflow"));
        }
        
        UARTFrame frame;
        frame.data = byte;
        frame.timestamp = std::chrono::steady_clock::now();
        tx_fifo_.push(frame);
    }
    
    statistics_.bytes_transmitted += data.size();
    
    // Update line status
    registers_.line_status &= ~static_cast<u32>(UARTLineStatus::TX_HOLDING_EMPTY);
    
    COMPONENT_LOG_DEBUG("UART controller {} queued {} bytes for transmission",
                       controller_id_, data.size());
    
    return {};
}

Result<void> UARTController::send_string(const std::string& str) {
    std::vector<u8> data(str.begin(), str.end());
    return send_data(data);
}

Result<std::vector<u8>> UARTController::receive_data(size_t max_bytes) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    std::vector<u8> result;
    size_t count = (max_bytes == 0) ? rx_fifo_.size() : std::min(max_bytes, rx_fifo_.size());
    result.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        if (!rx_fifo_.empty()) {
            result.push_back(rx_fifo_.front().data);
            rx_fifo_.pop();
        }
    }
    
    // Update line status
    if (rx_fifo_.empty()) {
        registers_.line_status &= ~static_cast<u32>(UARTLineStatus::DATA_READY);
    }
    
    statistics_.bytes_received += result.size();
    
    COMPONENT_LOG_DEBUG("UART controller {} received {} bytes", controller_id_, result.size());
    return result;
}

Result<std::string> UARTController::receive_string() {
    auto data_result = receive_data();
    if (!data_result) {
        return unexpected(data_result.error());
    }
    
    auto data = data_result.value();
    return std::string(data.begin(), data.end());
}

Result<void> UARTController::send_break(u32 duration_ms) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    // Create break frame
    UARTFrame break_frame;
    break_frame.data = 0;
    break_frame.timestamp = std::chrono::steady_clock::now();
    break_frame.is_break = true;
    
    if (tx_fifo_.size() >= TX_FIFO_SIZE) {
        statistics_.fifo_overflows++;
        return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
            "TX FIFO overflow"));
    }
    
    tx_fifo_.push(break_frame);
    statistics_.break_conditions++;
    
    COMPONENT_LOG_DEBUG("UART controller {} sending break condition for {} ms",
                       controller_id_, duration_ms);
    
    return {};
}

Result<void> UARTController::set_rts(bool state) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    rts_state_ = state;
    
    // Update modem control register
    if (state) {
        registers_.modem_control |= 0x02;
    } else {
        registers_.modem_control &= ~0x02;
    }
    
    COMPONENT_LOG_DEBUG("UART controller {} RTS {}", controller_id_, state ? "asserted" : "deasserted");
    return {};
}

Result<void> UARTController::set_dtr(bool state) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    dtr_state_ = state;
    
    // Update modem control register
    if (state) {
        registers_.modem_control |= 0x01;
    } else {
        registers_.modem_control &= ~0x01;
    }
    
    COMPONENT_LOG_DEBUG("UART controller {} DTR {}", controller_id_, state ? "asserted" : "deasserted");
    return {};
}

bool UARTController::get_cts() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return cts_state_;
}

bool UARTController::get_dsr() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return dsr_state_;
}

bool UARTController::get_cd() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return cd_state_;
}

bool UARTController::get_ri() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return ri_state_;
}

Result<void> UARTController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    Address offset = address - (UART0_BASE_ADDR + controller_id_ * 0x100);
    
    switch (offset) {
        case 0x00: // Data register / Divisor latch low
            if (registers_.line_control & 0x80) { // DLAB bit set
                registers_.divisor_latch_low = value & 0xFF;
                baud_rate_ = 115200 / ((registers_.divisor_latch_high << 8) | registers_.divisor_latch_low);
            } else {
                // Transmit data
                if (tx_fifo_.size() < TX_FIFO_SIZE) {
                    UARTFrame frame;
                    frame.data = value & 0xFF;
                    frame.timestamp = std::chrono::steady_clock::now();
                    tx_fifo_.push(frame);
                    
                    registers_.line_status &= ~static_cast<u32>(UARTLineStatus::TX_HOLDING_EMPTY);
                } else {
                    statistics_.fifo_overflows++;
                }
            }
            break;
            
        case 0x01: // Interrupt enable / Divisor latch high
            if (registers_.line_control & 0x80) { // DLAB bit set
                registers_.divisor_latch_high = value & 0xFF;
                baud_rate_ = 115200 / ((registers_.divisor_latch_high << 8) | registers_.divisor_latch_low);
            } else {
                registers_.interrupt_enable = value & 0xFF;
            }
            break;
            
        case 0x02: // FIFO control register
            registers_.fifo_control = value & 0xFF;
            if (value & 0x01) { // Enable FIFOs
                // FIFOs already enabled
            }
            if (value & 0x02) { // Clear RX FIFO
                while (!rx_fifo_.empty()) rx_fifo_.pop();
                registers_.line_status &= ~static_cast<u32>(UARTLineStatus::DATA_READY);
            }
            if (value & 0x04) { // Clear TX FIFO
                while (!tx_fifo_.empty()) tx_fifo_.pop();
                registers_.line_status |= static_cast<u32>(UARTLineStatus::TX_HOLDING_EMPTY | UARTLineStatus::TX_EMPTY);
            }
            break;
            
        case 0x03: // Line control register
            registers_.line_control = value & 0xFF;
            // Update configuration from register
            data_bits_ = static_cast<UARTDataBits>((value & 0x03) + 5);
            stop_bits_ = static_cast<UARTStopBits>(((value >> 2) & 0x01) + 1);
            parity_ = static_cast<UARTParity>((value >> 3) & 0x07);
            
            if (value & 0x40) { // Break control
                send_break(100); // 100ms break
            }
            break;
            
        case 0x04: // Modem control register
            registers_.modem_control = value & 0xFF;
            rts_state_ = (value & 0x02) != 0;
            dtr_state_ = (value & 0x01) != 0;
            break;
            
        case 0x07: // Scratch register
            registers_.scratch = value & 0xFF;
            break;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid UART register offset: 0x" + std::to_string(offset)));
    }
    
    return {};
}

Result<u32> UARTController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "UART controller not initialized"));
    }
    
    Address offset = address - (UART0_BASE_ADDR + controller_id_ * 0x100);
    
    switch (offset) {
        case 0x00: // Data register / Divisor latch low
            if (registers_.line_control & 0x80) { // DLAB bit set
                return registers_.divisor_latch_low;
            } else {
                // Receive data
                if (!rx_fifo_.empty()) {
                    u8 data = rx_fifo_.front().data;
                    rx_fifo_.pop();
                    
                    if (rx_fifo_.empty()) {
                        registers_.line_status &= ~static_cast<u32>(UARTLineStatus::DATA_READY);
                    }
                    
                    return data;
                } else {
                    statistics_.fifo_underflows++;
                    return 0;
                }
            }
            
        case 0x01: // Interrupt enable / Divisor latch high
            if (registers_.line_control & 0x80) { // DLAB bit set
                return registers_.divisor_latch_high;
            } else {
                return registers_.interrupt_enable;
            }
            
        case 0x02: // Interrupt identification register
            update_line_status();
            return registers_.interrupt_id;
            
        case 0x03: // Line control register
            return registers_.line_control;
            
        case 0x04: // Modem control register
            return registers_.modem_control;
            
        case 0x05: // Line status register
            update_line_status();
            return registers_.line_status;
            
        case 0x06: // Modem status register
            update_modem_status();
            return registers_.modem_status;
            
        case 0x07: // Scratch register
            return registers_.scratch;
            
        default:
            return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid UART register offset: 0x" + std::to_string(offset)));
    }
}

void UARTController::update() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    update_transmission();
    update_reception();
    process_tx_fifo();
    process_rx_fifo();
    simulate_external_device();
}

void UARTController::update_transmission() {
    if (!current_tx_frame_ && !tx_fifo_.empty()) {
        // Start transmitting next frame
        current_tx_frame_ = std::make_unique<UARTFrame>(tx_fifo_.front());
        tx_fifo_.pop();
        tx_start_time_ = std::chrono::steady_clock::now();
        
        registers_.line_status &= ~static_cast<u32>(UARTLineStatus::TX_EMPTY);
        
        if (tx_fifo_.empty()) {
            registers_.line_status |= static_cast<u32>(UARTLineStatus::TX_HOLDING_EMPTY);
            trigger_interrupt(UARTInterruptType::TX_HOLDING_EMPTY);
        }
    }
    
    if (current_tx_frame_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - tx_start_time_);
        u32 frame_time = calculate_frame_time_us();
        
        if (elapsed.count() >= frame_time) {
            // Frame transmission complete
            statistics_.frames_sent++;
            current_tx_frame_.reset();
            
            if (tx_fifo_.empty()) {
                registers_.line_status |= static_cast<u32>(UARTLineStatus::TX_EMPTY);
            }
        }
    }
}

void UARTController::update_reception() {
    // Check for RX timeout
    if (!rx_fifo_.empty()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_time_);
        
        if (elapsed.count() >= timeout_ms_) {
            trigger_interrupt(UARTInterruptType::RX_TIMEOUT);
        }
    }
}

void UARTController::process_tx_fifo() {
    // TX FIFO is processed in update_transmission()
}

void UARTController::process_rx_fifo() {
    // Check if we have data available
    if (!rx_fifo_.empty()) {
        registers_.line_status |= static_cast<u32>(UARTLineStatus::DATA_READY);
        trigger_interrupt(UARTInterruptType::RX_DATA_AVAILABLE);
    }
}

void UARTController::trigger_interrupt(UARTInterruptType interrupt_type) {
    if (registers_.interrupt_enable & static_cast<u8>(interrupt_type)) {
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(
                static_cast<InterruptType>(static_cast<u8>(InterruptType::UART0) + controller_id_));
        }
        
        // Update interrupt identification register
        registers_.interrupt_id = static_cast<u32>(interrupt_type);
    }
}

void UARTController::update_line_status() {
    // Update TX status
    if (tx_fifo_.empty() && !current_tx_frame_) {
        registers_.line_status |= static_cast<u32>(UARTLineStatus::TX_EMPTY | UARTLineStatus::TX_HOLDING_EMPTY);
    }
    
    // Update RX status
    if (!rx_fifo_.empty()) {
        registers_.line_status |= static_cast<u32>(UARTLineStatus::DATA_READY);
    } else {
        registers_.line_status &= ~static_cast<u32>(UARTLineStatus::DATA_READY);
    }
}

void UARTController::update_modem_status() {
    registers_.modem_status = (cts_state_ ? 0x10 : 0) |
                             (dsr_state_ ? 0x20 : 0) |
                             (cd_state_ ? 0x80 : 0) |
                             (ri_state_ ? 0x40 : 0);
}

void UARTController::simulate_external_device() {
    // Simple echo simulation for testing
    static auto last_echo = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_echo);
    
    // Simulate periodic data reception (e.g., from GPS, sensor)
    if (elapsed.count() >= 1000 && rx_fifo_.size() < RX_FIFO_SIZE / 2) {
        UARTFrame frame;
        frame.data = 0x30 + (rand() % 10); // Random digit
        frame.timestamp = now;
        rx_fifo_.push(frame);
        
        last_rx_time_ = now;
        last_echo = now;
        statistics_.frames_received++;
    }
}

u32 UARTController::calculate_frame_time_us() const {
    // Calculate time for one frame (start + data + parity + stop bits)
    u32 total_bits = 1; // Start bit
    total_bits += static_cast<u32>(data_bits_);
    
    if (parity_ != UARTParity::NONE) {
        total_bits += 1; // Parity bit
    }
    
    if (stop_bits_ == UARTStopBits::STOP_1) {
        total_bits += 1;
    } else if (stop_bits_ == UARTStopBits::STOP_1_5) {
        total_bits += 1; // Simplified to 1 bit
    } else {
        total_bits += 2;
    }
    
    return (total_bits * 1000000) / baud_rate_;
}

bool UARTController::validate_frame(const UARTFrame& frame) const {
    // Simple frame validation
    return !frame.has_parity_error && !frame.has_framing_error;
}

void UARTController::handle_break_condition() {
    statistics_.break_conditions++;
    registers_.line_status |= static_cast<u32>(UARTLineStatus::BREAK_INTERRUPT);
    trigger_interrupt(UARTInterruptType::BREAK_DETECTED);
}

bool UARTController::is_tx_busy() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return current_tx_frame_ != nullptr || !tx_fifo_.empty();
}

bool UARTController::is_rx_data_available() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return !rx_fifo_.empty();
}

size_t UARTController::get_rx_data_count() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    return rx_fifo_.size();
}

void UARTController::clear_statistics() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    statistics_ = {};
}

void UARTController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== UART Controller {} Status ===", controller_id_);
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Configuration: {} bps, {}-{}-{}, flow={}",
                          baud_rate_, static_cast<u8>(data_bits_),
                          static_cast<u8>(parity_), static_cast<u8>(stop_bits_),
                          static_cast<u8>(flow_control_));
        
        COMPONENT_LOG_INFO("TX busy: {}, RX data available: {}",
                          is_tx_busy(), is_rx_data_available());
        
        COMPONENT_LOG_INFO("TX FIFO: {}/{}", tx_fifo_.size(), TX_FIFO_SIZE);
        COMPONENT_LOG_INFO("RX FIFO: {}/{}", rx_fifo_.size(), RX_FIFO_SIZE);
        
        COMPONENT_LOG_INFO("Modem signals: RTS={}, DTR={}, CTS={}, DSR={}, CD={}, RI={}",
                          rts_state_, dtr_state_, cts_state_, dsr_state_, cd_state_, ri_state_);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Bytes transmitted: {}", statistics_.bytes_transmitted);
        COMPONENT_LOG_INFO("  Bytes received: {}", statistics_.bytes_received);
        COMPONENT_LOG_INFO("  Frames sent: {}", statistics_.frames_sent);
        COMPONENT_LOG_INFO("  Frames received: {}", statistics_.frames_received);
        COMPONENT_LOG_INFO("  Parity errors: {}", statistics_.parity_errors);
        COMPONENT_LOG_INFO("  Framing errors: {}", statistics_.framing_errors);
        COMPONENT_LOG_INFO("  Overrun errors: {}", statistics_.overrun_errors);
        COMPONENT_LOG_INFO("  Break conditions: {}", statistics_.break_conditions);
        COMPONENT_LOG_INFO("  FIFO overflows: {}", statistics_.fifo_overflows);
        COMPONENT_LOG_INFO("  FIFO underflows: {}", statistics_.fifo_underflows);
    }
}

}  // namespace m5tab5::emulator