#include "emulator/connectivity/rs485_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace m5tab5::emulator {

RS485Controller::RS485Controller()
    : initialized_(false)
    , interrupt_controller_(nullptr)
    , gpio_controller_(nullptr)
    , thread_running_(false) {
    
    // Initialize default configuration
    config_.mode = RS485Mode::HALF_DUPLEX;
    config_.baud_rate = RS485BaudRate::BAUD_115200;
    config_.data_bits = RS485DataBits::DATA_8;
    config_.parity = RS485Parity::NONE;
    config_.stop_bits = RS485StopBits::STOP_1;
    config_.station_address = 1;
    
    // Initialize hardware state
    hardware_.enabled = false;
    hardware_.direction_tx = false;
    hardware_.de_pin = 2;  // Driver Enable on GPIO2
    hardware_.re_pin = 3;  // Receiver Enable on GPIO3 (active low)
    hardware_.termination_enabled = true;
    hardware_.bias_enabled = true;
    
    // Initialize bus simulator
    bus_simulator_.cable_attenuation_db = 0.5f;
    bus_simulator_.ambient_noise_level = 5;
    bus_simulator_.termination_mismatch = false;
    
    // Initialize Modbus registers
    for (u16 addr = 0; addr < 1000; ++addr) {
        ModbusRegister reg;
        reg.address = addr;
        reg.value = 0;
        reg.readable = true;
        reg.writable = true;
        protocol_handler_.modbus_registers[addr] = reg;
    }
    
    LOG_DEBUG("RS485Controller", "RS-485 controller initialized");
}

RS485Controller::~RS485Controller() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> RS485Controller::initialize(const Configuration& config,
                                        InterruptController* interrupt_controller,
                                        GPIOController* gpio_controller) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "RS-485 controller already initialized");
    }
    
    if (!interrupt_controller || !gpio_controller) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Required controllers cannot be null");
    }
    
    interrupt_controller_ = interrupt_controller;
    gpio_controller_ = gpio_controller;
    
    // Configure GPIO pins for RS-485
    auto de_result = gpio_controller_->configure_pin(hardware_.de_pin, 
        GPIOMode::OUTPUT, GPIOPull::NONE);
    if (!de_result) {
        return make_error(ErrorCode::DEVICE_ERROR, "Failed to configure DE pin: " + de_result.error().message);
    }
    
    auto re_result = gpio_controller_->configure_pin(hardware_.re_pin, 
        GPIOMode::OUTPUT, GPIOPull::NONE);
    if (!re_result) {
        return make_error(ErrorCode::DEVICE_ERROR, "Failed to configure RE pin: " + re_result.error().message);
    }
    
    // Set initial direction to receive
    gpio_controller_->write_pin(hardware_.de_pin, false);  // DE = 0 (receive)
    gpio_controller_->write_pin(hardware_.re_pin, false);  // RE = 0 (enabled)
    
    // Calculate timing parameters
    calculate_bus_timing();
    
    // Start background processing thread
    thread_running_ = true;
    rs485_thread_ = std::make_unique<std::thread>([this]() {
        while (thread_running_) {
            {
                std::lock_guard<std::mutex> lock(rs485_mutex_);
                if (initialized_ && hardware_.enabled) {
                    simulate_bus_activity();
                    simulate_data_transmission();
                    simulate_data_reception();
                    simulate_collisions();
                    simulate_noise_and_errors();
                    process_tx_queue();
                    process_rx_queue();
                    update_bus_state();
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100)); // High frequency updates
        }
    });
    
    last_update_ = std::chrono::steady_clock::now();
    statistics_.last_reset = last_update_;
    
    initialized_ = true;
    
    LOG_INFO("RS485Controller", "RS-485 controller initialized successfully");
    trigger_rs485_event(RS485EventType::BUS_IDLE);
    
    return {};
}

Result<void> RS485Controller::shutdown() {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Stop background thread
    thread_running_ = false;
    if (rs485_thread_ && rs485_thread_->joinable()) {
        rs485_mutex_.unlock();
        rs485_thread_->join();
        rs485_mutex_.lock();
    }
    rs485_thread_.reset();
    
    // Clear queues and buffers
    while (!transceiver_.tx_queue.empty()) {
        transceiver_.tx_queue.pop();
    }
    while (!transceiver_.rx_queue.empty()) {
        transceiver_.rx_queue.pop();
    }
    transceiver_.rx_buffer.clear();
    
    // Disable hardware
    hardware_.enabled = false;
    if (gpio_controller_) {
        gpio_controller_->write_pin(hardware_.de_pin, false);
        gpio_controller_->write_pin(hardware_.re_pin, true); // Disable receiver
    }
    
    interrupt_controller_ = nullptr;
    gpio_controller_ = nullptr;
    initialized_ = false;
    
    LOG_INFO("RS485Controller", "RS-485 controller shutdown complete");
    return {};
}

Result<void> RS485Controller::configure(const RS485Config& config) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    // Validate configuration
    if (config.tx_buffer_size == 0 || config.rx_buffer_size == 0) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Buffer sizes must be non-zero");
    }
    
    if (config.station_address == 0 && config.address_mode != RS485AddressMode::DISABLED) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid station address for addressing mode");
    }
    
    config_ = config;
    
    // Update hardware settings
    hardware_.termination_enabled = config.termination_enabled;
    hardware_.bias_enabled = config.bias_resistors_enabled;
    
    // Update timing
    calculate_bus_timing();
    
    // Initialize protocol handler
    if (config.protocol == RS485Protocol::MODBUS_RTU) {
        protocol_handler_.modbus_slave_address = config.station_address;
        protocol_handler_.current_protocol = RS485Protocol::MODBUS_RTU;
    }
    
    // Configure error handling
    error_handler_.echo_suppression_enabled = config.echo_suppression;
    
    LOG_INFO("RS485Controller", "RS-485 configured: {}bps, {}-{}-{}, addr={}", 
             static_cast<u32>(config.baud_rate),
             static_cast<u8>(config.data_bits),
             static_cast<u8>(config.parity),
             static_cast<u8>(config.stop_bits),
             config.station_address);
    
    return {};
}

Result<void> RS485Controller::enable(bool enable) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (hardware_.enabled == enable) {
        return {}; // Already in requested state
    }
    
    hardware_.enabled = enable;
    
    if (enable) {
        // Set direction to receive by default
        hardware_.direction_tx = false;
        update_direction_control();
        
        // Clear any pending errors
        error_handler_.last_error = RS485ErrorType::NONE;
        
        trigger_rs485_event(RS485EventType::BUS_IDLE);
    } else {
        // Flush any pending transmissions
        while (!transceiver_.tx_queue.empty()) {
            transceiver_.tx_queue.pop();
        }
        transceiver_.tx_in_progress = false;
        
        // Disable transceivers
        if (gpio_controller_) {
            gpio_controller_->write_pin(hardware_.de_pin, false);
            gpio_controller_->write_pin(hardware_.re_pin, true);
        }
    }
    
    LOG_INFO("RS485Controller", "RS-485 {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> RS485Controller::set_direction(bool transmit) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (!hardware_.enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "RS-485 not enabled");
    }
    
    if (hardware_.direction_tx != transmit) {
        hardware_.direction_tx = transmit;
        update_direction_control();
        
        // Add turnaround delay
        if (config_.turnaround_delay_us > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(config_.turnaround_delay_us));
        }
        
        trigger_rs485_event(RS485EventType::DIRECTION_CHANGED);
    }
    
    return {};
}

Result<void> RS485Controller::send_frame(const RS485Frame& frame) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (!hardware_.enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "RS-485 not enabled");
    }
    
    if (frame.data.empty() || frame.data.size() > MAX_FRAME_SIZE) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid frame size");
    }
    
    if (transceiver_.tx_queue.size() >= config_.tx_buffer_size) {
        return make_error(ErrorCode::BUFFER_FULL, "TX buffer full");
    }
    
    RS485Frame tx_frame = frame;
    tx_frame.timestamp = std::chrono::steady_clock::now();
    
    // Add CRC if enabled
    if (config_.crc_enabled) {
        add_crc_to_frame(tx_frame);
    }
    
    transceiver_.tx_queue.push(tx_frame);
    
    LOG_DEBUG("RS485Controller", "Queued frame for transmission: {} bytes to address {}", 
              frame.data.size(), frame.address);
    
    return {};
}

Result<void> RS485Controller::send_data(const std::vector<u8>& data, u8 address) {
    RS485Frame frame;
    frame.address = address;
    frame.data = data;
    frame.is_broadcast = (address == BROADCAST_ADDRESS);
    
    return send_frame(frame);
}

Result<RS485Frame> RS485Controller::receive_frame(u32 timeout_ms) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (!hardware_.enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "RS-485 not enabled");
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (transceiver_.rx_queue.empty()) {
        if (timeout_ms > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            if (elapsed >= timeout_ms) {
                return make_error(ErrorCode::TIMEOUT, "Receive timeout");
            }
        }
        
        // Unlock and wait briefly to allow background thread to process
        rs485_mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        rs485_mutex_.lock();
        
        if (!initialized_ || !hardware_.enabled) {
            return make_error(ErrorCode::DEVICE_ERROR, "Device disabled during receive");
        }
    }
    
    RS485Frame frame = transceiver_.rx_queue.front();
    transceiver_.rx_queue.pop();
    
    return frame;
}

Result<void> RS485Controller::modbus_send_request(u8 slave_address, u8 function_code, const std::vector<u8>& data) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (config_.protocol != RS485Protocol::MODBUS_RTU) {
        return make_error(ErrorCode::INVALID_STATE, "Not in Modbus RTU mode");
    }
    
    ModbusFrame modbus_frame;
    modbus_frame.slave_address = slave_address;
    modbus_frame.function_code = function_code;
    modbus_frame.data = data;
    
    auto frame_data = build_modbus_frame(modbus_frame);
    
    RS485Frame rs485_frame;
    rs485_frame.address = slave_address;
    rs485_frame.data = frame_data;
    rs485_frame.is_broadcast = (slave_address == BROADCAST_ADDRESS);
    
    // Store request info for response matching
    protocol_handler_.last_modbus_request = std::chrono::steady_clock::now();
    protocol_handler_.pending_function_code = function_code;
    protocol_handler_.pending_data = data;
    protocol_handler_.modbus_master_mode = true;
    
    auto result = send_frame(rs485_frame);
    if (result) {
        statistics_.frames_sent++;
        LOG_DEBUG("RS485Controller", "Sent Modbus request: slave={}, func={:02X}, {} bytes", 
                  slave_address, function_code, data.size());
    }
    
    return result;
}

Result<ModbusFrame> RS485Controller::modbus_receive_response(u32 timeout_ms) {
    auto frame_result = receive_frame(timeout_ms);
    if (!frame_result) {
        return make_error(frame_result.error().code, frame_result.error().message);
    }
    
    auto rs485_frame = frame_result.value();
    
    // Parse Modbus frame
    auto modbus_frame = parse_modbus_frame(rs485_frame.data);
    
    // Validate response matches request
    if (modbus_frame.function_code != protocol_handler_.pending_function_code &&
        modbus_frame.function_code != (protocol_handler_.pending_function_code | 0x80)) {
        return make_error(ErrorCode::PROTOCOL_ERROR, "Response function code mismatch");
    }
    
    LOG_DEBUG("RS485Controller", "Received Modbus response: slave={}, func={:02X}, {} bytes", 
              modbus_frame.slave_address, modbus_frame.function_code, modbus_frame.data.size());
    
    return modbus_frame;
}

Result<void> RS485Controller::modbus_set_register(u16 address, u16 value) {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    auto it = protocol_handler_.modbus_registers.find(address);
    if (it == protocol_handler_.modbus_registers.end()) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Register address not found");
    }
    
    if (!it->second.writable) {
        return make_error(ErrorCode::ACCESS_DENIED, "Register is read-only");
    }
    
    it->second.value = value;
    
    LOG_DEBUG("RS485Controller", "Set Modbus register {}: {}", address, value);
    return {};
}

Result<u16> RS485Controller::modbus_get_register(u16 address) const {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    auto it = protocol_handler_.modbus_registers.find(address);
    if (it == protocol_handler_.modbus_registers.end()) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Register address not found");
    }
    
    if (!it->second.readable) {
        return make_error(ErrorCode::ACCESS_DENIED, "Register is write-only");
    }
    
    return it->second.value;
}

Result<std::vector<u8>> RS485Controller::scan_bus() {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "RS-485 controller not initialized");
    }
    
    if (!hardware_.enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "RS-485 not enabled");
    }
    
    std::vector<u8> active_stations;
    
    // Simulate scanning by checking recent activity
    auto now = std::chrono::steady_clock::now();
    for (const auto& [station, last_activity] : bus_simulator_.station_activity) {
        auto idle_time = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_activity).count();
        
        if (idle_time < 30) { // Consider active if seen in last 30 seconds
            active_stations.push_back(station);
        }
    }
    
    // Add some simulated stations for testing
    if (active_stations.empty()) {
        active_stations = {1, 2, 3, 5, 10}; // Common addresses
    }
    
    LOG_INFO("RS485Controller", "Bus scan found {} active stations", active_stations.size());
    
    return active_stations;
}

void RS485Controller::update() {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < 10) { // Update at 100Hz maximum
        return;
    }
    
    update_statistics();
    detect_active_stations();
    
    // Update power consumption
    statistics_.power_consumption_mw = calculate_power_consumption();
    
    last_update_ = now;
}

void RS485Controller::simulate_bus_activity() {
    // Simulate random bus activity from other stations
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> activity_dist(0, 10000);
    
    if (activity_dist(gen) < 10) { // 0.1% chance of activity per cycle
        // Generate random frame from another station
        std::uniform_int_distribution<u8> addr_dist(1, 10);
        std::uniform_int_distribution<size_t> size_dist(1, 32);
        std::uniform_int_distribution<u8> data_dist(0, 255);
        
        RS485Frame frame;
        frame.address = addr_dist(gen);
        frame.data.resize(size_dist(gen));
        for (auto& byte : frame.data) {
            byte = data_dist(gen);
        }
        frame.timestamp = std::chrono::steady_clock::now();
        frame.signal_quality = 85 + (activity_dist(gen) % 15); // 85-100%
        frame.rssi_dbm = -20 - (activity_dist(gen) % 40); // -20 to -60 dBm
        
        // Update station activity
        bus_simulator_.station_activity[frame.address] = frame.timestamp;
        
        if (!bus_simulator_.active_stations.empty() && 
            std::find(bus_simulator_.active_stations.begin(), 
                     bus_simulator_.active_stations.end(), 
                     frame.address) == bus_simulator_.active_stations.end()) {
            bus_simulator_.active_stations.push_back(frame.address);
        }
        
        // Add to bus frames if not from our station
        if (frame.address != config_.station_address) {
            bus_simulator_.bus_frames.push(frame);
        }
    }
}

void RS485Controller::simulate_data_transmission() {
    if (transceiver_.tx_in_progress) {
        auto now = std::chrono::steady_clock::now();
        auto tx_time = std::chrono::duration_cast<std::chrono::microseconds>(
            now - transceiver_.tx_start_time).count();
        
        // Calculate expected transmission time
        u32 frame_bits = (transceiver_.current_tx_frame.data.size() * 
                         (static_cast<u8>(config_.data_bits) + 1 + static_cast<u8>(config_.stop_bits)));
        if (config_.parity != RS485Parity::NONE) {
            frame_bits += transceiver_.current_tx_frame.data.size();
        }
        
        u32 expected_time_us = frame_bits * timing_.bit_time_us;
        
        if (tx_time >= expected_time_us) {
            // Transmission complete
            transceiver_.tx_in_progress = false;
            
            // Switch back to receive mode for half-duplex
            if (config_.mode == RS485Mode::HALF_DUPLEX) {
                hardware_.direction_tx = false;
                update_direction_control();
            }
            
            statistics_.frames_sent++;
            statistics_.bytes_sent += transceiver_.current_tx_frame.data.size();
            
            LOG_DEBUG("RS485Controller", "Frame transmission completed: {} bytes", 
                      transceiver_.current_tx_frame.data.size());
            
            trigger_rs485_event(RS485EventType::DATA_SENT, RS485ErrorType::NONE,
                               transceiver_.current_tx_frame.data);
            
            // Add inter-frame delay
            std::this_thread::sleep_for(std::chrono::microseconds(config_.inter_frame_delay_us));
        }
    }
}

void RS485Controller::simulate_data_reception() {
    // Process frames from the bus
    while (!bus_simulator_.bus_frames.empty()) {
        auto frame = bus_simulator_.bus_frames.front();
        bus_simulator_.bus_frames.pop();
        
        // Check if frame is for us (address match or broadcast)
        bool for_us = (frame.address == config_.station_address) || 
                      (frame.is_broadcast) ||
                      (config_.address_mode == RS485AddressMode::DISABLED);
        
        if (for_us) {
            // Simulate reception processing
            frame.error = RS485ErrorType::NONE;
            frame.is_valid = true;
            
            // Simulate potential errors
            if (should_inject_error()) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> error_dist(0, 4);
                
                switch (error_dist(gen)) {
                    case 0: frame.error = RS485ErrorType::FRAMING; break;
                    case 1: frame.error = RS485ErrorType::PARITY; break;
                    case 2: frame.error = RS485ErrorType::NOISE; break;
                    case 3: frame.error = RS485ErrorType::CRC; break;
                    default: break;
                }
                
                frame.is_valid = false;
                handle_frame_error(frame.error);
            }
            
            // Validate frame
            if (frame.is_valid && !validate_frame(frame)) {
                frame.is_valid = false;
                frame.error = RS485ErrorType::CRC;
                handle_frame_error(RS485ErrorType::CRC);
            }
            
            // Add to receive queue if valid
            if (frame.is_valid || !config_.crc_enabled) {
                if (transceiver_.rx_queue.size() < config_.rx_buffer_size) {
                    transceiver_.rx_queue.push(frame);
                    statistics_.frames_received++;
                    statistics_.bytes_received += frame.data.size();
                    
                    trigger_rs485_event(RS485EventType::DATA_RECEIVED, frame.error, frame.data, frame.address);
                    
                    if (data_callback_) {
                        data_callback_(frame);
                    }
                    
                    // Process protocol-specific handling
                    if (config_.protocol == RS485Protocol::MODBUS_RTU) {
                        process_modbus_frame(frame);
                    }
                } else {
                    statistics_.overrun_errors++;
                    handle_frame_error(RS485ErrorType::OVERRUN);
                }
            }
        }
    }
}

void RS485Controller::simulate_collisions() {
    if (config_.mode != RS485Mode::MULTI_DROP) {
        return;
    }
    
    // Simulate collision detection
    if (transceiver_.tx_in_progress && bus_simulator_.active_stations.size() > 1) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> collision_dist(0, 1000);
        
        if (collision_dist(gen) < 5) { // 0.5% chance of collision
            transceiver_.collision_detected = true;
            bus_simulator_.bus_busy = true;
            
            statistics_.collision_errors++;
            handle_frame_error(RS485ErrorType::COLLISION);
            
            LOG_WARN("RS485Controller", "Bus collision detected");
            trigger_rs485_event(RS485EventType::COLLISION_DETECTED, RS485ErrorType::COLLISION);
        }
    }
}

void RS485Controller::simulate_noise_and_errors() {
    if (error_handler_.noise_injection_level > 0) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> noise_dist(0, 100);
        
        if (noise_dist(gen) < error_handler_.noise_injection_level) {
            // Inject random error
            std::uniform_int_distribution<> error_type_dist(1, 7);
            RS485ErrorType error = static_cast<RS485ErrorType>(error_type_dist(gen));
            
            handle_frame_error(error);
            trigger_rs485_event(RS485EventType::ERROR_OCCURRED, error);
        }
    }
}

void RS485Controller::process_tx_queue() {
    if (transceiver_.tx_in_progress || transceiver_.tx_queue.empty()) {
        return;
    }
    
    // Check if bus is available
    if (bus_simulator_.bus_busy) {
        return;
    }
    
    // Get next frame to transmit
    auto frame = transceiver_.tx_queue.front();
    transceiver_.tx_queue.pop();
    
    // Switch to transmit mode for half-duplex
    if (config_.mode == RS485Mode::HALF_DUPLEX) {
        hardware_.direction_tx = true;
        update_direction_control();
        
        // Add turnaround delay
        std::this_thread::sleep_for(std::chrono::microseconds(config_.turnaround_delay_us));
    }
    
    // Start transmission
    transceiver_.tx_in_progress = true;
    transceiver_.tx_start_time = std::chrono::steady_clock::now();
    transceiver_.current_tx_frame = frame;
    transceiver_.current_tx_bytes = frame.data.size();
    
    bus_simulator_.bus_busy = true;
    bus_simulator_.current_transmitter = config_.station_address;
    
    LOG_DEBUG("RS485Controller", "Started transmission: {} bytes to address {}", 
              frame.data.size(), frame.address);
}

void RS485Controller::process_rx_queue() {
    // Process received frames and handle protocol-specific operations
    // This is handled in simulate_data_reception()
}

void RS485Controller::process_modbus_frame(const RS485Frame& frame) {
    if (frame.address != protocol_handler_.modbus_slave_address && 
        frame.address != BROADCAST_ADDRESS) {
        return; // Not for us
    }
    
    auto modbus_frame = parse_modbus_frame(frame.data);
    
    // Handle as slave if not in master mode or if broadcast
    if (!protocol_handler_.modbus_master_mode || frame.is_broadcast) {
        // Process Modbus slave request
        ModbusFrame response;
        response.slave_address = protocol_handler_.modbus_slave_address;
        response.function_code = modbus_frame.function_code;
        
        switch (modbus_frame.function_code) {
            case 0x03: { // Read Holding Registers
                if (modbus_frame.data.size() >= 4) {
                    u16 start_addr = (modbus_frame.data[0] << 8) | modbus_frame.data[1];
                    u16 count = (modbus_frame.data[2] << 8) | modbus_frame.data[3];
                    
                    response.data.push_back(count * 2); // Byte count
                    
                    for (u16 i = 0; i < count; ++i) {
                        u16 addr = start_addr + i;
                        auto it = protocol_handler_.modbus_registers.find(addr);
                        u16 value = (it != protocol_handler_.modbus_registers.end()) ? it->second.value : 0;
                        
                        response.data.push_back((value >> 8) & 0xFF);
                        response.data.push_back(value & 0xFF);
                    }
                }
                break;
            }
            
            case 0x06: { // Write Single Register
                if (modbus_frame.data.size() >= 4) {
                    u16 addr = (modbus_frame.data[0] << 8) | modbus_frame.data[1];
                    u16 value = (modbus_frame.data[2] << 8) | modbus_frame.data[3];
                    
                    auto it = protocol_handler_.modbus_registers.find(addr);
                    if (it != protocol_handler_.modbus_registers.end() && it->second.writable) {
                        it->second.value = value;
                        response.data = modbus_frame.data; // Echo back
                    } else {
                        response.is_exception = true;
                        response.function_code |= 0x80;
                        response.exception_code = 0x02; // Illegal data address
                    }
                }
                break;
            }
            
            default:
                response.is_exception = true;
                response.function_code |= 0x80;
                response.exception_code = 0x01; // Illegal function
                break;
        }
        
        // Send response (if not broadcast)
        if (!frame.is_broadcast) {
            auto response_data = build_modbus_frame(response);
            RS485Frame rs485_response;
            rs485_response.address = modbus_frame.slave_address;
            rs485_response.data = response_data;
            
            transceiver_.tx_queue.push(rs485_response);
        }
    }
}

ModbusFrame RS485Controller::parse_modbus_frame(const std::vector<u8>& data) {
    ModbusFrame frame;
    
    if (data.size() < 4) { // Minimum: addr + func + crc
        frame.is_exception = true;
        return frame;
    }
    
    frame.slave_address = data[0];
    frame.function_code = data[1];
    
    // Extract data (excluding address, function code, and CRC)
    if (data.size() > 4) {
        frame.data.assign(data.begin() + 2, data.end() - 2);
    }
    
    // Extract CRC
    frame.crc = (data[data.size() - 2] << 8) | data[data.size() - 1];
    
    // Verify CRC
    std::vector<u8> crc_data(data.begin(), data.end() - 2);
    u16 calculated_crc = calculate_modbus_crc(crc_data);
    
    if (calculated_crc != frame.crc) {
        frame.is_exception = true;
        frame.exception_code = 0x04; // Slave device failure
    }
    
    return frame;
}

std::vector<u8> RS485Controller::build_modbus_frame(const ModbusFrame& frame) {
    std::vector<u8> data;
    
    data.push_back(frame.slave_address);
    data.push_back(frame.function_code);
    
    if (frame.is_exception) {
        data.push_back(frame.exception_code);
    } else {
        data.insert(data.end(), frame.data.begin(), frame.data.end());
    }
    
    // Calculate and append CRC
    u16 crc = calculate_modbus_crc(data);
    data.push_back(crc & 0xFF);
    data.push_back((crc >> 8) & 0xFF);
    
    return data;
}

u16 RS485Controller::calculate_modbus_crc(const std::vector<u8>& data) {
    u16 crc = 0xFFFF;
    
    for (u8 byte : data) {
        crc ^= byte;
        
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void RS485Controller::update_bus_state() {
    auto now = std::chrono::steady_clock::now();
    
    // Update bus busy state
    if (transceiver_.tx_in_progress) {
        bus_simulator_.bus_busy = true;
        bus_simulator_.current_transmitter = config_.station_address;
    } else {
        // Check if any other station is transmitting
        bool other_activity = false;
        for (const auto& [station, activity_time] : bus_simulator_.station_activity) {
            auto idle_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - activity_time).count();
            
            if (idle_time < 100 && station != config_.station_address) { // Active in last 100ms
                other_activity = true;
                break;
            }
        }
        
        if (!other_activity) {
            bus_simulator_.bus_busy = false;
            bus_simulator_.current_transmitter = 0;
            
            if (bus_simulator_.bus_idle_start == std::chrono::steady_clock::time_point{}) {
                bus_simulator_.bus_idle_start = now;
                trigger_rs485_event(RS485EventType::BUS_IDLE);
            }
        } else {
            bus_simulator_.bus_idle_start = std::chrono::steady_clock::time_point{};
        }
    }
    
    // Calculate bus load
    static std::chrono::steady_clock::time_point last_load_calc;
    static u64 bytes_at_last_calc = 0;
    
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_load_calc).count() >= 1) {
        u64 bytes_delta = statistics_.bytes_sent + statistics_.bytes_received - bytes_at_last_calc;
        float theoretical_max = static_cast<u32>(config_.baud_rate) / 10; // Assuming 10 bits per byte
        
        bus_simulator_.bus_load_percentage = (bytes_delta / theoretical_max) * 100.0f;
        bus_simulator_.bus_load_percentage = std::min(bus_simulator_.bus_load_percentage, 100.0f);
        
        bytes_at_last_calc = statistics_.bytes_sent + statistics_.bytes_received;
        last_load_calc = now;
    }
}

void RS485Controller::detect_active_stations() {
    auto now = std::chrono::steady_clock::now();
    bus_simulator_.active_stations.clear();
    
    for (const auto& [station, last_activity] : bus_simulator_.station_activity) {
        auto idle_time = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_activity).count();
        
        if (idle_time < 60) { // Consider active if seen in last minute
            bus_simulator_.active_stations.push_back(station);
        }
    }
    
    // Always include our own station if enabled
    if (hardware_.enabled) {
        auto it = std::find(bus_simulator_.active_stations.begin(),
                           bus_simulator_.active_stations.end(),
                           config_.station_address);
        if (it == bus_simulator_.active_stations.end()) {
            bus_simulator_.active_stations.push_back(config_.station_address);
        }
    }
}

void RS485Controller::handle_collision_detection() {
    if (transceiver_.collision_detected) {
        // Stop current transmission
        transceiver_.tx_in_progress = false;
        
        // Switch back to receive mode
        if (config_.mode == RS485Mode::HALF_DUPLEX) {
            hardware_.direction_tx = false;
            update_direction_control();
        }
        
        transceiver_.collision_detected = false;
        
        // Implement collision recovery (backoff algorithm)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> backoff_dist(10, 100);
        u32 backoff_ms = backoff_dist(gen);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
    }
}

void RS485Controller::calculate_bus_timing() {
    u32 baud = static_cast<u32>(config_.baud_rate);
    timing_.bit_time_us = 1000000 / baud;
    
    // Calculate character time (data bits + start + stop + parity)
    u32 char_bits = 1 + static_cast<u8>(config_.data_bits) + static_cast<u8>(config_.stop_bits);
    if (config_.parity != RS485Parity::NONE) {
        char_bits += 1;
    }
    
    timing_.char_time_us = char_bits * timing_.bit_time_us;
    
    // Modbus requires 3.5 character times between frames
    timing_.frame_gap_us = (timing_.char_time_us * 35) / 10;
    
    LOG_DEBUG("RS485Controller", "Timing calculated: bit={}us, char={}us, gap={}us",
              timing_.bit_time_us, timing_.char_time_us, timing_.frame_gap_us);
}

void RS485Controller::update_direction_control() {
    if (!gpio_controller_) {
        return;
    }
    
    if (hardware_.direction_tx) {
        // Enable transmitter, disable receiver
        gpio_controller_->write_pin(hardware_.de_pin, config_.driver_enable_polarity != 0);
        gpio_controller_->write_pin(hardware_.re_pin, true); // Disable receiver (active low)
    } else {
        // Disable transmitter, enable receiver
        gpio_controller_->write_pin(hardware_.de_pin, config_.driver_enable_polarity == 0);
        gpio_controller_->write_pin(hardware_.re_pin, false); // Enable receiver (active low)
    }
}

bool RS485Controller::validate_frame(const RS485Frame& frame) {
    if (config_.crc_enabled) {
        return verify_frame_crc(frame);
    }
    
    // Basic validation
    return !frame.data.empty() && frame.data.size() <= MAX_FRAME_SIZE;
}

void RS485Controller::handle_frame_error(RS485ErrorType error) {
    error_handler_.last_error = error;
    error_handler_.last_error_time = std::chrono::steady_clock::now();
    
    if (error_handler_.error_history.size() >= error_handler_.max_error_history) {
        error_handler_.error_history.erase(error_handler_.error_history.begin());
    }
    error_handler_.error_history.push_back(error);
    
    // Update error statistics
    switch (error) {
        case RS485ErrorType::FRAMING:
            statistics_.framing_errors++;
            break;
        case RS485ErrorType::PARITY:
            statistics_.parity_errors++;
            break;
        case RS485ErrorType::OVERRUN:
            statistics_.overrun_errors++;
            break;
        case RS485ErrorType::COLLISION:
            statistics_.collision_errors++;
            break;
        case RS485ErrorType::TIMEOUT:
            statistics_.timeout_errors++;
            break;
        case RS485ErrorType::CRC:
            statistics_.crc_errors++;
            break;
        default:
            break;
    }
    
    if (error_callback_) {
        error_callback_(error, "RS-485 communication error");
    }
}

bool RS485Controller::should_inject_error() {
    if (error_handler_.noise_injection_level == 0) {
        return false;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> error_dist(0, 1000);
    
    return error_dist(gen) < error_handler_.noise_injection_level;
}

u16 RS485Controller::calculate_crc16(const std::vector<u8>& data) {
    u16 crc = 0xFFFF;
    
    for (u8 byte : data) {
        crc ^= byte;
        
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8005; // CRC-16-IBM polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void RS485Controller::add_crc_to_frame(RS485Frame& frame) {
    frame.crc = calculate_crc16(frame.data);
    frame.data.push_back(frame.crc & 0xFF);
    frame.data.push_back((frame.crc >> 8) & 0xFF);
}

bool RS485Controller::verify_frame_crc(const RS485Frame& frame) {
    if (frame.data.size() < 2) {
        return false;
    }
    
    std::vector<u8> data_without_crc(frame.data.begin(), frame.data.end() - 2);
    u16 calculated_crc = calculate_crc16(data_without_crc);
    
    u16 received_crc = frame.data[frame.data.size() - 2] | 
                       (frame.data[frame.data.size() - 1] << 8);
    
    return calculated_crc == received_crc;
}

void RS485Controller::trigger_rs485_event(RS485EventType type, RS485ErrorType error,
                                         const std::vector<u8>& data, u8 address) {
    RS485Event event;
    event.type = type;
    event.error = error;
    event.data = data;
    event.address = address;
    event.timestamp = std::chrono::steady_clock::now();
    event.event_data = 0;
    
    if (event_callback_) {
        event_callback_(event);
    }
    
    // Trigger hardware interrupt for important events
    if (interrupt_controller_) {
        switch (type) {
            case RS485EventType::DATA_RECEIVED:
            case RS485EventType::DATA_SENT:
            case RS485EventType::ERROR_OCCURRED:
            case RS485EventType::COLLISION_DETECTED:
                interrupt_controller_->trigger_interrupt(53); // RS-485 interrupt line
                break;
            default:
                break;
        }
    }
}

void RS485Controller::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - statistics_.last_reset).count();
    
    if (dt >= 1000) { // Update every second
        // Calculate error rate
        u64 total_frames = statistics_.frames_sent + statistics_.frames_received;
        u64 total_errors = statistics_.framing_errors + statistics_.parity_errors + 
                          statistics_.overrun_errors + statistics_.collision_errors +
                          statistics_.timeout_errors + statistics_.crc_errors;
        
        if (total_frames > 0) {
            statistics_.error_rate = static_cast<float>(total_errors) / total_frames;
        }
        
        // Calculate throughput
        u64 total_bytes = statistics_.bytes_sent + statistics_.bytes_received;
        statistics_.throughput_bps = (total_bytes * 8.0f * 1000.0f) / dt;
        statistics_.peak_throughput_bps = std::max(statistics_.peak_throughput_bps, 
                                                   statistics_.throughput_bps);
        
        // Update bus utilization
        statistics_.bus_utilization_percent = static_cast<u8>(bus_simulator_.bus_load_percentage);
    }
}

u32 RS485Controller::calculate_power_consumption() {
    u32 power_mw = 10; // Base power
    
    if (!hardware_.enabled) {
        return power_mw;
    }
    
    // Active power consumption
    power_mw += 50;
    
    // Transmission power
    if (hardware_.direction_tx) {
        power_mw += 100 + (hardware_.drive_strength * 10);
    }
    
    // Termination power
    if (hardware_.termination_enabled) {
        power_mw += 20;
    }
    
    // Bias resistor power
    if (hardware_.bias_enabled) {
        power_mw += 15;
    }
    
    return power_mw;
}

void RS485Controller::clear_statistics() {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    statistics_ = RS485Statistics{};
    statistics_.last_reset = std::chrono::steady_clock::now();
}

void RS485Controller::dump_status() const {
    std::lock_guard<std::mutex> lock(rs485_mutex_);
    
    LOG_INFO("RS485Controller", "=== RS-485 Controller Status ===");
    LOG_INFO("RS485Controller", "Initialized: {}", initialized_);
    LOG_INFO("RS485Controller", "Enabled: {}", hardware_.enabled);
    LOG_INFO("RS485Controller", "Mode: {}", static_cast<int>(config_.mode));
    LOG_INFO("RS485Controller", "Baud Rate: {} bps", static_cast<u32>(config_.baud_rate));
    LOG_INFO("RS485Controller", "Data Format: {}-{}-{}", 
             static_cast<u8>(config_.data_bits),
             static_cast<u8>(config_.parity),
             static_cast<u8>(config_.stop_bits));
    LOG_INFO("RS485Controller", "Station Address: {}", config_.station_address);
    LOG_INFO("RS485Controller", "Protocol: {}", static_cast<int>(config_.protocol));
    LOG_INFO("RS485Controller", "Direction: {}", hardware_.direction_tx ? "TX" : "RX");
    LOG_INFO("RS485Controller", "Termination: {}", hardware_.termination_enabled);
    LOG_INFO("RS485Controller", "Bus Busy: {}", bus_simulator_.bus_busy);
    LOG_INFO("RS485Controller", "Active Stations: {}", bus_simulator_.active_stations.size());
    LOG_INFO("RS485Controller", "TX Queue: {}", transceiver_.tx_queue.size());
    LOG_INFO("RS485Controller", "RX Queue: {}", transceiver_.rx_queue.size());
    LOG_INFO("RS485Controller", "Frames Sent: {}", statistics_.frames_sent);
    LOG_INFO("RS485Controller", "Frames Received: {}", statistics_.frames_received);
    LOG_INFO("RS485Controller", "Error Rate: {:.2f}%", statistics_.error_rate * 100.0f);
    LOG_INFO("RS485Controller", "Throughput: {:.1f} bps", statistics_.throughput_bps);
    LOG_INFO("RS485Controller", "Bus Utilization: {}%", statistics_.bus_utilization_percent);
    LOG_INFO("RS485Controller", "Power Consumption: {} mW", statistics_.power_consumption_mw);
    LOG_INFO("RS485Controller", "Last Error: {}", static_cast<int>(error_handler_.last_error));
}

}  // namespace m5tab5::emulator