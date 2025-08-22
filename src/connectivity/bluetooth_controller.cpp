#include "emulator/connectivity/bluetooth_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace m5tab5::emulator {

BluetoothController::BluetoothController()
    : initialized_(false)
    , current_mode_(BluetoothMode::OFF)
    , bluetooth_enabled_(false)
    , interrupt_controller_(nullptr)
    , device_name_("ESP32-BT")
    , thread_running_(false) {
    
    // Generate random local address
    local_address_ = generate_random_address(false);
    
    // Initialize security manager defaults
    security_manager_.security_level = 1;
    security_manager_.io_capabilities = 3; // NoInputNoOutput
    security_manager_.secure_connections = false;
    security_manager_.bonding = true;
    
    // Initialize power manager
    power_manager_.tx_power_dbm = 0;
    power_manager_.low_power_mode = false;
    
    LOG_DEBUG("BluetoothController", "Bluetooth controller initialized with address: {}", 
              local_address_.to_string());
}

BluetoothController::~BluetoothController() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> BluetoothController::initialize(const Configuration& config, 
                                           InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "Bluetooth controller already initialized");
    }
    
    if (!interrupt_controller) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Interrupt controller cannot be null");
    }
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize simulated environment
    simulate_bluetooth_environment();
    
    // Start background processing thread
    thread_running_ = true;
    bluetooth_thread_ = std::make_unique<std::thread>([this]() {
        while (thread_running_) {
            {
                std::lock_guard<std::mutex> lock(bluetooth_mutex_);
                if (initialized_ && bluetooth_enabled_) {
                    simulate_device_discovery();
                    simulate_ble_scanning();
                    simulate_ble_advertising();
                    simulate_connections();
                    simulate_data_transmission();
                    simulate_gatt_operations();
                    process_classic_bluetooth();
                    process_ble_operations();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    last_update_ = std::chrono::steady_clock::now();
    
    initialized_ = true;
    
    LOG_INFO("BluetoothController", "Bluetooth controller initialized successfully");
    trigger_bluetooth_event(BluetoothEventType::STATE_CHANGED);
    
    return {};
}

Result<void> BluetoothController::shutdown() {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Stop background thread
    thread_running_ = false;
    if (bluetooth_thread_ && bluetooth_thread_->joinable()) {
        bluetooth_mutex_.unlock();
        bluetooth_thread_->join();
        bluetooth_mutex_.lock();
    }
    bluetooth_thread_.reset();
    
    // Disconnect all devices
    for (auto& device : classic_state_.connected_devices) {
        device.is_connected = false;
        trigger_bluetooth_event(BluetoothEventType::ACL_DISCONNECTED, device.address);
    }
    classic_state_.connected_devices.clear();
    
    // Disconnect all BLE connections
    for (auto& [handle, connection] : ble_state_.connections) {
        connection.state = BLEConnectionState::DISCONNECTED;
        trigger_bluetooth_event(BluetoothEventType::BLE_DISCONNECT, connection.remote_address);
    }
    ble_state_.connections.clear();
    
    // Stop scanning and advertising
    classic_state_.discovering = false;
    ble_state_.scanning = false;
    ble_state_.advertising = false;
    
    // Clear queues and buffers
    while (!packet_processor_.tx_queue.empty()) {
        packet_processor_.tx_queue.pop();
    }
    while (!packet_processor_.rx_queue.empty()) {
        packet_processor_.rx_queue.pop();
    }
    
    bluetooth_enabled_ = false;
    current_mode_ = BluetoothMode::OFF;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    LOG_INFO("BluetoothController", "Bluetooth controller shutdown complete");
    return {};
}

Result<void> BluetoothController::set_mode(BluetoothMode mode) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ == mode) {
        return {}; // Already in requested mode
    }
    
    // Handle mode transitions
    if (current_mode_ != BluetoothMode::OFF) {
        // Stop current operations
        classic_state_.discovering = false;
        ble_state_.scanning = false;
        ble_state_.advertising = false;
    }
    
    current_mode_ = mode;
    
    switch (mode) {
        case BluetoothMode::OFF:
            bluetooth_enabled_ = false;
            classic_state_.state = BluetoothState::IDLE;
            break;
            
        case BluetoothMode::CLASSIC:
            bluetooth_enabled_ = true;
            classic_state_.state = BluetoothState::IDLE;
            break;
            
        case BluetoothMode::BLE:
            bluetooth_enabled_ = true;
            break;
            
        case BluetoothMode::DUAL:
            bluetooth_enabled_ = true;
            classic_state_.state = BluetoothState::IDLE;
            break;
    }
    
    LOG_INFO("BluetoothController", "Bluetooth mode set to {}", static_cast<int>(mode));
    trigger_bluetooth_event(BluetoothEventType::STATE_CHANGED);
    
    return {};
}

Result<void> BluetoothController::set_device_name(const std::string& name) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (name.length() > MAX_DEVICE_NAME_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Device name too long");
    }
    
    device_name_ = name;
    
    LOG_INFO("BluetoothController", "Device name set to: {}", name);
    return {};
}

Result<void> BluetoothController::start_discovery(u32 duration_s) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ != BluetoothMode::CLASSIC && current_mode_ != BluetoothMode::DUAL) {
        return make_error(ErrorCode::INVALID_STATE, "Not in Classic Bluetooth mode");
    }
    
    if (!bluetooth_enabled_) {
        return make_error(ErrorCode::DEVICE_ERROR, "Bluetooth not enabled");
    }
    
    if (classic_state_.discovering) {
        return make_error(ErrorCode::BUSY, "Discovery already in progress");
    }
    
    classic_state_.discovering = true;
    classic_state_.discovery_start = std::chrono::steady_clock::now();
    classic_state_.discovery_duration_s = duration_s;
    classic_state_.discovered_devices.clear();
    classic_state_.state = BluetoothState::DISCOVERING;
    
    statistics_.discovery_sessions++;
    
    LOG_INFO("BluetoothController", "Starting Classic Bluetooth discovery for {} seconds", duration_s);
    trigger_bluetooth_event(BluetoothEventType::DISCOVERY_STARTED);
    
    return {};
}

Result<void> BluetoothController::ble_start_scan(u32 duration_ms, bool active_scan) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ != BluetoothMode::BLE && current_mode_ != BluetoothMode::DUAL) {
        return make_error(ErrorCode::INVALID_STATE, "Not in BLE mode");
    }
    
    if (!bluetooth_enabled_) {
        return make_error(ErrorCode::DEVICE_ERROR, "Bluetooth not enabled");
    }
    
    if (ble_state_.scanning) {
        return make_error(ErrorCode::BUSY, "BLE scan already in progress");
    }
    
    ble_state_.scanning = true;
    ble_state_.scan_start = std::chrono::steady_clock::now();
    ble_state_.scan_duration_ms = duration_ms;
    ble_state_.active_scan = active_scan;
    ble_state_.scan_results.clear();
    
    LOG_INFO("BluetoothController", "Starting BLE scan for {} ms (active: {})", duration_ms, active_scan);
    
    return {};
}

Result<void> BluetoothController::ble_start_advertising(const std::vector<u8>& advertisement_data,
                                                       const std::vector<u8>& scan_response_data,
                                                       u16 interval_min, u16 interval_max) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ != BluetoothMode::BLE && current_mode_ != BluetoothMode::DUAL) {
        return make_error(ErrorCode::INVALID_STATE, "Not in BLE mode");
    }
    
    if (!bluetooth_enabled_) {
        return make_error(ErrorCode::DEVICE_ERROR, "Bluetooth not enabled");
    }
    
    if (advertisement_data.size() > MAX_ADVERTISEMENT_DATA_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Advertisement data too long");
    }
    
    if (scan_response_data.size() > MAX_SCAN_RESPONSE_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Scan response data too long");
    }
    
    ble_state_.advertisement_data = advertisement_data;
    ble_state_.scan_response_data = scan_response_data;
    ble_state_.advertising_interval_min = interval_min;
    ble_state_.advertising_interval_max = interval_max;
    ble_state_.advertising = true;
    ble_state_.last_advertisement = std::chrono::steady_clock::now();
    
    LOG_INFO("BluetoothController", "Started BLE advertising with {} byte payload", advertisement_data.size());
    
    return {};
}

Result<u16> BluetoothController::ble_connect(const BluetoothAddress& address, u16 conn_interval_min, u16 conn_interval_max) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ != BluetoothMode::BLE && current_mode_ != BluetoothMode::DUAL) {
        return make_error(ErrorCode::INVALID_STATE, "Not in BLE mode");
    }
    
    if (!bluetooth_enabled_) {
        return make_error(ErrorCode::DEVICE_ERROR, "Bluetooth not enabled");
    }
    
    if (ble_state_.connections.size() >= MAX_BLE_CONNECTIONS) {
        return make_error(ErrorCode::RESOURCE_LIMIT, "Maximum BLE connections reached");
    }
    
    // Check if already connected
    for (const auto& [handle, conn] : ble_state_.connections) {
        if (conn.remote_address == address) {
            return make_error(ErrorCode::ALREADY_CONNECTED, "Device already connected");
        }
    }
    
    // Create new connection
    u16 handle = ble_state_.next_connection_handle++;
    BLEConnection connection;
    connection.remote_address = address;
    connection.state = BLEConnectionState::CONNECTING;
    connection.connection_handle = handle;
    connection.connection_interval = conn_interval_min;
    connection.slave_latency = 0;
    connection.supervision_timeout = 400; // 4 seconds
    connection.connection_time = std::chrono::steady_clock::now();
    connection.rssi = simulate_rssi(address);
    
    ble_state_.connections[handle] = connection;
    
    statistics_.ble_connections++;
    
    LOG_INFO("BluetoothController", "Connecting to BLE device {} (handle: {})", address.to_string(), handle);
    
    return handle;
}

Result<std::vector<u8>> BluetoothController::gatt_read_characteristic(u16 connection_handle, const std::string& char_uuid) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    auto conn_it = ble_state_.connections.find(connection_handle);
    if (conn_it == ble_state_.connections.end()) {
        return make_error(ErrorCode::NOT_CONNECTED, "Connection handle not found");
    }
    
    if (conn_it->second.state != BLEConnectionState::CONNECTED) {
        return make_error(ErrorCode::NOT_CONNECTED, "Device not connected");
    }
    
    // Simulate characteristic read
    auto value = simulate_characteristic_read(char_uuid);
    
    statistics_.gatt_operations++;
    
    LOG_DEBUG("BluetoothController", "GATT read characteristic {} on handle {}: {} bytes", 
              char_uuid, connection_handle, value.size());
    
    trigger_bluetooth_event(BluetoothEventType::BLE_CHARACTERISTIC_READ, conn_it->second.remote_address, value);
    
    return value;
}

Result<void> BluetoothController::gatt_write_characteristic(u16 connection_handle, 
                                                           const std::string& char_uuid, 
                                                           const std::vector<u8>& value, 
                                                           bool with_response) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    auto conn_it = ble_state_.connections.find(connection_handle);
    if (conn_it == ble_state_.connections.end()) {
        return make_error(ErrorCode::NOT_CONNECTED, "Connection handle not found");
    }
    
    if (conn_it->second.state != BLEConnectionState::CONNECTED) {
        return make_error(ErrorCode::NOT_CONNECTED, "Device not connected");
    }
    
    if (value.size() > MAX_CHARACTERISTIC_VALUE_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Value too long");
    }
    
    // Simulate characteristic write
    bool success = simulate_characteristic_write(char_uuid, value);
    if (!success) {
        return make_error(ErrorCode::DEVICE_ERROR, "Write operation failed");
    }
    
    statistics_.gatt_operations++;
    
    LOG_DEBUG("BluetoothController", "GATT write characteristic {} on handle {}: {} bytes", 
              char_uuid, connection_handle, value.size());
    
    trigger_bluetooth_event(BluetoothEventType::BLE_CHARACTERISTIC_WRITE, conn_it->second.remote_address, value);
    
    return {};
}

Result<void> BluetoothController::send_data(const BluetoothAddress& device, 
                                           const std::vector<u8>& data, 
                                           BluetoothProfile profile) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    if (current_mode_ != BluetoothMode::CLASSIC && current_mode_ != BluetoothMode::DUAL) {
        return make_error(ErrorCode::INVALID_STATE, "Not in Classic Bluetooth mode");
    }
    
    // Find connected device
    auto device_it = std::find_if(classic_state_.connected_devices.begin(),
                                 classic_state_.connected_devices.end(),
                                 [&device](const BluetoothDevice& d) {
                                     return d.address == device && d.is_connected;
                                 });
    
    if (device_it == classic_state_.connected_devices.end()) {
        return make_error(ErrorCode::NOT_CONNECTED, "Device not connected");
    }
    
    // Add to TX queue
    packet_processor_.tx_queue.push(data);
    
    statistics_.data_packets_sent++;
    statistics_.bytes_sent += data.size();
    
    LOG_DEBUG("BluetoothController", "Sent {} bytes to device {}", data.size(), device.to_string());
    
    if (data_callback_) {
        data_callback_(device, data);
    }
    
    return {};
}

Result<void> BluetoothController::set_tx_power(i8 power_dbm) {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Bluetooth controller not initialized");
    }
    
    // ESP32-C6 Bluetooth power range: -12dBm to +9dBm
    if (power_dbm < -12 || power_dbm > 9) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "TX power out of range (-12 to +9 dBm)");
    }
    
    power_manager_.tx_power_dbm = power_dbm;
    
    LOG_DEBUG("BluetoothController", "TX power set to {} dBm", power_dbm);
    return {};
}

void BluetoothController::update() {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < 50) { // Update at 20Hz maximum
        return;
    }
    
    update_statistics();
    update_connection_states();
    
    // Update power consumption
    power_manager_.current_consumption_mw = calculate_power_consumption();
    statistics_.current_power_consumption_mw = power_manager_.current_consumption_mw;
    
    last_update_ = now;
}

void BluetoothController::simulate_bluetooth_environment() {
    // Generate simulated devices for discovery
    classic_state_.discovered_devices = generate_simulated_classic_devices();
    ble_state_.scan_results = generate_simulated_ble_devices();
}

void BluetoothController::simulate_device_discovery() {
    if (!classic_state_.discovering) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto discovery_time = std::chrono::duration_cast<std::chrono::seconds>(
        now - classic_state_.discovery_start).count();
    
    if (discovery_time >= classic_state_.discovery_duration_s) {
        // Discovery completed
        classic_state_.discovering = false;
        classic_state_.state = BluetoothState::IDLE;
        
        LOG_INFO("BluetoothController", "Classic Bluetooth discovery completed, found {} devices", 
                 classic_state_.discovered_devices.size());
        
        trigger_bluetooth_event(BluetoothEventType::DISCOVERY_FINISHED);
        return;
    }
    
    // Simulate discovering devices over time
    static std::chrono::steady_clock::time_point last_device_found;
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_device_found).count() > 2) {
        // Add a new discovered device every 2 seconds
        auto simulated_devices = generate_simulated_classic_devices();
        if (!simulated_devices.empty() && classic_state_.discovered_devices.size() < 10) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> device_dist(0, simulated_devices.size() - 1);
            
            auto new_device = simulated_devices[device_dist(gen)];
            new_device.last_seen = now;
            new_device.rssi = simulate_rssi(new_device.address);
            
            // Check if device already discovered
            auto existing = std::find_if(classic_state_.discovered_devices.begin(),
                                       classic_state_.discovered_devices.end(),
                                       [&new_device](const BluetoothDevice& d) {
                                           return d.address == new_device.address;
                                       });
            
            if (existing == classic_state_.discovered_devices.end()) {
                classic_state_.discovered_devices.push_back(new_device);
                
                LOG_DEBUG("BluetoothController", "Discovered Classic device: {} ({})", 
                         new_device.name, new_device.address.to_string());
                
                trigger_bluetooth_event(BluetoothEventType::DEVICE_FOUND, new_device.address);
            }
        }
        
        last_device_found = now;
    }
}

void BluetoothController::simulate_ble_scanning() {
    if (!ble_state_.scanning) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto scan_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - ble_state_.scan_start).count();
    
    if (scan_time >= ble_state_.scan_duration_ms) {
        // Scan completed
        ble_state_.scanning = false;
        
        LOG_INFO("BluetoothController", "BLE scan completed, found {} devices", 
                 ble_state_.scan_results.size());
        return;
    }
    
    // Simulate receiving advertisements
    static std::chrono::steady_clock::time_point last_advertisement;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_advertisement).count() > 100) {
        // Receive advertisement every 100ms
        auto simulated_devices = generate_simulated_ble_devices();
        if (!simulated_devices.empty()) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> device_dist(0, simulated_devices.size() - 1);
            
            auto device = simulated_devices[device_dist(gen)];
            device.last_advertisement = now;
            device.rssi = simulate_rssi(device.address);
            
            // Check if device already in scan results
            auto existing = std::find_if(ble_state_.scan_results.begin(),
                                       ble_state_.scan_results.end(),
                                       [&device](const BLEDevice& d) {
                                           return d.address == device.address;
                                       });
            
            if (existing == ble_state_.scan_results.end()) {
                ble_state_.scan_results.push_back(device);
                statistics_.ble_scan_results++;
                
                LOG_DEBUG("BluetoothController", "Received BLE advertisement from: {} ({})", 
                         device.name, device.address.to_string());
                
                trigger_bluetooth_event(BluetoothEventType::BLE_SCAN_RESULT, device.address);
                
                if (ble_scan_callback_) {
                    ble_scan_callback_(device);
                }
            } else {
                // Update existing device RSSI
                existing->rssi = device.rssi;
                existing->last_advertisement = now;
            }
        }
        
        last_advertisement = now;
    }
}

void BluetoothController::simulate_ble_advertising() {
    if (!ble_state_.advertising) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    // Calculate advertising interval (in ms)
    u32 interval_ms = (ble_state_.advertising_interval_min + ble_state_.advertising_interval_max) / 2;
    interval_ms = interval_ms * 1250 / 1000; // Convert from 0.625ms units to ms
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - ble_state_.last_advertisement).count() >= interval_ms) {
        
        // Send advertisement packet
        generate_advertisement_packet();
        ble_state_.last_advertisement = now;
        statistics_.ble_advertisements_sent++;
        
        trigger_bluetooth_event(BluetoothEventType::BLE_ADVERTISEMENT, local_address_);
    }
}

void BluetoothController::simulate_connections() {
    // Simulate BLE connection establishment
    for (auto& [handle, connection] : ble_state_.connections) {
        if (connection.state == BLEConnectionState::CONNECTING) {
            auto now = std::chrono::steady_clock::now();
            auto connect_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - connection.connection_time).count();
            
            if (connect_time > 3000) { // 3 second connection timeout
                // Connection established
                connection.state = BLEConnectionState::CONNECTED;
                connection.services = simulate_gatt_service_discovery();
                
                statistics_.active_connections++;
                
                LOG_INFO("BluetoothController", "BLE connection established to {} (handle: {})", 
                         connection.remote_address.to_string(), handle);
                
                trigger_bluetooth_event(BluetoothEventType::BLE_CONNECT, connection.remote_address);
            }
        }
    }
}

void BluetoothController::simulate_data_transmission() {
    // Process TX queue
    while (!packet_processor_.tx_queue.empty()) {
        auto packet = packet_processor_.tx_queue.front();
        packet_processor_.tx_queue.pop();
        
        // Simulate packet transmission
        statistics_.data_packets_sent++;
        statistics_.bytes_sent += packet.size();
        
        if (packet_processor_.packet_logging) {
            packet_processor_.packet_dump.push_back(packet);
            if (packet_processor_.packet_dump.size() > packet_processor_.max_dump_size) {
                packet_processor_.packet_dump.erase(packet_processor_.packet_dump.begin());
            }
        }
    }
    
    // Simulate incoming data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> data_dist(0, 1000);
    
    if (data_dist(gen) < 10) { // 1% chance of receiving data
        if (!classic_state_.connected_devices.empty() || !ble_state_.connections.empty()) {
            std::uniform_int_distribution<> size_dist(10, 100);
            size_t packet_size = size_dist(gen);
            
            std::vector<u8> packet(packet_size);
            std::uniform_int_distribution<u8> byte_dist(0, 255);
            for (auto& byte : packet) {
                byte = byte_dist(gen);
            }
            
            packet_processor_.rx_queue.push(packet);
            statistics_.data_packets_received++;
            statistics_.bytes_received += packet_size;
            
            // Trigger data callback for first connected device
            if (!classic_state_.connected_devices.empty()) {
                auto& device = classic_state_.connected_devices[0];
                trigger_bluetooth_event(BluetoothEventType::DATA_RECEIVED, device.address, packet);
                
                if (data_callback_) {
                    data_callback_(device.address, packet);
                }
            }
        }
    }
}

void BluetoothController::simulate_gatt_operations() {
    // Simulate GATT notifications from connected devices
    for (auto& [handle, connection] : ble_state_.connections) {
        if (connection.state == BLEConnectionState::CONNECTED) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> notify_dist(0, 1000);
            
            if (notify_dist(gen) < 5) { // 0.5% chance of notification
                // Simulate notification from battery service
                std::string battery_char_uuid = "2A19";
                std::uniform_int_distribution<u8> battery_dist(0, 100);
                std::vector<u8> battery_value = {battery_dist(gen)};
                
                trigger_bluetooth_event(BluetoothEventType::BLE_NOTIFICATION, 
                                       connection.remote_address, battery_value);
                
                if (characteristic_callback_) {
                    characteristic_callback_(battery_char_uuid, battery_value);
                }
            }
        }
    }
}

void BluetoothController::process_classic_bluetooth() {
    // Handle classic Bluetooth state transitions and operations
    if (classic_state_.discoverable) {
        auto now = std::chrono::steady_clock::now();
        if (classic_state_.discoverable_timeout_s > 0) {
            auto discoverable_time = std::chrono::duration_cast<std::chrono::seconds>(
                now - classic_state_.discoverable_start).count();
            
            if (discoverable_time >= classic_state_.discoverable_timeout_s) {
                classic_state_.discoverable = false;
                LOG_INFO("BluetoothController", "Discoverable mode timeout");
            }
        }
    }
}

void BluetoothController::process_ble_operations() {
    // Handle BLE connection parameter updates and maintenance
    for (auto& [handle, connection] : ble_state_.connections) {
        if (connection.state == BLEConnectionState::CONNECTED) {
            auto now = std::chrono::steady_clock::now();
            
            // Simulate RSSI updates
            if (std::chrono::duration_cast<std::chrono::seconds>(
                now - connection.connection_time).count() % 10 == 0) {
                connection.rssi = simulate_rssi(connection.remote_address);
            }
        }
    }
}

void BluetoothController::update_connection_states() {
    // Check for connection timeouts and state updates
    auto now = std::chrono::steady_clock::now();
    
    // Update classic connections
    for (auto& device : classic_state_.connected_devices) {
        if (device.is_connected) {
            auto idle_time = std::chrono::duration_cast<std::chrono::seconds>(
                now - device.last_seen).count();
            
            if (idle_time > 300) { // 5 minute timeout
                device.is_connected = false;
                statistics_.active_connections--;
                
                LOG_WARN("BluetoothController", "Classic connection lost to device: {}", 
                        device.address.to_string());
                
                trigger_bluetooth_event(BluetoothEventType::ACL_DISCONNECTED, device.address);
            }
        }
    }
    
    // Update BLE connections
    for (auto it = ble_state_.connections.begin(); it != ble_state_.connections.end();) {
        auto& connection = it->second;
        
        if (connection.state == BLEConnectionState::CONNECTED) {
            auto connection_age = std::chrono::duration_cast<std::chrono::seconds>(
                now - connection.connection_time).count();
            
            // Simulate connection supervision timeout
            if (connection_age > connection.supervision_timeout * 10) { // supervision_timeout is in 10ms units
                connection.state = BLEConnectionState::DISCONNECTED;
                statistics_.active_connections--;
                
                LOG_WARN("BluetoothController", "BLE connection supervison timeout for device: {}", 
                        connection.remote_address.to_string());
                
                trigger_bluetooth_event(BluetoothEventType::BLE_DISCONNECT, connection.remote_address);
                
                it = ble_state_.connections.erase(it);
                continue;
            }
        }
        
        ++it;
    }
}

void BluetoothController::trigger_bluetooth_event(BluetoothEventType type, 
                                                 const BluetoothAddress& address,
                                                 const std::vector<u8>& data, 
                                                 u32 event_data) {
    BluetoothEvent event;
    event.type = type;
    event.device_address = address;
    event.data = data;
    event.event_data = event_data;
    event.timestamp = std::chrono::steady_clock::now();
    
    if (event_callback_) {
        event_callback_(event);
    }
    
    // Trigger hardware interrupt for important events
    if (interrupt_controller_) {
        switch (type) {
            case BluetoothEventType::DEVICE_FOUND:
            case BluetoothEventType::CONNECTION_STATE_CHANGED:
            case BluetoothEventType::PAIRING_COMPLETE:
            case BluetoothEventType::BLE_CONNECT:
            case BluetoothEventType::BLE_DISCONNECT:
            case BluetoothEventType::DATA_RECEIVED:
                interrupt_controller_->trigger_interrupt(51); // Bluetooth interrupt line
                break;
            default:
                break;
        }
    }
}

std::vector<BluetoothDevice> BluetoothController::generate_simulated_classic_devices() {
    std::vector<BluetoothDevice> devices;
    
    std::vector<std::pair<std::string, BluetoothDeviceType>> device_templates = {
        {"John's iPhone", BluetoothDeviceType::PHONE},
        {"Samsung Galaxy", BluetoothDeviceType::PHONE},
        {"AirPods Pro", BluetoothDeviceType::HEADSET},
        {"Bluetooth Speaker", BluetoothDeviceType::SPEAKER},
        {"Gaming Headset", BluetoothDeviceType::HEADSET},
        {"Wireless Mouse", BluetoothDeviceType::MOUSE},
        {"BT Keyboard", BluetoothDeviceType::KEYBOARD},
        {"Fitness Tracker", BluetoothDeviceType::FITNESS_TRACKER}
    };
    
    for (const auto& [name, type] : device_templates) {
        BluetoothDevice device;
        device.address = generate_random_address(false);
        device.name = name;
        device.device_type = type;
        device.rssi = -40 - (rand() % 40); // -40 to -80 dBm
        device.class_of_device = static_cast<u32>(type) << 8;
        device.is_bonded = false;
        device.is_connected = false;
        device.last_seen = std::chrono::steady_clock::now();
        
        // Add supported profiles based on device type
        switch (type) {
            case BluetoothDeviceType::HEADSET:
                device.supported_profiles = {BluetoothProfile::A2DP, BluetoothProfile::HFP};
                break;
            case BluetoothDeviceType::SPEAKER:
                device.supported_profiles = {BluetoothProfile::A2DP, BluetoothProfile::AVRCP};
                break;
            case BluetoothDeviceType::MOUSE:
            case BluetoothDeviceType::KEYBOARD:
                device.supported_profiles = {BluetoothProfile::HID};
                break;
            case BluetoothDeviceType::PHONE:
                device.supported_profiles = {BluetoothProfile::SPP, BluetoothProfile::PBAP, BluetoothProfile::MAP};
                break;
            default:
                device.supported_profiles = {BluetoothProfile::SPP};
                break;
        }
        
        devices.push_back(device);
    }
    
    return devices;
}

std::vector<BLEDevice> BluetoothController::generate_simulated_ble_devices() {
    std::vector<BLEDevice> devices;
    
    std::vector<std::pair<std::string, std::vector<std::string>>> device_templates = {
        {"BLE Heart Rate", {"180D"}},  // Heart Rate Service
        {"Smart Watch", {"180F", "1805"}},  // Battery + Current Time
        {"BLE Beacon", {"1809"}},  // Health Thermometer
        {"Fitness Band", {"180D", "180F"}},  // Heart Rate + Battery
        {"Smart Thermostat", {"1809", "180F"}},  // Health Thermometer + Battery
        {"BLE Mouse", {"1812"}},  // HID over GATT
        {"Smart Light", {"180F"}},  // Battery Service
        {"Environment Sensor", {"181A"}}  // Environmental Sensing
    };
    
    for (const auto& [name, services] : device_templates) {
        BLEDevice device;
        device.address = generate_random_address(true);
        device.name = name;
        device.adv_type = BLEAdvertisementType::ADV_IND;
        device.rssi = -50 - (rand() % 30); // -50 to -80 dBm
        device.tx_power = 0;
        device.service_uuids = services;
        device.connectable = true;
        device.last_advertisement = std::chrono::steady_clock::now();
        
        // Build advertisement data
        device.advertisement_data = build_advertisement_data(name, services);
        
        devices.push_back(device);
    }
    
    return devices;
}

BluetoothAddress BluetoothController::generate_random_address(bool random_static) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<u8> byte_dist(0, 255);
    
    BluetoothAddress address;
    for (auto& byte : address.address) {
        byte = byte_dist(gen);
    }
    
    if (random_static) {
        // Set random static address bits (top two bits = 11)
        address.address[5] |= 0xC0;
    } else {
        // Set locally administered bit
        address.address[5] |= 0x02;
        address.address[5] &= 0xFE; // Clear multicast bit
    }
    
    return address;
}

i8 BluetoothController::simulate_rssi(const BluetoothAddress& device) {
    // Simulate RSSI based on address hash and add some randomness
    std::hash<std::string> hasher;
    size_t hash = hasher(device.to_string());
    
    i8 base_rssi = -40 - (hash % 40); // -40 to -80 dBm
    
    // Add random variation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> noise(0.0f, 3.0f);
    
    i8 rssi = base_rssi + static_cast<i8>(noise(gen));
    return std::clamp(rssi, i8(-100), i8(-20));
}

std::vector<BLEService> BluetoothController::simulate_gatt_service_discovery() {
    std::vector<BLEService> services;
    
    // Generic Access Service
    BLEService gas;
    gas.uuid = "1800";
    gas.start_handle = 1;
    gas.end_handle = 5;
    
    BLECharacteristic device_name_char;
    device_name_char.uuid = "2A00";
    device_name_char.handle = 3;
    device_name_char.properties = 0x02; // Read
    device_name_char.value = std::vector<u8>(device_name_.begin(), device_name_.end());
    gas.characteristics.push_back(device_name_char);
    
    services.push_back(gas);
    
    // Battery Service
    BLEService battery;
    battery.uuid = "180F";
    battery.start_handle = 10;
    battery.end_handle = 15;
    
    BLECharacteristic battery_level;
    battery_level.uuid = "2A19";
    battery_level.handle = 12;
    battery_level.properties = 0x12; // Read + Notify
    battery_level.value = {85}; // 85% battery
    battery.characteristics.push_back(battery_level);
    
    services.push_back(battery);
    
    return services;
}

std::vector<u8> BluetoothController::simulate_characteristic_read(const std::string& char_uuid) {
    // Simulate reading different characteristic types
    if (char_uuid == "2A19") { // Battery Level
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<u8> battery_dist(50, 100);
        return {battery_dist(gen)};
    } else if (char_uuid == "2A00") { // Device Name
        return std::vector<u8>(device_name_.begin(), device_name_.end());
    } else if (char_uuid == "2A37") { // Heart Rate Measurement
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<u8> hr_dist(60, 100);
        return {0x00, hr_dist(gen)}; // Flags + Heart Rate
    } else {
        // Generic data
        return {0x01, 0x02, 0x03, 0x04};
    }
}

bool BluetoothController::simulate_characteristic_write(const std::string& char_uuid, 
                                                       const std::vector<u8>& value) {
    // Simulate write operation
    LOG_DEBUG("BluetoothController", "Simulated write to characteristic {}: {} bytes", 
              char_uuid, value.size());
    
    // All writes succeed in simulation
    return true;
}

std::vector<u8> BluetoothController::build_advertisement_data(const std::string& name, 
                                                             const std::vector<std::string>& services) {
    std::vector<u8> data;
    
    // Flags
    add_advertisement_field(data, BLEDataType::FLAGS, {0x06}); // General Discoverable + BR/EDR Not Supported
    
    // Complete Local Name
    std::vector<u8> name_bytes(name.begin(), name.end());
    add_advertisement_field(data, BLEDataType::COMPLETE_LOCAL_NAME, name_bytes);
    
    // Complete 16-bit Service UUIDs
    if (!services.empty()) {
        std::vector<u8> service_uuids;
        for (const auto& service : services) {
            if (service.length() == 4) { // 16-bit UUID
                u16 uuid = std::stoi(service, nullptr, 16);
                service_uuids.push_back(uuid & 0xFF);
                service_uuids.push_back((uuid >> 8) & 0xFF);
            }
        }
        if (!service_uuids.empty()) {
            add_advertisement_field(data, BLEDataType::COMPLETE_16BIT_UUIDS, service_uuids);
        }
    }
    
    return data;
}

void BluetoothController::add_advertisement_field(std::vector<u8>& data, 
                                                 BLEDataType type, 
                                                 const std::vector<u8>& value) {
    if (data.size() + value.size() + 2 > MAX_ADVERTISEMENT_DATA_LEN) {
        return; // Not enough space
    }
    
    data.push_back(static_cast<u8>(value.size() + 1)); // Length
    data.push_back(static_cast<u8>(type));             // Type
    data.insert(data.end(), value.begin(), value.end()); // Value
}

void BluetoothController::generate_advertisement_packet() {
    if (ble_state_.advertisement_data.empty()) {
        // Generate default advertisement
        ble_state_.advertisement_data = build_advertisement_data(device_name_, {});
    }
    
    // Add to packet dump if logging enabled
    if (packet_processor_.packet_logging) {
        packet_processor_.packet_dump.push_back(ble_state_.advertisement_data);
        if (packet_processor_.packet_dump.size() > packet_processor_.max_dump_size) {
            packet_processor_.packet_dump.erase(packet_processor_.packet_dump.begin());
        }
    }
}

u32 BluetoothController::calculate_power_consumption() {
    u32 base_power = 0;
    
    if (!bluetooth_enabled_) {
        return 5; // Sleep power
    }
    
    // Base radio power
    base_power = 50; // mW
    
    // Add TX power contribution
    base_power += (power_manager_.tx_power_dbm + 12) * 5; // Approximate scaling
    
    // Add mode-specific power
    if (current_mode_ == BluetoothMode::CLASSIC || current_mode_ == BluetoothMode::DUAL) {
        base_power += 30; // Classic Bluetooth overhead
        
        if (classic_state_.discovering) {
            base_power += 20; // Discovery power
        }
        
        base_power += classic_state_.connected_devices.size() * 10; // Per connection
    }
    
    if (current_mode_ == BluetoothMode::BLE || current_mode_ == BluetoothMode::DUAL) {
        base_power += 10; // BLE overhead
        
        if (ble_state_.scanning) {
            base_power += 15; // Scanning power
        }
        
        if (ble_state_.advertising) {
            base_power += 5; // Advertising power
        }
        
        base_power += ble_state_.connections.size() * 3; // Per BLE connection
    }
    
    return base_power;
}

void BluetoothController::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    
    // Update active connections count
    statistics_.active_connections = classic_state_.connected_devices.size() + ble_state_.connections.size();
    
    // Calculate connection success rate
    if (statistics_.ble_connections + statistics_.classic_connections > 0) {
        u64 total_attempts = statistics_.ble_connections + statistics_.classic_connections;
        u64 successful = statistics_.active_connections; // Simplified
        statistics_.connection_success_rate = static_cast<float>(successful) / total_attempts;
    }
    
    // Update average RSSI
    float total_rssi = 0.0f;
    u32 rssi_count = 0;
    
    for (const auto& device : classic_state_.connected_devices) {
        if (device.is_connected) {
            total_rssi += device.rssi;
            rssi_count++;
        }
    }
    
    for (const auto& [handle, connection] : ble_state_.connections) {
        if (connection.state == BLEConnectionState::CONNECTED) {
            total_rssi += connection.rssi;
            rssi_count++;
        }
    }
    
    if (rssi_count > 0) {
        statistics_.average_rssi = total_rssi / rssi_count;
    }
}

void BluetoothController::clear_statistics() {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    statistics_ = BluetoothStatistics{};
}

void BluetoothController::dump_status() const {
    std::lock_guard<std::mutex> lock(bluetooth_mutex_);
    
    LOG_INFO("BluetoothController", "=== Bluetooth Controller Status ===");
    LOG_INFO("BluetoothController", "Initialized: {}", initialized_);
    LOG_INFO("BluetoothController", "Enabled: {}", bluetooth_enabled_);
    LOG_INFO("BluetoothController", "Mode: {}", static_cast<int>(current_mode_));
    LOG_INFO("BluetoothController", "Local Address: {}", local_address_.to_string());
    LOG_INFO("BluetoothController", "Device Name: {}", device_name_);
    LOG_INFO("BluetoothController", "Classic State: {}", static_cast<int>(classic_state_.state));
    
    if (classic_state_.discovering) {
        LOG_INFO("BluetoothController", "Classic Discovery: Active ({} devices found)", 
                 classic_state_.discovered_devices.size());
    }
    
    if (ble_state_.scanning) {
        LOG_INFO("BluetoothController", "BLE Scanning: Active ({} devices found)", 
                 ble_state_.scan_results.size());
    }
    
    if (ble_state_.advertising) {
        LOG_INFO("BluetoothController", "BLE Advertising: Active");
    }
    
    LOG_INFO("BluetoothController", "Connected Devices - Classic: {}, BLE: {}", 
             classic_state_.connected_devices.size(), ble_state_.connections.size());
    LOG_INFO("BluetoothController", "TX Power: {} dBm", power_manager_.tx_power_dbm);
    LOG_INFO("BluetoothController", "Power Consumption: {} mW", power_manager_.current_consumption_mw);
    LOG_INFO("BluetoothController", "Packets Sent: {}", statistics_.data_packets_sent);
    LOG_INFO("BluetoothController", "Packets Received: {}", statistics_.data_packets_received);
    LOG_INFO("BluetoothController", "BLE Advertisements Sent: {}", statistics_.ble_advertisements_sent);
    LOG_INFO("BluetoothController", "GATT Operations: {}", statistics_.gatt_operations);
    LOG_INFO("BluetoothController", "Average RSSI: {:.1f} dBm", statistics_.average_rssi);
    LOG_INFO("BluetoothController", "Connection Success Rate: {:.1f}%", 
             statistics_.connection_success_rate * 100.0f);
}

}  // namespace m5tab5::emulator