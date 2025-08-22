#include "emulator/connectivity/usb_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace m5tab5::emulator {

USBController::USBController()
    : initialized_(false)
    , current_mode_(USBMode::DISABLED)
    , interrupt_controller_(nullptr)
    , thread_running_(false) {
    
    // Initialize PHY settings
    phy_.enabled = false;
    phy_.speed = USBSpeed::FULL_SPEED;
    phy_.signal_strength_dbm = 0;
    phy_.drive_strength = 2;
    
    // Initialize power management
    power_manager_.vbus_enabled = false;
    power_manager_.power_budget_ma = 500;
    power_manager_.current_consumption_ma = 0;
    
    // Initialize host controller
    host_controller_.enabled = false;
    host_controller_.port_count = 1;
    host_controller_.port_power.fill(false);
    host_controller_.port_enabled.fill(false);
    host_controller_.frame_number = 0;
    
    // Initialize device controller with default device descriptor
    device_controller_.enabled = false;
    device_controller_.device.device_descriptor.idVendor = USB_VID_ESPRESSIF;
    device_controller_.device.device_descriptor.idProduct = USB_PID_ESP32_C6;
    device_controller_.device.state = USBState::DETACHED;
    
    // Initialize transfer manager
    transfer_manager_.next_transfer_id = 1;
    transfer_manager_.max_concurrent_transfers = 32;
    
    LOG_DEBUG("USBController", "USB controller initialized");
}

USBController::~USBController() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> USBController::initialize(const Configuration& config, 
                                      InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "USB controller already initialized");
    }
    
    if (!interrupt_controller) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Interrupt controller cannot be null");
    }
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize default string descriptors
    device_controller_.device.string_descriptors.resize(4);
    
    // Language descriptor (index 0)
    USBStringDescriptor lang_desc;
    lang_desc.bLength = 4;
    lang_desc.bString = {0x0409}; // English (US)
    device_controller_.device.string_descriptors[0] = lang_desc;
    
    // Manufacturer string (index 1)
    device_controller_.device.string_descriptors[1] = {0, static_cast<u8>(USBDescriptorType::STRING), {}};
    auto manufacturer = build_string_descriptor("Espressif Systems");
    device_controller_.device.string_descriptors[1].bLength = manufacturer.size();
    
    // Product string (index 2)
    auto product = build_string_descriptor("ESP32-C6 USB Device");
    device_controller_.device.string_descriptors[2].bLength = product.size();
    
    // Serial number string (index 3)
    auto serial = build_string_descriptor("123456789ABC");
    device_controller_.device.string_descriptors[3].bLength = serial.size();
    
    // Start background processing thread
    thread_running_ = true;
    usb_thread_ = std::make_unique<std::thread>([this]() {
        while (thread_running_) {
            {
                std::lock_guard<std::mutex> lock(usb_mutex_);
                if (initialized_) {
                    simulate_usb_enumeration();
                    simulate_usb_transfers();
                    simulate_sof_generation();
                    simulate_power_management();
                    simulate_device_connections();
                    process_host_operations();
                    process_device_operations();
                    process_transfer_queue();
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100)); // High frequency updates
        }
    });
    
    last_update_ = std::chrono::steady_clock::now();
    
    initialized_ = true;
    
    LOG_INFO("USBController", "USB controller initialized successfully");
    trigger_usb_event(USBEventType::RESET);
    
    return {};
}

Result<void> USBController::shutdown() {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Stop background thread
    thread_running_ = false;
    if (usb_thread_ && usb_thread_->joinable()) {
        usb_mutex_.unlock();
        usb_thread_->join();
        usb_mutex_.lock();
    }
    usb_thread_.reset();
    
    // Disconnect all devices
    if (current_mode_ == USBMode::HOST || current_mode_ == USBMode::OTG) {
        for (auto& device : host_controller_.connected_devices) {
            device.state = USBState::DETACHED;
            if (device_callback_) {
                device_callback_(device, false);
            }
        }
        host_controller_.connected_devices.clear();
    }
    
    // Cancel all active transfers
    for (auto& [id, transfer] : transfer_manager_.active_transfers) {
        transfer.status = USBTransferStatus::CANCELLED;
        transfer.completed = true;
    }
    transfer_manager_.active_transfers.clear();
    
    // Disable PHY and power
    phy_.enabled = false;
    power_manager_.vbus_enabled = false;
    
    current_mode_ = USBMode::DISABLED;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    LOG_INFO("USBController", "USB controller shutdown complete");
    return {};
}

Result<void> USBController::set_mode(USBMode mode) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ == mode) {
        return {}; // Already in requested mode
    }
    
    // Handle mode transitions
    if (current_mode_ != USBMode::DISABLED) {
        // Reset current mode
        host_controller_.enabled = false;
        device_controller_.enabled = false;
        phy_.enabled = false;
    }
    
    current_mode_ = mode;
    
    switch (mode) {
        case USBMode::DISABLED:
            phy_.enabled = false;
            power_manager_.vbus_enabled = false;
            break;
            
        case USBMode::DEVICE:
            device_controller_.enabled = true;
            device_controller_.device.state = USBState::DETACHED;
            phy_.enabled = true;
            break;
            
        case USBMode::HOST:
            host_controller_.enabled = true;
            host_controller_.root_hub_enabled = true;
            power_manager_.vbus_enabled = true;
            phy_.enabled = true;
            break;
            
        case USBMode::OTG:
            // OTG starts in device mode by default
            device_controller_.enabled = true;
            device_controller_.device.state = USBState::DETACHED;
            phy_.enabled = true;
            break;
    }
    
    LOG_INFO("USBController", "USB mode set to {}", static_cast<int>(mode));
    trigger_usb_event(USBEventType::RESET);
    
    return {};
}

Result<void> USBController::enable_vbus(bool enable) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "VBUS control only available in host/OTG mode");
    }
    
    power_manager_.vbus_enabled = enable;
    power_manager_.vbus_voltage_mv = enable ? 5000 : 0;
    power_manager_.session_valid = enable;
    
    if (enable) {
        // Power up ports
        for (size_t i = 0; i < host_controller_.port_count; ++i) {
            host_controller_.port_power[i] = true;
            host_controller_.port_enabled[i] = true;
        }
    } else {
        // Power down ports
        host_controller_.port_power.fill(false);
        host_controller_.port_enabled.fill(false);
        
        // Disconnect all devices
        for (auto& device : host_controller_.connected_devices) {
            device.state = USBState::DETACHED;
            trigger_usb_event(USBEventType::DISCONNECT);
        }
        host_controller_.connected_devices.clear();
    }
    
    LOG_INFO("USBController", "VBUS {}", enable ? "enabled" : "disabled");
    trigger_usb_event(USBEventType::VBUS_CHANGE);
    
    return {};
}

Result<void> USBController::device_configure_endpoint(u8 address, USBEndpointType type, 
                                                     u16 max_packet_size, u8 interval) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ != USBMode::DEVICE && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "Not in device mode");
    }
    
    if (!is_endpoint_address_valid(address)) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid endpoint address");
    }
    
    u8 ep_num = get_endpoint_number(address);
    if (ep_num >= MAX_ENDPOINTS) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Endpoint number out of range");
    }
    
    // Validate max packet size based on speed and type
    u16 max_allowed = calculate_max_packet_size(phy_.speed, type);
    if (max_packet_size > max_allowed) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Max packet size too large for speed/type");
    }
    
    USBEndpoint endpoint;
    endpoint.address = address;
    endpoint.type = type;
    endpoint.max_packet_size = max_packet_size;
    endpoint.interval = interval;
    endpoint.stalled = false;
    endpoint.bytes_transferred = 0;
    endpoint.transfer_count = 0;
    
    device_controller_.endpoints[address] = endpoint;
    
    LOG_DEBUG("USBController", "Configured endpoint 0x{:02X}: type={}, max_packet={}, interval={}", 
              address, static_cast<int>(type), max_packet_size, interval);
    
    return {};
}

Result<u32> USBController::device_submit_transfer(u8 endpoint_address, const std::vector<u8>& data, u32 timeout_ms) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ != USBMode::DEVICE && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "Not in device mode");
    }
    
    auto ep_it = device_controller_.endpoints.find(endpoint_address);
    if (ep_it == device_controller_.endpoints.end()) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Endpoint not configured");
    }
    
    if (ep_it->second.stalled) {
        return make_error(ErrorCode::DEVICE_ERROR, "Endpoint stalled");
    }
    
    if (transfer_manager_.active_transfers.size() >= transfer_manager_.max_concurrent_transfers) {
        return make_error(ErrorCode::RESOURCE_LIMIT, "Too many active transfers");
    }
    
    // Create transfer
    u32 transfer_id = transfer_manager_.next_transfer_id++;
    USBTransfer transfer;
    transfer.id = transfer_id;
    transfer.endpoint_address = endpoint_address;
    transfer.type = ep_it->second.type;
    transfer.data = data;
    transfer.bytes_transferred = 0;
    transfer.status = USBTransferStatus::COMPLETED;
    transfer.start_time = std::chrono::steady_clock::now();
    transfer.timeout_ms = timeout_ms;
    transfer.completed = false;
    
    transfer_manager_.active_transfers[transfer_id] = transfer;
    
    LOG_DEBUG("USBController", "Submitted device transfer {}: endpoint 0x{:02X}, {} bytes", 
              transfer_id, endpoint_address, data.size());
    
    return transfer_id;
}

Result<u32> USBController::host_control_transfer(u8 device_address, const USBSetupPacket& setup, 
                                                std::vector<u8>& data, u32 timeout_ms) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "Not in host mode");
    }
    
    if (!is_device_address_valid(device_address)) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid device address");
    }
    
    if (transfer_manager_.active_transfers.size() >= transfer_manager_.max_concurrent_transfers) {
        return make_error(ErrorCode::RESOURCE_LIMIT, "Too many active transfers");
    }
    
    // Find device
    auto device_it = std::find_if(host_controller_.connected_devices.begin(),
                                 host_controller_.connected_devices.end(),
                                 [device_address](const USBDevice& d) {
                                     return d.address == device_address;
                                 });
    
    if (device_it == host_controller_.connected_devices.end()) {
        return make_error(ErrorCode::NOT_CONNECTED, "Device not found");
    }
    
    // Create control transfer
    u32 transfer_id = transfer_manager_.next_transfer_id++;
    USBTransfer transfer;
    transfer.id = transfer_id;
    transfer.endpoint_address = 0; // Control endpoint
    transfer.type = USBEndpointType::CONTROL;
    transfer.data = data;
    transfer.bytes_transferred = 0;
    transfer.status = USBTransferStatus::COMPLETED;
    transfer.start_time = std::chrono::steady_clock::now();
    transfer.timeout_ms = timeout_ms;
    transfer.completed = false;
    
    transfer_manager_.active_transfers[transfer_id] = transfer;
    
    statistics_.control_transfers++;
    statistics_.setup_packets++;
    
    LOG_DEBUG("USBController", "Submitted control transfer {}: device {}, request 0x{:02X}", 
              transfer_id, device_address, setup.bRequest);
    
    // Simulate immediate processing for standard requests
    if ((setup.bmRequestType & 0x60) == 0x00) { // Standard request
        std::vector<u8> response;
        if (handle_standard_request(setup, response)) {
            data = response;
            complete_transfer(transfer_id, USBTransferStatus::COMPLETED, response.size());
        } else {
            complete_transfer(transfer_id, USBTransferStatus::STALLED);
        }
    }
    
    return transfer_id;
}

Result<std::vector<u8>> USBController::get_device_descriptor(u8 device_address) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (device_address == 0 && (current_mode_ == USBMode::DEVICE || current_mode_ == USBMode::OTG)) {
        // Return local device descriptor
        return build_device_descriptor(device_controller_.device.device_descriptor);
    }
    
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "Not in host mode");
    }
    
    // Find device
    auto device_it = std::find_if(host_controller_.connected_devices.begin(),
                                 host_controller_.connected_devices.end(),
                                 [device_address](const USBDevice& d) {
                                     return d.address == device_address;
                                 });
    
    if (device_it == host_controller_.connected_devices.end()) {
        return make_error(ErrorCode::NOT_CONNECTED, "Device not found");
    }
    
    return build_device_descriptor(device_it->device_descriptor);
}

Result<std::vector<u8>> USBController::get_string_descriptor(u8 device_address, u8 string_index, u16 language_id) {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (device_address == 0 && (current_mode_ == USBMode::DEVICE || current_mode_ == USBMode::OTG)) {
        // Return local string descriptor
        if (string_index >= device_controller_.device.string_descriptors.size()) {
            return make_error(ErrorCode::INVALID_ARGUMENT, "String index out of range");
        }
        
        switch (string_index) {
            case 0: return build_string_descriptor("", language_id); // Language descriptor
            case 1: return build_string_descriptor("Espressif Systems", language_id);
            case 2: return build_string_descriptor("ESP32-C6 USB Device", language_id);
            case 3: return build_string_descriptor("123456789ABC", language_id);
            default: return make_error(ErrorCode::INVALID_ARGUMENT, "String index not supported");
        }
    }
    
    return make_error(ErrorCode::NOT_IMPLEMENTED, "Host mode string descriptors not implemented");
}

Result<void> USBController::host_enumerate_devices() {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "USB controller not initialized");
    }
    
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return make_error(ErrorCode::INVALID_STATE, "Not in host mode");
    }
    
    if (!power_manager_.vbus_enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "VBUS not enabled");
    }
    
    // Clear existing devices
    host_controller_.connected_devices.clear();
    host_controller_.next_device_address = 1;
    
    // Simulate device detection and enumeration
    auto simulated_devices = generate_simulated_devices();
    
    for (auto& device : simulated_devices) {
        device.address = host_controller_.next_device_address++;
        device.state = USBState::CONFIGURED;
        device.connect_time = std::chrono::steady_clock::now();
        
        host_controller_.connected_devices.push_back(device);
        
        LOG_INFO("USBController", "Enumerated USB device: address={}, VID:PID={:04X}:{:04X}", 
                 device.address, device.device_descriptor.idVendor, device.device_descriptor.idProduct);
        
        trigger_usb_event(USBEventType::CONNECT);
        
        if (device_callback_) {
            device_callback_(device, true);
        }
    }
    
    statistics_.enumeration_time_ms = 100; // Simulated enumeration time
    
    LOG_INFO("USBController", "Device enumeration completed: {} devices found", 
             host_controller_.connected_devices.size());
    
    return {};
}

void USBController::update() {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < 10) { // Update at 100Hz maximum
        return;
    }
    
    update_statistics();
    update_frame_counter();
    
    // Update power consumption
    power_manager_.current_consumption_ma = calculate_power_consumption();
    statistics_.power_consumption_mw = power_manager_.current_consumption_ma * 5; // Assume 5V
    
    last_update_ = now;
}

void USBController::simulate_usb_enumeration() {
    if (current_mode_ == USBMode::DEVICE || current_mode_ == USBMode::OTG) {
        if (device_controller_.device.state == USBState::DETACHED && power_manager_.session_valid) {
            // Simulate device attachment
            device_controller_.device.state = USBState::ATTACHED;
            trigger_usb_event(USBEventType::CONNECT);
            
            // Transition through enumeration states
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            device_controller_.device.state = USBState::POWERED;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            device_controller_.device.state = USBState::DEFAULT;
            trigger_usb_event(USBEventType::RESET);
        }
    }
}

void USBController::simulate_usb_transfers() {
    // Process active transfers and simulate completion
    for (auto& [id, transfer] : transfer_manager_.active_transfers) {
        if (transfer.completed) {
            continue;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - transfer.start_time).count();
        
        if (elapsed >= transfer.timeout_ms) {
            // Transfer timeout
            transfer.status = USBTransferStatus::TIMEOUT;
            transfer.completed = true;
            statistics_.transfers_failed++;
            
            trigger_usb_event(USBEventType::ERROR, transfer.endpoint_address, id, transfer.status);
            continue;
        }
        
        // Simulate transfer processing based on type
        switch (transfer.type) {
            case USBEndpointType::CONTROL:
                handle_control_transfer(transfer);
                break;
            case USBEndpointType::BULK:
                handle_bulk_transfer(transfer);
                break;
            case USBEndpointType::INTERRUPT:
                handle_interrupt_transfer(transfer);
                break;
            case USBEndpointType::ISOCHRONOUS:
                handle_isochronous_transfer(transfer);
                break;
        }
    }
    
    // Clean up completed transfers
    for (auto it = transfer_manager_.active_transfers.begin(); 
         it != transfer_manager_.active_transfers.end();) {
        if (it->second.completed) {
            transfer_manager_.completed_transfers.push(it->second);
            it = transfer_manager_.active_transfers.erase(it);
        } else {
            ++it;
        }
    }
}

void USBController::simulate_sof_generation() {
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return;
    }
    
    if (!host_controller_.enabled) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto sof_time = std::chrono::duration_cast<std::chrono::microseconds>(
        now - host_controller_.last_sof).count();
    
    if (sof_time >= SOF_INTERVAL_US) {
        // Generate SOF (Start of Frame)
        host_controller_.frame_number++;
        host_controller_.last_sof = now;
        statistics_.sof_packets++;
        
        trigger_usb_event(USBEventType::SOF);
        
        // Update frame number for all devices
        for (auto& device : host_controller_.connected_devices) {
            device.current_frame = host_controller_.frame_number;
        }
    }
}

void USBController::simulate_power_management() {
    // Update VBUS voltage and current monitoring
    if (power_manager_.vbus_enabled) {
        power_manager_.vbus_voltage_mv = 5000; // 5V
        power_manager_.session_valid = true;
    } else {
        power_manager_.vbus_voltage_mv = 0;
        power_manager_.session_valid = false;
    }
    
    // Check for overcurrent condition
    if (power_manager_.current_consumption_ma > power_manager_.power_budget_ma * 1.1f) {
        if (!power_manager_.overcurrent_detected) {
            power_manager_.overcurrent_detected = true;
            LOG_WARN("USBController", "Overcurrent detected: {}mA > {}mA", 
                     power_manager_.current_consumption_ma, power_manager_.power_budget_ma);
            trigger_usb_event(USBEventType::OVERCURRENT);
        }
    } else {
        power_manager_.overcurrent_detected = false;
    }
}

void USBController::simulate_device_connections() {
    // Simulate random device connections/disconnections for host mode
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return;
    }
    
    if (!host_controller_.enabled || !power_manager_.vbus_enabled) {
        return;
    }
    
    static std::chrono::steady_clock::time_point last_connection_check;
    auto now = std::chrono::steady_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_connection_check).count() > 30) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> event_dist(0, 1000);
        
        if (event_dist(gen) < 10) { // 1% chance of connection event
            if (host_controller_.connected_devices.size() < MAX_DEVICES) {
                // Simulate new device connection
                auto new_devices = generate_simulated_devices();
                if (!new_devices.empty()) {
                    auto device = new_devices[0];
                    device.address = host_controller_.next_device_address++;
                    device.state = USBState::CONFIGURED;
                    device.connect_time = now;
                    
                    host_controller_.connected_devices.push_back(device);
                    
                    LOG_INFO("USBController", "New USB device connected: VID:PID={:04X}:{:04X}", 
                             device.device_descriptor.idVendor, device.device_descriptor.idProduct);
                    
                    trigger_usb_event(USBEventType::CONNECT);
                    
                    if (device_callback_) {
                        device_callback_(device, true);
                    }
                }
            } else if (event_dist(gen) < 5 && !host_controller_.connected_devices.empty()) {
                // Simulate device disconnection
                auto& device = host_controller_.connected_devices.back();
                
                LOG_INFO("USBController", "USB device disconnected: address {}", device.address);
                
                trigger_usb_event(USBEventType::DISCONNECT);
                
                if (device_callback_) {
                    device_callback_(device, false);
                }
                
                host_controller_.connected_devices.pop_back();
            }
        }
        
        last_connection_check = now;
    }
}

void USBController::process_host_operations() {
    if (current_mode_ != USBMode::HOST && current_mode_ != USBMode::OTG) {
        return;
    }
    
    // Process host controller operations
    if (host_controller_.enabled) {
        // Update device states
        for (auto& device : host_controller_.connected_devices) {
            if (device.state == USBState::CONFIGURED) {
                // Device is ready for communication
                device_controller_.last_activity = std::chrono::steady_clock::now();
            }
        }
    }
}

void USBController::process_device_operations() {
    if (current_mode_ != USBMode::DEVICE && current_mode_ != USBMode::OTG) {
        return;
    }
    
    // Process setup packets
    process_setup_packets();
    
    // Update device state machine
    if (device_controller_.enabled) {
        auto now = std::chrono::steady_clock::now();
        auto idle_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - device_controller_.last_activity).count();
        
        // Simulate suspend after 3ms of inactivity (USB spec)
        if (idle_time > 3 && device_controller_.device.state != USBState::SUSPENDED) {
            device_controller_.device.state = USBState::SUSPENDED;
            trigger_usb_event(USBEventType::SUSPEND);
        }
    }
}

void USBController::process_setup_packets() {
    while (!device_controller_.setup_queue.empty()) {
        auto setup = device_controller_.setup_queue.front();
        device_controller_.setup_queue.pop();
        
        std::vector<u8> response;
        bool handled = false;
        
        // Determine request type
        u8 request_type = setup.bmRequestType & 0x60;
        switch (request_type) {
            case 0x00: // Standard
                handled = handle_standard_request(setup, response);
                break;
            case 0x20: // Class
                handled = handle_class_request(setup, response);
                break;
            case 0x40: // Vendor
                handled = handle_vendor_request(setup, response);
                break;
        }
        
        if (!handled) {
            // Stall the control endpoint
            auto ep_it = device_controller_.endpoints.find(0x00);
            if (ep_it != device_controller_.endpoints.end()) {
                ep_it->second.stalled = true;
            }
        }
        
        trigger_usb_event(USBEventType::SETUP, 0, 0, 
                         handled ? USBTransferStatus::COMPLETED : USBTransferStatus::STALLED);
    }
}

void USBController::process_transfer_queue() {
    // Move completed transfers to completion queue
    while (transfer_manager_.completed_transfers.size() > 100) {
        transfer_manager_.completed_transfers.pop();
    }
}

void USBController::handle_control_transfer(USBTransfer& transfer) {
    // Simulate control transfer timing
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - transfer.start_time).count();
    
    if (elapsed > 10) { // 10ms processing time
        transfer.bytes_transferred = transfer.data.size();
        transfer.status = USBTransferStatus::COMPLETED;
        transfer.completed = true;
        statistics_.transfers_completed++;
        statistics_.bytes_sent += transfer.data.size();
        
        trigger_usb_event(USBEventType::TRANSFER_COMPLETE, transfer.endpoint_address, 
                         transfer.id, transfer.status);
    }
}

void USBController::handle_bulk_transfer(USBTransfer& transfer) {
    // Simulate bulk transfer with realistic timing
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - transfer.start_time).count();
    
    // Calculate transfer time based on data size and speed
    u32 bytes_per_ms = 0;
    switch (phy_.speed) {
        case USBSpeed::LOW_SPEED:  bytes_per_ms = 188;    break; // 1.5 Mbps
        case USBSpeed::FULL_SPEED: bytes_per_ms = 1500;   break; // 12 Mbps
        case USBSpeed::HIGH_SPEED: bytes_per_ms = 60000;  break; // 480 Mbps
    }
    
    u32 expected_time = transfer.data.size() / bytes_per_ms + 1;
    
    if (elapsed >= expected_time) {
        transfer.bytes_transferred = transfer.data.size();
        transfer.status = USBTransferStatus::COMPLETED;
        transfer.completed = true;
        statistics_.transfers_completed++;
        statistics_.bulk_transfers++;
        statistics_.bytes_sent += transfer.data.size();
        
        trigger_usb_event(USBEventType::TRANSFER_COMPLETE, transfer.endpoint_address, 
                         transfer.id, transfer.status);
    }
}

void USBController::handle_interrupt_transfer(USBTransfer& transfer) {
    // Interrupt transfers complete immediately for simulation
    transfer.bytes_transferred = transfer.data.size();
    transfer.status = USBTransferStatus::COMPLETED;
    transfer.completed = true;
    statistics_.transfers_completed++;
    statistics_.interrupt_transfers++;
    statistics_.bytes_sent += transfer.data.size();
    
    trigger_usb_event(USBEventType::TRANSFER_COMPLETE, transfer.endpoint_address, 
                     transfer.id, transfer.status);
}

void USBController::handle_isochronous_transfer(USBTransfer& transfer) {
    // Isochronous transfers complete within one frame
    transfer.bytes_transferred = transfer.data.size();
    transfer.status = USBTransferStatus::COMPLETED;
    transfer.completed = true;
    statistics_.transfers_completed++;
    statistics_.isochronous_transfers++;
    statistics_.bytes_sent += transfer.data.size();
    
    trigger_usb_event(USBEventType::TRANSFER_COMPLETE, transfer.endpoint_address, 
                     transfer.id, transfer.status);
}

void USBController::complete_transfer(u32 transfer_id, USBTransferStatus status, size_t bytes_transferred) {
    auto it = transfer_manager_.active_transfers.find(transfer_id);
    if (it != transfer_manager_.active_transfers.end()) {
        it->second.status = status;
        it->second.bytes_transferred = bytes_transferred;
        it->second.completed = true;
        
        if (status == USBTransferStatus::COMPLETED) {
            statistics_.transfers_completed++;
        } else {
            statistics_.transfers_failed++;
        }
        
        if (transfer_callback_) {
            transfer_callback_(it->second);
        }
    }
}

std::vector<USBDevice> USBController::generate_simulated_devices() {
    std::vector<USBDevice> devices;
    
    // Generate a few common device types
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> device_dist(0, 3);
    
    switch (device_dist(gen)) {
        case 0: devices.push_back(create_hid_device()); break;
        case 1: devices.push_back(create_msc_device()); break;
        case 2: devices.push_back(create_cdc_device()); break;
        case 3: devices.push_back(create_audio_device()); break;
    }
    
    return devices;
}

USBDevice USBController::create_hid_device() {
    USBDevice device;
    device.device_descriptor.bDeviceClass = static_cast<u8>(USBDeviceClass::HID);
    device.device_descriptor.idVendor = 0x046D; // Logitech
    device.device_descriptor.idProduct = 0xC077; // Mouse
    device.speed = USBSpeed::FULL_SPEED;
    
    // Add HID configuration
    USBConfiguration config;
    config.value = 1;
    config.name = "HID Configuration";
    config.attributes = 0xA0; // Remote wakeup, bus powered
    config.max_power = 50; // 100mA
    
    USBInterface interface;
    interface.number = 0;
    interface.interface_class = USBDeviceClass::HID;
    interface.name = "HID Mouse Interface";
    
    // Add interrupt IN endpoint
    USBEndpoint endpoint;
    endpoint.address = 0x81; // IN endpoint 1
    endpoint.type = USBEndpointType::INTERRUPT;
    endpoint.max_packet_size = 8;
    endpoint.interval = 10; // 10ms polling
    interface.endpoints.push_back(endpoint);
    
    config.interfaces.push_back(interface);
    device.configurations.push_back(config);
    
    return device;
}

USBDevice USBController::create_msc_device() {
    USBDevice device;
    device.device_descriptor.bDeviceClass = static_cast<u8>(USBDeviceClass::MASS_STORAGE);
    device.device_descriptor.idVendor = 0x0781; // SanDisk
    device.device_descriptor.idProduct = 0x5567; // USB Flash Drive
    device.speed = USBSpeed::HIGH_SPEED;
    
    // Add Mass Storage configuration
    USBConfiguration config;
    config.value = 1;
    config.name = "Mass Storage Configuration";
    config.attributes = 0x80; // Bus powered
    config.max_power = 100; // 200mA
    
    USBInterface interface;
    interface.number = 0;
    interface.interface_class = USBDeviceClass::MASS_STORAGE;
    interface.sub_class = 0x06; // SCSI
    interface.protocol = 0x50; // Bulk-Only Transport
    interface.name = "Mass Storage Interface";
    
    // Add bulk IN and OUT endpoints
    USBEndpoint bulk_in;
    bulk_in.address = 0x81; // IN endpoint 1
    bulk_in.type = USBEndpointType::BULK;
    bulk_in.max_packet_size = 512;
    interface.endpoints.push_back(bulk_in);
    
    USBEndpoint bulk_out;
    bulk_out.address = 0x02; // OUT endpoint 2
    bulk_out.type = USBEndpointType::BULK;
    bulk_out.max_packet_size = 512;
    interface.endpoints.push_back(bulk_out);
    
    config.interfaces.push_back(interface);
    device.configurations.push_back(config);
    
    return device;
}

USBDevice USBController::create_cdc_device() {
    USBDevice device;
    device.device_descriptor.bDeviceClass = static_cast<u8>(USBDeviceClass::CDC);
    device.device_descriptor.idVendor = 0x2341; // Arduino
    device.device_descriptor.idProduct = 0x0043; // Uno R3
    device.speed = USBSpeed::FULL_SPEED;
    
    // Add CDC configuration
    USBConfiguration config;
    config.value = 1;
    config.name = "CDC Configuration";
    config.attributes = 0x80; // Bus powered
    config.max_power = 50; // 100mA
    
    // CDC Control Interface
    USBInterface control_interface;
    control_interface.number = 0;
    control_interface.interface_class = USBDeviceClass::CDC;
    control_interface.sub_class = 0x02; // ACM
    control_interface.protocol = 0x01; // AT Commands
    control_interface.name = "CDC Control Interface";
    
    // Add interrupt IN endpoint for notifications
    USBEndpoint interrupt_in;
    interrupt_in.address = 0x82; // IN endpoint 2
    interrupt_in.type = USBEndpointType::INTERRUPT;
    interrupt_in.max_packet_size = 8;
    interrupt_in.interval = 255;
    control_interface.endpoints.push_back(interrupt_in);
    
    // CDC Data Interface
    USBInterface data_interface;
    data_interface.number = 1;
    data_interface.interface_class = USBDeviceClass::CDC_DATA;
    data_interface.name = "CDC Data Interface";
    
    // Add bulk IN and OUT endpoints
    USBEndpoint bulk_in;
    bulk_in.address = 0x83; // IN endpoint 3
    bulk_in.type = USBEndpointType::BULK;
    bulk_in.max_packet_size = 64;
    data_interface.endpoints.push_back(bulk_in);
    
    USBEndpoint bulk_out;
    bulk_out.address = 0x01; // OUT endpoint 1
    bulk_out.type = USBEndpointType::BULK;
    bulk_out.max_packet_size = 64;
    data_interface.endpoints.push_back(bulk_out);
    
    config.interfaces.push_back(control_interface);
    config.interfaces.push_back(data_interface);
    device.configurations.push_back(config);
    
    return device;
}

USBDevice USBController::create_audio_device() {
    USBDevice device;
    device.device_descriptor.bDeviceClass = static_cast<u8>(USBDeviceClass::AUDIO);
    device.device_descriptor.idVendor = 0x0b05; // ASUS
    device.device_descriptor.idProduct = 0x1234; // USB Audio
    device.speed = USBSpeed::FULL_SPEED;
    
    // Add Audio configuration
    USBConfiguration config;
    config.value = 1;
    config.name = "Audio Configuration";
    config.attributes = 0x80; // Bus powered
    config.max_power = 100; // 200mA
    
    USBInterface interface;
    interface.number = 0;
    interface.interface_class = USBDeviceClass::AUDIO;
    interface.sub_class = 0x01; // Audio Control
    interface.name = "Audio Control Interface";
    
    config.interfaces.push_back(interface);
    device.configurations.push_back(config);
    
    return device;
}

std::vector<u8> USBController::build_device_descriptor(const USBDeviceDescriptor& desc) {
    std::vector<u8> data(18);
    
    data[0] = desc.bLength;
    data[1] = desc.bDescriptorType;
    data[2] = desc.bcdUSB & 0xFF;
    data[3] = (desc.bcdUSB >> 8) & 0xFF;
    data[4] = desc.bDeviceClass;
    data[5] = desc.bDeviceSubClass;
    data[6] = desc.bDeviceProtocol;
    data[7] = desc.bMaxPacketSize0;
    data[8] = desc.idVendor & 0xFF;
    data[9] = (desc.idVendor >> 8) & 0xFF;
    data[10] = desc.idProduct & 0xFF;
    data[11] = (desc.idProduct >> 8) & 0xFF;
    data[12] = desc.bcdDevice & 0xFF;
    data[13] = (desc.bcdDevice >> 8) & 0xFF;
    data[14] = desc.iManufacturer;
    data[15] = desc.iProduct;
    data[16] = desc.iSerialNumber;
    data[17] = desc.bNumConfigurations;
    
    return data;
}

std::vector<u8> USBController::build_string_descriptor(const std::string& str, u16 language_id) {
    if (str.empty() && language_id == 0) {
        // Language descriptor
        std::vector<u8> data = {4, static_cast<u8>(USBDescriptorType::STRING), 0x09, 0x04};
        return data;
    }
    
    std::vector<u8> data;
    data.push_back((str.length() * 2) + 2); // Length
    data.push_back(static_cast<u8>(USBDescriptorType::STRING));
    
    // Convert ASCII to UTF-16LE
    for (char c : str) {
        data.push_back(static_cast<u8>(c));
        data.push_back(0);
    }
    
    return data;
}

bool USBController::handle_standard_request(const USBSetupPacket& setup, std::vector<u8>& response) {
    switch (setup.bRequest) {
        case static_cast<u8>(USBStandardRequest::GET_DESCRIPTOR): {
            u8 desc_type = (setup.wValue >> 8) & 0xFF;
            u8 desc_index = setup.wValue & 0xFF;
            
            switch (desc_type) {
                case static_cast<u8>(USBDescriptorType::DEVICE):
                    response = build_device_descriptor(device_controller_.device.device_descriptor);
                    return true;
                    
                case static_cast<u8>(USBDescriptorType::STRING):
                    if (desc_index == 0) {
                        response = build_string_descriptor("", 0);
                    } else if (desc_index <= 3) {
                        std::string strings[] = {"", "Espressif Systems", "ESP32-C6 USB Device", "123456789ABC"};
                        response = build_string_descriptor(strings[desc_index], setup.wIndex);
                    }
                    return true;
                    
                default:
                    return false;
            }
        }
        
        case static_cast<u8>(USBStandardRequest::SET_ADDRESS):
            device_controller_.device.address = setup.wValue & 0x7F;
            device_controller_.device.state = USBState::ADDRESS;
            device_controller_.address_assigned = true;
            return true;
            
        case static_cast<u8>(USBStandardRequest::SET_CONFIGURATION):
            device_controller_.device.current_configuration = setup.wValue & 0xFF;
            if (setup.wValue > 0) {
                device_controller_.device.state = USBState::CONFIGURED;
                device_controller_.configured = true;
            }
            return true;
            
        case static_cast<u8>(USBStandardRequest::GET_STATUS):
            response = {0x00, 0x00}; // Self-powered = 0, Remote wakeup = 0
            return true;
            
        default:
            return false;
    }
}

bool USBController::handle_class_request(const USBSetupPacket& setup, std::vector<u8>& response) {
    // Handle class-specific requests (HID, CDC, etc.)
    LOG_DEBUG("USBController", "Class request: 0x{:02X}", setup.bRequest);
    return false; // Not implemented for simulation
}

bool USBController::handle_vendor_request(const USBSetupPacket& setup, std::vector<u8>& response) {
    // Handle vendor-specific requests
    LOG_DEBUG("USBController", "Vendor request: 0x{:02X}", setup.bRequest);
    return false; // Not implemented for simulation
}

void USBController::trigger_usb_event(USBEventType type, u8 endpoint, u32 transfer_id, 
                                     USBTransferStatus status, const std::vector<u8>& data) {
    USBEvent event;
    event.type = type;
    event.endpoint_address = endpoint;
    event.transfer_id = transfer_id;
    event.status = status;
    event.data = data;
    event.timestamp = std::chrono::steady_clock::now();
    
    if (event_callback_) {
        event_callback_(event);
    }
    
    // Trigger hardware interrupt for important events
    if (interrupt_controller_) {
        switch (type) {
            case USBEventType::CONNECT:
            case USBEventType::DISCONNECT:
            case USBEventType::RESET:
            case USBEventType::TRANSFER_COMPLETE:
            case USBEventType::ERROR:
                interrupt_controller_->trigger_interrupt(52); // USB interrupt line
                break;
            default:
                break;
        }
    }
}

bool USBController::is_endpoint_address_valid(u8 address) const {
    u8 ep_num = address & 0x0F;
    return ep_num < MAX_ENDPOINTS;
}

bool USBController::is_device_address_valid(u8 address) const {
    return address <= 127;
}

u8 USBController::get_endpoint_number(u8 address) const {
    return address & 0x0F;
}

bool USBController::is_endpoint_in(u8 address) const {
    return (address & 0x80) != 0;
}

u16 USBController::calculate_max_packet_size(USBSpeed speed, USBEndpointType type) {
    switch (speed) {
        case USBSpeed::LOW_SPEED:
            return (type == USBEndpointType::INTERRUPT) ? 8 : 8;
        case USBSpeed::FULL_SPEED:
            switch (type) {
                case USBEndpointType::CONTROL:
                case USBEndpointType::BULK:
                    return 64;
                case USBEndpointType::INTERRUPT:
                    return 64;
                case USBEndpointType::ISOCHRONOUS:
                    return 1023;
            }
            break;
        case USBSpeed::HIGH_SPEED:
            switch (type) {
                case USBEndpointType::CONTROL:
                    return 64;
                case USBEndpointType::BULK:
                    return 512;
                case USBEndpointType::INTERRUPT:
                    return 1024;
                case USBEndpointType::ISOCHRONOUS:
                    return 1024;
            }
            break;
    }
    return 64;
}

void USBController::update_frame_counter() {
    statistics_.current_frame_number = host_controller_.frame_number;
}

u32 USBController::calculate_power_consumption() {
    u32 power_mw = 0;
    
    if (!phy_.enabled) {
        return 10; // Standby power
    }
    
    // Base PHY power
    power_mw = 50;
    
    // Mode-specific power
    switch (current_mode_) {
        case USBMode::DEVICE:
            power_mw += 30;
            if (device_controller_.configured) {
                power_mw += 20;
            }
            break;
            
        case USBMode::HOST:
            power_mw += 80;
            if (power_manager_.vbus_enabled) {
                power_mw += 100; // VBUS generation
            }
            power_mw += host_controller_.connected_devices.size() * 50;
            break;
            
        case USBMode::OTG:
            power_mw += 40;
            break;
            
        default:
            break;
    }
    
    // Speed-based power adjustment
    switch (phy_.speed) {
        case USBSpeed::HIGH_SPEED:
            power_mw += 20;
            break;
        default:
            break;
    }
    
    return power_mw;
}

void USBController::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    
    // Calculate throughput
    static u64 last_bytes_sent = 0;
    static auto last_throughput_update = now;
    
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_throughput_update).count();
    
    if (dt_ms >= 1000) { // Update every second
        u64 bytes_delta = statistics_.bytes_sent - last_bytes_sent;
        statistics_.current_throughput_mbps = (bytes_delta * 8.0f) / (dt_ms * 1000.0f);
        statistics_.peak_throughput_mbps = std::max(statistics_.peak_throughput_mbps, 
                                                   statistics_.current_throughput_mbps);
        
        last_bytes_sent = statistics_.bytes_sent;
        last_throughput_update = now;
    }
    
    // Calculate error rate
    u64 total_transfers = statistics_.transfers_completed + statistics_.transfers_failed;
    if (total_transfers > 0) {
        statistics_.error_rate = static_cast<float>(statistics_.transfers_failed) / total_transfers;
    }
}

void USBController::clear_statistics() {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    statistics_ = USBStatistics{};
}

void USBController::dump_status() const {
    std::lock_guard<std::mutex> lock(usb_mutex_);
    
    LOG_INFO("USBController", "=== USB Controller Status ===");
    LOG_INFO("USBController", "Initialized: {}", initialized_);
    LOG_INFO("USBController", "Mode: {}", static_cast<int>(current_mode_));
    LOG_INFO("USBController", "PHY Enabled: {}", phy_.enabled);
    LOG_INFO("USBController", "Speed: {}", static_cast<int>(phy_.speed));
    LOG_INFO("USBController", "VBUS Enabled: {}", power_manager_.vbus_enabled);
    LOG_INFO("USBController", "VBUS Voltage: {} mV", power_manager_.vbus_voltage_mv);
    
    if (current_mode_ == USBMode::DEVICE || current_mode_ == USBMode::OTG) {
        LOG_INFO("USBController", "Device State: {}", static_cast<int>(device_controller_.device.state));
        LOG_INFO("USBController", "Device Address: {}", device_controller_.device.address);
        LOG_INFO("USBController", "Configuration: {}", device_controller_.device.current_configuration);
        LOG_INFO("USBController", "Endpoints Configured: {}", device_controller_.endpoints.size());
    }
    
    if (current_mode_ == USBMode::HOST || current_mode_ == USBMode::OTG) {
        LOG_INFO("USBController", "Connected Devices: {}", host_controller_.connected_devices.size());
        LOG_INFO("USBController", "Frame Number: {}", host_controller_.frame_number);
        
        for (const auto& device : host_controller_.connected_devices) {
            LOG_INFO("USBController", "  Device {}: VID:PID={:04X}:{:04X}, Class={}", 
                     device.address, device.device_descriptor.idVendor, 
                     device.device_descriptor.idProduct, device.device_descriptor.bDeviceClass);
        }
    }
    
    LOG_INFO("USBController", "Active Transfers: {}", transfer_manager_.active_transfers.size());
    LOG_INFO("USBController", "Power Consumption: {} mW", power_manager_.current_consumption_ma * 5);
    LOG_INFO("USBController", "Transfers Completed: {}", statistics_.transfers_completed);
    LOG_INFO("USBController", "Transfers Failed: {}", statistics_.transfers_failed);
    LOG_INFO("USBController", "Bytes Sent: {}", statistics_.bytes_sent);
    LOG_INFO("USBController", "Bytes Received: {}", statistics_.bytes_received);
    LOG_INFO("USBController", "Current Throughput: {:.2f} Mbps", statistics_.current_throughput_mbps);
    LOG_INFO("USBController", "Error Rate: {:.2f}%", statistics_.error_rate * 100.0f);
}

}  // namespace m5tab5::emulator