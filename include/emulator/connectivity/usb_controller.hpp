#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <array>
#include <string>

namespace m5tab5::emulator {

enum class USBMode : u8 {
    DISABLED = 0,
    DEVICE = 1,
    HOST = 2,
    OTG = 3         // On-The-Go (dual role)
};

enum class USBSpeed : u8 {
    LOW_SPEED = 0,      // 1.5 Mbps
    FULL_SPEED = 1,     // 12 Mbps
    HIGH_SPEED = 2      // 480 Mbps
};

enum class USBState : u8 {
    DETACHED = 0,
    ATTACHED = 1,
    POWERED = 2,
    DEFAULT = 3,
    ADDRESS = 4,
    CONFIGURED = 5,
    SUSPENDED = 6
};

enum class USBDeviceClass : u8 {
    INTERFACE_SPECIFIC = 0x00,
    AUDIO = 0x01,
    CDC = 0x02,         // Communications Device Class
    HID = 0x03,         // Human Interface Device
    PHYSICAL = 0x05,
    IMAGE = 0x06,
    PRINTER = 0x07,
    MASS_STORAGE = 0x08,
    HUB = 0x09,
    CDC_DATA = 0x0A,
    SMART_CARD = 0x0B,
    CONTENT_SECURITY = 0x0D,
    VIDEO = 0x0E,
    PERSONAL_HEALTHCARE = 0x0F,
    AUDIO_VIDEO = 0x10,
    BILLBOARD = 0x11,
    DIAGNOSTIC = 0xDC,
    WIRELESS = 0xE0,
    MISCELLANEOUS = 0xEF,
    APPLICATION_SPECIFIC = 0xFE,
    VENDOR_SPECIFIC = 0xFF
};

enum class USBRequestType : u8 {
    STANDARD = 0,
    CLASS = 1,
    VENDOR = 2
};

enum class USBStandardRequest : u8 {
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    SET_FEATURE = 3,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12
};

enum class USBDescriptorType : u8 {
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,
    DEVICE_QUALIFIER = 6,
    OTHER_SPEED_CONFIG = 7,
    INTERFACE_POWER = 8,
    OTG = 9,
    DEBUG = 10,
    INTERFACE_ASSOCIATION = 11,
    HID = 0x21,
    HID_REPORT = 0x22,
    HID_PHYSICAL = 0x23
};

enum class USBEndpointType : u8 {
    CONTROL = 0,
    ISOCHRONOUS = 1,
    BULK = 2,
    INTERRUPT = 3
};

enum class USBTransferStatus : u8 {
    COMPLETED = 0,
    ERROR = 1,
    STALLED = 2,
    NAK = 3,
    TIMEOUT = 4,
    BABBLE = 5,
    CANCELLED = 6
};

enum class USBEventType : u8 {
    CONNECT = 0,
    DISCONNECT = 1,
    RESET = 2,
    SUSPEND = 3,
    RESUME = 4,
    SOF = 5,            // Start of Frame
    SETUP = 6,
    TRANSFER_COMPLETE = 7,
    ERROR = 8,
    OVERCURRENT = 9,
    VBUS_CHANGE = 10
};

struct USBSetupPacket {
    u8 bmRequestType;   // Request type and direction
    u8 bRequest;        // Specific request
    u16 wValue;         // Request-specific value
    u16 wIndex;         // Request-specific index
    u16 wLength;        // Length of data phase
};

struct USBDeviceDescriptor {
    u8 bLength = 18;
    u8 bDescriptorType = static_cast<u8>(USBDescriptorType::DEVICE);
    u16 bcdUSB = 0x0200;        // USB 2.0
    u8 bDeviceClass = 0;
    u8 bDeviceSubClass = 0;
    u8 bDeviceProtocol = 0;
    u8 bMaxPacketSize0 = 64;    // Control endpoint packet size
    u16 idVendor = 0x303A;      // Espressif vendor ID
    u16 idProduct = 0x1001;
    u16 bcdDevice = 0x0100;     // Device release number
    u8 iManufacturer = 1;       // String descriptor index
    u8 iProduct = 2;
    u8 iSerialNumber = 3;
    u8 bNumConfigurations = 1;
};

struct USBConfigurationDescriptor {
    u8 bLength = 9;
    u8 bDescriptorType = static_cast<u8>(USBDescriptorType::CONFIGURATION);
    u16 wTotalLength = 0;       // Total length including interfaces/endpoints
    u8 bNumInterfaces = 1;
    u8 bConfigurationValue = 1;
    u8 iConfiguration = 0;
    u8 bmAttributes = 0x80;     // Bus powered
    u8 bMaxPower = 50;          // 100mA (units of 2mA)
};

struct USBInterfaceDescriptor {
    u8 bLength = 9;
    u8 bDescriptorType = static_cast<u8>(USBDescriptorType::INTERFACE);
    u8 bInterfaceNumber = 0;
    u8 bAlternateSetting = 0;
    u8 bNumEndpoints = 0;
    u8 bInterfaceClass = 0;
    u8 bInterfaceSubClass = 0;
    u8 bInterfaceProtocol = 0;
    u8 iInterface = 0;
};

struct USBEndpointDescriptor {
    u8 bLength = 7;
    u8 bDescriptorType = static_cast<u8>(USBDescriptorType::ENDPOINT);
    u8 bEndpointAddress = 0;    // Endpoint number and direction
    u8 bmAttributes = 0;        // Transfer type
    u16 wMaxPacketSize = 0;
    u8 bInterval = 0;           // Polling interval
};

struct USBStringDescriptor {
    u8 bLength;
    u8 bDescriptorType = static_cast<u8>(USBDescriptorType::STRING);
    std::vector<u16> bString;   // UTF-16LE encoded string
};

struct USBEndpoint {
    u8 address;
    USBEndpointType type;
    u16 max_packet_size;
    u8 interval;
    bool stalled;
    std::queue<std::vector<u8>> tx_queue;
    std::queue<std::vector<u8>> rx_queue;
    u64 bytes_transferred;
    u32 transfer_count;
};

struct USBInterface {
    u8 number;
    u8 alternate_setting;
    USBDeviceClass interface_class;
    u8 sub_class;
    u8 protocol;
    std::string name;
    std::vector<USBEndpoint> endpoints;
    bool claimed;
};

struct USBConfiguration {
    u8 value;
    std::string name;
    u8 attributes;
    u8 max_power;
    std::vector<USBInterface> interfaces;
};

struct USBDevice {
    u8 address = 0;
    USBSpeed speed = USBSpeed::FULL_SPEED;
    USBState state = USBState::DETACHED;
    USBDeviceDescriptor device_descriptor;
    std::vector<USBConfiguration> configurations;
    std::vector<USBStringDescriptor> string_descriptors;
    u8 current_configuration = 0;
    std::chrono::steady_clock::time_point connect_time;
    bool self_powered = false;
    bool remote_wakeup = false;
    u16 current_frame = 0;
};

struct USBTransfer {
    u32 id;
    u8 endpoint_address;
    USBEndpointType type;
    std::vector<u8> data;
    size_t bytes_transferred;
    USBTransferStatus status;
    std::chrono::steady_clock::time_point start_time;
    u32 timeout_ms;
    bool completed;
};

struct USBEvent {
    USBEventType type;
    u8 endpoint_address;
    u32 transfer_id;
    USBTransferStatus status;
    std::vector<u8> data;
    std::chrono::steady_clock::time_point timestamp;
};

struct USBStatistics {
    u64 transfers_completed = 0;
    u64 transfers_failed = 0;
    u64 bytes_sent = 0;
    u64 bytes_received = 0;
    u64 control_transfers = 0;
    u64 bulk_transfers = 0;
    u64 interrupt_transfers = 0;
    u64 isochronous_transfers = 0;
    u64 setup_packets = 0;
    u64 sof_packets = 0;
    u64 nak_responses = 0;
    u64 stall_responses = 0;
    u32 current_frame_number = 0;
    float current_throughput_mbps = 0.0f;
    float peak_throughput_mbps = 0.0f;
    u32 power_consumption_mw = 0;
    u32 enumeration_time_ms = 0;
    float error_rate = 0.0f;
};

using USBEventCallback = std::function<void(const USBEvent& event)>;
using USBTransferCallback = std::function<void(const USBTransfer& transfer)>;
using USBDeviceCallback = std::function<void(const USBDevice& device, bool connected)>;

class USBController {
public:
    static constexpr u16 USB_VID_ESPRESSIF = 0x303A;
    static constexpr u16 USB_PID_ESP32_C6 = 0x1001;
    static constexpr u8 MAX_ENDPOINTS = 16;
    static constexpr u8 MAX_DEVICES = 8;
    static constexpr u16 MAX_PACKET_SIZE_FS = 64;
    static constexpr u16 MAX_PACKET_SIZE_HS = 512;
    static constexpr u32 SOF_INTERVAL_US = 1000;  // 1ms for full speed
    
    USBController();
    ~USBController();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller);
    Result<void> shutdown();

    // Mode and power management
    Result<void> set_mode(USBMode mode);
    Result<USBMode> get_mode() const;
    Result<void> enable_vbus(bool enable);
    Result<bool> is_vbus_enabled() const;
    Result<void> set_speed(USBSpeed speed);
    Result<USBSpeed> get_speed() const;
    
    // Device mode operations
    Result<void> device_set_descriptor(USBDescriptorType type, u8 index, const std::vector<u8>& descriptor);
    Result<std::vector<u8>> device_get_descriptor(USBDescriptorType type, u8 index) const;
    Result<void> device_configure_endpoint(u8 address, USBEndpointType type, u16 max_packet_size, u8 interval = 0);
    Result<void> device_enable_endpoint(u8 address, bool enable);
    Result<void> device_stall_endpoint(u8 address, bool stall);
    Result<USBState> device_get_state() const;
    
    // Device mode data transfer
    Result<u32> device_submit_transfer(u8 endpoint_address, const std::vector<u8>& data, u32 timeout_ms = 1000);
    Result<std::vector<u8>> device_receive_data(u8 endpoint_address, size_t max_length = 0, u32 timeout_ms = 1000);
    Result<void> device_send_zlp(u8 endpoint_address); // Zero Length Packet
    
    // Host mode operations
    Result<void> host_enumerate_devices();
    Result<std::vector<USBDevice>> host_get_devices() const;
    Result<USBDevice> host_get_device(u8 address) const;
    Result<void> host_reset_device(u8 address);
    Result<void> host_set_device_configuration(u8 address, u8 config_value);
    
    // Host mode transfers
    Result<u32> host_control_transfer(u8 device_address, const USBSetupPacket& setup, 
                                     std::vector<u8>& data, u32 timeout_ms = 5000);
    Result<u32> host_bulk_transfer(u8 device_address, u8 endpoint_address, 
                                  const std::vector<u8>& data, u32 timeout_ms = 5000);
    Result<u32> host_interrupt_transfer(u8 device_address, u8 endpoint_address, 
                                       const std::vector<u8>& data, u32 timeout_ms = 5000);
    Result<std::vector<u8>> host_receive_data(u8 device_address, u8 endpoint_address, 
                                             size_t max_length, u32 timeout_ms = 5000);
    
    // Transfer management
    Result<USBTransfer> get_transfer_status(u32 transfer_id) const;
    Result<void> cancel_transfer(u32 transfer_id);
    Result<std::vector<USBTransfer>> get_pending_transfers() const;
    
    // Standard USB requests
    Result<std::vector<u8>> get_device_descriptor(u8 device_address = 0);
    Result<std::vector<u8>> get_configuration_descriptor(u8 device_address, u8 config_index);
    Result<std::vector<u8>> get_string_descriptor(u8 device_address, u8 string_index, u16 language_id = 0x0409);
    Result<void> set_device_address(u8 old_address, u8 new_address);
    Result<void> set_device_configuration(u8 device_address, u8 config_value);
    
    // USB class-specific operations
    Result<void> hid_set_report(u8 device_address, u8 interface_number, const std::vector<u8>& report);
    Result<std::vector<u8>> hid_get_report(u8 device_address, u8 interface_number, u8 report_id);
    Result<void> msc_send_cbw(u8 device_address, const std::vector<u8>& cbw); // Command Block Wrapper
    Result<std::vector<u8>> msc_receive_csw(u8 device_address); // Command Status Wrapper
    
    // OTG (On-The-Go) operations
    Result<void> otg_start_hnp(); // Host Negotiation Protocol
    Result<void> otg_start_srp(); // Session Request Protocol
    Result<bool> otg_is_a_device() const;
    Result<bool> otg_is_b_device() const;
    
    // Power and suspend management
    Result<void> suspend_bus();
    Result<void> resume_bus();
    Result<void> remote_wakeup();
    Result<bool> is_suspended() const;
    Result<void> set_power_budget(u16 power_ma);
    Result<u16> get_power_budget() const;
    
    // Error handling and diagnostics
    Result<void> clear_error_conditions();
    Result<std::vector<u8>> get_error_log();
    Result<void> enable_debug_mode(bool enable);
    Result<void> enable_test_mode(u8 test_selector);
    Result<void> perform_compliance_test();
    
    // Event handling
    Result<void> set_event_callback(USBEventCallback callback);
    Result<void> set_transfer_callback(USBTransferCallback callback);
    Result<void> set_device_callback(USBDeviceCallback callback);
    
    // Real-time monitoring
    void update();
    Result<void> force_update();
    
    bool is_initialized() const { return initialized_; }
    const USBStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct USBPhy {
        bool enabled = false;
        USBSpeed speed = USBSpeed::FULL_SPEED;
        bool differential_receiver = true;
        bool single_ended_receiver = false;
        i8 signal_strength_dbm = 0;
        bool termination_enabled = true;
        u8 drive_strength = 2; // 0-7 scale
    };

    struct USBHostController {
        bool enabled = false;
        std::vector<USBDevice> connected_devices;
        u8 next_device_address = 1;
        bool root_hub_enabled = false;
        u8 port_count = 1;
        std::array<bool, 4> port_power;
        std::array<bool, 4> port_enabled;
        std::chrono::steady_clock::time_point last_sof;
        u16 frame_number = 0;
        bool periodic_list_enabled = false;
        bool async_list_enabled = false;
    };

    struct USBDeviceController {
        bool enabled = false;
        USBDevice device;
        std::map<u8, USBEndpoint> endpoints;
        bool address_assigned = false;
        bool configured = false;
        std::queue<USBSetupPacket> setup_queue;
        std::chrono::steady_clock::time_point last_activity;
        bool remote_wakeup_enabled = false;
        bool self_powered = false;
    };

    struct USBTransferManager {
        std::map<u32, USBTransfer> active_transfers;
        u32 next_transfer_id = 1;
        std::queue<USBTransfer> completed_transfers;
        std::chrono::steady_clock::time_point last_transfer_check;
        u32 max_concurrent_transfers = 32;
    };

    struct USBPowerManager {
        bool vbus_enabled = false;
        bool session_valid = false;
        u16 vbus_voltage_mv = 0;
        u16 power_budget_ma = 500;
        u16 current_consumption_ma = 0;
        bool overcurrent_detected = false;
        std::chrono::steady_clock::time_point last_power_measurement;
    };

    struct USBErrorHandler {
        std::vector<std::string> error_log;
        u32 crc_errors = 0;
        u32 timeout_errors = 0;
        u32 protocol_errors = 0;
        u32 babble_errors = 0;
        bool debug_mode = false;
        u8 test_mode = 0;
        size_t max_log_entries = 1000;
    };

    // Core USB operations
    void simulate_usb_enumeration();
    void simulate_usb_transfers();
    void simulate_sof_generation();
    void simulate_power_management();
    void simulate_device_connections();
    
    void process_host_operations();
    void process_device_operations();
    void process_setup_packets();
    void process_transfer_queue();
    void update_frame_counter();
    
    // Transfer handling
    void handle_control_transfer(USBTransfer& transfer);
    void handle_bulk_transfer(USBTransfer& transfer);
    void handle_interrupt_transfer(USBTransfer& transfer);
    void handle_isochronous_transfer(USBTransfer& transfer);
    void complete_transfer(u32 transfer_id, USBTransferStatus status, size_t bytes_transferred = 0);
    
    // Device simulation
    std::vector<USBDevice> generate_simulated_devices();
    USBDevice create_hid_device();
    USBDevice create_msc_device();
    USBDevice create_cdc_device();
    USBDevice create_audio_device();
    
    // Descriptor generation
    std::vector<u8> build_device_descriptor(const USBDeviceDescriptor& desc);
    std::vector<u8> build_configuration_descriptor(const USBConfiguration& config);
    std::vector<u8> build_interface_descriptor(const USBInterface& interface);
    std::vector<u8> build_endpoint_descriptor(const USBEndpoint& endpoint);
    std::vector<u8> build_string_descriptor(const std::string& str, u16 language_id = 0x0409);
    
    // Protocol handling
    bool handle_standard_request(const USBSetupPacket& setup, std::vector<u8>& response);
    bool handle_class_request(const USBSetupPacket& setup, std::vector<u8>& response);
    bool handle_vendor_request(const USBSetupPacket& setup, std::vector<u8>& response);
    
    void trigger_usb_event(USBEventType type, u8 endpoint = 0, u32 transfer_id = 0, 
                          USBTransferStatus status = USBTransferStatus::COMPLETED,
                          const std::vector<u8>& data = {});
    
    // Utility functions
    bool is_endpoint_address_valid(u8 address) const;
    bool is_device_address_valid(u8 address) const;
    u8 get_endpoint_number(u8 address) const;
    bool is_endpoint_in(u8 address) const;
    USBSpeed negotiate_speed(USBSpeed host_speed, USBSpeed device_speed);
    u16 calculate_max_packet_size(USBSpeed speed, USBEndpointType type);
    void update_statistics();
    u32 calculate_power_consumption();
    
    bool initialized_;
    USBMode current_mode_;
    
    InterruptController* interrupt_controller_;
    
    // Hardware simulation
    USBPhy phy_;
    USBHostController host_controller_;
    USBDeviceController device_controller_;
    USBTransferManager transfer_manager_;
    USBPowerManager power_manager_;
    USBErrorHandler error_handler_;
    
    // Callback interfaces
    USBEventCallback event_callback_;
    USBTransferCallback transfer_callback_;
    USBDeviceCallback device_callback_;
    
    // Background processing
    std::unique_ptr<std::thread> usb_thread_;
    std::atomic<bool> thread_running_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    USBStatistics statistics_;
    
    mutable std::mutex usb_mutex_;
};

}  // namespace m5tab5::emulator