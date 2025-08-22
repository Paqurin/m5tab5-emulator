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

enum class BluetoothMode : u8 {
    OFF = 0,
    CLASSIC = 1,        // Bluetooth Classic (BR/EDR)
    BLE = 2,           // Bluetooth Low Energy
    DUAL = 3           // Both Classic and BLE
};

enum class BluetoothState : u8 {
    IDLE = 0,
    DISCOVERING = 1,
    DISCOVERABLE = 2,
    CONNECTING = 3,
    CONNECTED = 4,
    DISCONNECTING = 5,
    PAIRING = 6,
    BONDED = 7
};

enum class BLEConnectionState : u8 {
    DISCONNECTED = 0,
    SCANNING = 1,
    CONNECTING = 2,
    CONNECTED = 3,
    DISCONNECTING = 4
};

enum class BluetoothEventType : u8 {
    STATE_CHANGED = 0,
    DISCOVERY_STARTED = 1,
    DISCOVERY_FINISHED = 2,
    DEVICE_FOUND = 3,
    PAIRING_REQUEST = 4,
    PAIRING_COMPLETE = 5,
    CONNECTION_STATE_CHANGED = 6,
    DATA_RECEIVED = 7,
    ACL_CONNECTED = 8,
    ACL_DISCONNECTED = 9,
    BOND_STATE_CHANGED = 10,
    // BLE specific events
    BLE_SCAN_RESULT = 20,
    BLE_CONNECT = 21,
    BLE_DISCONNECT = 22,
    BLE_GATT_DISCOVERED = 23,
    BLE_CHARACTERISTIC_READ = 24,
    BLE_CHARACTERISTIC_WRITE = 25,
    BLE_NOTIFICATION = 26,
    BLE_ADVERTISEMENT = 27
};

enum class BluetoothDeviceType : u8 {
    UNKNOWN = 0,
    COMPUTER = 1,
    PHONE = 2,
    HEADSET = 3,
    SPEAKER = 4,
    KEYBOARD = 5,
    MOUSE = 6,
    GAMEPAD = 7,
    SENSOR = 8,
    BEACON = 9,
    FITNESS_TRACKER = 10,
    SMARTWATCH = 11
};

enum class BluetoothProfile : u8 {
    SPP = 0,           // Serial Port Profile
    A2DP = 1,          // Advanced Audio Distribution Profile
    AVRCP = 2,         // Audio/Video Remote Control Profile
    HFP = 3,           // Hands-Free Profile
    HSP = 4,           // Headset Profile
    HID = 5,           // Human Interface Device Profile
    OBEX = 6,          // Object Exchange
    PAN = 7,           // Personal Area Network
    PBAP = 8,          // Phone Book Access Profile
    MAP = 9            // Message Access Profile
};

enum class BLEAdvertisementType : u8 {
    ADV_IND = 0,           // Connectable undirected advertising
    ADV_DIRECT_IND = 1,    // Connectable directed advertising
    ADV_SCAN_IND = 2,      // Scannable undirected advertising
    ADV_NONCONN_IND = 3,   // Non-connectable undirected advertising
    SCAN_RSP = 4           // Scan response
};

enum class BLEDataType : u8 {
    FLAGS = 0x01,
    INCOMPLETE_16BIT_UUIDS = 0x02,
    COMPLETE_16BIT_UUIDS = 0x03,
    INCOMPLETE_32BIT_UUIDS = 0x04,
    COMPLETE_32BIT_UUIDS = 0x05,
    INCOMPLETE_128BIT_UUIDS = 0x06,
    COMPLETE_128BIT_UUIDS = 0x07,
    SHORTENED_LOCAL_NAME = 0x08,
    COMPLETE_LOCAL_NAME = 0x09,
    TX_POWER_LEVEL = 0x0A,
    MANUFACTURER_DATA = 0xFF
};

struct BluetoothAddress {
    std::array<u8, 6> address;
    
    BluetoothAddress() {
        address.fill(0);
    }
    
    BluetoothAddress(const std::array<u8, 6>& addr) : address(addr) {}
    
    std::string to_string() const {
        char buffer[18];
        snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
                address[0], address[1], address[2], address[3], address[4], address[5]);
        return std::string(buffer);
    }
    
    bool operator==(const BluetoothAddress& other) const {
        return address == other.address;
    }
    
    bool operator<(const BluetoothAddress& other) const {
        return address < other.address;
    }
};

struct BluetoothDevice {
    BluetoothAddress address;
    std::string name;
    BluetoothDeviceType device_type;
    std::vector<BluetoothProfile> supported_profiles;
    i8 rssi;
    u32 class_of_device;
    bool is_bonded;
    bool is_connected;
    std::chrono::steady_clock::time_point last_seen;
    std::map<std::string, std::vector<u8>> service_data;
};

struct BLEDevice {
    BluetoothAddress address;
    std::string name;
    BLEAdvertisementType adv_type;
    i8 rssi;
    u8 tx_power;
    std::vector<u8> advertisement_data;
    std::vector<u8> scan_response_data;
    std::vector<std::string> service_uuids;
    std::map<u16, std::vector<u8>> manufacturer_data;
    std::chrono::steady_clock::time_point last_advertisement;
    bool connectable;
};

struct BLECharacteristic {
    std::string uuid;
    u16 handle;
    u8 properties;      // Read, Write, Notify, Indicate flags
    std::vector<u8> value;
    std::vector<u8> descriptors;
    bool notifications_enabled;
};

struct BLEService {
    std::string uuid;
    u16 start_handle;
    u16 end_handle;
    std::vector<BLECharacteristic> characteristics;
};

struct BLEConnection {
    BluetoothAddress remote_address;
    BLEConnectionState state;
    u16 connection_handle;
    u16 connection_interval;    // In 1.25ms units
    u16 slave_latency;
    u16 supervision_timeout;    // In 10ms units
    std::vector<BLEService> services;
    std::chrono::steady_clock::time_point connection_time;
    i8 rssi;
};

struct BluetoothEvent {
    BluetoothEventType type;
    BluetoothAddress device_address;
    std::vector<u8> data;
    u32 event_data;
    std::chrono::steady_clock::time_point timestamp;
};

struct BluetoothStatistics {
    u64 classic_connections = 0;
    u64 ble_connections = 0;
    u64 discovery_sessions = 0;
    u64 pairing_attempts = 0;
    u64 successful_pairings = 0;
    u64 data_packets_sent = 0;
    u64 data_packets_received = 0;
    u64 bytes_sent = 0;
    u64 bytes_received = 0;
    u64 ble_advertisements_sent = 0;
    u64 ble_scan_results = 0;
    u64 gatt_operations = 0;
    u32 active_connections = 0;
    float average_rssi = 0.0f;
    float connection_success_rate = 0.0f;
    u32 current_power_consumption_mw = 0;
};

using BluetoothEventCallback = std::function<void(const BluetoothEvent& event)>;
using BluetoothDataCallback = std::function<void(const BluetoothAddress& device, const std::vector<u8>& data)>;
using BLEScanCallback = std::function<void(const BLEDevice& device)>;
using BLECharacteristicCallback = std::function<void(const std::string& uuid, const std::vector<u8>& value)>;

class BluetoothController {
public:
    static constexpr size_t MAX_DEVICE_NAME_LEN = 248;
    static constexpr size_t MAX_ADVERTISEMENT_DATA_LEN = 31;
    static constexpr size_t MAX_SCAN_RESPONSE_LEN = 31;
    static constexpr size_t MAX_CHARACTERISTIC_VALUE_LEN = 512;
    static constexpr u16 MAX_BLE_CONNECTIONS = 20;
    static constexpr u16 DEFAULT_SCAN_WINDOW = 0x30;    // 30ms
    static constexpr u16 DEFAULT_SCAN_INTERVAL = 0x60;  // 60ms
    
    BluetoothController();
    ~BluetoothController();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller);
    Result<void> shutdown();

    // General Bluetooth control
    Result<void> set_mode(BluetoothMode mode);
    Result<BluetoothMode> get_mode() const;
    Result<void> enable(bool enable);
    Result<bool> is_enabled() const;
    Result<BluetoothState> get_state() const;
    
    // Device configuration
    Result<void> set_device_name(const std::string& name);
    Result<std::string> get_device_name() const;
    Result<void> set_local_address(const BluetoothAddress& address);
    Result<BluetoothAddress> get_local_address() const;
    Result<void> set_discoverable(bool discoverable, u32 timeout_s = 0);
    Result<bool> is_discoverable() const;
    
    // Classic Bluetooth operations
    Result<void> start_discovery(u32 duration_s = 10);
    Result<void> stop_discovery();
    Result<bool> is_discovering() const;
    Result<std::vector<BluetoothDevice>> get_discovered_devices() const;
    Result<void> clear_discovered_devices();
    
    // Connection management
    Result<void> connect_device(const BluetoothAddress& address, BluetoothProfile profile = BluetoothProfile::SPP);
    Result<void> disconnect_device(const BluetoothAddress& address);
    Result<std::vector<BluetoothDevice>> get_connected_devices() const;
    Result<bool> is_device_connected(const BluetoothAddress& address) const;
    
    // Pairing and bonding
    Result<void> pair_device(const BluetoothAddress& address, const std::string& pin = "");
    Result<void> unpair_device(const BluetoothAddress& address);
    Result<std::vector<BluetoothDevice>> get_bonded_devices() const;
    Result<bool> is_device_bonded(const BluetoothAddress& address) const;
    
    // Data transmission (Classic)
    Result<void> send_data(const BluetoothAddress& device, const std::vector<u8>& data, BluetoothProfile profile = BluetoothProfile::SPP);
    Result<std::vector<u8>> receive_data(const BluetoothAddress& device, size_t max_length = 1024);
    
    // BLE operations
    Result<void> ble_start_scan(u32 duration_ms = 10000, bool active_scan = true);
    Result<void> ble_stop_scan();
    Result<bool> ble_is_scanning() const;
    Result<std::vector<BLEDevice>> ble_get_scan_results() const;
    Result<void> ble_clear_scan_results();
    
    // BLE advertising
    Result<void> ble_start_advertising(const std::vector<u8>& advertisement_data, 
                                      const std::vector<u8>& scan_response_data = {},
                                      u16 interval_min = 0x20, u16 interval_max = 0x40);
    Result<void> ble_stop_advertising();
    Result<bool> ble_is_advertising() const;
    Result<void> ble_set_advertisement_data(const std::vector<u8>& data);
    Result<void> ble_set_scan_response_data(const std::vector<u8>& data);
    
    // BLE connection management
    Result<u16> ble_connect(const BluetoothAddress& address, u16 conn_interval_min = 0x06, u16 conn_interval_max = 0x0C);
    Result<void> ble_disconnect(u16 connection_handle);
    Result<std::vector<BLEConnection>> ble_get_connections() const;
    Result<BLEConnection> ble_get_connection_info(u16 connection_handle) const;
    
    // GATT operations
    Result<void> gatt_discover_services(u16 connection_handle);
    Result<std::vector<BLEService>> gatt_get_services(u16 connection_handle) const;
    Result<void> gatt_discover_characteristics(u16 connection_handle, const std::string& service_uuid);
    Result<std::vector<BLECharacteristic>> gatt_get_characteristics(u16 connection_handle, const std::string& service_uuid) const;
    
    // GATT characteristic operations
    Result<std::vector<u8>> gatt_read_characteristic(u16 connection_handle, const std::string& char_uuid);
    Result<void> gatt_write_characteristic(u16 connection_handle, const std::string& char_uuid, const std::vector<u8>& value, bool with_response = true);
    Result<void> gatt_enable_notifications(u16 connection_handle, const std::string& char_uuid, bool enable);
    
    // GATT server (peripheral) operations
    Result<void> gatt_server_add_service(const BLEService& service);
    Result<void> gatt_server_remove_service(const std::string& service_uuid);
    Result<void> gatt_server_update_characteristic(const std::string& char_uuid, const std::vector<u8>& value);
    Result<void> gatt_server_notify_characteristic(const std::string& char_uuid, const std::vector<u8>& value);
    
    // Power management
    Result<void> set_tx_power(i8 power_dbm);
    Result<i8> get_tx_power() const;
    Result<void> set_scan_parameters(u16 scan_interval, u16 scan_window);
    Result<void> set_connection_parameters(u16 conn_interval_min, u16 conn_interval_max, u16 slave_latency, u16 supervision_timeout);
    
    // Security and encryption
    Result<void> set_security_level(u8 level); // 0=None, 1=Low, 2=Medium, 3=High
    Result<u8> get_security_level() const;
    Result<void> set_io_capabilities(u8 capabilities); // 0=DisplayOnly, 1=DisplayYesNo, 2=KeyboardOnly, 3=NoInputNoOutput, 4=KeyboardDisplay
    Result<void> enable_secure_connections(bool enable);
    
    // Event handling
    Result<void> set_event_callback(BluetoothEventCallback callback);
    Result<void> set_data_callback(BluetoothDataCallback callback);
    Result<void> set_ble_scan_callback(BLEScanCallback callback);
    Result<void> set_characteristic_callback(BLECharacteristicCallback callback);
    
    // Diagnostics and utilities
    Result<std::vector<u8>> get_raw_packet_dump(size_t max_packets = 100);
    Result<void> enable_packet_logging(bool enable);
    Result<void> perform_rssi_measurement(const BluetoothAddress& device);
    Result<void> reset_controller();
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    const BluetoothStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct ClassicBluetoothState {
        BluetoothState state = BluetoothState::IDLE;
        bool discoverable = false;
        u32 discoverable_timeout_s = 0;
        std::chrono::steady_clock::time_point discoverable_start;
        bool discovering = false;
        std::chrono::steady_clock::time_point discovery_start;
        u32 discovery_duration_s = 10;
        std::vector<BluetoothDevice> discovered_devices;
        std::vector<BluetoothDevice> connected_devices;
        std::vector<BluetoothDevice> bonded_devices;
        std::queue<std::vector<u8>> data_queue;
    };

    struct BLEControllerState {
        bool scanning = false;
        bool advertising = false;
        std::chrono::steady_clock::time_point scan_start;
        u32 scan_duration_ms = 10000;
        bool active_scan = true;
        u16 scan_interval = DEFAULT_SCAN_INTERVAL;
        u16 scan_window = DEFAULT_SCAN_WINDOW;
        
        std::vector<u8> advertisement_data;
        std::vector<u8> scan_response_data;
        u16 advertising_interval_min = 0x20;
        u16 advertising_interval_max = 0x40;
        std::chrono::steady_clock::time_point last_advertisement;
        
        std::vector<BLEDevice> scan_results;
        std::map<u16, BLEConnection> connections;
        u16 next_connection_handle = 1;
        
        // GATT server state
        std::vector<BLEService> server_services;
        std::map<std::string, std::vector<u16>> characteristic_subscriptions; // char_uuid -> connection handles
    };

    struct SecurityManager {
        u8 security_level = 1;          // Low security by default
        u8 io_capabilities = 3;         // NoInputNoOutput
        bool secure_connections = false;
        std::map<BluetoothAddress, std::string> pairing_keys;
        std::map<BluetoothAddress, u32> pairing_passkeys;
        bool mitm_protection = false;
        bool bonding = true;
    };

    struct PowerManager {
        i8 tx_power_dbm = 0;
        u32 current_consumption_mw = 0;
        bool low_power_mode = false;
        std::chrono::steady_clock::time_point last_activity;
    };

    struct PacketProcessor {
        std::queue<std::vector<u8>> tx_queue;
        std::queue<std::vector<u8>> rx_queue;
        bool packet_logging = false;
        std::vector<std::vector<u8>> packet_dump;
        size_t max_dump_size = 1000;
    };

    void simulate_bluetooth_environment();
    void simulate_device_discovery();
    void simulate_ble_scanning();
    void simulate_ble_advertising();
    void simulate_connections();
    void simulate_data_transmission();
    void simulate_gatt_operations();
    
    void process_classic_bluetooth();
    void process_ble_operations();
    void process_pairing_requests();
    void update_connection_states();
    void handle_gatt_requests();
    void generate_advertisement_packet();
    
    void trigger_bluetooth_event(BluetoothEventType type, const BluetoothAddress& address = BluetoothAddress(),
                                const std::vector<u8>& data = {}, u32 event_data = 0);
    
    // Device simulation
    std::vector<BluetoothDevice> generate_simulated_classic_devices();
    std::vector<BLEDevice> generate_simulated_ble_devices();
    BluetoothAddress generate_random_address(bool random_static = false);
    std::string generate_device_name(BluetoothDeviceType type);
    i8 simulate_rssi(const BluetoothAddress& device);
    
    // Protocol simulation
    bool simulate_pairing_process(const BluetoothAddress& device, const std::string& pin);
    bool simulate_ble_connection_process(const BluetoothAddress& device);
    std::vector<BLEService> simulate_gatt_service_discovery();
    std::vector<u8> simulate_characteristic_read(const std::string& char_uuid);
    bool simulate_characteristic_write(const std::string& char_uuid, const std::vector<u8>& value);
    
    // Advertisement data helpers
    std::vector<u8> build_advertisement_data(const std::string& name, const std::vector<std::string>& services);
    std::vector<u8> parse_advertisement_data(const std::vector<u8>& data, BLEDataType type);
    void add_advertisement_field(std::vector<u8>& data, BLEDataType type, const std::vector<u8>& value);
    
    // Utility functions
    u32 calculate_power_consumption();
    void update_statistics();
    bool is_address_valid(const BluetoothAddress& address);
    std::string uuid_to_string(const std::array<u8, 16>& uuid);
    std::array<u8, 16> string_to_uuid(const std::string& uuid_str);
    
    bool initialized_;
    BluetoothMode current_mode_;
    bool bluetooth_enabled_;
    
    InterruptController* interrupt_controller_;
    
    // Configuration
    std::string device_name_;
    BluetoothAddress local_address_;
    
    // Controller states
    ClassicBluetoothState classic_state_;
    BLEControllerState ble_state_;
    SecurityManager security_manager_;
    PowerManager power_manager_;
    PacketProcessor packet_processor_;
    
    // Callback interfaces
    BluetoothEventCallback event_callback_;
    BluetoothDataCallback data_callback_;
    BLEScanCallback ble_scan_callback_;
    BLECharacteristicCallback characteristic_callback_;
    
    // Background processing
    std::unique_ptr<std::thread> bluetooth_thread_;
    std::atomic<bool> thread_running_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    BluetoothStatistics statistics_;
    
    mutable std::mutex bluetooth_mutex_;
};

}  // namespace m5tab5::emulator