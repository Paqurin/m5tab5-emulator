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

namespace m5tab5::emulator {

enum class WiFiMode : u8 {
    OFF = 0,
    STATION = 1,
    ACCESS_POINT = 2,
    STATION_AP = 3,
    MONITOR = 4
};

enum class WiFiConnectionState : u8 {
    DISCONNECTED = 0,
    CONNECTING = 1,
    CONNECTED = 2,
    DISCONNECTING = 3,
    CONNECTION_FAILED = 4,
    LOST_CONNECTION = 5
};

enum class WiFiSecurityType : u8 {
    OPEN = 0,
    WEP = 1,
    WPA_PSK = 2,
    WPA2_PSK = 3,
    WPA_WPA2_PSK = 4,
    WPA2_ENTERPRISE = 5,
    WPA3_PSK = 6,
    WPA2_WPA3_PSK = 7
};

enum class WiFiPhyMode : u8 {
    B = 0,      // 802.11b
    G = 1,      // 802.11g
    N = 2,      // 802.11n (2.4GHz)
    LR = 3,     // Long Range
    AC = 4,     // 802.11ac (5GHz)
    AX = 5      // 802.11ax (Wi-Fi 6)
};

enum class WiFiPowerSaveMode : u8 {
    NONE = 0,
    MIN_MODEM = 1,
    MAX_MODEM = 2,
    LIGHT_SLEEP = 3,
    DEEP_SLEEP = 4
};

enum class WiFiEventType : u8 {
    READY = 0,
    SCAN_DONE = 1,
    STA_START = 2,
    STA_STOP = 3,
    STA_CONNECTED = 4,
    STA_DISCONNECTED = 5,
    STA_AUTHMODE_CHANGE = 6,
    STA_GOT_IP = 7,
    STA_LOST_IP = 8,
    AP_START = 9,
    AP_STOP = 10,
    AP_STACONNECTED = 11,
    AP_STADISCONNECTED = 12,
    AP_PROBEREQRECVED = 13,
    GOT_IP6 = 14,
    ETH_START = 15,
    ETH_STOP = 16,
    ETH_CONNECTED = 17,
    ETH_DISCONNECTED = 18,
    ETH_GOT_IP = 19
};

struct WiFiAccessPoint {
    std::string ssid;
    std::array<u8, 6> bssid;
    u8 channel;
    i8 rssi;
    WiFiSecurityType auth_mode;
    bool wps;
    bool is_hidden;
    u32 phy_11b : 1;
    u32 phy_11g : 1;
    u32 phy_11n : 1;
    u32 phy_lr : 1;
    u32 phy_11ac : 1;
    u32 phy_11ax : 1;
    u32 wps_disable : 1;
    u16 country_info;
};

struct WiFiStationConfig {
    std::string ssid;
    std::string password;
    std::array<u8, 6> bssid;
    u8 channel = 0;         // 0 = auto
    bool bssid_set = false;
    WiFiSecurityType threshold_authmode = WiFiSecurityType::WPA2_PSK;
    u8 pmf_cfg = 0;         // Protected Management Frames
    u32 sae_pwe_h2e = 0;    // SAE Password Element
    u32 sae_pk = 0;         // SAE-PK
    u8 failure_retry_cnt = 1;
    u32 he_dcm_set : 1;
    u32 he_dcm_max_constellation_tx : 2;
    u32 he_dcm_max_constellation_rx : 2;
    u32 he_mcs9_enabled : 1;
    u32 he_su_beamformee_disabled : 1;
    u32 he_trig_su_bmforming_feedback_disabled : 1;
    u32 he_trig_mu_bmforming_partial_feedback_disabled : 1;
    u32 he_trig_cqi_feedback_disabled : 1;
    u32 he_reserved : 21;
};

struct WiFiAPConfig {
    std::string ssid;
    std::string password;
    u8 ssid_len = 0;
    u8 channel = 1;
    WiFiSecurityType authmode = WiFiSecurityType::WPA2_PSK;
    u8 ssid_hidden = 0;
    u8 max_connection = 4;
    u16 beacon_interval = 100;
    u8 pairwise_cipher = 0;
    bool ftm_responder = false;
    WiFiPhyMode phy_mode = WiFiPhyMode::N;
    u8 pmf_cfg = 0;
    u32 sae_pwe_h2e = 0;
};

struct WiFiIPInfo {
    u32 ip;
    u32 netmask;
    u32 gateway;
};

struct WiFiEvent {
    WiFiEventType event_id;
    u32 event_info;
    std::chrono::steady_clock::time_point timestamp;
    std::array<u8, 64> event_data;
    size_t data_len;
};

struct WiFiStatistics {
    u64 packets_sent = 0;
    u64 packets_received = 0;
    u64 bytes_sent = 0;
    u64 bytes_received = 0;
    u64 scan_requests = 0;
    u64 connection_attempts = 0;
    u64 successful_connections = 0;
    u64 connection_failures = 0;
    u64 disconnections = 0;
    u64 beacons_sent = 0;
    u64 beacons_received = 0;
    u32 current_throughput_bps = 0;
    u32 peak_throughput_bps = 0;
    i8 current_rssi = 0;
    u8 current_channel = 0;
    float packet_loss_rate = 0.0f;
    float connection_uptime_hours = 0.0f;
};

struct WiFiPowerStats {
    u32 active_time_ms = 0;
    u32 sleep_time_ms = 0;
    u32 tx_time_ms = 0;
    u32 rx_time_ms = 0;
    float average_power_mw = 0.0f;
    float peak_power_mw = 0.0f;
    u32 wake_events = 0;
};

using WiFiEventCallback = std::function<void(const WiFiEvent& event)>;
using WiFiScanCallback = std::function<void(const std::vector<WiFiAccessPoint>& access_points)>;
using WiFiDataCallback = std::function<void(const std::vector<u8>& data, const std::string& source_ip)>;

class WiFiController {
public:
    static constexpr size_t MAX_SSID_LEN = 32;
    static constexpr size_t MAX_PASSWORD_LEN = 64;
    static constexpr size_t MAX_SCAN_RESULTS = 32;
    static constexpr size_t MAX_CONNECTED_STATIONS = 10;
    static constexpr size_t MAX_PACKET_SIZE = 1500;
    static constexpr u16 DEFAULT_PORT = 80;
    
    WiFiController();
    ~WiFiController();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller);
    Result<void> shutdown();

    // Mode and configuration
    Result<void> set_mode(WiFiMode mode);
    Result<WiFiMode> get_mode() const;
    Result<void> configure_station(const WiFiStationConfig& config);
    Result<void> configure_access_point(const WiFiAPConfig& config);
    Result<WiFiStationConfig> get_station_config() const;
    Result<WiFiAPConfig> get_access_point_config() const;
    
    // Network scanning
    Result<void> start_scan(bool show_hidden = false, bool passive = false, u32 max_scan_time_ms = 1500);
    Result<bool> is_scan_done() const;
    Result<std::vector<WiFiAccessPoint>> get_scan_results();
    Result<void> clear_scan_results();
    
    // Station mode operations
    Result<void> connect();
    Result<void> disconnect();
    Result<WiFiConnectionState> get_connection_state() const;
    Result<WiFiAccessPoint> get_current_ap_info() const;
    Result<i8> get_rssi() const;
    
    // Access Point mode operations
    Result<void> start_ap();
    Result<void> stop_ap();
    Result<std::vector<std::array<u8, 6>>> get_connected_stations() const;
    Result<void> disconnect_station(const std::array<u8, 6>& mac_address);
    
    // IP configuration
    Result<void> set_ip_info(const WiFiIPInfo& ip_info, bool dhcp_enable = false);
    Result<WiFiIPInfo> get_ip_info() const;
    Result<void> enable_dhcp(bool enable);
    Result<bool> is_dhcp_enabled() const;
    
    // Advanced networking
    Result<void> set_hostname(const std::string& hostname);
    Result<std::string> get_hostname() const;
    Result<void> set_dns_server(u32 primary_dns, u32 secondary_dns = 0);
    Result<std::pair<u32, u32>> get_dns_servers() const;
    
    // Power management
    Result<void> set_power_save_mode(WiFiPowerSaveMode mode);
    Result<WiFiPowerSaveMode> get_power_save_mode() const;
    Result<void> set_tx_power(i8 power_dbm);
    Result<i8> get_tx_power() const;
    Result<WiFiPowerStats> get_power_statistics() const;
    
    // PHY configuration
    Result<void> set_phy_mode(WiFiPhyMode mode);
    Result<WiFiPhyMode> get_phy_mode() const;
    Result<void> set_channel(u8 channel);
    Result<u8> get_channel() const;
    Result<void> set_bandwidth(u8 bandwidth); // 20MHz = 0, 40MHz = 1
    
    // Data transmission
    Result<void> send_raw_packet(const std::vector<u8>& packet, u8 channel = 0);
    Result<void> send_udp_packet(const std::vector<u8>& data, const std::string& dest_ip, u16 dest_port, u16 src_port = 0);
    Result<void> send_tcp_packet(const std::vector<u8>& data, const std::string& dest_ip, u16 dest_port);
    
    // Monitoring mode
    Result<void> enable_promiscuous_mode(bool enable);
    Result<bool> is_promiscuous_mode_enabled() const;
    Result<void> set_channel_hopping(bool enable, u32 hop_interval_ms = 500);
    
    // Security and encryption
    Result<void> set_wpa_config(const std::string& identity, const std::string& ca_cert = "");
    Result<void> enable_wps(bool enable);
    Result<void> start_wps_pbc(); // Push Button Configuration
    Result<void> start_wps_pin(const std::string& pin);
    
    // Event handling
    Result<void> set_event_callback(WiFiEventCallback callback);
    Result<void> set_scan_callback(WiFiScanCallback callback);
    Result<void> set_data_callback(WiFiDataCallback callback);
    
    // Diagnostics and debugging
    Result<std::vector<u8>> capture_packet_dump(size_t max_packets = 100);
    Result<void> enable_packet_logging(bool enable);
    Result<void> perform_spectrum_analysis(u8 start_channel = 1, u8 end_channel = 13);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    const WiFiStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct WiFiHardwareState {
        bool radio_enabled = false;
        bool antenna_connected = true;
        i8 tx_power_dbm = 20;
        u8 current_channel = 1;
        WiFiPhyMode phy_mode = WiFiPhyMode::N;
        u8 bandwidth = 0; // 20MHz
        bool cca_enabled = true; // Clear Channel Assessment
        u32 phy_rate_kbps = 150000; // Current PHY rate
        float noise_floor_dbm = -95.0f;
    };

    struct NetworkSimulation {
        std::vector<WiFiAccessPoint> simulated_aps;
        std::map<std::string, i8> rssi_map; // SSID -> RSSI
        std::map<std::string, std::chrono::steady_clock::time_point> last_beacon;
        bool internet_connectivity = true;
        u32 simulated_latency_ms = 50;
        float packet_loss_probability = 0.001f;
        u32 bandwidth_limit_kbps = 100000;
    };

    struct ConnectionManager {
        WiFiConnectionState state = WiFiConnectionState::DISCONNECTED;
        WiFiAccessPoint current_ap;
        std::chrono::steady_clock::time_point connection_start;
        std::chrono::steady_clock::time_point last_activity;
        u32 connection_timeout_ms = 10000;
        u32 keepalive_interval_ms = 30000;
        bool auto_reconnect = true;
        u8 retry_count = 0;
        u8 max_retries = 5;
    };

    struct AccessPointManager {
        bool ap_active = false;
        std::vector<std::array<u8, 6>> connected_stations;
        u32 beacon_interval_ms = 100;
        std::chrono::steady_clock::time_point last_beacon;
        u32 dhcp_lease_time_s = 3600;
        u32 next_ip_assignment = 0xC0A80102; // 192.168.1.2
    };

    struct PowerManager {
        WiFiPowerSaveMode mode = WiFiPowerSaveMode::NONE;
        bool sleep_enabled = false;
        std::chrono::steady_clock::time_point last_activity;
        u32 sleep_threshold_ms = 1000;
        float current_power_mw = 0.0f;
        std::chrono::steady_clock::time_point power_measurement_start;
    };

    struct PacketProcessor {
        std::queue<std::vector<u8>> tx_queue;
        std::queue<std::vector<u8>> rx_queue;
        bool promiscuous_mode = false;
        bool packet_logging = false;
        std::vector<std::vector<u8>> packet_dump;
        size_t max_dump_size = 1000;
        bool channel_hopping = false;
        u32 hop_interval_ms = 500;
        std::chrono::steady_clock::time_point last_hop;
        u8 hop_channel_index = 0;
    };

    void simulate_network_environment();
    void simulate_wifi_scanning();
    void simulate_connection_process();
    void simulate_access_point_operation();
    void simulate_data_transmission();
    void simulate_power_consumption();
    
    void process_tx_queue();
    void process_rx_queue();
    void generate_beacon_frames();
    void handle_dhcp_requests();
    void update_rssi_simulation();
    void simulate_interference();
    
    void trigger_wifi_event(WiFiEventType event_type, u32 event_info = 0, 
                           const u8* event_data = nullptr, size_t data_len = 0);
    void update_connection_state();
    void handle_station_management();
    void perform_channel_hopping();
    
    // Network packet generation
    std::vector<u8> generate_beacon_frame(const WiFiAPConfig& ap_config);
    std::vector<u8> generate_probe_response(const std::string& ssid);
    std::vector<u8> generate_association_response();
    std::vector<u8> generate_deauth_frame(const std::array<u8, 6>& mac);
    
    // Security simulation
    bool simulate_wpa_handshake(const std::string& password);
    bool validate_wep_key(const std::string& key);
    std::vector<u8> encrypt_packet(const std::vector<u8>& data, WiFiSecurityType security);
    std::vector<u8> decrypt_packet(const std::vector<u8>& data, WiFiSecurityType security);
    
    // Utility functions
    std::array<u8, 6> generate_random_mac();
    u32 string_to_ip(const std::string& ip_str);
    std::string ip_to_string(u32 ip);
    i8 calculate_rssi(const std::string& ssid, u8 channel);
    bool is_channel_valid(u8 channel);
    u32 calculate_throughput();
    void update_statistics();
    
    bool initialized_;
    WiFiMode current_mode_;
    
    InterruptController* interrupt_controller_;
    
    // Configuration
    WiFiStationConfig station_config_;
    WiFiAPConfig ap_config_;
    WiFiIPInfo ip_info_;
    bool dhcp_enabled_;
    std::string hostname_;
    std::pair<u32, u32> dns_servers_;
    
    // Hardware simulation
    WiFiHardwareState hardware_state_;
    NetworkSimulation network_simulation_;
    ConnectionManager connection_manager_;
    AccessPointManager ap_manager_;
    PowerManager power_manager_;
    PacketProcessor packet_processor_;
    
    // Scanning state
    bool scan_active_;
    std::chrono::steady_clock::time_point scan_start_;
    u32 scan_duration_ms_;
    bool scan_show_hidden_;
    bool scan_passive_;
    std::vector<WiFiAccessPoint> scan_results_;
    
    // Callback interfaces
    WiFiEventCallback event_callback_;
    WiFiScanCallback scan_callback_;
    WiFiDataCallback data_callback_;
    
    // Background processing
    std::unique_ptr<std::thread> wifi_thread_;
    std::atomic<bool> thread_running_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    WiFiStatistics statistics_;
    WiFiPowerStats power_stats_;
    
    mutable std::mutex wifi_mutex_;
};

}  // namespace m5tab5::emulator