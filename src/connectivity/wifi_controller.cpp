#include "emulator/connectivity/wifi_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace m5tab5::emulator {

WiFiController::WiFiController()
    : initialized_(false)
    , current_mode_(WiFiMode::OFF)
    , interrupt_controller_(nullptr)
    , dhcp_enabled_(true)
    , hostname_("esp32")
    , scan_active_(false)
    , scan_duration_ms_(1500)
    , scan_show_hidden_(false)
    , scan_passive_(false)
    , thread_running_(false) {
    
    // Initialize default configurations
    station_config_.ssid = "";
    station_config_.password = "";
    station_config_.channel = 0;
    station_config_.bssid_set = false;
    
    ap_config_.ssid = "ESP32-AP";
    ap_config_.password = "12345678";
    ap_config_.channel = 1;
    ap_config_.authmode = WiFiSecurityType::WPA2_PSK;
    ap_config_.max_connection = 4;
    
    // Initialize IP configuration
    ip_info_.ip = string_to_ip("192.168.1.100");
    ip_info_.netmask = string_to_ip("255.255.255.0");
    ip_info_.gateway = string_to_ip("192.168.1.1");
    
    dns_servers_ = {string_to_ip("8.8.8.8"), string_to_ip("8.8.4.4")};
    
    // Initialize hardware state
    hardware_state_.radio_enabled = false;
    hardware_state_.tx_power_dbm = 20;
    hardware_state_.current_channel = 1;
    hardware_state_.phy_mode = WiFiPhyMode::N;
    
    // Initialize network simulation with common APs
    network_simulation_.simulated_aps = {
        {"TestNetwork", {0x00, 0x11, 0x22, 0x33, 0x44, 0x55}, 1, -45, WiFiSecurityType::WPA2_PSK, false, false, 1, 1, 1, 0, 0, 0, 0, 0},
        {"OpenWiFi", {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 6, -60, WiFiSecurityType::OPEN, false, false, 1, 1, 1, 0, 0, 0, 0, 0},
        {"SecureAP", {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}, 11, -70, WiFiSecurityType::WPA3_PSK, false, false, 1, 1, 1, 0, 0, 1, 0, 0}
    };
    
    LOG_DEBUG("WiFiController", "WiFi controller initialized");
}

WiFiController::~WiFiController() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> WiFiController::initialize(const Configuration& config, 
                                       InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "WiFi controller already initialized");
    }
    
    if (!interrupt_controller) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Interrupt controller cannot be null");
    }
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize network simulation
    simulate_network_environment();
    
    // Start background processing thread
    thread_running_ = true;
    wifi_thread_ = std::make_unique<std::thread>([this]() {
        while (thread_running_) {
            {
                std::lock_guard<std::mutex> lock(wifi_mutex_);
                if (initialized_) {
                    simulate_wifi_scanning();
                    simulate_connection_process();
                    simulate_access_point_operation();
                    simulate_data_transmission();
                    simulate_power_consumption();
                    process_tx_queue();
                    process_rx_queue();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    last_update_ = std::chrono::steady_clock::now();
    
    initialized_ = true;
    
    LOG_INFO("WiFiController", "WiFi controller initialized successfully");
    trigger_wifi_event(WiFiEventType::READY);
    
    return {};
}

Result<void> WiFiController::shutdown() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Stop background thread
    thread_running_ = false;
    if (wifi_thread_ && wifi_thread_->joinable()) {
        wifi_mutex_.unlock();
        wifi_thread_->join();
        wifi_mutex_.lock();
    }
    wifi_thread_.reset();
    
    // Disconnect if connected
    if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
        connection_manager_.state = WiFiConnectionState::DISCONNECTED;
        trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
    }
    
    // Stop AP if running
    if (ap_manager_.ap_active) {
        ap_manager_.ap_active = false;
        trigger_wifi_event(WiFiEventType::AP_STOP);
    }
    
    // Clear queues and buffers
    while (!packet_processor_.tx_queue.empty()) {
        packet_processor_.tx_queue.pop();
    }
    while (!packet_processor_.rx_queue.empty()) {
        packet_processor_.rx_queue.pop();
    }
    scan_results_.clear();
    packet_processor_.packet_dump.clear();
    
    current_mode_ = WiFiMode::OFF;
    hardware_state_.radio_enabled = false;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    LOG_INFO("WiFiController", "WiFi controller shutdown complete");
    return {};
}

Result<void> WiFiController::set_mode(WiFiMode mode) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (current_mode_ == mode) {
        return {}; // Already in requested mode
    }
    
    // Handle mode transitions
    if (current_mode_ != WiFiMode::OFF) {
        // Disconnect/stop current mode operations
        if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
            connection_manager_.state = WiFiConnectionState::DISCONNECTED;
            trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
        }
        
        if (ap_manager_.ap_active) {
            ap_manager_.ap_active = false;
            trigger_wifi_event(WiFiEventType::AP_STOP);
        }
    }
    
    current_mode_ = mode;
    
    switch (mode) {
        case WiFiMode::OFF:
            hardware_state_.radio_enabled = false;
            break;
            
        case WiFiMode::STATION:
            hardware_state_.radio_enabled = true;
            trigger_wifi_event(WiFiEventType::STA_START);
            break;
            
        case WiFiMode::ACCESS_POINT:
            hardware_state_.radio_enabled = true;
            trigger_wifi_event(WiFiEventType::AP_START);
            break;
            
        case WiFiMode::STATION_AP:
            hardware_state_.radio_enabled = true;
            trigger_wifi_event(WiFiEventType::STA_START);
            trigger_wifi_event(WiFiEventType::AP_START);
            break;
            
        case WiFiMode::MONITOR:
            hardware_state_.radio_enabled = true;
            packet_processor_.promiscuous_mode = true;
            break;
    }
    
    LOG_INFO("WiFiController", "WiFi mode set to {}", static_cast<int>(mode));
    return {};
}

Result<void> WiFiController::configure_station(const WiFiStationConfig& config) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (config.ssid.length() > MAX_SSID_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "SSID too long");
    }
    
    if (config.password.length() > MAX_PASSWORD_LEN) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Password too long");
    }
    
    station_config_ = config;
    
    LOG_INFO("WiFiController", "Station configured for SSID: {}", config.ssid);
    return {};
}

Result<void> WiFiController::start_scan(bool show_hidden, bool passive, u32 max_scan_time_ms) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (!hardware_state_.radio_enabled) {
        return make_error(ErrorCode::DEVICE_ERROR, "WiFi radio not enabled");
    }
    
    if (scan_active_) {
        return make_error(ErrorCode::BUSY, "Scan already in progress");
    }
    
    scan_active_ = true;
    scan_start_ = std::chrono::steady_clock::now();
    scan_duration_ms_ = max_scan_time_ms;
    scan_show_hidden_ = show_hidden;
    scan_passive_ = passive;
    scan_results_.clear();
    
    statistics_.scan_requests++;
    
    LOG_INFO("WiFiController", "WiFi scan started (hidden: {}, passive: {}, timeout: {}ms)", 
             show_hidden, passive, max_scan_time_ms);
    
    return {};
}

Result<bool> WiFiController::is_scan_done() const {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    return !scan_active_;
}

Result<std::vector<WiFiAccessPoint>> WiFiController::get_scan_results() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    return scan_results_;
}

Result<void> WiFiController::connect() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (current_mode_ != WiFiMode::STATION && current_mode_ != WiFiMode::STATION_AP) {
        return make_error(ErrorCode::INVALID_STATE, "Not in station mode");
    }
    
    if (station_config_.ssid.empty()) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "No SSID configured");
    }
    
    if (connection_manager_.state == WiFiConnectionState::CONNECTED ||
        connection_manager_.state == WiFiConnectionState::CONNECTING) {
        return make_error(ErrorCode::BUSY, "Already connected or connecting");
    }
    
    connection_manager_.state = WiFiConnectionState::CONNECTING;
    connection_manager_.connection_start = std::chrono::steady_clock::now();
    connection_manager_.retry_count = 0;
    
    statistics_.connection_attempts++;
    
    LOG_INFO("WiFiController", "Connecting to SSID: {}", station_config_.ssid);
    
    return {};
}

Result<void> WiFiController::disconnect() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (connection_manager_.state == WiFiConnectionState::DISCONNECTED) {
        return {}; // Already disconnected
    }
    
    connection_manager_.state = WiFiConnectionState::DISCONNECTING;
    
    LOG_INFO("WiFiController", "Disconnecting from WiFi");
    
    return {};
}

Result<WiFiConnectionState> WiFiController::get_connection_state() const {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    return connection_manager_.state;
}

Result<void> WiFiController::start_ap() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (current_mode_ != WiFiMode::ACCESS_POINT && current_mode_ != WiFiMode::STATION_AP) {
        return make_error(ErrorCode::INVALID_STATE, "Not in access point mode");
    }
    
    if (ap_manager_.ap_active) {
        return {}; // Already active
    }
    
    ap_manager_.ap_active = true;
    ap_manager_.last_beacon = std::chrono::steady_clock::now();
    ap_manager_.connected_stations.clear();
    
    hardware_state_.current_channel = ap_config_.channel;
    
    LOG_INFO("WiFiController", "Access Point started: {} on channel {}", 
             ap_config_.ssid, ap_config_.channel);
    
    trigger_wifi_event(WiFiEventType::AP_START);
    
    return {};
}

Result<void> WiFiController::send_udp_packet(const std::vector<u8>& data, 
                                            const std::string& dest_ip, 
                                            u16 dest_port, 
                                            u16 src_port) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (connection_manager_.state != WiFiConnectionState::CONNECTED && !ap_manager_.ap_active) {
        return make_error(ErrorCode::NOT_CONNECTED, "Not connected to network");
    }
    
    if (data.size() > MAX_PACKET_SIZE) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Packet too large");
    }
    
    // Simulate UDP packet construction
    std::vector<u8> packet;
    packet.reserve(data.size() + 64); // Extra space for headers
    
    // Add simulated headers (simplified)
    packet.insert(packet.end(), data.begin(), data.end());
    
    packet_processor_.tx_queue.push(packet);
    
    statistics_.packets_sent++;
    statistics_.bytes_sent += data.size();
    
    LOG_DEBUG("WiFiController", "UDP packet queued: {} bytes to {}:{}", 
              data.size(), dest_ip, dest_port);
    
    return {};
}

Result<void> WiFiController::set_tx_power(i8 power_dbm) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    // ESP32-C6 power range: 2.5dBm to 20dBm
    if (power_dbm < 2 || power_dbm > 20) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "TX power out of range (2-20 dBm)");
    }
    
    hardware_state_.tx_power_dbm = power_dbm;
    
    // Update power consumption simulation
    power_manager_.current_power_mw = 100.0f + (power_dbm - 2) * 15.0f; // Approximate
    
    LOG_DEBUG("WiFiController", "TX power set to {} dBm", power_dbm);
    return {};
}

Result<i8> WiFiController::get_rssi() const {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "WiFi controller not initialized");
    }
    
    if (connection_manager_.state != WiFiConnectionState::CONNECTED) {
        return make_error(ErrorCode::NOT_CONNECTED, "Not connected");
    }
    
    return statistics_.current_rssi;
}

void WiFiController::update() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < 50) { // Update at 20Hz maximum
        return;
    }
    
    update_statistics();
    update_rssi_simulation();
    update_connection_state();
    
    // Update power statistics
    if (hardware_state_.radio_enabled) {
        power_stats_.active_time_ms += dt;
        if (connection_manager_.state == WiFiConnectionState::CONNECTED || ap_manager_.ap_active) {
            power_stats_.tx_time_ms += dt / 10; // Approximate TX activity
            power_stats_.rx_time_ms += dt / 8;  // Approximate RX activity
        }
    } else {
        power_stats_.sleep_time_ms += dt;
    }
    
    last_update_ = now;
}

void WiFiController::simulate_network_environment() {
    // Add some realistic interference simulation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> interference_dist(0.0f, 0.1f);
    
    network_simulation_.packet_loss_probability = interference_dist(gen);
    network_simulation_.simulated_latency_ms = 20 + static_cast<u32>(interference_dist(gen) * 100);
}

void WiFiController::simulate_wifi_scanning() {
    if (!scan_active_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto scan_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - scan_start_).count();
    
    if (scan_time >= scan_duration_ms_) {
        // Scan completed
        scan_active_ = false;
        
        // Copy simulated APs to scan results
        scan_results_ = network_simulation_.simulated_aps;
        
        // Add some realistic RSSI variation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> rssi_variation(0.0f, 3.0f);
        
        for (auto& ap : scan_results_) {
            ap.rssi += static_cast<i8>(rssi_variation(gen));
            ap.rssi = std::clamp(ap.rssi, i8(-100), i8(-20));
            
            // Update channel-based RSSI
            statistics_.current_rssi = calculate_rssi(ap.ssid, ap.channel);
        }
        
        // Filter hidden networks if not requested
        if (!scan_show_hidden_) {
            scan_results_.erase(
                std::remove_if(scan_results_.begin(), scan_results_.end(),
                              [](const WiFiAccessPoint& ap) { return ap.is_hidden; }),
                scan_results_.end());
        }
        
        LOG_INFO("WiFiController", "Scan completed, found {} access points", scan_results_.size());
        
        trigger_wifi_event(WiFiEventType::SCAN_DONE);
        
        if (scan_callback_) {
            scan_callback_(scan_results_);
        }
    }
}

void WiFiController::simulate_connection_process() {
    if (connection_manager_.state != WiFiConnectionState::CONNECTING) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto connect_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - connection_manager_.connection_start).count();
    
    if (connect_time >= connection_manager_.connection_timeout_ms) {
        // Connection timeout
        connection_manager_.state = WiFiConnectionState::CONNECTION_FAILED;
        connection_manager_.retry_count++;
        
        statistics_.connection_failures++;
        
        LOG_WARN("WiFiController", "Connection attempt failed for SSID: {}", station_config_.ssid);
        trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
        
        // Auto-retry if enabled
        if (connection_manager_.auto_reconnect && 
            connection_manager_.retry_count < connection_manager_.max_retries) {
            connection_manager_.state = WiFiConnectionState::CONNECTING;
            connection_manager_.connection_start = now;
            LOG_INFO("WiFiController", "Retrying connection (attempt {})", 
                     connection_manager_.retry_count + 1);
        } else {
            connection_manager_.state = WiFiConnectionState::DISCONNECTED;
        }
        
        return;
    }
    
    // Simulate connection stages
    if (connect_time > 1000 && connect_time < 2000) {
        // Authentication phase
        bool auth_success = simulate_wpa_handshake(station_config_.password);
        if (!auth_success) {
            connection_manager_.state = WiFiConnectionState::CONNECTION_FAILED;
            statistics_.connection_failures++;
            trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
            return;
        }
    } else if (connect_time > 2000 && connect_time < 3000) {
        // Association phase
        connection_manager_.state = WiFiConnectionState::CONNECTED;
        connection_manager_.last_activity = now;
        
        // Find the AP in scan results
        auto ap_it = std::find_if(network_simulation_.simulated_aps.begin(),
                                 network_simulation_.simulated_aps.end(),
                                 [this](const WiFiAccessPoint& ap) {
                                     return ap.ssid == station_config_.ssid;
                                 });
        
        if (ap_it != network_simulation_.simulated_aps.end()) {
            connection_manager_.current_ap = *ap_it;
            hardware_state_.current_channel = ap_it->channel;
        }
        
        statistics_.successful_connections++;
        statistics_.current_channel = hardware_state_.current_channel;
        
        LOG_INFO("WiFiController", "Connected to SSID: {} on channel {}", 
                 station_config_.ssid, hardware_state_.current_channel);
        
        trigger_wifi_event(WiFiEventType::STA_CONNECTED);
        
        // Simulate getting IP address
        if (dhcp_enabled_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            trigger_wifi_event(WiFiEventType::STA_GOT_IP);
        }
    }
}

void WiFiController::simulate_access_point_operation() {
    if (!ap_manager_.ap_active) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    // Send beacon frames
    auto beacon_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - ap_manager_.last_beacon).count();
    
    if (beacon_time >= ap_manager_.beacon_interval_ms) {
        generate_beacon_frames();
        ap_manager_.last_beacon = now;
        statistics_.beacons_sent++;
    }
    
    // Simulate station connections/disconnections
    static std::chrono::steady_clock::time_point last_station_event;
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_station_event).count() > 30) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> event_dist(0, 100);
        
        if (event_dist(gen) < 10 && ap_manager_.connected_stations.size() < ap_config_.max_connection) {
            // Simulate new station connection
            auto new_mac = generate_random_mac();
            ap_manager_.connected_stations.push_back(new_mac);
            
            LOG_INFO("WiFiController", "Station connected to AP: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                     new_mac[0], new_mac[1], new_mac[2], new_mac[3], new_mac[4], new_mac[5]);
            
            trigger_wifi_event(WiFiEventType::AP_STACONNECTED);
        } else if (event_dist(gen) < 5 && !ap_manager_.connected_stations.empty()) {
            // Simulate station disconnection
            auto disconnected_mac = ap_manager_.connected_stations.back();
            ap_manager_.connected_stations.pop_back();
            
            LOG_INFO("WiFiController", "Station disconnected from AP: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                     disconnected_mac[0], disconnected_mac[1], disconnected_mac[2], 
                     disconnected_mac[3], disconnected_mac[4], disconnected_mac[5]);
            
            trigger_wifi_event(WiFiEventType::AP_STADISCONNECTED);
        }
        
        last_station_event = now;
    }
}

void WiFiController::simulate_data_transmission() {
    // Simulate network throughput based on connection quality
    if (connection_manager_.state == WiFiConnectionState::CONNECTED || ap_manager_.ap_active) {
        i8 rssi = statistics_.current_rssi;
        u32 base_throughput = 0;
        
        // Calculate throughput based on PHY mode and RSSI
        switch (hardware_state_.phy_mode) {
            case WiFiPhyMode::B:  base_throughput = 11000;   break; // 11 Mbps
            case WiFiPhyMode::G:  base_throughput = 54000;   break; // 54 Mbps
            case WiFiPhyMode::N:  base_throughput = 150000;  break; // 150 Mbps
            case WiFiPhyMode::AC: base_throughput = 866000;  break; // 866 Mbps
            case WiFiPhyMode::AX: base_throughput = 1200000; break; // 1.2 Gbps
            default: base_throughput = 54000; break;
        }
        
        // Adjust for RSSI (signal quality)
        float rssi_factor = 1.0f;
        if (rssi < -80) rssi_factor = 0.1f;
        else if (rssi < -70) rssi_factor = 0.3f;
        else if (rssi < -60) rssi_factor = 0.6f;
        else if (rssi < -50) rssi_factor = 0.8f;
        else rssi_factor = 1.0f;
        
        statistics_.current_throughput_bps = static_cast<u32>(base_throughput * rssi_factor);
        statistics_.peak_throughput_bps = std::max(statistics_.peak_throughput_bps, 
                                                  statistics_.current_throughput_bps);
    }
}

void WiFiController::simulate_power_consumption() {
    float base_power = 0.0f;
    
    if (!hardware_state_.radio_enabled) {
        base_power = 5.0f; // Deep sleep power
    } else {
        switch (power_manager_.mode) {
            case WiFiPowerSaveMode::NONE:
                base_power = 160.0f; // Active mode
                break;
            case WiFiPowerSaveMode::MIN_MODEM:
                base_power = 120.0f; // Reduced modem power
                break;
            case WiFiPowerSaveMode::MAX_MODEM:
                base_power = 80.0f; // Maximum modem power saving
                break;
            case WiFiPowerSaveMode::LIGHT_SLEEP:
                base_power = 20.0f; // Light sleep
                break;
            case WiFiPowerSaveMode::DEEP_SLEEP:
                base_power = 5.0f; // Deep sleep
                break;
        }
        
        // Add TX power contribution
        base_power += (hardware_state_.tx_power_dbm - 2) * 3.0f;
        
        // Add throughput-based power
        if (statistics_.current_throughput_bps > 0) {
            base_power += statistics_.current_throughput_bps / 100000.0f * 20.0f;
        }
    }
    
    power_manager_.current_power_mw = base_power;
    power_stats_.average_power_mw = power_stats_.average_power_mw * 0.99f + base_power * 0.01f;
    power_stats_.peak_power_mw = std::max(power_stats_.peak_power_mw, base_power);
}

void WiFiController::process_tx_queue() {
    while (!packet_processor_.tx_queue.empty()) {
        auto packet = packet_processor_.tx_queue.front();
        packet_processor_.tx_queue.pop();
        
        // Simulate packet transmission delay
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> loss_dist(0.0f, 1.0f);
        
        if (loss_dist(gen) > network_simulation_.packet_loss_probability) {
            // Packet transmitted successfully
            statistics_.packets_sent++;
            statistics_.bytes_sent += packet.size();
            
            if (packet_processor_.packet_logging) {
                packet_processor_.packet_dump.push_back(packet);
                if (packet_processor_.packet_dump.size() > packet_processor_.max_dump_size) {
                    packet_processor_.packet_dump.erase(packet_processor_.packet_dump.begin());
                }
            }
        } else {
            // Packet lost
            statistics_.packet_loss_rate = statistics_.packet_loss_rate * 0.99f + 0.01f;
        }
    }
}

void WiFiController::process_rx_queue() {
    // Simulate receiving packets
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> packet_dist(0, 1000);
    
    if (packet_dist(gen) < 10 && (connection_manager_.state == WiFiConnectionState::CONNECTED || 
                                  ap_manager_.ap_active)) {
        // Generate random incoming packet
        std::uniform_int_distribution<> size_dist(64, 1500);
        size_t packet_size = size_dist(gen);
        
        std::vector<u8> packet(packet_size);
        std::uniform_int_distribution<u8> byte_dist(0, 255);
        for (auto& byte : packet) {
            byte = byte_dist(gen);
        }
        
        packet_processor_.rx_queue.push(packet);
        statistics_.packets_received++;
        statistics_.bytes_received += packet_size;
        
        if (data_callback_) {
            data_callback_(packet, "192.168.1.1");
        }
    }
}

void WiFiController::generate_beacon_frames() {
    if (!ap_manager_.ap_active) {
        return;
    }
    
    auto beacon = generate_beacon_frame(ap_config_);
    packet_processor_.tx_queue.push(beacon);
}

void WiFiController::trigger_wifi_event(WiFiEventType event_type, u32 event_info, 
                                       const u8* event_data, size_t data_len) {
    WiFiEvent event;
    event.event_id = event_type;
    event.event_info = event_info;
    event.timestamp = std::chrono::steady_clock::now();
    event.data_len = std::min(data_len, event.event_data.size());
    
    if (event_data && event.data_len > 0) {
        std::memcpy(event.event_data.data(), event_data, event.data_len);
    }
    
    if (event_callback_) {
        event_callback_(event);
    }
    
    // Trigger hardware interrupt for important events
    if (interrupt_controller_) {
        switch (event_type) {
            case WiFiEventType::STA_CONNECTED:
            case WiFiEventType::STA_DISCONNECTED:
            case WiFiEventType::STA_GOT_IP:
            case WiFiEventType::AP_STACONNECTED:
            case WiFiEventType::AP_STADISCONNECTED:
            case WiFiEventType::SCAN_DONE:
                interrupt_controller_->trigger_interrupt(50); // WiFi interrupt line
                break;
            default:
                break;
        }
    }
}

void WiFiController::update_connection_state() {
    if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
        auto now = std::chrono::steady_clock::now();
        auto idle_time = std::chrono::duration_cast<std::chrono::seconds>(
            now - connection_manager_.last_activity).count();
        
        // Simulate connection loss due to inactivity or poor signal
        if (idle_time > 300 || statistics_.current_rssi < -90) { // 5 minutes or very poor signal
            connection_manager_.state = WiFiConnectionState::LOST_CONNECTION;
            statistics_.disconnections++;
            
            LOG_WARN("WiFiController", "Connection lost to SSID: {}", station_config_.ssid);
            trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
            
            // Auto-reconnect if enabled
            if (connection_manager_.auto_reconnect) {
                connection_manager_.state = WiFiConnectionState::CONNECTING;
                connection_manager_.connection_start = now;
                connection_manager_.retry_count = 0;
            } else {
                connection_manager_.state = WiFiConnectionState::DISCONNECTED;
            }
        }
    } else if (connection_manager_.state == WiFiConnectionState::DISCONNECTING) {
        // Complete disconnection after brief delay
        connection_manager_.state = WiFiConnectionState::DISCONNECTED;
        statistics_.disconnections++;
        trigger_wifi_event(WiFiEventType::STA_DISCONNECTED);
    }
}

void WiFiController::update_rssi_simulation() {
    if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
        // Simulate RSSI fluctuation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> rssi_noise(0.0f, 2.0f);
        
        i8 base_rssi = connection_manager_.current_ap.rssi;
        statistics_.current_rssi = base_rssi + static_cast<i8>(rssi_noise(gen));
        statistics_.current_rssi = std::clamp(statistics_.current_rssi, i8(-100), i8(-20));
    }
}

bool WiFiController::simulate_wpa_handshake(const std::string& password) {
    // Simulate WPA/WPA2 handshake based on password
    if (password.empty()) {
        return false;
    }
    
    // Find target AP
    auto ap_it = std::find_if(network_simulation_.simulated_aps.begin(),
                             network_simulation_.simulated_aps.end(),
                             [this](const WiFiAccessPoint& ap) {
                                 return ap.ssid == station_config_.ssid;
                             });
    
    if (ap_it == network_simulation_.simulated_aps.end()) {
        return false;
    }
    
    // Simulate authentication based on security type
    switch (ap_it->auth_mode) {
        case WiFiSecurityType::OPEN:
            return true;
            
        case WiFiSecurityType::WEP:
            return validate_wep_key(password);
            
        case WiFiSecurityType::WPA_PSK:
        case WiFiSecurityType::WPA2_PSK:
        case WiFiSecurityType::WPA_WPA2_PSK:
            // Simple password validation (in real implementation, this would be much more complex)
            return password.length() >= 8;
            
        case WiFiSecurityType::WPA3_PSK:
        case WiFiSecurityType::WPA2_WPA3_PSK:
            return password.length() >= 8;
            
        default:
            return false;
    }
}

bool WiFiController::validate_wep_key(const std::string& key) {
    // WEP key validation (simplified)
    return key.length() == 10 || key.length() == 26; // 64-bit or 128-bit hex keys
}

std::vector<u8> WiFiController::generate_beacon_frame(const WiFiAPConfig& ap_config) {
    std::vector<u8> beacon;
    beacon.reserve(256);
    
    // Simplified beacon frame structure
    // Frame Control
    beacon.push_back(0x80); // Type: Beacon
    beacon.push_back(0x00);
    
    // Duration
    beacon.push_back(0x00);
    beacon.push_back(0x00);
    
    // Destination Address (broadcast)
    for (int i = 0; i < 6; ++i) {
        beacon.push_back(0xFF);
    }
    
    // Source Address (AP MAC)
    auto ap_mac = generate_random_mac();
    for (int i = 0; i < 6; ++i) {
        beacon.push_back(ap_mac[i]);
    }
    
    // BSSID (same as source)
    for (int i = 0; i < 6; ++i) {
        beacon.push_back(ap_mac[i]);
    }
    
    // Sequence Control
    beacon.push_back(0x00);
    beacon.push_back(0x00);
    
    // Fixed Parameters (timestamp, beacon interval, capability)
    for (int i = 0; i < 12; ++i) {
        beacon.push_back(0x00);
    }
    
    // SSID Element
    beacon.push_back(0x00); // Element ID
    beacon.push_back(static_cast<u8>(ap_config.ssid.length())); // Length
    for (char c : ap_config.ssid) {
        beacon.push_back(static_cast<u8>(c));
    }
    
    // Supported Rates Element
    beacon.push_back(0x01); // Element ID
    beacon.push_back(0x08); // Length
    u8 rates[] = {0x82, 0x84, 0x8B, 0x96, 0x12, 0x24, 0x48, 0x6C};
    for (u8 rate : rates) {
        beacon.push_back(rate);
    }
    
    // DS Parameter Set (channel)
    beacon.push_back(0x03); // Element ID
    beacon.push_back(0x01); // Length
    beacon.push_back(ap_config.channel);
    
    return beacon;
}

std::array<u8, 6> WiFiController::generate_random_mac() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<u8> byte_dist(0, 255);
    
    std::array<u8, 6> mac;
    for (auto& byte : mac) {
        byte = byte_dist(gen);
    }
    
    // Set locally administered bit
    mac[0] |= 0x02;
    mac[0] &= 0xFE; // Clear multicast bit
    
    return mac;
}

u32 WiFiController::string_to_ip(const std::string& ip_str) {
    std::istringstream iss(ip_str);
    std::string octet;
    u32 ip = 0;
    int shift = 24;
    
    while (std::getline(iss, octet, '.') && shift >= 0) {
        ip |= (std::stoi(octet) << shift);
        shift -= 8;
    }
    
    return ip;
}

std::string WiFiController::ip_to_string(u32 ip) {
    std::ostringstream oss;
    oss << ((ip >> 24) & 0xFF) << "."
        << ((ip >> 16) & 0xFF) << "."
        << ((ip >> 8) & 0xFF) << "."
        << (ip & 0xFF);
    return oss.str();
}

i8 WiFiController::calculate_rssi(const std::string& ssid, u8 channel) {
    // Simulate RSSI based on distance and interference
    std::hash<std::string> hasher;
    size_t hash = hasher(ssid);
    
    i8 base_rssi = -30 - (hash % 50); // -30 to -80 dBm range
    
    // Add channel-based interference
    i8 channel_penalty = std::abs(static_cast<i8>(channel - hardware_state_.current_channel)) * 2;
    
    return base_rssi - channel_penalty;
}

void WiFiController::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    
    if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
        auto uptime = std::chrono::duration<float, std::ratio<3600>>(
            now - connection_manager_.connection_start).count();
        statistics_.connection_uptime_hours = uptime;
    }
    
    // Update packet loss rate
    if (statistics_.packets_sent > 0) {
        statistics_.packet_loss_rate = 1.0f - (static_cast<float>(statistics_.packets_received) / 
                                              statistics_.packets_sent);
    }
}

void WiFiController::clear_statistics() {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    statistics_ = WiFiStatistics{};
    power_stats_ = WiFiPowerStats{};
}

void WiFiController::dump_status() const {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    
    LOG_INFO("WiFiController", "=== WiFi Controller Status ===");
    LOG_INFO("WiFiController", "Initialized: {}", initialized_);
    LOG_INFO("WiFiController", "Mode: {}", static_cast<int>(current_mode_));
    LOG_INFO("WiFiController", "Radio Enabled: {}", hardware_state_.radio_enabled);
    LOG_INFO("WiFiController", "Connection State: {}", static_cast<int>(connection_manager_.state));
    
    if (connection_manager_.state == WiFiConnectionState::CONNECTED) {
        LOG_INFO("WiFiController", "Connected SSID: {}", connection_manager_.current_ap.ssid);
        LOG_INFO("WiFiController", "Channel: {}", hardware_state_.current_channel);
        LOG_INFO("WiFiController", "RSSI: {} dBm", statistics_.current_rssi);
        LOG_INFO("WiFiController", "IP Address: {}", ip_to_string(ip_info_.ip));
    }
    
    if (ap_manager_.ap_active) {
        LOG_INFO("WiFiController", "AP SSID: {}", ap_config_.ssid);
        LOG_INFO("WiFiController", "AP Channel: {}", ap_config_.channel);
        LOG_INFO("WiFiController", "Connected Stations: {}", ap_manager_.connected_stations.size());
    }
    
    LOG_INFO("WiFiController", "TX Power: {} dBm", hardware_state_.tx_power_dbm);
    LOG_INFO("WiFiController", "PHY Mode: {}", static_cast<int>(hardware_state_.phy_mode));
    LOG_INFO("WiFiController", "Throughput: {} bps", statistics_.current_throughput_bps);
    LOG_INFO("WiFiController", "Packets Sent: {}", statistics_.packets_sent);
    LOG_INFO("WiFiController", "Packets Received: {}", statistics_.packets_received);
    LOG_INFO("WiFiController", "Packet Loss Rate: {:.2f}%", statistics_.packet_loss_rate * 100.0f);
    LOG_INFO("WiFiController", "Connection Attempts: {}", statistics_.connection_attempts);
    LOG_INFO("WiFiController", "Successful Connections: {}", statistics_.successful_connections);
    LOG_INFO("WiFiController", "Power Consumption: {:.1f} mW", power_manager_.current_power_mw);
    LOG_INFO("WiFiController", "Uptime: {:.2f} hours", statistics_.connection_uptime_hours);
}

}  // namespace m5tab5::emulator