#include <gtest/gtest.h>
#include "emulator/connectivity/wifi_controller.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <thread>
#include <chrono>

namespace emulator::connectivity::test {

class WiFiControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<memory::MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value());
        
        auto wifi_result = WiFiController::create(memory_controller);
        ASSERT_TRUE(wifi_result.has_value()) << "Failed to create WiFi controller";
        wifi_controller = std::move(wifi_result.value());
        
        ASSERT_TRUE(wifi_controller->initialize().has_value()) 
            << "Failed to initialize WiFi controller";
    }
    
    void TearDown() override {
        if (wifi_controller) {
            wifi_controller->disconnect();
            wifi_controller->shutdown();
        }
    }
    
    std::shared_ptr<memory::MemoryController> memory_controller;
    std::unique_ptr<WiFiController> wifi_controller;
};

TEST_F(WiFiControllerTest, BasicInitialization) {
    auto status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status.value().state, WiFiController::State::IDLE);
    EXPECT_FALSE(status.value().connected);
}

TEST_F(WiFiControllerTest, APModeConfiguration) {
    WiFiController::APConfig config;
    config.ssid = "TestAP";
    config.password = "testpass123";
    config.channel = 6;
    config.max_connections = 4;
    config.security = WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_ap(config).has_value());
    EXPECT_TRUE(wifi_controller->start_ap().has_value());
    
    auto status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status.value().state, WiFiController::State::AP_STARTED);
    
    EXPECT_TRUE(wifi_controller->stop_ap().has_value());
}

TEST_F(WiFiControllerTest, StationModeConfiguration) {
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "password123";
    config.security = WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    
    auto stored_config = wifi_controller->get_station_config();
    ASSERT_TRUE(stored_config.has_value());
    EXPECT_EQ(stored_config.value().ssid, "TestNetwork");
    EXPECT_EQ(stored_config.value().security, WiFiController::SecurityType::WPA2);
}

TEST_F(WiFiControllerTest, NetworkScanning) {
    wifi_controller->add_simulated_network("TestAP1", -45, WiFiController::SecurityType::WPA2);
    wifi_controller->add_simulated_network("TestAP2", -60, WiFiController::SecurityType::WPA3);
    wifi_controller->add_simulated_network("OpenAP", -75, WiFiController::SecurityType::NONE);
    
    auto scan_result = wifi_controller->scan_networks();
    ASSERT_TRUE(scan_result.has_value()) << "Network scan failed";
    
    auto networks = scan_result.value();
    EXPECT_GE(networks.size(), 3);
    
    bool found_test_ap1 = false;
    for (const auto& network : networks) {
        if (network.ssid == "TestAP1") {
            found_test_ap1 = true;
            EXPECT_EQ(network.rssi, -45);
            EXPECT_EQ(network.security, WiFiController::SecurityType::WPA2);
        }
    }
    EXPECT_TRUE(found_test_ap1);
}

TEST_F(WiFiControllerTest, ConnectionProcess) {
    wifi_controller->add_simulated_network("TestNetwork", -50, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "correctpassword";
    config.security = WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    
    auto connect_result = wifi_controller->connect();
    ASSERT_TRUE(connect_result.has_value()) << "Connection failed";
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(status.value().connected);
    EXPECT_EQ(status.value().state, WiFiController::State::CONNECTED);
}

TEST_F(WiFiControllerTest, ConnectionFailure) {
    wifi_controller->add_simulated_network("TestNetwork", -50, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "wrongpassword";
    config.security = WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    
    auto connect_result = wifi_controller->connect();
    EXPECT_FALSE(connect_result.has_value());
    EXPECT_EQ(connect_result.error(), utils::ErrorCode::AUTHENTICATION_FAILED);
}

TEST_F(WiFiControllerTest, IPAddressAssignment) {
    wifi_controller->add_simulated_network("TestNetwork", -50, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "correctpassword";
    config.security = WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto ip_info = wifi_controller->get_ip_info();
    ASSERT_TRUE(ip_info.has_value());
    
    EXPECT_FALSE(ip_info.value().ip_address.empty());
    EXPECT_FALSE(ip_info.value().subnet_mask.empty());
    EXPECT_FALSE(ip_info.value().gateway.empty());
}

TEST_F(WiFiControllerTest, SignalStrengthMonitoring) {
    wifi_controller->add_simulated_network("TestNetwork", -50, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "correctpassword";
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto rssi = wifi_controller->get_rssi();
    ASSERT_TRUE(rssi.has_value());
    EXPECT_LE(rssi.value(), -30);
    EXPECT_GE(rssi.value(), -90);
}

TEST_F(WiFiControllerTest, PowerManagement) {
    auto power_mode = wifi_controller->get_power_mode();
    ASSERT_TRUE(power_mode.has_value());
    EXPECT_EQ(power_mode.value(), WiFiController::PowerMode::ACTIVE);
    
    EXPECT_TRUE(wifi_controller->set_power_mode(WiFiController::PowerMode::POWER_SAVE).has_value());
    
    power_mode = wifi_controller->get_power_mode();
    ASSERT_TRUE(power_mode.has_value());
    EXPECT_EQ(power_mode.value(), WiFiController::PowerMode::POWER_SAVE);
}

TEST_F(WiFiControllerTest, EventCallbacks) {
    bool connected_event_received = false;
    bool disconnected_event_received = false;
    
    auto connect_callback = [&connected_event_received]() {
        connected_event_received = true;
    };
    
    auto disconnect_callback = [&disconnected_event_received]() {
        disconnected_event_received = true;
    };
    
    EXPECT_TRUE(wifi_controller->set_connect_callback(connect_callback).has_value());
    EXPECT_TRUE(wifi_controller->set_disconnect_callback(disconnect_callback).has_value());
    
    wifi_controller->add_simulated_network("TestNetwork", -50, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "correctpassword";
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_TRUE(connected_event_received);
    
    EXPECT_TRUE(wifi_controller->disconnect().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(disconnected_event_received);
}

TEST_F(WiFiControllerTest, WPS_Support) {
    wifi_controller->add_simulated_network("WPSNetwork", -55, WiFiController::SecurityType::WPA2, true);
    
    auto wps_result = wifi_controller->start_wps(WiFiController::WPSMode::PBC);
    ASSERT_TRUE(wps_result.has_value()) << "WPS start failed";
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(status.value().connected);
    
    auto config = wifi_controller->get_station_config();
    ASSERT_TRUE(config.has_value());
    EXPECT_EQ(config.value().ssid, "WPSNetwork");
}

TEST_F(WiFiControllerTest, MACAddressHandling) {
    auto mac_address = wifi_controller->get_mac_address();
    ASSERT_TRUE(mac_address.has_value());
    
    EXPECT_EQ(mac_address.value().length(), 17);
    EXPECT_EQ(mac_address.value().substr(2, 1), ":");
    EXPECT_EQ(mac_address.value().substr(5, 1), ":");
    
    std::string new_mac = "AA:BB:CC:DD:EE:FF";
    EXPECT_TRUE(wifi_controller->set_mac_address(new_mac).has_value());
    
    auto updated_mac = wifi_controller->get_mac_address();
    ASSERT_TRUE(updated_mac.has_value());
    EXPECT_EQ(updated_mac.value(), new_mac);
}

TEST_F(WiFiControllerTest, SecurityProtocols) {
    std::vector<WiFiController::SecurityType> security_types = {
        WiFiController::SecurityType::NONE,
        WiFiController::SecurityType::WEP,
        WiFiController::SecurityType::WPA,
        WiFiController::SecurityType::WPA2,
        WiFiController::SecurityType::WPA3
    };
    
    for (auto security : security_types) {
        std::string ssid = "TestNet_" + std::to_string(static_cast<int>(security));
        wifi_controller->add_simulated_network(ssid, -60, security);
        
        WiFiController::StationConfig config;
        config.ssid = ssid;
        config.password = (security == WiFiController::SecurityType::NONE) ? "" : "testpass123";
        config.security = security;
        
        EXPECT_TRUE(wifi_controller->configure_station(config).has_value())
            << "Failed to configure for security type: " << static_cast<int>(security);
        
        if (security != WiFiController::SecurityType::WEP) {
            EXPECT_TRUE(wifi_controller->connect().has_value())
                << "Failed to connect with security type: " << static_cast<int>(security);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            EXPECT_TRUE(wifi_controller->disconnect().has_value());
        }
    }
}

TEST_F(WiFiControllerTest, PerformanceMetrics) {
    wifi_controller->add_simulated_network("PerfTest", -40, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "PerfTest";
    config.password = "testpass";
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto throughput = wifi_controller->measure_throughput();
    ASSERT_TRUE(throughput.has_value());
    EXPECT_GT(throughput.value().download_mbps, 0.0f);
    EXPECT_GT(throughput.value().upload_mbps, 0.0f);
    
    auto latency = wifi_controller->measure_latency("8.8.8.8");
    ASSERT_TRUE(latency.has_value());
    EXPECT_GT(latency.value(), 0);
}

TEST_F(WiFiControllerTest, ConcurrentOperations) {
    const int num_threads = 3;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            std::string ssid = "TestNet_" + std::to_string(i);
            wifi_controller->add_simulated_network(ssid, -50 - i * 5, WiFiController::SecurityType::WPA2);
            
            auto scan_result = wifi_controller->scan_networks();
            if (scan_result.has_value()) {
                for (const auto& network : scan_result.value()) {
                    if (network.ssid == ssid) {
                        success_count++;
                        break;
                    }
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    EXPECT_EQ(success_count.load(), num_threads);
}

TEST_F(WiFiControllerTest, ErrorRecovery) {
    wifi_controller->add_simulated_network("TestNetwork", -80, WiFiController::SecurityType::WPA2);
    
    WiFiController::StationConfig config;
    config.ssid = "TestNetwork";
    config.password = "testpass";
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    wifi_controller->simulate_connection_drop();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_FALSE(status.value().connected);
    
    EXPECT_TRUE(wifi_controller->connect().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    status = wifi_controller->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(status.value().connected);
}

} // namespace emulator::connectivity::test