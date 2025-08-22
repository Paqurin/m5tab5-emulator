#include <gtest/gtest.h>
#include "emulator/core/emulator_core.hpp"
#include "emulator/connectivity/wifi_controller.hpp"
#include "emulator/connectivity/bluetooth_controller.hpp"
#include "emulator/connectivity/usb_controller.hpp"
#include "emulator/connectivity/rs485_controller.hpp"
#include "emulator/peripherals/uart_controller.hpp"
#include "emulator/config/configuration.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <random>

namespace emulator::integration::test {

class ConnectivityStackTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::string config_content = R"({
            "emulator": {
                "enable_debugging": true,
                "log_level": "info"
            },
            "cpu": {
                "core_count": 2,
                "frequency_mhz": 240
            },
            "connectivity": {
                "wifi": {
                    "enabled": true,
                    "default_power_mode": "active",
                    "max_scan_results": 50
                },
                "bluetooth": {
                    "enabled": true,
                    "classic_enabled": true,
                    "ble_enabled": true,
                    "max_connections": 7
                },
                "usb": {
                    "enabled": true,
                    "host_enabled": true,
                    "device_enabled": true
                },
                "rs485": {
                    "enabled": true,
                    "default_baud_rate": 115200
                }
            }
        })";
        
        config_path = std::filesystem::temp_directory_path() / "connectivity_config.json";
        std::ofstream(config_path) << config_content;
        
        auto config_result = config::Configuration::load(config_path.string());
        ASSERT_TRUE(config_result.has_value());
        configuration = config_result.value();
        
        auto core_result = EmulatorCore::create(configuration);
        ASSERT_TRUE(core_result.has_value());
        emulator_core = std::move(core_result.value());
        
        ASSERT_TRUE(emulator_core->initialize().has_value());
        ASSERT_TRUE(emulator_core->start().has_value());
    }
    
    void TearDown() override {
        if (emulator_core) {
            emulator_core->stop();
            emulator_core->shutdown();
        }
        if (std::filesystem::exists(config_path)) {
            std::filesystem::remove(config_path);
        }
    }
    
    std::filesystem::path config_path;
    std::shared_ptr<config::Configuration> configuration;
    std::unique_ptr<EmulatorCore> emulator_core;
};

TEST_F(ConnectivityStackTest, WiFiBluetoothCoexistence) {
    auto wifi_controller = emulator_core->get_wifi_controller();
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    
    ASSERT_TRUE(wifi_controller != nullptr);
    ASSERT_TRUE(bluetooth_controller != nullptr);
    
    ASSERT_TRUE(bluetooth_controller->initialize().has_value());
    EXPECT_TRUE(bluetooth_controller->enable().has_value());
    
    wifi_controller->add_simulated_network("TestNetwork", -50, connectivity::WiFiController::SecurityType::WPA2);
    
    connectivity::WiFiController::StationConfig wifi_config;
    wifi_config.ssid = "TestNetwork";
    wifi_config.password = "testpass123";
    wifi_config.security = connectivity::WiFiController::SecurityType::WPA2;
    
    EXPECT_TRUE(wifi_controller->configure_station(wifi_config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto wifi_status = wifi_controller->get_status();
    ASSERT_TRUE(wifi_status.has_value());
    EXPECT_TRUE(wifi_status.value().connected);
    
    bluetooth_controller->add_simulated_device("TestPhone", "AA:BB:CC:DD:EE:FF", 
        connectivity::BluetoothController::DeviceClass::SMARTPHONE);
    
    auto bt_scan_result = bluetooth_controller->start_discovery();
    EXPECT_TRUE(bt_scan_result.has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto bt_devices = bluetooth_controller->get_discovered_devices();
    ASSERT_TRUE(bt_devices.has_value());
    EXPECT_FALSE(bt_devices.value().empty());
    
    EXPECT_TRUE(bluetooth_controller->stop_discovery().has_value());
    
    auto wifi_rssi = wifi_controller->get_rssi();
    ASSERT_TRUE(wifi_rssi.has_value());
    EXPECT_LE(wifi_rssi.value(), -30);  // Should still have good signal
    
    auto bt_status = bluetooth_controller->get_status();
    ASSERT_TRUE(bt_status.has_value());
    EXPECT_TRUE(bt_status.value().enabled);
    EXPECT_FALSE(bt_status.value().discovering);
}

TEST_F(ConnectivityStackTest, USBDeviceEnumeration) {
    auto usb_controller = emulator_core->get_usb_controller();
    
    ASSERT_TRUE(usb_controller != nullptr);
    ASSERT_TRUE(usb_controller->initialize().has_value());
    
    EXPECT_TRUE(usb_controller->set_mode(connectivity::USBController::Mode::HOST).has_value());
    
    usb_controller->simulate_device_connection(connectivity::USBController::DeviceClass::HID, 0x1234, 0x5678);
    usb_controller->simulate_device_connection(connectivity::USBController::DeviceClass::MASS_STORAGE, 0xABCD, 0xEF01);
    usb_controller->simulate_device_connection(connectivity::USBController::DeviceClass::CDC_ACM, 0x2341, 0x0043);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto connected_devices = usb_controller->get_connected_devices();
    ASSERT_TRUE(connected_devices.has_value());
    EXPECT_EQ(connected_devices.value().size(), 3);
    
    bool found_hid = false, found_storage = false, found_cdc = false;
    
    for (const auto& device : connected_devices.value()) {
        if (device.device_class == connectivity::USBController::DeviceClass::HID) {
            found_hid = true;
            EXPECT_EQ(device.vendor_id, 0x1234);
            EXPECT_EQ(device.product_id, 0x5678);
        } else if (device.device_class == connectivity::USBController::DeviceClass::MASS_STORAGE) {
            found_storage = true;
            EXPECT_EQ(device.vendor_id, 0xABCD);
            EXPECT_EQ(device.product_id, 0xEF01);
        } else if (device.device_class == connectivity::USBController::DeviceClass::CDC_ACM) {
            found_cdc = true;
            EXPECT_EQ(device.vendor_id, 0x2341);
            EXPECT_EQ(device.product_id, 0x0043);
        }
    }
    
    EXPECT_TRUE(found_hid && found_storage && found_cdc);
    
    usb_controller->simulate_device_disconnection(0x1234, 0x5678);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto devices_after_disconnect = usb_controller->get_connected_devices();
    ASSERT_TRUE(devices_after_disconnect.has_value());
    EXPECT_EQ(devices_after_disconnect.value().size(), 2);
}

TEST_F(ConnectivityStackTest, RS485ModbusCommunication) {
    auto rs485_controller = emulator_core->get_rs485_controller();
    
    ASSERT_TRUE(rs485_controller != nullptr);
    ASSERT_TRUE(rs485_controller->initialize().has_value());
    
    connectivity::RS485Controller::Configuration config;
    config.baud_rate = 115200;
    config.data_bits = 8;
    config.parity = connectivity::RS485Controller::Parity::NONE;
    config.stop_bits = 1;
    config.protocol = connectivity::RS485Controller::Protocol::MODBUS_RTU;
    
    EXPECT_TRUE(rs485_controller->configure(config).has_value());
    
    rs485_controller->add_virtual_device(0x01, connectivity::RS485Controller::DeviceType::SENSOR);
    rs485_controller->add_virtual_device(0x02, connectivity::RS485Controller::DeviceType::ACTUATOR);
    rs485_controller->add_virtual_device(0x03, connectivity::RS485Controller::DeviceType::CONTROLLER);
    
    std::vector<uint8_t> modbus_read_cmd = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    
    auto write_result = rs485_controller->write_frame(modbus_read_cmd);
    ASSERT_TRUE(write_result.has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto read_result = rs485_controller->read_frame();
    ASSERT_TRUE(read_result.has_value());
    
    auto& response = read_result.value();
    EXPECT_GE(response.size(), 7);  // Minimum Modbus response size
    EXPECT_EQ(response[0], 0x01);   // Device address
    EXPECT_EQ(response[1], 0x03);   // Function code
    
    std::vector<uint8_t> modbus_write_cmd = {0x02, 0x06, 0x00, 0x01, 0x00, 0xFF, 0x59, 0xFA};
    
    auto write_cmd_result = rs485_controller->write_frame(modbus_write_cmd);
    ASSERT_TRUE(write_cmd_result.has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto write_response = rs485_controller->read_frame();
    ASSERT_TRUE(write_response.has_value());
    EXPECT_EQ(write_response.value().size(), modbus_write_cmd.size());
    EXPECT_EQ(write_response.value()[0], 0x02);
}

TEST_F(ConnectivityStackTest, MultiProtocolDataBridge) {
    auto wifi_controller = emulator_core->get_wifi_controller();
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    auto rs485_controller = emulator_core->get_rs485_controller();
    auto uart_controller = emulator_core->get_uart_controller(0);
    
    ASSERT_TRUE(wifi_controller != nullptr);
    ASSERT_TRUE(bluetooth_controller != nullptr);
    ASSERT_TRUE(rs485_controller != nullptr);
    ASSERT_TRUE(uart_controller != nullptr);
    
    wifi_controller->add_simulated_network("BridgeNet", -45, connectivity::WiFiController::SecurityType::WPA2);
    
    connectivity::WiFiController::StationConfig wifi_config;
    wifi_config.ssid = "BridgeNet";
    wifi_config.password = "bridge123";
    
    EXPECT_TRUE(wifi_controller->configure_station(wifi_config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    ASSERT_TRUE(bluetooth_controller->initialize().has_value());
    EXPECT_TRUE(bluetooth_controller->enable().has_value());
    
    connectivity::RS485Controller::Configuration rs485_config;
    rs485_config.baud_rate = 9600;
    rs485_config.protocol = connectivity::RS485Controller::Protocol::MODBUS_RTU;
    EXPECT_TRUE(rs485_controller->configure(rs485_config).has_value());
    
    EXPECT_TRUE(uart_controller->configure(115200, 8, peripherals::UARTController::Parity::NONE, 1).has_value());
    
    struct BridgePacket {
        uint8_t protocol_id;  // 1=WiFi, 2=BT, 3=RS485, 4=UART
        uint16_t data_length;
        std::vector<uint8_t> payload;
        uint16_t checksum;
    };
    
    std::vector<BridgePacket> processed_packets;
    std::mutex packet_mutex;
    
    auto process_bridge_data = [&]() {
        for (int i = 0; i < 20; ++i) {
            BridgePacket packet = {};
            
            switch (i % 4) {
                case 0: {  // WiFi to RS485 bridge
                    packet.protocol_id = 1;
                    packet.payload = {0x01, 0x03, 0x00, static_cast<uint8_t>(i), 0x00, 0x01};
                    packet.data_length = packet.payload.size();
                    
                    std::vector<uint8_t> rs485_frame = packet.payload;
                    if (rs485_controller->write_frame(rs485_frame).has_value()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        auto response = rs485_controller->read_frame();
                        if (response.has_value()) {
                            packet.payload = response.value();
                            packet.data_length = packet.payload.size();
                        }
                    }
                    break;
                }
                
                case 1: {  // Bluetooth to UART bridge
                    packet.protocol_id = 2;
                    packet.payload = {0xAA, 0x55, static_cast<uint8_t>(i), 0x00};
                    packet.data_length = packet.payload.size();
                    
                    if (uart_controller->write_bulk(packet.payload).has_value()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        auto uart_data = uart_controller->read_available();
                        if (uart_data.has_value() && !uart_data.value().empty()) {
                            packet.payload = uart_data.value();
                            packet.data_length = packet.payload.size();
                        }
                    }
                    break;
                }
                
                case 2: {  // RS485 to WiFi bridge
                    packet.protocol_id = 3;
                    std::vector<uint8_t> sensor_cmd = {0x02, 0x04, 0x00, static_cast<uint8_t>(i), 0x00, 0x01};
                    packet.payload = sensor_cmd;
                    packet.data_length = packet.payload.size();
                    
                    if (rs485_controller->write_frame(sensor_cmd).has_value()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        auto rs485_response = rs485_controller->read_frame();
                        if (rs485_response.has_value()) {
                            packet.payload = rs485_response.value();
                            packet.data_length = packet.payload.size();
                        }
                    }
                    break;
                }
                
                case 3: {  // UART to Bluetooth bridge
                    packet.protocol_id = 4;
                    std::vector<uint8_t> uart_cmd = {0xCC, 0xDD, static_cast<uint8_t>(i)};
                    packet.payload = uart_cmd;
                    packet.data_length = packet.payload.size();
                    
                    uart_controller->write_bulk(uart_cmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    break;
                }
            }
            
            packet.checksum = 0;
            for (uint8_t byte : packet.payload) {
                packet.checksum += byte;
            }
            packet.checksum ^= packet.protocol_id ^ packet.data_length;
            
            {
                std::lock_guard<std::mutex> lock(packet_mutex);
                processed_packets.push_back(packet);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    };
    
    std::thread bridge_thread(process_bridge_data);
    bridge_thread.join();
    
    std::lock_guard<std::mutex> lock(packet_mutex);
    EXPECT_EQ(processed_packets.size(), 20);
    
    std::array<int, 5> protocol_counts = {0};
    for (const auto& packet : processed_packets) {
        EXPECT_GE(packet.protocol_id, 1);
        EXPECT_LE(packet.protocol_id, 4);
        EXPECT_GT(packet.data_length, 0);
        EXPECT_FALSE(packet.payload.empty());
        
        protocol_counts[packet.protocol_id]++;
    }
    
    for (int i = 1; i <= 4; ++i) {
        EXPECT_EQ(protocol_counts[i], 5);
    }
}

TEST_F(ConnectivityStackTest, ConcurrentConnectivityOperations) {
    const int num_operations = 100;
    
    auto wifi_controller = emulator_core->get_wifi_controller();
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    auto usb_controller = emulator_core->get_usb_controller();
    auto rs485_controller = emulator_core->get_rs485_controller();
    
    ASSERT_TRUE(wifi_controller != nullptr);
    ASSERT_TRUE(bluetooth_controller != nullptr);
    ASSERT_TRUE(usb_controller != nullptr);
    ASSERT_TRUE(rs485_controller != nullptr);
    
    wifi_controller->add_simulated_network("ConcurrentNet", -50, connectivity::WiFiController::SecurityType::WPA2);
    ASSERT_TRUE(bluetooth_controller->initialize().has_value());
    ASSERT_TRUE(usb_controller->initialize().has_value());
    ASSERT_TRUE(rs485_controller->initialize().has_value());
    
    std::atomic<int> wifi_operations{0};
    std::atomic<int> bt_operations{0};
    std::atomic<int> usb_operations{0};
    std::atomic<int> rs485_operations{0};
    
    std::vector<std::thread> threads;
    
    threads.emplace_back([&]() {
        connectivity::WiFiController::StationConfig config;
        config.ssid = "ConcurrentNet";
        config.password = "concurrent123";
        
        for (int i = 0; i < num_operations / 4; ++i) {
            if (wifi_controller->configure_station(config).has_value()) {
                if (wifi_controller->connect().has_value()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    wifi_controller->disconnect();
                    wifi_operations++;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    
    threads.emplace_back([&]() {
        bluetooth_controller->enable();
        
        for (int i = 0; i < num_operations / 4; ++i) {
            if (bluetooth_controller->start_discovery().has_value()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                bluetooth_controller->stop_discovery();
                bt_operations++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    
    threads.emplace_back([&]() {
        usb_controller->set_mode(connectivity::USBController::Mode::HOST);
        
        for (int i = 0; i < num_operations / 4; ++i) {
            usb_controller->simulate_device_connection(
                connectivity::USBController::DeviceClass::HID, 
                0x1000 + i, 0x2000 + i
            );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            usb_controller->simulate_device_disconnection(0x1000 + i, 0x2000 + i);
            usb_operations++;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    
    threads.emplace_back([&]() {
        connectivity::RS485Controller::Configuration config;
        config.baud_rate = 115200;
        config.protocol = connectivity::RS485Controller::Protocol::MODBUS_RTU;
        rs485_controller->configure(config);
        
        for (int i = 0; i < num_operations / 4; ++i) {
            std::vector<uint8_t> frame = {0x01, 0x03, 0x00, static_cast<uint8_t>(i % 256), 0x00, 0x01};
            if (rs485_controller->write_frame(frame).has_value()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                auto response = rs485_controller->read_frame();
                if (response.has_value()) {
                    rs485_operations++;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    std::cout << "Concurrent Connectivity Operations:" << std::endl;
    std::cout << "  WiFi operations: " << wifi_operations.load() << std::endl;
    std::cout << "  Bluetooth operations: " << bt_operations.load() << std::endl;
    std::cout << "  USB operations: " << usb_operations.load() << std::endl;
    std::cout << "  RS485 operations: " << rs485_operations.load() << std::endl;
    
    int total_successful = wifi_operations.load() + bt_operations.load() + 
                          usb_operations.load() + rs485_operations.load();
    
    EXPECT_GT(total_successful, num_operations * 0.8);  // Expect >80% success rate
    EXPECT_GT(wifi_operations.load(), 0);
    EXPECT_GT(bt_operations.load(), 0);
    EXPECT_GT(usb_operations.load(), 0);
    EXPECT_GT(rs485_operations.load(), 0);
}

TEST_F(ConnectivityStackTest, NetworkFailoverAndRecovery) {
    auto wifi_controller = emulator_core->get_wifi_controller();
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    
    ASSERT_TRUE(wifi_controller != nullptr);
    ASSERT_TRUE(bluetooth_controller != nullptr);
    
    wifi_controller->add_simulated_network("PrimaryNet", -40, connectivity::WiFiController::SecurityType::WPA2);
    wifi_controller->add_simulated_network("BackupNet", -60, connectivity::WiFiController::SecurityType::WPA2);
    
    ASSERT_TRUE(bluetooth_controller->initialize().has_value());
    EXPECT_TRUE(bluetooth_controller->enable().has_value());
    
    connectivity::WiFiController::StationConfig primary_config;
    primary_config.ssid = "PrimaryNet";
    primary_config.password = "primary123";
    
    connectivity::WiFiController::StationConfig backup_config;
    backup_config.ssid = "BackupNet";
    backup_config.password = "backup123";
    
    EXPECT_TRUE(wifi_controller->configure_station(primary_config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto initial_status = wifi_controller->get_status();
    ASSERT_TRUE(initial_status.has_value());
    EXPECT_TRUE(initial_status.value().connected);
    
    wifi_controller->simulate_connection_drop();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto dropped_status = wifi_controller->get_status();
    ASSERT_TRUE(dropped_status.has_value());
    EXPECT_FALSE(dropped_status.value().connected);
    
    auto reconnect_result = wifi_controller->connect();
    if (!reconnect_result.has_value()) {
        EXPECT_TRUE(wifi_controller->configure_station(backup_config).has_value());
        EXPECT_TRUE(wifi_controller->connect().has_value());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto backup_status = wifi_controller->get_status();
        ASSERT_TRUE(backup_status.has_value());
        EXPECT_TRUE(backup_status.value().connected);
        
        auto current_config = wifi_controller->get_station_config();
        ASSERT_TRUE(current_config.has_value());
        EXPECT_EQ(current_config.value().ssid, "BackupNet");
    }
    
    bluetooth_controller->add_simulated_device("BackupDevice", "11:22:33:44:55:66", 
        connectivity::BluetoothController::DeviceClass::SMARTPHONE);
    
    EXPECT_TRUE(bluetooth_controller->start_discovery().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto bt_devices = bluetooth_controller->get_discovered_devices();
    ASSERT_TRUE(bt_devices.has_value());
    EXPECT_FALSE(bt_devices.value().empty());
    
    EXPECT_TRUE(bluetooth_controller->connect_device("11:22:33:44:55:66").has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto bt_connections = bluetooth_controller->get_connected_devices();
    ASSERT_TRUE(bt_connections.has_value());
    EXPECT_FALSE(bt_connections.value().empty());
}

TEST_F(ConnectivityStackTest, ProtocolInterferenceMitigation) {
    auto wifi_controller = emulator_core->get_wifi_controller();
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    
    ASSERT_TRUE(wifi_controller != nullptr);
    ASSERT_TRUE(bluetooth_controller != nullptr);
    
    wifi_controller->add_simulated_network("InterferenceTest", -30, connectivity::WiFiController::SecurityType::WPA2);
    
    connectivity::WiFiController::StationConfig config;
    config.ssid = "InterferenceTest";
    config.password = "interference123";
    
    EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    EXPECT_TRUE(wifi_controller->connect().has_value());
    
    ASSERT_TRUE(bluetooth_controller->initialize().has_value());
    EXPECT_TRUE(bluetooth_controller->enable().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto initial_wifi_rssi = wifi_controller->get_rssi();
    ASSERT_TRUE(initial_wifi_rssi.has_value());
    
    bluetooth_controller->add_simulated_device("InterfereDevice", "AA:BB:CC:DD:EE:FF", 
        connectivity::BluetoothController::DeviceClass::SMARTPHONE);
    
    EXPECT_TRUE(bluetooth_controller->start_discovery().has_value());
    
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto wifi_rssi = wifi_controller->get_rssi();
        auto bt_scan_results = bluetooth_controller->get_discovered_devices();
        
        ASSERT_TRUE(wifi_rssi.has_value());
        ASSERT_TRUE(bt_scan_results.has_value());
        
        EXPECT_LE(wifi_rssi.value(), -25);
        
        if (wifi_rssi.value() < -70) {
            wifi_controller->set_power_mode(connectivity::WiFiController::PowerMode::PERFORMANCE);
        }
        
        if (bt_scan_results.value().size() > 10) {
            bluetooth_controller->stop_discovery();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            bluetooth_controller->start_discovery();
        }
    }
    
    EXPECT_TRUE(bluetooth_controller->stop_discovery().has_value());
    
    auto final_wifi_status = wifi_controller->get_status();
    ASSERT_TRUE(final_wifi_status.has_value());
    EXPECT_TRUE(final_wifi_status.value().connected);
    
    auto final_bt_status = bluetooth_controller->get_status();
    ASSERT_TRUE(final_bt_status.has_value());
    EXPECT_TRUE(final_bt_status.value().enabled);
}

} // namespace emulator::integration::test