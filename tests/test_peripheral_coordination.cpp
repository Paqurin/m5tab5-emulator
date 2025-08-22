#include <gtest/gtest.h>
#include "emulator/core/emulator_core.hpp"
#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/peripherals/spi_controller.hpp"
#include "emulator/peripherals/uart_controller.hpp"
#include "emulator/peripherals/pin_mux_controller.hpp"
#include "emulator/config/configuration.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>

namespace emulator::integration::test {

class PeripheralCoordinationTest : public ::testing::Test {
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
            "memory": {
                "psram_size_mb": 8,
                "enable_cache_coherency": true
            },
            "peripherals": {
                "gpio_pins": 55,
                "i2c_controllers": 2,
                "spi_controllers": 3,
                "uart_controllers": 5
            }
        })";
        
        config_path = std::filesystem::temp_directory_path() / "peripheral_config.json";
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

TEST_F(PeripheralCoordinationTest, GPIOInterruptToI2CTransaction) {
    auto gpio_controller = emulator_core->get_gpio_controller();
    auto i2c_controller = emulator_core->get_i2c_controller(0);
    
    ASSERT_TRUE(gpio_controller != nullptr);
    ASSERT_TRUE(i2c_controller != nullptr);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(0x68);
    
    uint8_t interrupt_pin = 2;
    std::atomic<bool> i2c_transaction_completed{false};
    
    EXPECT_TRUE(gpio_controller->configure_pin(interrupt_pin, peripherals::GPIOController::Mode::INPUT).has_value());
    
    auto interrupt_callback = [&](uint8_t pin, bool state) {
        if (pin == interrupt_pin && state) {
            auto write_result = i2c_controller->write_register(0x68, 0x6B, 0x80);
            if (write_result.has_value()) {
                auto read_result = i2c_controller->read_register(0x68, 0x6B);
                if (read_result.has_value() && read_result.value() == 0x80) {
                    i2c_transaction_completed = true;
                }
            }
        }
    };
    
    EXPECT_TRUE(gpio_controller->attach_interrupt(interrupt_pin, 
        peripherals::GPIOController::InterruptTrigger::RISING_EDGE, interrupt_callback).has_value());
    
    gpio_controller->simulate_external_input(interrupt_pin, true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(i2c_transaction_completed.load());
}

TEST_F(PeripheralCoordinationTest, MultiPeripheralSensorFusion) {
    auto gpio_controller = emulator_core->get_gpio_controller();
    auto i2c_controller = emulator_core->get_i2c_controller(0);
    auto spi_controller = emulator_core->get_spi_controller(0);
    
    ASSERT_TRUE(gpio_controller != nullptr);
    ASSERT_TRUE(i2c_controller != nullptr);
    ASSERT_TRUE(spi_controller != nullptr);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(0x68);  // BMI270 IMU
    i2c_controller->add_virtual_device(0x1D);  // Accelerometer
    
    EXPECT_TRUE(spi_controller->configure(8000000, peripherals::SPIController::Mode::MASTER).has_value());
    spi_controller->add_virtual_device(0, 0x42);  // Gyroscope via SPI
    
    uint8_t data_ready_pin = 5;
    EXPECT_TRUE(gpio_controller->configure_pin(data_ready_pin, peripherals::GPIOController::Mode::INPUT).has_value());
    
    struct SensorData {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
        bool valid;
    };
    
    std::vector<SensorData> sensor_readings;
    std::mutex readings_mutex;
    
    auto data_ready_callback = [&](uint8_t pin, bool state) {
        if (pin == data_ready_pin && state) {
            SensorData data = {};
            
            auto accel_x_result = i2c_controller->read_registers(0x1D, 0x32, 6);
            if (accel_x_result.has_value() && accel_x_result.value().size() >= 6) {
                auto& accel_data = accel_x_result.value();
                data.accel_x = (accel_data[1] << 8) | accel_data[0];
                data.accel_y = (accel_data[3] << 8) | accel_data[2];
                data.accel_z = (accel_data[5] << 8) | accel_data[4];
            }
            
            auto spi_transaction = spi_controller->begin_transaction(0);
            if (spi_transaction.has_value()) {
                spi_controller->write_byte(0x43);  // Gyro register
                auto gyro_data = spi_controller->read_bytes(6);
                if (gyro_data.has_value() && gyro_data.value().size() >= 6) {
                    auto& gyro_bytes = gyro_data.value();
                    data.gyro_x = (gyro_bytes[1] << 8) | gyro_bytes[0];
                    data.gyro_y = (gyro_bytes[3] << 8) | gyro_bytes[2];
                    data.gyro_z = (gyro_bytes[5] << 8) | gyro_bytes[4];
                }
                spi_controller->end_transaction();
            }
            
            data.valid = true;
            
            std::lock_guard<std::mutex> lock(readings_mutex);
            sensor_readings.push_back(data);
        }
    };
    
    EXPECT_TRUE(gpio_controller->attach_interrupt(data_ready_pin, 
        peripherals::GPIOController::InterruptTrigger::RISING_EDGE, data_ready_callback).has_value());
    
    for (int i = 0; i < 10; ++i) {
        gpio_controller->simulate_external_input(data_ready_pin, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        gpio_controller->simulate_external_input(data_ready_pin, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::lock_guard<std::mutex> lock(readings_mutex);
    EXPECT_EQ(sensor_readings.size(), 10);
    
    for (const auto& reading : sensor_readings) {
        EXPECT_TRUE(reading.valid);
    }
}

TEST_F(PeripheralCoordinationTest, UARTToGPIOControl) {
    auto gpio_controller = emulator_core->get_gpio_controller();
    auto uart_controller = emulator_core->get_uart_controller(0);
    
    ASSERT_TRUE(gpio_controller != nullptr);
    ASSERT_TRUE(uart_controller != nullptr);
    
    std::vector<uint8_t> control_pins = {10, 11, 12, 13};
    for (auto pin : control_pins) {
        EXPECT_TRUE(gpio_controller->configure_pin(pin, peripherals::GPIOController::Mode::OUTPUT).has_value());
        EXPECT_TRUE(gpio_controller->digital_write(pin, false).has_value());
    }
    
    EXPECT_TRUE(uart_controller->configure(115200, 8, peripherals::UARTController::Parity::NONE, 1).has_value());
    
    auto command_handler = [&](const std::vector<uint8_t>& data) {
        if (data.size() >= 3 && data[0] == 0xAA && data[1] == 0x55) {
            uint8_t command = data[2];
            
            switch (command) {
                case 0x01:  // Turn on LED pattern
                    for (size_t i = 0; i < control_pins.size(); ++i) {
                        bool state = (command >> i) & 1;
                        gpio_controller->digital_write(control_pins[i], state);
                    }
                    break;
                    
                case 0x02:  // Blink sequence
                    for (int blink = 0; blink < 3; ++blink) {
                        for (auto pin : control_pins) {
                            gpio_controller->digital_write(pin, true);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        for (auto pin : control_pins) {
                            gpio_controller->digital_write(pin, false);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    break;
                    
                case 0x03:  // Pattern scan
                    for (auto pin : control_pins) {
                        gpio_controller->digital_write(pin, true);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        gpio_controller->digital_write(pin, false);
                    }
                    break;
            }
            
            std::vector<uint8_t> response = {0xBB, 0x66, command, 0x00};
            uart_controller->write_bulk(response);
        }
    };
    
    EXPECT_TRUE(uart_controller->set_receive_callback(command_handler).has_value());
    
    std::vector<uint8_t> commands = {
        {0xAA, 0x55, 0x01},  // LED pattern
        {0xAA, 0x55, 0x02},  // Blink sequence  
        {0xAA, 0x55, 0x03}   // Pattern scan
    };
    
    for (const auto& command : commands) {
        uart_controller->simulate_receive(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        auto pin_state = gpio_controller->digital_read(control_pins[0]);
        ASSERT_TRUE(pin_state.has_value());
    }
}

TEST_F(PeripheralCoordinationTest, PinMuxingCoordination) {
    auto gpio_controller = emulator_core->get_gpio_controller();
    auto i2c_controller = emulator_core->get_i2c_controller(1);
    auto pin_mux_controller = emulator_core->get_pin_mux_controller();
    
    ASSERT_TRUE(gpio_controller != nullptr);
    ASSERT_TRUE(i2c_controller != nullptr);
    ASSERT_TRUE(pin_mux_controller != nullptr);
    
    uint8_t sda_pin = 21;
    uint8_t scl_pin = 22;
    
    EXPECT_TRUE(gpio_controller->configure_pin(sda_pin, peripherals::GPIOController::Mode::OUTPUT).has_value());
    EXPECT_TRUE(gpio_controller->configure_pin(scl_pin, peripherals::GPIOController::Mode::OUTPUT).has_value());
    
    EXPECT_TRUE(gpio_controller->digital_write(sda_pin, true).has_value());
    EXPECT_TRUE(gpio_controller->digital_write(scl_pin, true).has_value());
    
    auto gpio_state_sda = gpio_controller->digital_read(sda_pin);
    auto gpio_state_scl = gpio_controller->digital_read(scl_pin);
    ASSERT_TRUE(gpio_state_sda.has_value() && gpio_state_scl.has_value());
    EXPECT_TRUE(gpio_state_sda.value() && gpio_state_scl.value());
    
    EXPECT_TRUE(pin_mux_controller->configure_pin(sda_pin, peripherals::PinMuxController::Function::I2C1_SDA).has_value());
    EXPECT_TRUE(pin_mux_controller->configure_pin(scl_pin, peripherals::PinMuxController::Function::I2C1_SCL).has_value());
    
    auto gpio_write_after_mux = gpio_controller->digital_write(sda_pin, false);
    EXPECT_FALSE(gpio_write_after_mux.has_value());
    EXPECT_EQ(gpio_write_after_mux.error(), utils::ErrorCode::INVALID_OPERATION);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(0x3C);
    
    auto i2c_result = i2c_controller->write_register(0x3C, 0x00, 0xFF);
    EXPECT_TRUE(i2c_result.has_value());
    
    EXPECT_TRUE(pin_mux_controller->configure_pin(sda_pin, peripherals::PinMuxController::Function::GPIO).has_value());
    EXPECT_TRUE(pin_mux_controller->configure_pin(scl_pin, peripherals::PinMuxController::Function::GPIO).has_value());
    
    EXPECT_TRUE(gpio_controller->configure_pin(sda_pin, peripherals::GPIOController::Mode::OUTPUT).has_value());
    EXPECT_TRUE(gpio_controller->digital_write(sda_pin, false).has_value());
    
    auto final_gpio_state = gpio_controller->digital_read(sda_pin);
    ASSERT_TRUE(final_gpio_state.has_value());
    EXPECT_FALSE(final_gpio_state.value());
}

TEST_F(PeripheralCoordinationTest, InterPeripheralCommunication) {
    auto i2c_controller = emulator_core->get_i2c_controller(0);
    auto spi_controller = emulator_core->get_spi_controller(0);
    auto uart_controller = emulator_core->get_uart_controller(0);
    
    ASSERT_TRUE(i2c_controller != nullptr);
    ASSERT_TRUE(spi_controller != nullptr);
    ASSERT_TRUE(uart_controller != nullptr);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    EXPECT_TRUE(spi_controller->configure(1000000, peripherals::SPIController::Mode::MASTER).has_value());
    EXPECT_TRUE(uart_controller->configure(115200, 8, peripherals::UARTController::Parity::NONE, 1).has_value());
    
    i2c_controller->add_virtual_device(0x48);
    spi_controller->add_virtual_device(0, 0x55);
    
    struct DataPacket {
        uint8_t source;
        uint16_t sensor_data;
        uint32_t timestamp;
        uint8_t checksum;
    };
    
    std::vector<DataPacket> collected_data;
    std::mutex data_mutex;
    
    auto process_sensor_data = [&]() {
        for (int i = 0; i < 10; ++i) {
            DataPacket packet = {};
            
            auto i2c_temp_data = i2c_controller->read_registers(0x48, 0x00, 2);
            if (i2c_temp_data.has_value() && i2c_temp_data.value().size() >= 2) {
                packet.source = 0x01;  // I2C sensor
                packet.sensor_data = (i2c_temp_data.value()[1] << 8) | i2c_temp_data.value()[0];
            }
            
            auto spi_transaction = spi_controller->begin_transaction(0);
            if (spi_transaction.has_value()) {
                spi_controller->write_byte(0xD0);  // WHO_AM_I register
                auto spi_data = spi_controller->read_bytes(2);
                if (spi_data.has_value() && spi_data.value().size() >= 2) {
                    packet.sensor_data = (spi_data.value()[0] << 8) | spi_data.value()[1];
                    packet.source = 0x02;  // SPI sensor
                }
                spi_controller->end_transaction();
            }
            
            packet.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            
            packet.checksum = packet.source ^ (packet.sensor_data & 0xFF) ^ 
                             ((packet.sensor_data >> 8) & 0xFF) ^ (packet.timestamp & 0xFF);
            
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                collected_data.push_back(packet);
            }
            
            std::vector<uint8_t> uart_data = {
                packet.source,
                static_cast<uint8_t>(packet.sensor_data & 0xFF),
                static_cast<uint8_t>((packet.sensor_data >> 8) & 0xFF),
                static_cast<uint8_t>(packet.timestamp & 0xFF),
                packet.checksum
            };
            
            uart_controller->write_bulk(uart_data);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    };
    
    std::thread sensor_thread(process_sensor_data);
    sensor_thread.join();
    
    std::lock_guard<std::mutex> lock(data_mutex);
    EXPECT_EQ(collected_data.size(), 10);
    
    for (const auto& packet : collected_data) {
        EXPECT_TRUE(packet.source == 0x01 || packet.source == 0x02);
        EXPECT_GT(packet.timestamp, 0);
        
        uint8_t calculated_checksum = packet.source ^ (packet.sensor_data & 0xFF) ^ 
                                    ((packet.sensor_data >> 8) & 0xFF) ^ (packet.timestamp & 0xFF);
        EXPECT_EQ(packet.checksum, calculated_checksum);
    }
}

TEST_F(PeripheralCoordinationTest, ConcurrentPeripheralAccess) {
    const int num_threads = 6;
    const int operations_per_thread = 200;
    
    auto gpio_controller = emulator_core->get_gpio_controller();
    auto i2c_controller = emulator_core->get_i2c_controller(0);
    auto spi_controller = emulator_core->get_spi_controller(0);
    
    ASSERT_TRUE(gpio_controller != nullptr);
    ASSERT_TRUE(i2c_controller != nullptr);
    ASSERT_TRUE(spi_controller != nullptr);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    EXPECT_TRUE(spi_controller->configure(1000000, peripherals::SPIController::Mode::MASTER).has_value());
    
    for (int i = 0; i < 5; ++i) {
        i2c_controller->add_virtual_device(0x20 + i);
        spi_controller->add_virtual_device(i, 0x30 + i);
    }
    
    std::vector<std::thread> threads;
    std::atomic<int> successful_operations{0};
    std::atomic<int> failed_operations{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            for (int j = 0; j < operations_per_thread; ++j) {
                bool success = true;
                
                if (i % 3 == 0) {  // GPIO operations
                    uint8_t pin = (i * 10 + j) % 40;
                    if (gpio_controller->configure_pin(pin, peripherals::GPIOController::Mode::OUTPUT).has_value()) {
                        if (gpio_controller->digital_write(pin, j % 2 == 0).has_value()) {
                            auto read_result = gpio_controller->digital_read(pin);
                            success = read_result.has_value();
                        } else {
                            success = false;
                        }
                    } else {
                        success = false;
                    }
                } else if (i % 3 == 1) {  // I2C operations
                    uint8_t device_addr = 0x20 + (j % 5);
                    auto write_result = i2c_controller->write_register(device_addr, j % 256, (j * 7) % 256);
                    if (write_result.has_value()) {
                        auto read_result = i2c_controller->read_register(device_addr, j % 256);
                        success = read_result.has_value() && read_result.value() == ((j * 7) % 256);
                    } else {
                        success = false;
                    }
                } else {  // SPI operations
                    uint8_t cs_pin = j % 5;
                    auto transaction = spi_controller->begin_transaction(cs_pin);
                    if (transaction.has_value()) {
                        if (spi_controller->write_byte(j % 256).has_value()) {
                            auto read_result = spi_controller->read_byte();
                            success = read_result.has_value();
                        } else {
                            success = false;
                        }
                        spi_controller->end_transaction();
                    } else {
                        success = false;
                    }
                }
                
                if (success) {
                    successful_operations++;
                } else {
                    failed_operations++;
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    int total_operations = num_threads * operations_per_thread;
    double success_rate = double(successful_operations.load()) / total_operations;
    
    std::cout << "Concurrent Peripheral Access Results:" << std::endl;
    std::cout << "  Total operations: " << total_operations << std::endl;
    std::cout << "  Successful operations: " << successful_operations.load() << std::endl;
    std::cout << "  Failed operations: " << failed_operations.load() << std::endl;
    std::cout << "  Success rate: " << (success_rate * 100) << "%" << std::endl;
    
    EXPECT_GT(success_rate, 0.95);  // Expect >95% success rate
    EXPECT_EQ(successful_operations.load() + failed_operations.load(), total_operations);
}

} // namespace emulator::integration::test