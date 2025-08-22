#include <gtest/gtest.h>
#include "emulator/core/emulator_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/config/configuration.hpp"
#include <memory>
#include <thread>
#include <chrono>

namespace emulator::integration::test {

class SystemIntegrationTest : public ::testing::Test {
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
            "display": {
                "width": 1280,
                "height": 720,
                "refresh_rate": 60
            }
        })";
        
        config_path = std::filesystem::temp_directory_path() / "integration_config.json";
        std::ofstream(config_path) << config_content;
        
        auto config_result = config::Configuration::load(config_path.string());
        ASSERT_TRUE(config_result.has_value());
        configuration = config_result.value();
        
        auto core_result = EmulatorCore::create(configuration);
        ASSERT_TRUE(core_result.has_value());
        emulator_core = std::move(core_result.value());
    }
    
    void TearDown() override {
        if (emulator_core) {
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

TEST_F(SystemIntegrationTest, FullSystemInitialization) {
    ASSERT_TRUE(emulator_core->initialize().has_value()) 
        << "Failed to initialize emulator core";
    
    auto status = emulator_core->get_status();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status.value().state, EmulatorCore::State::INITIALIZED);
    
    EXPECT_TRUE(emulator_core->start().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    status = emulator_core->get_status();
    ASSERT_TRUE(status.value().state == EmulatorCore::State::RUNNING);
}

TEST_F(SystemIntegrationTest, CPUMemoryIntegration) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    auto memory_controller = emulator_core->get_memory_controller();
    ASSERT_TRUE(memory_controller != nullptr);
    
    uint32_t test_address = 0x20000000;
    uint32_t test_value = 0xDEADBEEF;
    
    EXPECT_TRUE(memory_controller->write32(test_address, test_value).has_value());
    
    auto read_result = memory_controller->read32(test_address);
    ASSERT_TRUE(read_result.has_value());
    EXPECT_EQ(read_result.value(), test_value);
    
    auto cpu_cores = emulator_core->get_cpu_cores();
    EXPECT_EQ(cpu_cores.size(), 2);
    
    for (auto& core : cpu_cores) {
        EXPECT_TRUE(core->set_pc(test_address).has_value());
        EXPECT_EQ(core->get_pc(), test_address);
    }
}

TEST_F(SystemIntegrationTest, DualCoreCoordination) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    auto cpu_cores = emulator_core->get_cpu_cores();
    ASSERT_EQ(cpu_cores.size(), 2);
    
    auto memory_controller = emulator_core->get_memory_controller();
    
    uint32_t shared_address = 0x20001000;
    uint32_t core0_data = 0x12345678;
    uint32_t core1_data = 0x87654321;
    
    EXPECT_TRUE(cpu_cores[0]->set_register(1, core0_data).has_value());
    EXPECT_TRUE(cpu_cores[1]->set_register(1, core1_data).has_value());
    
    EXPECT_TRUE(memory_controller->write32(shared_address, core0_data).has_value());
    EXPECT_TRUE(memory_controller->write32(shared_address + 4, core1_data).has_value());
    
    auto read0 = memory_controller->read32(shared_address);
    auto read1 = memory_controller->read32(shared_address + 4);
    
    ASSERT_TRUE(read0.has_value() && read1.has_value());
    EXPECT_EQ(read0.value(), core0_data);
    EXPECT_EQ(read1.value(), core1_data);
}

TEST_F(SystemIntegrationTest, PeripheralSystemIntegration) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    auto gpio_controller = emulator_core->get_gpio_controller();
    ASSERT_TRUE(gpio_controller != nullptr);
    
    uint8_t test_pin = 5;
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, peripherals::GPIOController::Mode::OUTPUT).has_value());
    EXPECT_TRUE(gpio_controller->digital_write(test_pin, true).has_value());
    
    auto pin_state = gpio_controller->digital_read(test_pin);
    ASSERT_TRUE(pin_state.has_value());
    EXPECT_TRUE(pin_state.value());
    
    auto i2c_controller = emulator_core->get_i2c_controller(0);
    ASSERT_TRUE(i2c_controller != nullptr);
    
    EXPECT_TRUE(i2c_controller->configure(400000, peripherals::I2CController::Mode::MASTER).has_value());
    
    i2c_controller->add_virtual_device(0x68);
    auto scan_result = i2c_controller->scan_devices();
    ASSERT_TRUE(scan_result.has_value());
    EXPECT_FALSE(scan_result.value().empty());
}

TEST_F(SystemIntegrationTest, DisplaySystemIntegration) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    auto graphics_engine = emulator_core->get_graphics_engine();
    ASSERT_TRUE(graphics_engine != nullptr);
    
    auto framebuffer = graphics_engine->get_framebuffer();
    ASSERT_TRUE(framebuffer != nullptr);
    
    graphics::Color test_color = {255, 128, 64, 255};
    EXPECT_TRUE(framebuffer->set_pixel(100, 100, test_color).has_value());
    
    auto pixel_result = framebuffer->get_pixel(100, 100);
    ASSERT_TRUE(pixel_result.has_value());
    EXPECT_EQ(pixel_result.value().r, test_color.r);
    EXPECT_EQ(pixel_result.value().g, test_color.g);
    EXPECT_EQ(pixel_result.value().b, test_color.b);
    
    auto touch_input = graphics_engine->get_touch_input();
    ASSERT_TRUE(touch_input != nullptr);
    
    graphics::TouchPoint test_touch = {200, 300, 1, true};
    touch_input->simulate_touch_event(test_touch);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    auto touch_events = touch_input->get_touch_events();
    ASSERT_TRUE(touch_events.has_value());
    EXPECT_FALSE(touch_events.value().empty());
}

TEST_F(SystemIntegrationTest, ConnectivityIntegration) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    auto wifi_controller = emulator_core->get_wifi_controller();
    if (wifi_controller) {
        wifi_controller->add_simulated_network("TestAP", -50, connectivity::WiFiController::SecurityType::WPA2);
        
        auto scan_result = wifi_controller->scan_networks();
        ASSERT_TRUE(scan_result.has_value());
        EXPECT_FALSE(scan_result.value().empty());
        
        connectivity::WiFiController::StationConfig config;
        config.ssid = "TestAP";
        config.password = "testpass";
        config.security = connectivity::WiFiController::SecurityType::WPA2;
        
        EXPECT_TRUE(wifi_controller->configure_station(config).has_value());
    }
    
    auto bluetooth_controller = emulator_core->get_bluetooth_controller();
    if (bluetooth_controller) {
        EXPECT_TRUE(bluetooth_controller->initialize().has_value());
        EXPECT_TRUE(bluetooth_controller->enable().has_value());
        
        auto status = bluetooth_controller->get_status();
        ASSERT_TRUE(status.has_value());
        EXPECT_TRUE(status.value().enabled);
    }
}

TEST_F(SystemIntegrationTest, ErrorPropagation) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    
    auto memory_controller = emulator_core->get_memory_controller();
    
    auto invalid_write = memory_controller->write32(0xFFFFFFFF, 0x12345678);
    EXPECT_FALSE(invalid_write.has_value());
    EXPECT_EQ(invalid_write.error(), utils::ErrorCode::INVALID_ADDRESS);
    
    auto error_stats = emulator_core->get_error_statistics();
    ASSERT_TRUE(error_stats.has_value());
    EXPECT_GT(error_stats.value().total_errors, 0);
}

TEST_F(SystemIntegrationTest, PerformanceUnderLoad) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto memory_controller = emulator_core->get_memory_controller();
    auto gpio_controller = emulator_core->get_gpio_controller();
    
    const int operations = 1000;
    std::atomic<int> completed_operations{0};
    
    std::vector<std::thread> worker_threads;
    
    worker_threads.emplace_back([&]() {
        for (int i = 0; i < operations / 2; ++i) {
            uint32_t addr = 0x20000000 + (i * 4);
            if (memory_controller->write32(addr, i).has_value()) {
                completed_operations++;
            }
        }
    });
    
    worker_threads.emplace_back([&]() {
        for (int i = 0; i < operations / 2; ++i) {
            uint8_t pin = i % 40;
            if (gpio_controller->configure_pin(pin, peripherals::GPIOController::Mode::OUTPUT).has_value()) {
                if (gpio_controller->digital_write(pin, i % 2 == 0).has_value()) {
                    completed_operations++;
                }
            }
        }
    });
    
    for (auto& thread : worker_threads) {
        thread.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_GT(completed_operations.load(), operations * 0.8);
    EXPECT_LT(duration.count(), 5000);
    
    auto perf_stats = emulator_core->get_performance_statistics();
    ASSERT_TRUE(perf_stats.has_value());
    EXPECT_GT(perf_stats.value().operations_per_second, 100);
}

TEST_F(SystemIntegrationTest, ShutdownSequence) {
    ASSERT_TRUE(emulator_core->initialize().has_value());
    ASSERT_TRUE(emulator_core->start().has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto status_before = emulator_core->get_status();
    ASSERT_TRUE(status_before.has_value());
    EXPECT_EQ(status_before.value().state, EmulatorCore::State::RUNNING);
    
    EXPECT_TRUE(emulator_core->stop().has_value());
    
    auto status_after = emulator_core->get_status();
    ASSERT_TRUE(status_after.has_value());
    EXPECT_EQ(status_after.value().state, EmulatorCore::State::STOPPED);
    
    emulator_core->shutdown();
    
    auto final_status = emulator_core->get_status();
    ASSERT_TRUE(final_status.has_value());
    EXPECT_EQ(final_status.value().state, EmulatorCore::State::SHUTDOWN);
}

} // namespace emulator::integration::test