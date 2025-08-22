#include <gtest/gtest.h>
#include "emulator/config/configuration.hpp"
#include "emulator/utils/types.hpp"
#include <filesystem>
#include <fstream>

namespace emulator::config::test {

class ConfigurationTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_config_path = std::filesystem::temp_directory_path() / "test_config.json";
        
        test_config_content = R"({
            "emulator": {
                "enable_debugging": true,
                "log_level": "debug",
                "performance_monitoring": true
            },
            "cpu": {
                "core_count": 2,
                "frequency_mhz": 240,
                "cache_size_kb": 64
            },
            "memory": {
                "psram_size_mb": 8,
                "enable_cache_coherency": true,
                "dma_channels": 8
            },
            "display": {
                "width": 1280,
                "height": 720,
                "refresh_rate": 60
            },
            "audio": {
                "sample_rate": 48000,
                "bit_depth": 16,
                "channels": 2
            },
            "connectivity": {
                "wifi": {
                    "enabled": true,
                    "default_power_mode": "active"
                },
                "bluetooth": {
                    "enabled": true,
                    "classic_enabled": true,
                    "ble_enabled": true
                }
            }
        })";
        
        std::ofstream config_file(test_config_path);
        config_file << test_config_content;
        config_file.close();
    }
    
    void TearDown() override {
        if (std::filesystem::exists(test_config_path)) {
            std::filesystem::remove(test_config_path);
        }
    }
    
    std::filesystem::path test_config_path;
    std::string test_config_content;
};

TEST_F(ConfigurationTest, LoadValidConfiguration) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value()) << "Failed to load configuration";
    
    auto config = config_result.value();
    
    EXPECT_TRUE(config->get_bool("emulator.enable_debugging").value_or(false));
    EXPECT_EQ(config->get_string("emulator.log_level").value_or(""), "debug");
    EXPECT_EQ(config->get_int("cpu.core_count").value_or(0), 2);
    EXPECT_EQ(config->get_int("cpu.frequency_mhz").value_or(0), 240);
}

TEST_F(ConfigurationTest, GettersWithDefaults) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    EXPECT_EQ(config->get_int("cpu.core_count", 1), 2);
    EXPECT_EQ(config->get_int("nonexistent.value", 42), 42);
    
    EXPECT_EQ(config->get_string("emulator.log_level", "info"), "debug");
    EXPECT_EQ(config->get_string("nonexistent.string", "default"), "default");
    
    EXPECT_TRUE(config->get_bool("emulator.enable_debugging", false));
    EXPECT_FALSE(config->get_bool("nonexistent.bool", false));
}

TEST_F(ConfigurationTest, SetAndSaveConfiguration) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    EXPECT_TRUE(config->set_int("cpu.core_count", 4).has_value());
    EXPECT_TRUE(config->set_string("emulator.log_level", "info").has_value());
    EXPECT_TRUE(config->set_bool("emulator.enable_debugging", false).has_value());
    
    EXPECT_EQ(config->get_int("cpu.core_count").value_or(0), 4);
    EXPECT_EQ(config->get_string("emulator.log_level").value_or(""), "info");
    EXPECT_FALSE(config->get_bool("emulator.enable_debugging").value_or(true));
    
    auto save_path = std::filesystem::temp_directory_path() / "saved_config.json";
    EXPECT_TRUE(config->save(save_path.string()).has_value());
    
    auto reloaded_config = Configuration::load(save_path.string());
    ASSERT_TRUE(reloaded_config.has_value());
    
    EXPECT_EQ(reloaded_config.value()->get_int("cpu.core_count").value_or(0), 4);
    EXPECT_EQ(reloaded_config.value()->get_string("emulator.log_level").value_or(""), "info");
    EXPECT_FALSE(reloaded_config.value()->get_bool("emulator.enable_debugging").value_or(true));
    
    std::filesystem::remove(save_path);
}

TEST_F(ConfigurationTest, NestedKeyHandling) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    EXPECT_TRUE(config->get_bool("connectivity.wifi.enabled").value_or(false));
    EXPECT_TRUE(config->get_bool("connectivity.bluetooth.classic_enabled").value_or(false));
    EXPECT_EQ(config->get_string("connectivity.wifi.default_power_mode").value_or(""), "active");
    
    EXPECT_TRUE(config->set_string("connectivity.wifi.default_power_mode", "power_save").has_value());
    EXPECT_EQ(config->get_string("connectivity.wifi.default_power_mode").value_or(""), "power_save");
}

TEST_F(ConfigurationTest, ArrayHandling) {
    std::string array_config = R"({
        "peripherals": {
            "gpio_pins": [1, 2, 3, 5, 8, 13],
            "i2c_addresses": ["0x68", "0x1D", "0x3C"],
            "uart_configs": [
                {"baud_rate": 115200, "data_bits": 8},
                {"baud_rate": 9600, "data_bits": 7}
            ]
        }
    })";
    
    auto array_config_path = std::filesystem::temp_directory_path() / "array_config.json";
    std::ofstream config_file(array_config_path);
    config_file << array_config;
    config_file.close();
    
    auto config_result = Configuration::load(array_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    auto gpio_pins = config->get_int_array("peripherals.gpio_pins");
    ASSERT_TRUE(gpio_pins.has_value());
    EXPECT_EQ(gpio_pins.value().size(), 6);
    EXPECT_EQ(gpio_pins.value()[0], 1);
    EXPECT_EQ(gpio_pins.value()[5], 13);
    
    auto i2c_addresses = config->get_string_array("peripherals.i2c_addresses");
    ASSERT_TRUE(i2c_addresses.has_value());
    EXPECT_EQ(i2c_addresses.value().size(), 3);
    EXPECT_EQ(i2c_addresses.value()[0], "0x68");
    
    std::filesystem::remove(array_config_path);
}

TEST_F(ConfigurationTest, ValidationRules) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    config->add_validation_rule("cpu.core_count", [](const auto& value) {
        if (value.is_number_integer()) {
            int cores = value.get<int>();
            return cores >= 1 && cores <= 8;
        }
        return false;
    });
    
    EXPECT_TRUE(config->set_int("cpu.core_count", 4).has_value());
    
    auto invalid_result = config->set_int("cpu.core_count", 16);
    EXPECT_FALSE(invalid_result.has_value());
    EXPECT_EQ(invalid_result.error(), utils::ErrorCode::VALIDATION_FAILED);
}

TEST_F(ConfigurationTest, ConfigurationMerging) {
    std::string base_config = R"({
        "emulator": {
            "enable_debugging": false,
            "log_level": "info"
        },
        "cpu": {
            "core_count": 1,
            "frequency_mhz": 160
        }
    })";
    
    std::string override_config = R"({
        "emulator": {
            "enable_debugging": true
        },
        "cpu": {
            "core_count": 2
        },
        "display": {
            "width": 800,
            "height": 600
        }
    })";
    
    auto base_path = std::filesystem::temp_directory_path() / "base_config.json";
    auto override_path = std::filesystem::temp_directory_path() / "override_config.json";
    
    std::ofstream(base_path) << base_config;
    std::ofstream(override_path) << override_config;
    
    auto base_config_obj = Configuration::load(base_path.string());
    auto override_config_obj = Configuration::load(override_path.string());
    
    ASSERT_TRUE(base_config_obj.has_value());
    ASSERT_TRUE(override_config_obj.has_value());
    
    EXPECT_TRUE(base_config_obj.value()->merge(*override_config_obj.value()).has_value());
    
    EXPECT_TRUE(base_config_obj.value()->get_bool("emulator.enable_debugging").value_or(false));
    EXPECT_EQ(base_config_obj.value()->get_string("emulator.log_level").value_or(""), "info");
    EXPECT_EQ(base_config_obj.value()->get_int("cpu.core_count").value_or(0), 2);
    EXPECT_EQ(base_config_obj.value()->get_int("cpu.frequency_mhz").value_or(0), 160);
    EXPECT_EQ(base_config_obj.value()->get_int("display.width").value_or(0), 800);
    
    std::filesystem::remove(base_path);
    std::filesystem::remove(override_path);
}

TEST_F(ConfigurationTest, EnvironmentVariableOverrides) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    config->enable_environment_overrides("M5TAB5_EMULATOR");
    
    setenv("M5TAB5_EMULATOR_CPU_CORE_COUNT", "6", 1);
    setenv("M5TAB5_EMULATOR_EMULATOR_LOG_LEVEL", "error", 1);
    
    EXPECT_EQ(config->get_int("cpu.core_count").value_or(0), 6);
    EXPECT_EQ(config->get_string("emulator.log_level").value_or(""), "error");
    
    unsetenv("M5TAB5_EMULATOR_CPU_CORE_COUNT");
    unsetenv("M5TAB5_EMULATOR_EMULATOR_LOG_LEVEL");
}

TEST_F(ConfigurationTest, ConfigurationWatching) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    bool change_detected = false;
    std::string changed_key;
    
    auto callback = [&change_detected, &changed_key](const std::string& key, const auto& value) {
        change_detected = true;
        changed_key = key;
    };
    
    EXPECT_TRUE(config->watch_changes(callback).has_value());
    
    std::string modified_config = R"({
        "emulator": {
            "enable_debugging": false,
            "log_level": "error",
            "performance_monitoring": true
        },
        "cpu": {
            "core_count": 4,
            "frequency_mhz": 240,
            "cache_size_kb": 64
        }
    })";
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::ofstream config_file(test_config_path);
    config_file << modified_config;
    config_file.close();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    EXPECT_TRUE(change_detected);
    EXPECT_FALSE(changed_key.empty());
}

TEST_F(ConfigurationTest, InvalidConfigurationHandling) {
    std::string invalid_config = "{ invalid json syntax }{";
    
    auto invalid_path = std::filesystem::temp_directory_path() / "invalid_config.json";
    std::ofstream(invalid_path) << invalid_config;
    
    auto config_result = Configuration::load(invalid_path.string());
    EXPECT_FALSE(config_result.has_value());
    EXPECT_EQ(config_result.error(), utils::ErrorCode::PARSE_ERROR);
    
    std::filesystem::remove(invalid_path);
}

TEST_F(ConfigurationTest, NonExistentFileHandling) {
    auto config_result = Configuration::load("/nonexistent/path/config.json");
    EXPECT_FALSE(config_result.has_value());
    EXPECT_EQ(config_result.error(), utils::ErrorCode::FILE_NOT_FOUND);
}

TEST_F(ConfigurationTest, TypeConversions) {
    auto config_result = Configuration::load(test_config_path.string());
    ASSERT_TRUE(config_result.has_value());
    auto config = config_result.value();
    
    EXPECT_TRUE(config->set_string("test.number_as_string", "42").has_value());
    EXPECT_EQ(config->get_int("test.number_as_string").value_or(0), 42);
    
    EXPECT_TRUE(config->set_int("test.bool_as_int", 1).has_value());
    EXPECT_TRUE(config->get_bool("test.bool_as_int").value_or(false));
    
    EXPECT_TRUE(config->set_bool("test.string_as_bool", true).has_value());
    EXPECT_EQ(config->get_string("test.string_as_bool").value_or(""), "true");
}

} // namespace emulator::config::test