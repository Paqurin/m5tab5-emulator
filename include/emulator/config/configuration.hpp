#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <variant>

namespace m5tab5::emulator {

/**
 * @brief Comprehensive configuration system for the emulator
 * 
 * Supports:
 * - JSON/TOML configuration files
 * - Runtime configuration updates
 * - Configuration validation
 * - Device variant profiles
 * - Plugin configurations
 */
class Configuration {
public:
    using ConfigValue = std::variant<bool, int64_t, double, std::string>;
    using ConfigSection = std::unordered_map<std::string, ConfigValue>;
    using ConfigTree = std::unordered_map<std::string, ConfigSection>;

    // Device variants
    enum class DeviceVariant {
        Standard,     // Base M5Tab5
        Industrial,   // Industrial variant
        Custom        // Custom configuration
    };

    Configuration();
    explicit Configuration(DeviceVariant variant);
    ~Configuration();

    // File operations
    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename) const;
    bool loadFromString(const std::string& config_data);
    std::string saveToString() const;

    // Value access
    template<typename T>
    T getValue(const std::string& section, const std::string& key, const T& default_value = T{}) const;
    
    template<typename T>
    void setValue(const std::string& section, const std::string& key, const T& value);

    bool hasSection(const std::string& section) const;
    bool hasKey(const std::string& section, const std::string& key) const;

    // Section management
    ConfigSection getSection(const std::string& section) const;
    void setSection(const std::string& section, const ConfigSection& values);
    void removeSection(const std::string& section);
    std::vector<std::string> getSectionNames() const;

    // Device variant management
    void applyDeviceVariant(DeviceVariant variant);
    DeviceVariant getDeviceVariant() const { return device_variant_; }

    // Validation
    struct ValidationRule {
        std::string section;
        std::string key;
        std::string type; // "bool", "int", "float", "string"
        ConfigValue min_value = 0;
        ConfigValue max_value = 0;
        std::vector<ConfigValue> allowed_values;
        bool required = false;
    };

    void addValidationRule(const ValidationRule& rule);
    bool validate(std::vector<std::string>& errors) const;

    // Configuration profiles for common setups
    static Configuration createDevelopmentConfig();
    static Configuration createProductionConfig();
    static Configuration createDebugConfig();
    static Configuration createPerformanceConfig();

    // Default configurations
    void setDefaults();

    // Hardware-specific configurations
    struct CPUConfig {
        uint32_t main_core_freq = 400000000; // 400MHz
        uint32_t lp_core_freq = 40000000;    // 40MHz
        bool enable_dual_core = true;
        bool enable_fpu = true;
        bool enable_compressed_instructions = true;
        uint32_t cache_size = 8192; // 8KB
    };

    struct MemoryConfig {
        size_t flash_size = 16 * 1024 * 1024;  // 16MB
        size_t psram_size = 32 * 1024 * 1024;  // 32MB
        size_t sram_size = 768 * 1024;         // 768KB
        bool enable_cache = true;
        bool enable_mmu = true;
        uint32_t cache_line_size = 32;
    };

    struct DisplayConfig {
        uint32_t width = 1280;
        uint32_t height = 720;
        uint32_t refresh_rate = 60;
        bool enable_vsync = true;
        bool enable_touch = true;
        float scale_factor = 1.0f;
        std::string window_title = "M5Tab5 Emulator";
    };

    struct AudioConfig {
        uint32_t sample_rate = 44100;
        uint32_t buffer_size = 1024;
        uint8_t channels = 2;
        bool enable_input = true;
        bool enable_output = true;
        std::string codec = "ES8388";
    };

    struct NetworkConfig {
        bool enable_wifi = true;
        bool enable_bluetooth = true;
        std::string wifi_ssid = "";
        std::string wifi_password = "";
        bool enable_ethernet = false;
    };

    struct DebugConfig {
        bool enable_debugger = false;
        uint16_t debugger_port = 3333;
        bool enable_profiling = false;
        bool enable_logging = true;
        std::string log_level = "INFO";
        std::string log_file = "";
    };

    // Typed configuration accessors
    CPUConfig getCPUConfig() const;
    MemoryConfig getMemoryConfig() const;
    DisplayConfig getDisplayConfig() const;
    AudioConfig getAudioConfig() const;
    NetworkConfig getNetworkConfig() const;
    DebugConfig getDebugConfig() const;

    void setCPUConfig(const CPUConfig& config);
    void setMemoryConfig(const MemoryConfig& config);
    void setDisplayConfig(const DisplayConfig& config);
    void setAudioConfig(const AudioConfig& config);
    void setNetworkConfig(const NetworkConfig& config);
    void setDebugConfig(const DebugConfig& config);

private:
    // Configuration storage
    ConfigTree config_tree_;
    DeviceVariant device_variant_ = DeviceVariant::Standard;
    
    // Validation rules
    std::vector<ValidationRule> validation_rules_;

    // Helper methods
    void setupDefaultValidationRules();
    void applyStandardDefaults();
    void applyIndustrialDefaults();
    
    template<typename T>
    T convertValue(const ConfigValue& value) const;
    
    ConfigValue parseValue(const std::string& value, const std::string& type) const;
};

// Template implementations
template<typename T>
T Configuration::getValue(const std::string& section, const std::string& key, const T& default_value) const {
    auto section_it = config_tree_.find(section);
    if (section_it == config_tree_.end()) {
        return default_value;
    }
    
    auto key_it = section_it->second.find(key);
    if (key_it == section_it->second.end()) {
        return default_value;
    }
    
    return convertValue<T>(key_it->second);
}

template<typename T>
void Configuration::setValue(const std::string& section, const std::string& key, const T& value) {
    config_tree_[section][key] = value;
}

} // namespace m5tab5::emulator