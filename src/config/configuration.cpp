#include "emulator/config/configuration.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace m5tab5::emulator {

using json = nlohmann::json;

Configuration::Configuration() {
    setDefaults();
}

Configuration::Configuration(DeviceVariant variant) : device_variant_(variant) {
    setDefaults();
}

Configuration::~Configuration() = default;

bool Configuration::loadFromFile(const std::string& filename) {
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        return loadFromString(content);
    } catch (const std::exception&) {
        return false;
    }
}

bool Configuration::saveToFile(const std::string& filename) const {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        file << saveToString();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool Configuration::loadFromString(const std::string& config_data) {
    try {
        json config_json = json::parse(config_data);
        
        // Basic stub implementation - just validate JSON structure
        if (config_json.contains("device")) {
            const auto& device = config_json["device"];
            if (device.contains("variant")) {
                auto variant_str = device["variant"].get<std::string>();
                if (variant_str == "standard") {
                    device_variant_ = DeviceVariant::Standard;
                } else if (variant_str == "industrial") {
                    device_variant_ = DeviceVariant::Industrial;
                } else if (variant_str == "custom") {
                    device_variant_ = DeviceVariant::Custom;
                }
            }
        }
        
        return true;
    } catch (const json::exception&) {
        return false;
    }
}

std::string Configuration::saveToString() const {
    json config;
    
    // Basic configuration structure
    config["device"]["variant"] = device_variant_ == DeviceVariant::Standard ? "standard" :
                                   device_variant_ == DeviceVariant::Industrial ? "industrial" : "custom";
    
    return config.dump(2);
}

bool Configuration::hasSection(const std::string& section) const {
    return config_tree_.find(section) != config_tree_.end();
}

bool Configuration::hasKey(const std::string& section, const std::string& key) const {
    auto section_it = config_tree_.find(section);
    if (section_it == config_tree_.end()) {
        return false;
    }
    return section_it->second.find(key) != section_it->second.end();
}

Configuration::ConfigSection Configuration::getSection(const std::string& section) const {
    auto section_it = config_tree_.find(section);
    if (section_it != config_tree_.end()) {
        return section_it->second;
    }
    return ConfigSection{};
}

void Configuration::setSection(const std::string& section, const ConfigSection& values) {
    config_tree_[section] = values;
}

void Configuration::removeSection(const std::string& section) {
    config_tree_.erase(section);
}

std::vector<std::string> Configuration::getSectionNames() const {
    std::vector<std::string> names;
    names.reserve(config_tree_.size());
    for (const auto& pair : config_tree_) {
        names.push_back(pair.first);
    }
    return names;
}

void Configuration::applyDeviceVariant(DeviceVariant variant) {
    device_variant_ = variant;
    // Apply variant-specific defaults
    setDefaults();
}

void Configuration::addValidationRule(const ValidationRule& rule) {
    validation_rules_.push_back(rule);
}

bool Configuration::validate(std::vector<std::string>& errors) const {
    errors.clear();
    // Basic validation stub
    return true;
}

Configuration Configuration::createDevelopmentConfig() {
    Configuration config(DeviceVariant::Standard);
    return config;
}

Configuration Configuration::createProductionConfig() {
    Configuration config(DeviceVariant::Standard);
    return config;
}

Configuration Configuration::createDebugConfig() {
    Configuration config(DeviceVariant::Standard);
    return config;
}

Configuration Configuration::createPerformanceConfig() {
    Configuration config(DeviceVariant::Standard);
    return config;
}

void Configuration::setDefaults() {
    config_tree_.clear();
    
    // Basic default configuration
    setValue("device", "variant", device_variant_ == DeviceVariant::Standard ? "standard" :
                                   device_variant_ == DeviceVariant::Industrial ? "industrial" : "custom");
    setValue("device", "model_name", std::string("M5Stack Tab5"));
    
    setValue("cpu", "main_frequency", static_cast<int64_t>(400000000));  // 400MHz
    setValue("cpu", "lp_frequency", static_cast<int64_t>(40000000));     // 40MHz
    setValue("cpu", "enable_dual_core", true);
    setValue("cpu", "enable_fpu", true);
    
    setValue("memory", "flash_size", static_cast<int64_t>(16 * 1024 * 1024));  // 16MB
    setValue("memory", "psram_size", static_cast<int64_t>(32 * 1024 * 1024));  // 32MB
    setValue("memory", "sram_size", static_cast<int64_t>(768 * 1024));         // 768KB
    
    setValue("display", "width", static_cast<int64_t>(1280));
    setValue("display", "height", static_cast<int64_t>(720));
    setValue("display", "refresh_rate", static_cast<int64_t>(60));
    setValue("display", "enable_vsync", true);
    
    setValue("audio", "sample_rate", static_cast<int64_t>(44100));
    setValue("audio", "buffer_size", static_cast<int64_t>(1024));
    setValue("audio", "channels", static_cast<int64_t>(2));
    
    setValue("debug", "enable_debugger", false);
    setValue("debug", "debugger_port", static_cast<int64_t>(3333));
    setValue("debug", "enable_profiling", false);
    setValue("debug", "log_level", std::string("INFO"));
}

// Typed configuration accessors - stub implementations
Configuration::CPUConfig Configuration::getCPUConfig() const {
    CPUConfig config;
    config.main_core_freq = static_cast<uint32_t>(getValue<int64_t>("cpu", "main_frequency", 400000000));
    config.lp_core_freq = static_cast<uint32_t>(getValue<int64_t>("cpu", "lp_frequency", 40000000));
    config.enable_dual_core = getValue<bool>("cpu", "enable_dual_core", true);
    config.enable_fpu = getValue<bool>("cpu", "enable_fpu", true);
    return config;
}

Configuration::MemoryConfig Configuration::getMemoryConfig() const {
    MemoryConfig config;
    config.flash_size = static_cast<size_t>(getValue<int64_t>("memory", "flash_size", 16 * 1024 * 1024));
    config.psram_size = static_cast<size_t>(getValue<int64_t>("memory", "psram_size", 32 * 1024 * 1024));
    config.sram_size = static_cast<size_t>(getValue<int64_t>("memory", "sram_size", 768 * 1024));
    return config;
}

Configuration::DisplayConfig Configuration::getDisplayConfig() const {
    DisplayConfig config;
    config.width = static_cast<uint32_t>(getValue<int64_t>("display", "width", 1280));
    config.height = static_cast<uint32_t>(getValue<int64_t>("display", "height", 720));
    config.refresh_rate = static_cast<uint32_t>(getValue<int64_t>("display", "refresh_rate", 60));
    config.enable_vsync = getValue<bool>("display", "enable_vsync", true);
    return config;
}

Configuration::AudioConfig Configuration::getAudioConfig() const {
    AudioConfig config;
    config.sample_rate = static_cast<uint32_t>(getValue<int64_t>("audio", "sample_rate", 44100));
    config.buffer_size = static_cast<uint32_t>(getValue<int64_t>("audio", "buffer_size", 1024));
    config.channels = static_cast<uint8_t>(getValue<int64_t>("audio", "channels", 2));
    return config;
}

Configuration::NetworkConfig Configuration::getNetworkConfig() const {
    NetworkConfig config;
    return config;
}

Configuration::DebugConfig Configuration::getDebugConfig() const {
    DebugConfig config;
    config.enable_debugger = getValue<bool>("debug", "enable_debugger", false);
    config.debugger_port = static_cast<uint16_t>(getValue<int64_t>("debug", "debugger_port", 3333));
    config.enable_profiling = getValue<bool>("debug", "enable_profiling", false);
    config.log_level = getValue<std::string>("debug", "log_level", "INFO");
    return config;
}

void Configuration::setCPUConfig(const CPUConfig& config) {
    setValue("cpu", "main_frequency", static_cast<int64_t>(config.main_core_freq));
    setValue("cpu", "lp_frequency", static_cast<int64_t>(config.lp_core_freq));
    setValue("cpu", "enable_dual_core", config.enable_dual_core);
    setValue("cpu", "enable_fpu", config.enable_fpu);
}

void Configuration::setMemoryConfig(const MemoryConfig& config) {
    setValue("memory", "flash_size", static_cast<int64_t>(config.flash_size));
    setValue("memory", "psram_size", static_cast<int64_t>(config.psram_size));
    setValue("memory", "sram_size", static_cast<int64_t>(config.sram_size));
}

void Configuration::setDisplayConfig(const DisplayConfig& config) {
    setValue("display", "width", static_cast<int64_t>(config.width));
    setValue("display", "height", static_cast<int64_t>(config.height));
    setValue("display", "refresh_rate", static_cast<int64_t>(config.refresh_rate));
    setValue("display", "enable_vsync", config.enable_vsync);
}

void Configuration::setAudioConfig(const AudioConfig& config) {
    setValue("audio", "sample_rate", static_cast<int64_t>(config.sample_rate));
    setValue("audio", "buffer_size", static_cast<int64_t>(config.buffer_size));
    setValue("audio", "channels", static_cast<int64_t>(config.channels));
}

void Configuration::setNetworkConfig(const NetworkConfig& config) {
    // Stub implementation
}

void Configuration::setDebugConfig(const DebugConfig& config) {
    setValue("debug", "enable_debugger", config.enable_debugger);
    setValue("debug", "debugger_port", static_cast<int64_t>(config.debugger_port));
    setValue("debug", "enable_profiling", config.enable_profiling);
    setValue("debug", "log_level", config.log_level);
}

// Template method specializations
template<>
std::string Configuration::convertValue<std::string>(const ConfigValue& value) const {
    return std::visit([](auto&& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::string>) {
            return arg;
        } else if constexpr (std::is_same_v<T, bool>) {
            return arg ? "true" : "false";
        } else if constexpr (std::is_same_v<T, int64_t>) {
            return std::to_string(arg);
        } else if constexpr (std::is_same_v<T, double>) {
            return std::to_string(arg);
        } else {
            return "";
        }
    }, value);
}

template<>
bool Configuration::convertValue<bool>(const ConfigValue& value) const {
    return std::visit([](auto&& arg) -> bool {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, bool>) {
            return arg;
        } else if constexpr (std::is_same_v<T, std::string>) {
            std::string lower = arg;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
            return lower == "true" || lower == "1" || lower == "yes" || lower == "on";
        } else if constexpr (std::is_same_v<T, int64_t>) {
            return arg != 0;
        } else if constexpr (std::is_same_v<T, double>) {
            return arg != 0.0;
        } else {
            return false;
        }
    }, value);
}

template<>
int64_t Configuration::convertValue<int64_t>(const ConfigValue& value) const {
    return std::visit([](auto&& arg) -> int64_t {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int64_t>) {
            return arg;
        } else if constexpr (std::is_same_v<T, double>) {
            return static_cast<int64_t>(arg);
        } else if constexpr (std::is_same_v<T, bool>) {
            return arg ? 1 : 0;
        } else if constexpr (std::is_same_v<T, std::string>) {
            try {
                return std::stoll(arg);
            } catch (...) {
                return 0;
            }
        } else {
            return 0;
        }
    }, value);
}

// Note: long and int64_t are the same type on this platform, so we don't need separate specialization

} // namespace m5tab5::emulator