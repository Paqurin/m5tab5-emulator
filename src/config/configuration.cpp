#include "emulator/config/configuration.hpp"
#include "emulator/utils/error.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace m5tab5::emulator {

using json = nlohmann::json;

Configuration::Configuration() {
    set_defaults();
}

Configuration::~Configuration() = default;

Result<void> Configuration::load_from_file(const std::string& config_path) {
    if (!std::filesystem::exists(config_path)) {
        return std::unexpected(MAKE_ERROR(CONFIG_FILE_NOT_FOUND, 
            "Configuration file not found: " + config_path));
    }
    
    try {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            return std::unexpected(MAKE_ERROR(CONFIG_FILE_NOT_FOUND,
                "Failed to open configuration file: " + config_path));
        }
        
        json config_json;
        file >> config_json;
        
        return load_from_json(config_json);
    } catch (const json::exception& e) {
        return std::unexpected(MAKE_ERROR(CONFIG_INVALID_FORMAT,
            "JSON parsing error: " + std::string(e.what())));
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to load configuration: " + std::string(e.what())));
    }
}

Result<void> Configuration::load_from_json(const nlohmann::json& config_json) {
    try {
        // Device configuration
        if (config_json.contains("device")) {
            const auto& device = config_json["device"];
            
            if (device.contains("variant")) {
                auto variant_str = device["variant"].get<std::string>();
                if (variant_str == "standard") {
                    device_variant_ = DeviceVariant::STANDARD;
                } else if (variant_str == "industrial") {
                    device_variant_ = DeviceVariant::INDUSTRIAL;
                } else if (variant_str == "custom") {
                    device_variant_ = DeviceVariant::CUSTOM;
                } else {
                    return std::unexpected(MAKE_ERROR(CONFIG_INVALID_VALUE,
                        "Invalid device variant: " + variant_str));
                }
            }
            
            if (device.contains("model_name")) {
                model_name_ = device["model_name"].get<std::string>();
            }
        }
        
        // CPU configuration
        if (config_json.contains("cpu")) {
            const auto& cpu = config_json["cpu"];
            
            if (cpu.contains("main_frequency")) {
                main_cpu_frequency_ = cpu["main_frequency"].get<u32>();
            }
            
            if (cpu.contains("lp_frequency")) {
                lp_cpu_frequency_ = cpu["lp_frequency"].get<u32>();
            }
            
            if (cpu.contains("enable_debugging")) {
                enable_cpu_debugging_ = cpu["enable_debugging"].get<bool>();
            }
        }
        
        // Memory configuration
        if (config_json.contains("memory")) {
            const auto& memory = config_json["memory"];
            
            if (memory.contains("flash_size")) {
                flash_size_ = memory["flash_size"].get<size_t>();
            }
            
            if (memory.contains("psram_size")) {
                psram_size_ = memory["psram_size"].get<size_t>();
            }
            
            if (memory.contains("sram_size")) {
                sram_size_ = memory["sram_size"].get<size_t>();
            }
            
            if (memory.contains("cache_line_size")) {
                cache_line_size_ = memory["cache_line_size"].get<size_t>();
            }
        }
        
        // Display configuration
        if (config_json.contains("display")) {
            const auto& display = config_json["display"];
            
            if (display.contains("width")) {
                display_width_ = display["width"].get<u32>();
            }
            
            if (display.contains("height")) {
                display_height_ = display["height"].get<u32>();
            }
            
            if (display.contains("refresh_rate")) {
                display_refresh_rate_ = display["refresh_rate"].get<u32>();
            }
            
            if (display.contains("enable_vsync")) {
                enable_vsync_ = display["enable_vsync"].get<bool>();
            }
        }
        
        // Audio configuration
        if (config_json.contains("audio")) {
            const auto& audio = config_json["audio"];
            
            if (audio.contains("sample_rate")) {
                audio_sample_rate_ = audio["sample_rate"].get<u32>();
            }
            
            if (audio.contains("buffer_size")) {
                audio_buffer_size_ = audio["buffer_size"].get<u32>();
            }
            
            if (audio.contains("enable_input")) {
                enable_audio_input_ = audio["enable_input"].get<bool>();
            }
            
            if (audio.contains("enable_output")) {
                enable_audio_output_ = audio["enable_output"].get<bool>();
            }
        }
        
        // Logging configuration
        if (config_json.contains("logging")) {
            const auto& logging = config_json["logging"];
            
            if (logging.contains("level")) {
                auto level_str = logging["level"].get<std::string>();
                if (level_str == "trace") {
                    log_level_ = LogLevel::TRACE;
                } else if (level_str == "debug") {
                    log_level_ = LogLevel::DEBUG;
                } else if (level_str == "info") {
                    log_level_ = LogLevel::INFO;
                } else if (level_str == "warn") {
                    log_level_ = LogLevel::WARN;
                } else if (level_str == "error") {
                    log_level_ = LogLevel::ERROR;
                } else {
                    return std::unexpected(MAKE_ERROR(CONFIG_INVALID_VALUE,
                        "Invalid log level: " + level_str));
                }
            }
            
            if (logging.contains("file_path")) {
                log_file_path_ = logging["file_path"].get<std::string>();
            }
            
            if (logging.contains("enable_console")) {
                enable_console_logging_ = logging["enable_console"].get<bool>();
            }
        }
        
        // Plugins configuration
        if (config_json.contains("plugins")) {
            const auto& plugins = config_json["plugins"];
            
            if (plugins.contains("search_paths")) {
                plugin_search_paths_.clear();
                for (const auto& path : plugins["search_paths"]) {
                    plugin_search_paths_.push_back(path.get<std::string>());
                }
            }
            
            if (plugins.contains("enabled_plugins")) {
                enabled_plugins_.clear();
                for (const auto& plugin : plugins["enabled_plugins"]) {
                    enabled_plugins_.push_back(plugin.get<std::string>());
                }
            }
        }
        
        return {};
    } catch (const json::exception& e) {
        return std::unexpected(MAKE_ERROR(CONFIG_INVALID_FORMAT,
            "JSON processing error: " + std::string(e.what())));
    }
}

Result<void> Configuration::save_to_file(const std::string& config_path) const {
    try {
        json config_json = to_json();
        
        std::ofstream file(config_path);
        if (!file.is_open()) {
            return std::unexpected(MAKE_ERROR(IO_WRITE_FAILED,
                "Failed to open file for writing: " + config_path));
        }
        
        file << config_json.dump(2);
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to save configuration: " + std::string(e.what())));
    }
}

nlohmann::json Configuration::to_json() const {
    json config;
    
    // Device configuration
    config["device"]["variant"] = device_variant_to_string(device_variant_);
    config["device"]["model_name"] = model_name_;
    
    // CPU configuration
    config["cpu"]["main_frequency"] = main_cpu_frequency_;
    config["cpu"]["lp_frequency"] = lp_cpu_frequency_;
    config["cpu"]["enable_debugging"] = enable_cpu_debugging_;
    
    // Memory configuration
    config["memory"]["flash_size"] = flash_size_;
    config["memory"]["psram_size"] = psram_size_;
    config["memory"]["sram_size"] = sram_size_;
    config["memory"]["cache_line_size"] = cache_line_size_;
    
    // Display configuration
    config["display"]["width"] = display_width_;
    config["display"]["height"] = display_height_;
    config["display"]["refresh_rate"] = display_refresh_rate_;
    config["display"]["enable_vsync"] = enable_vsync_;
    
    // Audio configuration
    config["audio"]["sample_rate"] = audio_sample_rate_;
    config["audio"]["buffer_size"] = audio_buffer_size_;
    config["audio"]["enable_input"] = enable_audio_input_;
    config["audio"]["enable_output"] = enable_audio_output_;
    
    // Logging configuration
    config["logging"]["level"] = log_level_to_string(log_level_);
    config["logging"]["file_path"] = log_file_path_;
    config["logging"]["enable_console"] = enable_console_logging_;
    
    // Plugins configuration
    config["plugins"]["search_paths"] = plugin_search_paths_;
    config["plugins"]["enabled_plugins"] = enabled_plugins_;
    
    return config;
}

void Configuration::set_defaults() {
    device_variant_ = DeviceVariant::STANDARD;
    model_name_ = "M5Stack Tab5";
    
    main_cpu_frequency_ = 400000000;  // 400MHz
    lp_cpu_frequency_ = 40000000;     // 40MHz
    enable_cpu_debugging_ = false;
    
    flash_size_ = 16 * 1024 * 1024;   // 16MB
    psram_size_ = 32 * 1024 * 1024;   // 32MB
    sram_size_ = 768 * 1024;          // 768KB
    cache_line_size_ = 32;            // 32 bytes
    
    display_width_ = 1280;
    display_height_ = 720;
    display_refresh_rate_ = 60;
    enable_vsync_ = true;
    
    audio_sample_rate_ = 44100;
    audio_buffer_size_ = 1024;
    enable_audio_input_ = true;
    enable_audio_output_ = true;
    
    log_level_ = LogLevel::INFO;
    log_file_path_ = "emulator.log";
    enable_console_logging_ = true;
    
    plugin_search_paths_ = {"./plugins", "/usr/local/lib/m5tab5-emulator/plugins"};
    enabled_plugins_.clear();
}

const char* Configuration::device_variant_to_string(DeviceVariant variant) {
    switch (variant) {
        case DeviceVariant::STANDARD: return "standard";
        case DeviceVariant::INDUSTRIAL: return "industrial";
        case DeviceVariant::CUSTOM: return "custom";
        default: return "unknown";
    }
}

const char* Configuration::log_level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::TRACE: return "trace";
        case LogLevel::DEBUG: return "debug";
        case LogLevel::INFO: return "info";
        case LogLevel::WARN: return "warn";
        case LogLevel::ERROR: return "error";
        default: return "unknown";
    }
}

}  // namespace m5tab5::emulator