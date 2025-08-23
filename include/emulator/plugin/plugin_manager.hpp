#pragma once

#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/config/configuration.hpp"
#include "plugin_interface.hpp"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace m5tab5::emulator {

/**
 * @brief Plugin management system for M5Stack Tab5 emulator
 * 
 * Handles loading, initializing, and coordinating plugins that extend
 * emulator functionality with custom behaviors and peripherals.
 */
class PluginManager {
public:
    PluginManager();
    ~PluginManager();

    // Lifecycle management
    Result<void> initialize(const Configuration& config);
    Result<void> shutdown();

    // Plugin loading and management
    Result<void> load_plugin(const std::string& plugin_name);
    Result<void> unload_plugin(const std::string& plugin_name);
    Result<void> reload_plugin(const std::string& plugin_name);

    // Plugin querying
    bool is_plugin_loaded(const std::string& plugin_name) const;
    std::vector<std::string> get_loaded_plugins() const;
    Result<Plugin*> get_plugin(const std::string& plugin_name);

    // Plugin lifecycle coordination
    Result<void> tick_all_plugins(Cycles cycle);
    Result<void> configure_plugin(const std::string& plugin_name, const std::string& config_json);

    // Plugin discovery
    std::vector<std::string> discover_available_plugins() const;

private:
    struct LoadedPlugin {
        std::unique_ptr<Plugin> plugin;
        void* library_handle;
        std::string library_path;
        bool initialized;
    };

    // Plugin storage
    std::unordered_map<std::string, LoadedPlugin> loaded_plugins_;
    
    // Configuration
    std::vector<std::string> plugin_search_paths_;
    bool initialized_;

    // Internal methods
    Result<void> load_plugin_library(const std::string& plugin_name, const std::string& library_path);
    Result<std::string> find_plugin_library(const std::string& plugin_name) const;
    void cleanup_plugin(const std::string& plugin_name);
};

} // namespace m5tab5::emulator