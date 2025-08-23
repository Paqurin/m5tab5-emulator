#include "emulator/plugin/plugin_interface.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/logging.hpp"

#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <dlfcn.h>

namespace m5tab5::emulator {

class PluginLoaderImpl {
private:
    struct LoadedPlugin {
        void* handle;
        std::unique_ptr<Plugin> plugin;
        PluginInfo info;
        
        ~LoadedPlugin() {
            if (plugin) {
                plugin->shutdown();
                plugin.reset();
            }
            if (handle) {
                dlclose(handle);
            }
        }
    };
    
    std::vector<std::unique_ptr<LoadedPlugin>> loaded_plugins_;

public:
    std::unique_ptr<Plugin> loadPlugin(const std::string& path) {
        // Clear any previous errors
        dlerror();
        
        // Load the shared library
        void* handle = dlopen(path.c_str(), RTLD_LAZY | RTLD_LOCAL);
        if (!handle) {
            const char* error_msg = dlerror();
            LOG_ERROR("Failed to load plugin '{}': {}", path, error_msg ? error_msg : "Unknown error");
            return nullptr;
        }
        
        // Clear any previous errors
        dlerror();
        
        // Get the required function pointers
        auto createFunc = reinterpret_cast<CreatePluginFunc>(dlsym(handle, "createPlugin"));
        auto destroyFunc = reinterpret_cast<DestroyPluginFunc>(dlsym(handle, "destroyPlugin"));
        auto getInfoFunc = reinterpret_cast<GetPluginInfoFunc>(dlsym(handle, "getPluginInfo"));
        auto getAPIVersionFunc = reinterpret_cast<GetAPIVersionFunc>(dlsym(handle, "getAPIVersion"));
        
        const char* error_msg = dlerror();
        if (error_msg || !createFunc || !destroyFunc || !getInfoFunc || !getAPIVersionFunc) {
            LOG_ERROR("Failed to find required plugin functions in '{}': {}", 
                     path, error_msg ? error_msg : "Missing required functions");
            dlclose(handle);
            return nullptr;
        }
        
        // Check API version compatibility
        uint32_t plugin_api_version = getAPIVersionFunc();
        if (plugin_api_version != PLUGIN_API_VERSION) {
            LOG_ERROR("Plugin '{}' API version {} incompatible with expected version {}", 
                     path, plugin_api_version, PLUGIN_API_VERSION);
            dlclose(handle);
            return nullptr;
        }
        
        // Get plugin info
        PluginInfo info = getInfoFunc();
        LOG_INFO("Loading plugin '{}' version {} by {}", info.name, info.version, info.author);
        
        // Create plugin instance
        Plugin* plugin_ptr = createFunc();
        if (!plugin_ptr) {
            LOG_ERROR("Plugin '{}' createPlugin() returned null", path);
            dlclose(handle);
            return nullptr;
        }
        
        // Create wrapper that will manage the plugin lifetime
        auto loaded_plugin = std::make_unique<LoadedPlugin>();
        loaded_plugin->handle = handle;
        loaded_plugin->plugin = std::unique_ptr<Plugin>(plugin_ptr);
        loaded_plugin->info = info;
        
        // Return the plugin (ownership transferred)
        auto result = std::move(loaded_plugin->plugin);
        loaded_plugins_.push_back(std::move(loaded_plugin));
        
        return result;
    }
    
    bool validatePlugin(const std::string& path) {
        // Try to load and immediately unload to validate
        void* handle = dlopen(path.c_str(), RTLD_LAZY | RTLD_LOCAL);
        if (!handle) {
            return false;
        }
        
        // Check for required functions
        bool valid = true;
        valid &= (dlsym(handle, "createPlugin") != nullptr);
        valid &= (dlsym(handle, "destroyPlugin") != nullptr);
        valid &= (dlsym(handle, "getPluginInfo") != nullptr);
        valid &= (dlsym(handle, "getAPIVersion") != nullptr);
        
        // Check API version
        if (valid) {
            auto getAPIVersionFunc = reinterpret_cast<GetAPIVersionFunc>(dlsym(handle, "getAPIVersion"));
            valid &= (getAPIVersionFunc() == PLUGIN_API_VERSION);
        }
        
        dlclose(handle);
        return valid;
    }
    
    PluginInfo getPluginInfo(const std::string& path) {
        void* handle = dlopen(path.c_str(), RTLD_LAZY | RTLD_LOCAL);
        if (!handle) {
            return PluginInfo{}; // Return empty info on error
        }
        
        auto getInfoFunc = reinterpret_cast<GetPluginInfoFunc>(dlsym(handle, "getPluginInfo"));
        PluginInfo info{};
        if (getInfoFunc) {
            info = getInfoFunc();
        }
        
        dlclose(handle);
        return info;
    }
    
    std::vector<std::string> findPlugins(const std::string& directory) {
        std::vector<std::string> plugin_paths;
        
        try {
            if (!std::filesystem::exists(directory)) {
                LOG_WARN("Plugin directory '{}' does not exist", directory);
                return plugin_paths;
            }
            
            for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
                if (entry.is_regular_file()) {
                    const auto& path = entry.path();
                    const auto extension = path.extension().string();
                    
                    // Look for shared library files
                    if (extension == ".so" || extension == ".dylib" || extension == ".dll") {
                        std::string path_str = path.string();
                        if (validatePlugin(path_str)) {
                            plugin_paths.push_back(path_str);
                            LOG_DEBUG("Found valid plugin: {}", path_str);
                        }
                    }
                }
            }
        } catch (const std::filesystem::filesystem_error& e) {
            LOG_ERROR("Error scanning plugin directory '{}': {}", directory, e.what());
        }
        
        return plugin_paths;
    }
};

// Static instance for the implementation
static PluginLoaderImpl impl;

// Static method implementations
std::unique_ptr<Plugin> PluginLoader::loadPlugin(const std::string& path) {
    return impl.loadPlugin(path);
}

bool PluginLoader::validatePlugin(const std::string& path) {
    return impl.validatePlugin(path);
}

PluginInfo PluginLoader::getPluginInfo(const std::string& path) {
    return impl.getPluginInfo(path);
}

std::vector<std::string> PluginLoader::findPlugins(const std::string& directory) {
    return impl.findPlugins(directory);
}

} // namespace m5tab5::emulator