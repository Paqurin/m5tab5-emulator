#pragma once

#include "emulator/core/types.hpp"
#include "emulator/peripherals/peripheral_base.hpp"

#include <string>
#include <memory>
#include <functional>

namespace m5tab5::emulator {

/**
 * @brief Plugin interface for extending emulator functionality
 * 
 * Enables dynamic loading of:
 * - Custom peripheral implementations
 * - Protocol analyzers
 * - Debug tools
 * - Test automation scripts
 * - Hardware variants
 */

// Plugin API version for compatibility checking
constexpr uint32_t PLUGIN_API_VERSION = 1;

// Plugin types
enum class PluginType {
    Peripheral,     // Hardware peripheral implementation
    Analyzer,       // Protocol/signal analyzer
    Debugger,       // Debug tool extension
    Automation,     // Test automation
    Visualization,  // Custom visualization
    Other
};

// Plugin metadata
struct PluginInfo {
    std::string name;
    std::string version;
    std::string author;
    std::string description;
    PluginType type;
    uint32_t api_version;
    std::vector<std::string> dependencies;
    std::string license;
};

// Host interface for plugins to interact with emulator
class PluginHost {
public:
    virtual ~PluginHost() = default;

    // Memory access
    virtual EmulatorError readMemory(Address address, uint32_t& value) = 0;
    virtual EmulatorError writeMemory(Address address, uint32_t value) = 0;

    // Register access
    virtual EmulatorError readRegister(const std::string& peripheral, 
                                      Address address, uint32_t& value) = 0;
    virtual EmulatorError writeRegister(const std::string& peripheral,
                                       Address address, uint32_t value) = 0;

    // Interrupt handling
    virtual EmulatorError triggerInterrupt(uint32_t interrupt_id) = 0;
    virtual EmulatorError setInterruptHandler(uint32_t interrupt_id,
                                             std::function<void()> handler) = 0;

    // Timing and synchronization
    virtual ClockCycle getCurrentCycle() = 0;
    virtual EmulatorError scheduleCallback(ClockCycle target_cycle,
                                          std::function<void()> callback) = 0;

    // Logging and debugging
    virtual void log(const std::string& level, const std::string& message) = 0;
    virtual EmulatorError setBreakpoint(Address address) = 0;
    virtual EmulatorError removeBreakpoint(Address address) = 0;

    // Configuration access
    virtual std::string getConfigValue(const std::string& section, 
                                      const std::string& key) = 0;
    virtual void setConfigValue(const std::string& section,
                               const std::string& key,
                               const std::string& value) = 0;

    // File I/O
    virtual EmulatorError loadFile(const std::string& filename,
                                  std::vector<uint8_t>& data) = 0;
    virtual EmulatorError saveFile(const std::string& filename,
                                  const std::vector<uint8_t>& data) = 0;

    // GUI integration (if available)
    virtual void* getDisplayContext() = 0; // Returns SDL_Renderer* or similar
    virtual EmulatorError addUIElement(const std::string& name,
                                      void* element) = 0;
};

// Base plugin interface
class Plugin {
public:
    virtual ~Plugin() = default;

    // Plugin lifecycle
    virtual EmulatorError initialize(PluginHost* host) = 0;
    virtual EmulatorError shutdown() = 0;
    virtual EmulatorError tick(ClockCycle cycle) = 0;

    // Plugin information
    virtual PluginInfo getInfo() const = 0;
    virtual std::string getConfigSchema() const = 0; // JSON schema for config

    // Configuration
    virtual EmulatorError configure(const std::string& config_json) = 0;
    virtual std::string getConfiguration() const = 0;

    // Plugin-specific interface (override in derived classes)
    virtual void* getInterface(const std::string& interface_name) {
        return nullptr;
    }
};

// Peripheral plugin interface
class PeripheralPlugin : public Plugin {
public:
    // Create the peripheral instance
    virtual std::unique_ptr<PeripheralBase> createPeripheral() = 0;
    
    // Get supported register addresses
    virtual std::vector<Address> getSupportedAddresses() = 0;
    
    // Get interrupt IDs this peripheral can generate
    virtual std::vector<uint32_t> getInterruptIds() = 0;
};

// Analyzer plugin interface
class AnalyzerPlugin : public Plugin {
public:
    // Capture and analyze signals
    virtual EmulatorError startCapture() = 0;
    virtual EmulatorError stopCapture() = 0;
    virtual EmulatorError analyzeSignal(const std::string& signal_name,
                                       const std::vector<uint8_t>& data) = 0;
    
    // Results and visualization
    virtual std::string getAnalysisResults() = 0; // JSON format
    virtual void* getVisualization() = 0; // Plugin-specific visualization
};

// Standard plugin entry points (C interface for dynamic loading)
extern "C" {
    // Required exports for all plugins
    typedef Plugin* (*CreatePluginFunc)();
    typedef void (*DestroyPluginFunc)(Plugin*);
    typedef PluginInfo (*GetPluginInfoFunc)();
    typedef uint32_t (*GetAPIVersionFunc)();

    // Plugin factory functions (must be implemented by plugins)
    Plugin* createPlugin();
    void destroyPlugin(Plugin* plugin);
    PluginInfo getPluginInfo();
    uint32_t getAPIVersion();
}

// Helper macros for plugin implementation
#define DECLARE_PLUGIN(ClassName) \
    extern "C" { \
        Plugin* createPlugin() { return new ClassName(); } \
        void destroyPlugin(Plugin* plugin) { delete plugin; } \
        uint32_t getAPIVersion() { return PLUGIN_API_VERSION; } \
    }

#define IMPLEMENT_PLUGIN_INFO(name, version, author, desc, type) \
    extern "C" { \
        PluginInfo getPluginInfo() { \
            return PluginInfo{ \
                name, version, author, desc, type, \
                PLUGIN_API_VERSION, {}, "MIT" \
            }; \
        } \
    }

// Plugin manager utilities
class PluginLoader {
public:
    static std::unique_ptr<Plugin> loadPlugin(const std::string& path);
    static bool validatePlugin(const std::string& path);
    static PluginInfo getPluginInfo(const std::string& path);
    static std::vector<std::string> findPlugins(const std::string& directory);
};

} // namespace m5tab5::emulator