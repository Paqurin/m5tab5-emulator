#pragma once

#include "types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/peripherals/peripheral_manager.hpp"
#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/plugin/plugin_manager.hpp"
#include "emulator/debug/debugger.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/memory/boot_rom.hpp"
#include "emulator/firmware/elf_loader.hpp"
#include "emulator/storage/flash_controller.hpp"
#include "emulator/storage/spiffs_filesystem.hpp"
#include "emulator/storage/vfs_manager.hpp"
#include "emulator/storage/ota_manager.hpp"

#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <typeindex>

namespace m5tab5::emulator {

// EmulatorState enum for tracking emulator lifecycle
enum class EmulatorState {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING,
    PAUSED,
    STOPPING,
    STOPPED,
    SHUTDOWN,
    ERROR
};

/**
 * @brief Core emulator engine that orchestrates all components
 * 
 * This class manages the overall emulation lifecycle, coordinates between
 * different subsystems, and provides the main execution loop.
 */
class EmulatorCore {
public:
    explicit EmulatorCore(const Configuration& config);
    ~EmulatorCore();

    // Core lifecycle  
    Result<void> initialize(const Configuration& config);
    Result<void> start();
    Result<void> pause();
    Result<void> resume();
    Result<void> stop();
    Result<void> shutdown();
    Result<void> reset();
    
    // ESP32-P4 boot sequence
    Result<void> cold_boot();
    Result<void> warm_restart();
    Result<void> execute_complete_boot_sequence();
    
    // Firmware loading and execution
    Result<firmware::ELFLoader::LoadingResult> load_elf_application(const std::string& file_path,
                                                                    firmware::ELFLoader::ProgressCallback progress_callback = nullptr);
    Result<firmware::ELFLoader::LoadingResult> load_esp_application_image(const std::string& file_path,
                                                                          firmware::ELFLoader::ProgressCallback progress_callback = nullptr);
    Result<void> set_application_entry_point(Address entry_point);
    Result<void> start_application_execution();

    // State management
    EmulatorState get_state() const;
    Cycles get_cycles_executed() const;
    double get_execution_speed() const;

    // Component access - Public interface for plugins and external systems
    std::shared_ptr<MemoryController> getMemoryController() const;
    
    // Template-based component access using registry pattern
    template<typename T>
    std::shared_ptr<T> getComponent(const std::string& name) const;
    
    // Non-template version returning void* for generic access (plugins)
    std::shared_ptr<void> getComponent(const std::string& name) const;
    
    // Overloaded version for type-based lookup (no name required)
    template<typename T>
    std::shared_ptr<T> getComponent() const;
    
    // Performance monitoring and timing
    Cycles getCurrentCycle() const;

private:
    // Main execution loops
    void execution_loop();
    void graphics_loop();

    // Configuration and state
    Configuration config_;
    EmulatorState state_;
    std::atomic<bool> running_{false};
    
    // Target frequency and performance tracking
    uint32_t target_frequency_;
    Cycles cycles_executed_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Core components (to match implementation)
    std::unique_ptr<MemoryController> memory_controller_;
    std::unique_ptr<DualCoreManager> cpu_manager_; 
    std::unique_ptr<PeripheralManager> peripheral_manager_;
    std::unique_ptr<GraphicsEngine> graphics_engine_;
    std::unique_ptr<PluginManager> plugin_manager_;
    std::unique_ptr<Debugger> debugger_;
    std::unique_ptr<BootROM> boot_rom_;
    std::unique_ptr<firmware::ELFLoader> elf_loader_;
    
    // Storage subsystem components
    std::unique_ptr<storage::FlashController> flash_controller_;
    std::unique_ptr<storage::SPIFFSFileSystem> spiffs_filesystem_;
    std::unique_ptr<storage::VFSManager> vfs_manager_;
    std::unique_ptr<storage::OTAManager> ota_manager_;
    
    // Execution threads
    std::thread execution_thread_;
    std::thread graphics_thread_;
    
    // Component registry for type-safe access
    mutable std::mutex component_registry_mutex_;
    std::unordered_map<std::string, std::shared_ptr<void>> component_registry_;
    std::unordered_map<std::type_index, std::shared_ptr<void>> type_registry_;
    
    // Component registration helpers (internal use)
    template<typename T>
    void registerComponent(const std::string& name, std::shared_ptr<T> component);
    
    template<typename T>
    void registerComponent(std::shared_ptr<T> component);
    
    // Initialize all components and populate registry
    Result<void> initializeComponents();
    void shutdownComponents();
};

//
// Template method implementations (must be in header for templates)
//

template<typename T>
std::shared_ptr<T> EmulatorCore::getComponent(const std::string& name) const {
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    auto it = component_registry_.find(name);
    if (it == component_registry_.end()) {
        return nullptr;
    }
    
    // Use dynamic_pointer_cast for safe type conversion
    return std::dynamic_pointer_cast<T>(
        std::static_pointer_cast<T>(it->second)
    );
}

template<typename T>
std::shared_ptr<T> EmulatorCore::getComponent() const {
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    std::type_index type_key = std::type_index(typeid(T));
    auto it = type_registry_.find(type_key);
    if (it == type_registry_.end()) {
        return nullptr;
    }
    
    // Use dynamic_pointer_cast for safe type conversion
    return std::dynamic_pointer_cast<T>(
        std::static_pointer_cast<T>(it->second)
    );
}

template<typename T>
void EmulatorCore::registerComponent(const std::string& name, std::shared_ptr<T> component) {
    if (!component) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    // Store in name registry
    component_registry_[name] = std::static_pointer_cast<void>(component);
    
    // Store in type registry
    std::type_index type_key = std::type_index(typeid(T));
    type_registry_[type_key] = std::static_pointer_cast<void>(component);
}

template<typename T>
void EmulatorCore::registerComponent(std::shared_ptr<T> component) {
    if (!component) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    // Store only in type registry (no name)
    std::type_index type_key = std::type_index(typeid(T));
    type_registry_[type_key] = std::static_pointer_cast<void>(component);
}

} // namespace m5tab5::emulator