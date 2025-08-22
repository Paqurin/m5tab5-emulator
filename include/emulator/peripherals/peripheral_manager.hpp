#pragma once

#include "emulator/core/types.hpp"
#include "emulator/peripherals/peripheral_base.hpp"
#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/peripherals/display_controller.hpp"
#include "emulator/peripherals/audio_controller.hpp"
#include "emulator/peripherals/camera_controller.hpp"
#include "emulator/peripherals/imu_controller.hpp"
#include "emulator/peripherals/communication_controller.hpp"

#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <mutex>
#include <queue>

namespace m5tab5::emulator {

/**
 * @brief Central manager for all peripheral devices
 * 
 * Coordinates all peripheral controllers and provides:
 * - Unified peripheral registration and discovery
 * - Memory-mapped I/O routing
 * - Interrupt aggregation and routing
 * - Plugin system for extensibility
 * - Cross-peripheral communication
 */
class PeripheralManager {
public:
    using InterruptCallback = std::function<void(uint32_t interrupt_id, bool level)>;

    explicit PeripheralManager(InterruptCallback interrupt_callback);
    ~PeripheralManager();

    // Lifecycle management
    EmulatorError initialize();
    EmulatorError reset();
    EmulatorError shutdown();

    // Peripheral registration
    EmulatorError registerPeripheral(std::unique_ptr<PeripheralBase> peripheral);
    EmulatorError unregisterPeripheral(const std::string& name);
    
    // Peripheral access
    PeripheralBase* getPeripheral(const std::string& name);
    template<typename T>
    T* getPeripheral(const std::string& name) {
        return dynamic_cast<T*>(getPeripheral(name));
    }

    // Specialized peripheral access
    GPIOController& getGPIO() { return *gpio_; }
    DisplayController& getDisplay() { return *display_; }
    AudioController& getAudio() { return *audio_; }
    CameraController& getCamera() { return *camera_; }
    IMUController& getIMU() { return *imu_; }
    CommunicationController& getCommunication() { return *communication_; }

    // Memory-mapped I/O
    EmulatorError readRegister(Address address, uint32_t& value);
    EmulatorError writeRegister(Address address, uint32_t value);
    
    // Interrupt handling
    EmulatorError triggerInterrupt(uint32_t interrupt_id);
    EmulatorError setInterruptEnabled(uint32_t interrupt_id, bool enabled);
    bool isInterruptPending(uint32_t interrupt_id) const;

    // Timing and synchronization
    EmulatorError tick(ClockCycle cycle); // Called each emulation cycle
    EmulatorError syncPeripherals(); // Synchronize all peripherals

    // Plugin system
    EmulatorError loadPlugin(const std::string& plugin_path);
    EmulatorError unloadPlugin(const std::string& plugin_name);
    std::vector<std::string> getLoadedPlugins() const;

    // Configuration
    EmulatorError configurePeripheral(const std::string& name, 
                                     const std::string& config_json);
    
    // Statistics and monitoring
    struct PeripheralStatistics {
        std::unordered_map<std::string, uint64_t> register_reads;
        std::unordered_map<std::string, uint64_t> register_writes;
        std::unordered_map<uint32_t, uint64_t> interrupts_triggered;
        uint64_t total_operations = 0;
    };

    const PeripheralStatistics& getStatistics() const { return stats_; }
    void resetStatistics();

    // Cross-peripheral communication
    EmulatorError sendMessage(const std::string& from_peripheral,
                             const std::string& to_peripheral,
                             const std::string& message_type,
                             const void* data, size_t size);

private:
    // Core peripheral initialization
    EmulatorError initializeCorePeripherals();
    
    // Address routing
    PeripheralBase* findPeripheralByAddress(Address address);
    
    // Plugin management
    struct PluginInfo {
        std::string name;
        std::string path;
        void* handle;
        std::unique_ptr<PeripheralBase> peripheral;
    };

    // Interrupt management
    struct InterruptState {
        bool enabled = false;
        bool pending = false;
        std::string source_peripheral;
    };

    // Core components
    InterruptCallback interrupt_callback_;
    
    // Core peripherals (always present)
    std::unique_ptr<GPIOController> gpio_;
    std::unique_ptr<DisplayController> display_;
    std::unique_ptr<AudioController> audio_;
    std::unique_ptr<CameraController> camera_;
    std::unique_ptr<IMUController> imu_;
    std::unique_ptr<CommunicationController> communication_;

    // All peripherals registry
    std::unordered_map<std::string, std::unique_ptr<PeripheralBase>> peripherals_;
    std::unordered_map<Address, PeripheralBase*> address_map_;

    // Plugin system
    std::unordered_map<std::string, PluginInfo> loaded_plugins_;

    // Interrupt management
    static constexpr uint32_t MAX_INTERRUPTS = 64;
    std::array<InterruptState, MAX_INTERRUPTS> interrupt_states_;
    mutable std::mutex interrupt_mutex_;

    // Statistics
    mutable std::mutex stats_mutex_;
    PeripheralStatistics stats_;

    // Cross-peripheral messaging
    struct Message {
        std::string from;
        std::string to;
        std::string type;
        std::vector<uint8_t> data;
    };
    
    std::queue<Message> message_queue_;
    std::mutex message_mutex_;

    // Performance optimization
    mutable Address last_address_ = 0;
    mutable PeripheralBase* last_peripheral_ = nullptr;
};

} // namespace m5tab5::emulator