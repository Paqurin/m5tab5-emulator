#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <functional>
#include <vector>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>

namespace m5tab5::emulator::utils {

/**
 * @brief Manages orderly shutdown of emulator components
 * 
 * This class ensures that all components are shutdown in the correct order
 * and prevents hanging during application exit.
 */
class ShutdownManager {
public:
    using ShutdownCallback = std::function<void()>;
    
    enum class Priority {
        Critical = 0,   // Graphics, UI components
        High = 1,       // Network, audio
        Normal = 2,     // Peripherals, memory
        Low = 3,        // Logging, configuration
        Cleanup = 4     // Final cleanup tasks
    };

    static ShutdownManager& instance();
    
    // Register components for shutdown
    void register_callback(Priority priority, const std::string& name, ShutdownCallback callback);
    void unregister_callback(const std::string& name);
    
    // Shutdown control
    void request_shutdown();
    bool is_shutdown_requested() const { return shutdown_requested_.load(); }
    
    void execute_shutdown(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    void force_shutdown();
    
    // Emergency shutdown for signal handlers
    void emergency_shutdown();

private:
    struct CallbackInfo {
        Priority priority;
        std::string name;
        ShutdownCallback callback;
    };
    
    std::vector<CallbackInfo> callbacks_;
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> shutdown_complete_{false};
    
    ShutdownManager() = default;
    ~ShutdownManager() = default;
    
    void execute_priority_group(Priority priority, std::chrono::milliseconds timeout);
    void wait_for_threads();
};

/**
 * @brief RAII helper for automatic shutdown registration
 */
class ShutdownGuard {
public:
    ShutdownGuard(ShutdownManager::Priority priority, const std::string& name, 
                  ShutdownManager::ShutdownCallback callback);
    ~ShutdownGuard();
    
    ShutdownGuard(const ShutdownGuard&) = delete;
    ShutdownGuard& operator=(const ShutdownGuard&) = delete;
    
    ShutdownGuard(ShutdownGuard&& other) noexcept;
    ShutdownGuard& operator=(ShutdownGuard&& other) noexcept;

private:
    std::string name_;
    bool registered_ = false;
};

// Convenience macros for shutdown registration
#define REGISTER_SHUTDOWN(priority, name, callback) \
    static auto shutdown_guard_##__LINE__ = utils::ShutdownGuard(priority, name, callback)

#define REGISTER_SHUTDOWN_CRITICAL(name, callback) \
    REGISTER_SHUTDOWN(utils::ShutdownManager::Priority::Critical, name, callback)

#define REGISTER_SHUTDOWN_HIGH(name, callback) \
    REGISTER_SHUTDOWN(utils::ShutdownManager::Priority::High, name, callback)

#define REGISTER_SHUTDOWN_NORMAL(name, callback) \
    REGISTER_SHUTDOWN(utils::ShutdownManager::Priority::Normal, name, callback)

} // namespace m5tab5::emulator::utils