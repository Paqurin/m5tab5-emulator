#include "emulator/utils/shutdown_manager.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <future>
#include <mutex>

namespace m5tab5::emulator::utils {

DECLARE_LOGGER("ShutdownManager");

ShutdownManager& ShutdownManager::instance() {
    static ShutdownManager instance;
    return instance;
}

void ShutdownManager::register_callback(Priority priority, const std::string& name, ShutdownCallback callback) {
    if (!callback) {
        COMPONENT_LOG_WARN("Attempted to register null shutdown callback for '{}'", name);
        return;
    }
    
    callbacks_.push_back({priority, name, callback});
    
    // Sort by priority (Critical = 0 executes first, Cleanup = 4 executes last)
    std::sort(callbacks_.begin(), callbacks_.end(), 
        [](const CallbackInfo& a, const CallbackInfo& b) {
            return static_cast<int>(a.priority) < static_cast<int>(b.priority);
        });
    
    COMPONENT_LOG_DEBUG("Registered shutdown callback '{}' with priority {}", 
                       name, static_cast<int>(priority));
}

void ShutdownManager::unregister_callback(const std::string& name) {
    auto it = std::remove_if(callbacks_.begin(), callbacks_.end(),
        [&name](const CallbackInfo& info) { return info.name == name; });
    
    if (it != callbacks_.end()) {
        callbacks_.erase(it, callbacks_.end());
        COMPONENT_LOG_DEBUG("Unregistered shutdown callback '{}'", name);
    }
}

void ShutdownManager::request_shutdown() {
    bool expected = false;
    if (shutdown_requested_.compare_exchange_strong(expected, true)) {
        COMPONENT_LOG_INFO("Shutdown requested");
    }
}

void ShutdownManager::execute_shutdown(std::chrono::milliseconds timeout) {
    if (shutdown_complete_.load()) {
        COMPONENT_LOG_DEBUG("Shutdown already completed");
        return;
    }
    
    COMPONENT_LOG_INFO("Beginning coordinated shutdown with {}ms timeout", timeout.count());
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Execute callbacks by priority groups
    for (int priority = 0; priority <= static_cast<int>(Priority::Cleanup); ++priority) {
        auto remaining_time = timeout - std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        
        if (remaining_time <= std::chrono::milliseconds(0)) {
            COMPONENT_LOG_WARN("Shutdown timeout reached, forcing remaining shutdowns");
            force_shutdown();
            return;
        }
        
        execute_priority_group(static_cast<Priority>(priority), remaining_time);
    }
    
    shutdown_complete_.store(true);
    COMPONENT_LOG_INFO("Coordinated shutdown completed successfully");
}

void ShutdownManager::force_shutdown() {
    COMPONENT_LOG_WARN("Forcing immediate shutdown");
    
    // Execute all remaining callbacks without timeout
    for (const auto& callback : callbacks_) {
        try {
            COMPONENT_LOG_DEBUG("Force executing shutdown callback '{}'", callback.name);
            callback.callback();
        } catch (const std::exception& e) {
            COMPONENT_LOG_ERROR("Exception in force shutdown callback '{}': {}", 
                               callback.name, e.what());
        } catch (...) {
            COMPONENT_LOG_ERROR("Unknown exception in force shutdown callback '{}'", 
                               callback.name);
        }
    }
    
    shutdown_complete_.store(true);
}

void ShutdownManager::emergency_shutdown() {
    COMPONENT_LOG_ERROR("Emergency shutdown initiated");
    
    // Set flags immediately
    shutdown_requested_.store(true);
    
    // Execute only critical shutdown callbacks
    for (const auto& callback : callbacks_) {
        if (callback.priority == Priority::Critical) {
            try {
                callback.callback();
            } catch (...) {
                // Ignore all exceptions in emergency mode
            }
        }
    }
    
    shutdown_complete_.store(true);
}

void ShutdownManager::execute_priority_group(Priority priority, std::chrono::milliseconds timeout) {
    std::vector<std::future<void>> futures;
    std::mutex completion_mutex;
    std::vector<std::string> completed_callbacks;
    
    COMPONENT_LOG_DEBUG("Executing priority group {} with timeout {}ms", 
                       static_cast<int>(priority), timeout.count());
    
    // Launch all callbacks for this priority group
    for (const auto& callback : callbacks_) {
        if (callback.priority == priority) {
            auto future = std::async(std::launch::async, [&callback, &completion_mutex, &completed_callbacks]() {
                try {
                    callback.callback();
                    std::lock_guard<std::mutex> lock(completion_mutex);
                    completed_callbacks.push_back(callback.name);
                } catch (const std::exception& e) {
                    COMPONENT_LOG_ERROR("Exception in shutdown callback '{}': {}", 
                                       callback.name, e.what());
                } catch (...) {
                    COMPONENT_LOG_ERROR("Unknown exception in shutdown callback '{}'", 
                                       callback.name);
                }
            });
            futures.push_back(std::move(future));
        }
    }
    
    // Wait for all callbacks to complete or timeout
    auto deadline = std::chrono::steady_clock::now() + timeout;
    
    for (auto& future : futures) {
        auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::milliseconds(0)) {
            COMPONENT_LOG_WARN("Timeout waiting for priority group {} to complete", 
                               static_cast<int>(priority));
            break;
        }
        
        if (future.wait_for(remaining) != std::future_status::ready) {
            COMPONENT_LOG_WARN("Shutdown callback timed out in priority group {}", 
                               static_cast<int>(priority));
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(completion_mutex);
        COMPONENT_LOG_DEBUG("Priority group {} completed {} callbacks", 
                           static_cast<int>(priority), completed_callbacks.size());
    }
}

void ShutdownManager::wait_for_threads() {
    // This would be implemented if we tracked specific threads
    // For now, we rely on individual components to handle their thread cleanup
}

//
// ShutdownGuard implementation
//

ShutdownGuard::ShutdownGuard(ShutdownManager::Priority priority, const std::string& name, 
                             ShutdownManager::ShutdownCallback callback)
    : name_(name), registered_(true) {
    ShutdownManager::instance().register_callback(priority, name, callback);
}

ShutdownGuard::~ShutdownGuard() {
    if (registered_) {
        ShutdownManager::instance().unregister_callback(name_);
    }
}

ShutdownGuard::ShutdownGuard(ShutdownGuard&& other) noexcept
    : name_(std::move(other.name_)), registered_(other.registered_) {
    other.registered_ = false;
}

ShutdownGuard& ShutdownGuard::operator=(ShutdownGuard&& other) noexcept {
    if (this != &other) {
        if (registered_) {
            ShutdownManager::instance().unregister_callback(name_);
        }
        
        name_ = std::move(other.name_);
        registered_ = other.registered_;
        other.registered_ = false;
    }
    return *this;
}

} // namespace m5tab5::emulator::utils