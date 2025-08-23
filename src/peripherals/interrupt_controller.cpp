#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("InterruptController");

InterruptController::InterruptController()
    : initialized_(false) {
    interrupts_.resize(MAX_INTERRUPTS);
    COMPONENT_LOG_DEBUG("InterruptController created");
}

InterruptController::~InterruptController() {
    if (initialized_) {
        auto result = shutdown();
        if (!result) {
            COMPONENT_LOG_ERROR("Failed to shutdown interrupt controller in destructor");
        }
    }
    COMPONENT_LOG_DEBUG("InterruptController destroyed");
}

Result<void> InterruptController::initialize(const Configuration& config) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Interrupt controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing interrupt controller");
    
    // Initialize all interrupt entries
    for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
        interrupts_[i] = InterruptEntry{};
    }
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Interrupt controller initialized successfully");
    return {};
}

Result<void> InterruptController::start() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    COMPONENT_LOG_INFO("Starting interrupt controller");
    return {};
}

Result<void> InterruptController::stop() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Stopping interrupt controller");
    
    // Clear all pending interrupts
    for (auto& interrupt : interrupts_) {
        interrupt.pending = false;
    }
    
    return {};
}

Result<void> InterruptController::reset() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    COMPONENT_LOG_INFO("Resetting interrupt controller");
    
    // Reset all interrupt entries to default state
    for (auto& interrupt : interrupts_) {
        interrupt.enabled = false;
        interrupt.pending = false;
        interrupt.priority = InterruptPriority::MEDIUM;
        interrupt.handler = nullptr;
    }
    
    return {};
}

Result<void> InterruptController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down interrupt controller");
    
    // Stop and clear everything
    RETURN_IF_ERROR(stop());
    
    initialized_ = false;
    return {};
}

Result<void> InterruptController::register_handler(InterruptType type, InterruptHandler handler) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].handler = handler;
    COMPONENT_LOG_DEBUG("Registered handler for interrupt type {}", static_cast<int>(type));
    return {};
}

Result<void> InterruptController::unregister_handler(InterruptType type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].handler = nullptr;
    COMPONENT_LOG_DEBUG("Unregistered handler for interrupt type {}", static_cast<int>(type));
    return {};
}

Result<void> InterruptController::trigger_interrupt(InterruptType type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    if (interrupts_[index].enabled) {
        interrupts_[index].pending = true;
        COMPONENT_LOG_TRACE("Triggered interrupt type {}", static_cast<int>(type));
    } else {
        COMPONENT_LOG_DEBUG("Interrupt type {} triggered but disabled", static_cast<int>(type));
    }
    
    return {};
}

Result<void> InterruptController::trigger_interrupt(u32 core_id, InterruptType type, u32 data) {
    // For now, we'll ignore the core_id and data parameters and delegate to simple trigger
    // In a more sophisticated implementation, we'd handle per-core interrupts differently
    COMPONENT_LOG_TRACE("Triggering interrupt type {} for core {} with data 0x{:x}", 
                       static_cast<int>(type), core_id, data);
    return trigger_interrupt(type);
}

Result<void> InterruptController::clear_interrupt(InterruptType type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].pending = false;
    COMPONENT_LOG_TRACE("Cleared interrupt type {}", static_cast<int>(type));
    return {};
}

Result<void> InterruptController::enable_interrupt(InterruptType type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].enabled = true;
    COMPONENT_LOG_DEBUG("Enabled interrupt type {}", static_cast<int>(type));
    return {};
}

Result<void> InterruptController::disable_interrupt(InterruptType type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].enabled = false;
    COMPONENT_LOG_DEBUG("Disabled interrupt type {}", static_cast<int>(type));
    return {};
}

Result<void> InterruptController::set_priority(InterruptType type, InterruptPriority priority) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    interrupts_[index].priority = priority;
    COMPONENT_LOG_DEBUG("Set priority {} for interrupt type {}", 
                       static_cast<int>(priority), static_cast<int>(type));
    return {};
}

Result<InterruptPriority> InterruptController::get_priority(InterruptType type) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid interrupt type"));
    }
    
    return interrupts_[index].priority;
}

bool InterruptController::is_interrupt_enabled(InterruptType type) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS || !initialized_) {
        return false;
    }
    
    return interrupts_[index].enabled;
}

bool InterruptController::is_interrupt_pending(InterruptType type) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    size_t index = static_cast<size_t>(type);
    if (index >= MAX_INTERRUPTS || !initialized_) {
        return false;
    }
    
    return interrupts_[index].pending;
}

void InterruptController::process_interrupts() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Find and process interrupts by priority
    std::vector<std::pair<InterruptType, InterruptPriority>> pending_interrupts;
    
    for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
        if (interrupts_[i].enabled && interrupts_[i].pending && interrupts_[i].handler) {
            pending_interrupts.emplace_back(static_cast<InterruptType>(i), interrupts_[i].priority);
        }
    }
    
    // Sort by priority (HIGHEST = 0 comes first)
    std::sort(pending_interrupts.begin(), pending_interrupts.end(),
              [](const auto& a, const auto& b) {
                  return static_cast<int>(a.second) < static_cast<int>(b.second);
              });
    
    // Process interrupts in priority order
    for (const auto& [type, priority] : pending_interrupts) {
        size_t index = static_cast<size_t>(type);
        
        // Call the handler
        interrupts_[index].handler(type);
        
        // Clear the pending flag after processing
        interrupts_[index].pending = false;
        
        COMPONENT_LOG_TRACE("Processed interrupt type {} with priority {}", 
                           static_cast<int>(type), static_cast<int>(priority));
    }
}

Result<void> InterruptController::clear_processed_interrupts(u32 core_id) {
    // For the current implementation, we clear all pending interrupts
    // In a multi-core implementation, this would be core-specific
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Interrupt controller not initialized"));
    }
    
    for (auto& interrupt : interrupts_) {
        interrupt.pending = false;
    }
    
    COMPONENT_LOG_DEBUG("Cleared processed interrupts for core {}", core_id);
    return {};
}

std::vector<InterruptType> InterruptController::get_pending_interrupts(u32 core_id) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    std::vector<InterruptType> pending;
    
    if (!initialized_) {
        return pending;
    }
    
    for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
        if (interrupts_[i].enabled && interrupts_[i].pending) {
            pending.push_back(static_cast<InterruptType>(i));
        }
    }
    
    return pending;
}

void InterruptController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        COMPONENT_LOG_INFO("Interrupt controller not initialized");
        return;
    }
    
    COMPONENT_LOG_INFO("=== Interrupt Controller Status ===");
    
    size_t enabled_count = 0;
    size_t pending_count = 0;
    size_t with_handlers = 0;
    
    for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
        if (interrupts_[i].enabled) {
            enabled_count++;
        }
        if (interrupts_[i].pending) {
            pending_count++;
        }
        if (interrupts_[i].handler) {
            with_handlers++;
        }
    }
    
    COMPONENT_LOG_INFO("Total interrupts: {}", MAX_INTERRUPTS);
    COMPONENT_LOG_INFO("Enabled: {}", enabled_count);
    COMPONENT_LOG_INFO("Pending: {}", pending_count);
    COMPONENT_LOG_INFO("With handlers: {}", with_handlers);
    
    if (pending_count > 0) {
        COMPONENT_LOG_INFO("Pending interrupts:");
        for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
            if (interrupts_[i].pending) {
                COMPONENT_LOG_INFO("  Type {}: priority {}, enabled {}", 
                                  i, static_cast<int>(interrupts_[i].priority), 
                                  interrupts_[i].enabled);
            }
        }
    }
}

}  // namespace m5tab5::emulator