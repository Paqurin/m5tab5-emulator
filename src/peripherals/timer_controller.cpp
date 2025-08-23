#include "emulator/peripherals/timer_controller.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("TimerController");

TimerController::TimerController() {
    COMPONENT_LOG_DEBUG("TimerController created");
}

TimerController::~TimerController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("TimerController destroyed");
}

Result<void> TimerController::initialize(const Configuration& config) {
    if (initialized_) {
        COMPONENT_LOG_WARN("TimerController already initialized");
        return {};
    }
    
    COMPONENT_LOG_INFO("Initializing TimerController");
    initialized_ = true;
    total_cycles_ = 0;
    
    // Initialize all timers to inactive state
    for (auto& timer : timers_) {
        timer.active = false;
        timer.timeout_cycles = 0;
        timer.elapsed_cycles = 0;
        timer.target_core = CoreId::CORE_0;
    }
    
    return {};
}

Result<void> TimerController::start() {
    if (!initialized_) {
        COMPONENT_LOG_ERROR("TimerController not initialized");
        return unexpected(MAKE_ERROR(INVALID_STATE, "TimerController not initialized"));
    }
    
    COMPONENT_LOG_INFO("Starting TimerController");
    return {};
}

void TimerController::stop() {
    if (!initialized_) {
        return;
    }
    
    COMPONENT_LOG_INFO("Stopping TimerController");
    
    // Stop all active timers
    for (auto& timer : timers_) {
        timer.active = false;
    }
}

Result<void> TimerController::reset() {
    if (!initialized_) {
        COMPONENT_LOG_WARN("TimerController not initialized");
        return unexpected(MAKE_ERROR(INVALID_STATE, "TimerController not initialized"));
    }
    
    COMPONENT_LOG_DEBUG("Resetting TimerController");
    total_cycles_ = 0;
    
    // Reset all timers
    for (auto& timer : timers_) {
        timer.active = false;
        timer.elapsed_cycles = 0;
    }
    
    return {};
}

void TimerController::shutdown() {
    if (!initialized_) {
        return;
    }
    
    COMPONENT_LOG_DEBUG("Shutting down TimerController");
    stop();
    initialized_ = false;
}

void TimerController::update(Cycles cycles) {
    if (!initialized_) {
        return;
    }
    
    total_cycles_ += cycles;
    
    // Update all active timers
    for (auto& timer : timers_) {
        if (timer.active) {
            timer.elapsed_cycles += static_cast<u32>(cycles);
            
            // Check for timeout
            if (timer.elapsed_cycles >= timer.timeout_cycles) {
                COMPONENT_LOG_DEBUG("Timer timeout reached for core {}", static_cast<int>(timer.target_core));
                timer.active = false;
                timer.elapsed_cycles = 0;
                // TODO: Trigger interrupt or callback
            }
        }
    }
}

Result<void> TimerController::set_timeout(CoreId core_id, u32 timeout_cycles) {
    if (!initialized_) {
        COMPONENT_LOG_ERROR("TimerController not initialized");
        return unexpected(MAKE_ERROR(INVALID_STATE, "TimerController not initialized"));
    }
    
    // Find available timer
    for (auto& timer : timers_) {
        if (!timer.active) {
            timer.active = true;
            timer.timeout_cycles = timeout_cycles;
            timer.elapsed_cycles = 0;
            timer.target_core = core_id;
            
            COMPONENT_LOG_DEBUG("Set timeout {} cycles for core {}", timeout_cycles, static_cast<int>(core_id));
            return {};
        }
    }
    
    COMPONENT_LOG_ERROR("No available timers");
    return unexpected(MAKE_ERROR(SYSTEM_RESOURCE_EXHAUSTED, "No available timers"));
}

} // namespace m5tab5::emulator