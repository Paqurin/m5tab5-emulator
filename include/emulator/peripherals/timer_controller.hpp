#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/config/configuration.hpp"

namespace m5tab5::emulator {

/**
 * @brief Timer controller for ESP32-P4 timer peripherals
 * 
 * Manages hardware timers, watchdog timers, and system tick timing
 * for CPU core synchronization and peripheral timing.
 */
class TimerController {
public:
    TimerController();
    ~TimerController();

    // Lifecycle management
    Result<void> initialize(const Configuration& config);
    Result<void> start();
    void stop();
    Result<void> reset();
    void shutdown();

    // Timer operations
    void update(Cycles cycles);
    Result<void> set_timeout(CoreId core_id, u32 timeout_cycles);
    
    // State queries
    bool is_initialized() const { return initialized_; }

private:
    bool initialized_ = false;
    u64 total_cycles_ = 0;
    
    // Timer state
    struct TimerState {
        bool active = false;
        u32 timeout_cycles = 0;
        u32 elapsed_cycles = 0;
        CoreId target_core;
    };
    
    std::array<TimerState, 8> timers_; // 8 hardware timers
};

} // namespace m5tab5::emulator