#pragma once

#include "types.hpp"
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/peripherals/peripheral_manager.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/debug/debugger.hpp"

#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>

namespace m5tab5::emulator {

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
    EmulatorError initialize();
    EmulatorError start();
    EmulatorError pause();
    EmulatorError resume();
    EmulatorError stop();
    EmulatorError reset();

    // State management
    bool isRunning() const { return running_.load(); }
    bool isPaused() const { return paused_.load(); }
    ClockCycle getCurrentCycle() const { return current_cycle_.load(); }

    // Component access
    CPUCore& getCPU() { return *cpu_; }
    MemoryController& getMemory() { return *memory_; }
    PeripheralManager& getPeripherals() { return *peripherals_; }
    Debugger& getDebugger() { return *debugger_; }

    // Configuration
    const Configuration& getConfig() const { return config_; }
    EmulatorError updateConfig(const Configuration& new_config);

    // Performance monitoring
    struct Statistics {
        uint64_t cycles_executed = 0;
        uint64_t instructions_executed = 0;
        double cpu_utilization = 0.0;
        double real_time_factor = 0.0; // How fast compared to real hardware
    };
    
    Statistics getStatistics() const;
    void resetStatistics();

private:
    // Main execution loop
    void executionLoop();
    void updateTiming();

    // Component initialization
    EmulatorError initializeCPU();
    EmulatorError initializeMemory();
    EmulatorError initializePeripherals();

    // Configuration and state
    Configuration config_;
    
    // Core components
    std::unique_ptr<CPUCore> cpu_;
    std::unique_ptr<MemoryController> memory_;
    std::unique_ptr<PeripheralManager> peripherals_;
    std::unique_ptr<Debugger> debugger_;

    // Execution control
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> should_stop_{false};
    std::unique_ptr<std::thread> execution_thread_;
    
    // Synchronization
    mutable std::mutex state_mutex_;
    std::condition_variable pause_cv_;
    std::condition_variable resume_cv_;

    // Timing and performance
    std::atomic<ClockCycle> current_cycle_{0};
    TimeStamp start_time_;
    TimeStamp last_stats_update_;
    mutable std::mutex stats_mutex_;
    Statistics current_stats_;

    // Execution control parameters
    static constexpr auto EXECUTION_QUANTUM = std::chrono::microseconds(100);
    static constexpr uint32_t CYCLES_PER_QUANTUM = CPU_FREQ_HZ / 10000; // 100µs worth
};

} // namespace m5tab5::emulator