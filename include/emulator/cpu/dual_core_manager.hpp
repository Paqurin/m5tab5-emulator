#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/timer_controller.hpp"

#include <memory>
#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace m5tab5::emulator {

// Forward declarations
class MemoryController;

/**
 * @brief Manages dual RISC-V CPU cores for ESP32-P4 emulation
 * 
 * The ESP32-P4 has two main RISC-V cores plus a low-power core.
 * This manager coordinates execution, inter-core communication,
 * and shared resource access.
 */
class DualCoreManager {
public:
    enum class CpuState {
        STOPPED,
        RUNNING,
        PAUSED,
        HALTED,
        ERROR
    };

    enum class CoreId {
        CORE_0 = 0,
        CORE_1 = 1, 
        LP_CORE = 2
    };

    static constexpr size_t NUM_CORES = 3;

    struct InterCoreMessage {
        CoreId source;
        CoreId target;
        u32 data;
        u64 timestamp;
    };

    DualCoreManager();
    ~DualCoreManager();

    // Lifecycle management
    Result<void> initialize(const Configuration& config, MemoryController& memory_controller);
    Result<void> start();
    Result<void> stop();
    Result<void> pause();
    Result<void> resume();
    Result<void> reset();
    Result<void> shutdown();

    // Execution control
    Result<Cycles> execute_cycles(Cycles max_cycles);
    
    // Core access
    Result<CpuCore*> get_core(CoreId core_id);
    Result<const CpuCore*> get_core(CoreId core_id) const;
    
    // State queries
    CpuState get_state() const { return state_; }
    bool is_running() const { return state_ == CpuState::RUNNING; }
    u64 get_total_cycles_executed() const;
    u64 get_total_instructions_executed() const;
    
    // Inter-core communication
    Result<void> send_inter_core_interrupt(CoreId source, CoreId target, u32 data);
    Result<void> wait_for_event(CoreId core_id, u32 timeout_cycles);
    Result<void> signal_event(CoreId core_id);
    
    // Debug and monitoring
    void dump_all_cores_state() const;
    void collect_performance_stats();

private:
    // Internal state
    std::atomic<CpuState> state_;
    MemoryController* memory_controller_;
    std::unique_ptr<InterruptController> interrupt_controller_;
    std::unique_ptr<TimerController> timer_controller_;
    
    // CPU cores
    std::array<std::unique_ptr<CpuCore>, NUM_CORES> cores_;
    
    // Synchronization
    mutable std::mutex cores_mutex_;
    std::condition_variable execution_cv_;
    
    // Inter-core communication
    std::mutex message_queue_mutex_;
    std::vector<InterCoreMessage> message_queue_;
    std::array<std::condition_variable, NUM_CORES> core_events_;
    
    // Performance tracking
    mutable std::mutex stats_mutex_;
    u64 total_cycles_executed_;
    
    // Private methods
    Result<void> setup_inter_core_communication();
    Result<void> process_interrupts();
    bool is_valid_core_id(CoreId core_id) const;
    size_t core_id_to_index(CoreId core_id) const;
};

} // namespace m5tab5::emulator