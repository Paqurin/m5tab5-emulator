#pragma once

#include "emulator/freertos/task.hpp"
#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

namespace m5tab5::emulator::freertos {

/**
 * @brief ESP32-P4 dual-core FreeRTOS scheduler emulation
 * 
 * Implements preemptive multi-tasking scheduler supporting:
 * - Dual-core RISC-V ESP32-P4 architecture
 * - Priority-based preemptive scheduling
 * - Inter-core task migration
 * - Tick-based time slicing
 * - Task synchronization primitives
 */
class TaskScheduler {
public:
    struct SchedulerConfig {
        uint32_t tick_frequency_hz = 1000;  // Default 1ms ticks
        uint32_t idle_task_stack_size = 1024;
        bool enable_task_statistics = true;
        bool enable_stack_overflow_checking = true;
        uint32_t max_task_name_length = 16;
    };

    // Core assignment for tasks
    enum class CoreAssignment : uint8_t {
        CORE_0 = 0,
        CORE_1 = 1,
        ANY_CORE = 2,
        NO_PREFERENCE = 2
    };

    explicit TaskScheduler(const SchedulerConfig& config, DualCoreManager& cpu_manager);
    ~TaskScheduler();

    // Scheduler lifecycle
    Result<void> initialize();
    Result<void> start();
    Result<void> stop();
    Result<void> suspend_all();
    Result<void> resume_all();
    
    // Task management
    Result<TaskHandle_t> create_task(const Task::TaskParameters& params);
    Result<void> delete_task(TaskHandle_t task_handle);
    Result<void> suspend_task(TaskHandle_t task_handle);
    Result<void> resume_task(TaskHandle_t task_handle);
    
    // Priority management
    Result<void> set_task_priority(TaskHandle_t task_handle, UBaseType_t new_priority);
    UBaseType_t get_task_priority(TaskHandle_t task_handle) const;
    
    // Core affinity management  
    Result<void> set_task_affinity(TaskHandle_t task_handle, CoreAssignment core);
    CoreAssignment get_task_affinity(TaskHandle_t task_handle) const;
    
    // Current task access
    TaskHandle_t get_current_task(CoreAssignment core) const;
    TaskHandle_t get_current_task() const; // Current core
    
    // Scheduler state
    bool is_running() const { return scheduler_running_; }
    bool is_scheduler_suspended() const { return scheduler_suspended_; }
    UBaseType_t get_task_count() const;
    
    // Time management
    TickType_t get_tick_count() const { return tick_count_; }
    void increment_tick();
    void delay_task(TaskHandle_t task_handle, TickType_t ticks);
    void delay_until(TaskHandle_t task_handle, TickType_t wake_time);
    
    // Scheduler algorithms
    void schedule(); // Main scheduling decision
    void yield_current_task(); // Voluntary yield
    void preempt_current_task(); // Forced preemption
    
    // Interrupt handling
    void handle_tick_interrupt();
    void handle_context_switch_interrupt(CoreAssignment core);
    bool is_context_switch_pending(CoreAssignment core) const;
    
    // Performance and debugging
    struct SchedulerStats {
        uint64_t total_context_switches = 0;
        uint64_t total_ticks = 0;
        uint64_t idle_ticks_core_0 = 0;
        uint64_t idle_ticks_core_1 = 0;
        uint32_t max_interrupt_disabled_time = 0;
        uint32_t task_switches_per_second = 0;
    };
    
    const SchedulerStats& get_statistics() const { return stats_; }
    void reset_statistics();
    
    // Stack monitoring
    Result<UBaseType_t> get_stack_high_water_mark(TaskHandle_t task_handle) const;
    Result<bool> check_stack_overflow(TaskHandle_t task_handle) const;
    
    // Task list management
    std::vector<TaskHandle_t> get_all_tasks() const;
    std::vector<TaskHandle_t> get_ready_tasks() const;
    std::vector<TaskHandle_t> get_blocked_tasks() const;

private:
    // Scheduler configuration
    SchedulerConfig config_;
    DualCoreManager& cpu_manager_;
    
    // Core scheduler state
    std::atomic<bool> scheduler_running_{false};
    std::atomic<bool> scheduler_suspended_{false};
    std::atomic<TickType_t> tick_count_{0};
    
    // Task storage and management
    mutable std::mutex tasks_mutex_;
    std::vector<std::unique_ptr<Task>> all_tasks_;
    
    // Ready task queues (priority-based)
    std::array<std::queue<Task*>, configMAX_PRIORITIES> ready_queues_core_0_;
    std::array<std::queue<Task*>, configMAX_PRIORITIES> ready_queues_core_1_;
    mutable std::mutex ready_queues_mutex_;
    
    // Currently running tasks
    std::atomic<Task*> current_task_core_0_{nullptr};
    std::atomic<Task*> current_task_core_1_{nullptr};
    
    // Delayed task management  
    struct DelayedTask {
        Task* task;
        TickType_t wake_time;
        bool operator>(const DelayedTask& other) const {
            return wake_time > other.wake_time; // Min-heap
        }
    };
    std::priority_queue<DelayedTask, std::vector<DelayedTask>, std::greater<DelayedTask>> delayed_tasks_;
    mutable std::mutex delayed_tasks_mutex_;
    
    // Idle tasks
    std::unique_ptr<Task> idle_task_core_0_;
    std::unique_ptr<Task> idle_task_core_1_;
    
    // Context switching
    std::array<std::atomic<bool>, 2> context_switch_pending_{false, false};
    std::array<std::condition_variable, 2> context_switch_cv_;
    std::array<std::mutex, 2> context_switch_mutex_;
    
    // Performance statistics
    SchedulerStats stats_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    // Internal scheduling algorithms
    Task* select_next_task(CoreAssignment core);
    void add_task_to_ready_queue(Task* task, CoreAssignment preferred_core);
    void remove_task_from_ready_queue(Task* task);
    void process_delayed_tasks();
    void balance_cores(); // Load balancing between cores
    
    // Context switching support
    void perform_context_switch(CoreAssignment core, Task* new_task);
    void save_task_context(Task* task, CoreAssignment core);
    void restore_task_context(Task* task, CoreAssignment core);
    
    // Idle task implementation
    void create_idle_tasks();
    static void idle_task_function_core_0(void* parameters);
    static void idle_task_function_core_1(void* parameters);
    
    // Priority queue management
    UBaseType_t find_highest_priority_ready(CoreAssignment core) const;
    bool has_ready_tasks(CoreAssignment core) const;
    
    // Utility functions
    CoreAssignment determine_best_core(const Task* task) const;
    size_t core_to_index(CoreAssignment core) const;
    bool is_valid_core(CoreAssignment core) const;
    
    // Task validation
    bool is_valid_task_handle(TaskHandle_t handle) const;
    Task* handle_to_task_ptr(TaskHandle_t handle) const;
    
    // Performance monitoring
    void update_statistics();
    void track_context_switch(CoreAssignment core);
};

/**
 * @brief Global scheduler instance management
 * 
 * Provides singleton access to the scheduler for FreeRTOS API calls.
 */
class SchedulerManager {
public:
    static void set_instance(TaskScheduler* scheduler);
    static TaskScheduler* get_instance();
    static bool has_instance();

private:
    static TaskScheduler* instance_;
    static std::mutex instance_mutex_;
};

} // namespace m5tab5::emulator::freertos