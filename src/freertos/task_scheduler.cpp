#include "emulator/freertos/scheduler.hpp"
#include "emulator/freertos/task.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cassert>

namespace m5tab5::emulator::freertos {

// Global scheduler instance
TaskScheduler* SchedulerManager::instance_ = nullptr;
std::mutex SchedulerManager::instance_mutex_;

TaskScheduler::TaskScheduler(const SchedulerConfig& config, DualCoreManager& cpu_manager)
    : config_(config), cpu_manager_(cpu_manager), scheduler_running_(false), scheduler_suspended_(false),
      tick_count_(0), current_task_core_0_(nullptr), current_task_core_1_(nullptr) {
    
    LOG_INFO("Creating TaskScheduler with tick frequency: {} Hz", config_.tick_frequency_hz);
    
    // Initialize performance statistics
    last_stats_update_ = std::chrono::steady_clock::now();
    
    // Clear all ready queues
    for (auto& queue : ready_queues_core_0_) {
        while (!queue.empty()) queue.pop();
    }
    for (auto& queue : ready_queues_core_1_) {
        while (!queue.empty()) queue.pop();
    }
}

TaskScheduler::~TaskScheduler() {
    if (scheduler_running_) {
        stop();
    }
    LOG_INFO("TaskScheduler destroyed");
}

Result<void> TaskScheduler::initialize() {
    LOG_INFO("Initializing TaskScheduler");
    
    // Create idle tasks for both cores
    create_idle_tasks();
    
    // Set up the scheduler as the global instance
    SchedulerManager::set_instance(this);
    TaskAPI::set_scheduler(this);
    
    LOG_INFO("TaskScheduler initialized successfully");
    return {};
}

Result<void> TaskScheduler::start() {
    if (scheduler_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler already running"));
    }
    
    LOG_INFO("Starting TaskScheduler");
    
    // Reset tick count and statistics
    tick_count_ = 0;
    reset_statistics();
    
    // Start with idle tasks on both cores
    current_task_core_0_ = idle_task_core_0_.get();
    current_task_core_1_ = idle_task_core_1_.get();
    
    // Mark idle tasks as running
    idle_task_core_0_->set_state(TaskState::RUNNING);
    idle_task_core_1_->set_state(TaskState::RUNNING);
    
    scheduler_running_ = true;
    
    LOG_INFO("TaskScheduler started successfully");
    return {};
}

Result<void> TaskScheduler::stop() {
    if (!scheduler_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler not running"));
    }
    
    LOG_INFO("Stopping TaskScheduler");
    
    scheduler_running_ = false;
    scheduler_suspended_ = false;
    
    // Mark all tasks as suspended
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    for (auto& task : all_tasks_) {
        if (task->get_state() == TaskState::RUNNING || task->get_state() == TaskState::READY) {
            task->set_state(TaskState::SUSPENDED);
        }
    }
    
    // Clear current tasks
    current_task_core_0_ = nullptr;
    current_task_core_1_ = nullptr;
    
    LOG_INFO("TaskScheduler stopped");
    return {};
}

Result<void> TaskScheduler::suspend_all() {
    if (!scheduler_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler not running"));
    }
    
    LOG_DEBUG("Suspending all tasks");
    scheduler_suspended_ = true;
    
    // Wake all waiting context switches to let them know about suspension
    context_switch_cv_[0].notify_all();
    context_switch_cv_[1].notify_all();
    
    return {};
}

Result<void> TaskScheduler::resume_all() {
    if (!scheduler_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler not running"));
    }
    
    LOG_DEBUG("Resuming all tasks");
    scheduler_suspended_ = false;
    
    // Trigger scheduling to resume normal operation
    schedule();
    
    return {};
}

Result<TaskHandle_t> TaskScheduler::create_task(const Task::TaskParameters& params) {
    LOG_DEBUG("Creating task: {} with priority {} on core {}", params.name, params.priority, params.core_affinity);
    
    // Validate parameters
    if (params.priority >= configMAX_PRIORITIES) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Task priority too high"));
    }
    
    if (params.core_affinity > 2) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid core affinity"));
    }
    
    if (params.stack_size < 512) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Stack size too small"));
    }
    
    // Create the task
    auto task = std::make_unique<Task>(params);
    Task* task_ptr = task.get();
    
    // Add to task list
    {
        std::lock_guard<std::mutex> lock(tasks_mutex_);
        all_tasks_.push_back(std::move(task));
    }
    
    // Add to appropriate ready queue
    if (scheduler_running_ && !scheduler_suspended_) {
        CoreAssignment preferred_core = static_cast<CoreAssignment>(params.core_affinity);
        add_task_to_ready_queue(task_ptr, preferred_core);
        
        // Trigger scheduling if this task has higher priority than current
        schedule();
    }
    
    LOG_INFO("Created task: {} (handle: {})", params.name, static_cast<void*>(task_ptr));
    return static_cast<TaskHandle_t>(task_ptr);
}

Result<void> TaskScheduler::delete_task(TaskHandle_t task_handle) {
    if (!is_valid_task_handle(task_handle)) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid task handle"));
    }
    
    Task* task = handle_to_task_ptr(task_handle);
    LOG_DEBUG("Deleting task: {}", task->get_name());
    
    // Remove from ready queue if present
    remove_task_from_ready_queue(task);
    
    // Mark as deleted
    task->set_state(TaskState::DELETED);
    
    // If this is the current task, trigger context switch
    if (current_task_core_0_ == task) {
        current_task_core_0_ = nullptr;
        context_switch_pending_[0] = true;
        context_switch_cv_[0].notify_one();
    }
    if (current_task_core_1_ == task) {
        current_task_core_1_ = nullptr;
        context_switch_pending_[1] = true;
        context_switch_cv_[1].notify_one();
    }
    
    // Remove from task list (will be done in next cleanup cycle)
    LOG_INFO("Deleted task: {}", task->get_name());
    return {};
}

void TaskScheduler::schedule() {
    if (!scheduler_running_ || scheduler_suspended_) {
        return;
    }
    
    // Select next tasks for both cores
    Task* next_task_core_0 = select_next_task(CoreAssignment::CORE_0);
    Task* next_task_core_1 = select_next_task(CoreAssignment::CORE_1);
    
    // Perform context switches if needed
    if (next_task_core_0 != current_task_core_0_ && next_task_core_0 != nullptr) {
        perform_context_switch(CoreAssignment::CORE_0, next_task_core_0);
    }
    
    if (next_task_core_1 != current_task_core_1_ && next_task_core_1 != nullptr) {
        perform_context_switch(CoreAssignment::CORE_1, next_task_core_1);
    }
    
    update_statistics();
}

Task* TaskScheduler::select_next_task(CoreAssignment core) {
    std::lock_guard<std::mutex> lock(ready_queues_mutex_);
    
    // Select appropriate ready queues
    auto& ready_queues = (core == CoreAssignment::CORE_0) ? ready_queues_core_0_ : ready_queues_core_1_;
    
    // Find highest priority non-empty queue
    for (int priority = configMAX_PRIORITIES - 1; priority >= 0; --priority) {
        if (!ready_queues[priority].empty()) {
            Task* task = ready_queues[priority].front();
            ready_queues[priority].pop();
            return task;
        }
    }
    
    // No ready tasks, return idle task
    return (core == CoreAssignment::CORE_0) ? idle_task_core_0_.get() : idle_task_core_1_.get();
}

void TaskScheduler::add_task_to_ready_queue(Task* task, CoreAssignment preferred_core) {
    if (!task || task->get_state() != TaskState::READY) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(ready_queues_mutex_);
    
    CoreAssignment target_core = determine_best_core(task);
    if (preferred_core != CoreAssignment::NO_PREFERENCE) {
        target_core = preferred_core;
    }
    
    auto& ready_queues = (target_core == CoreAssignment::CORE_0) ? ready_queues_core_0_ : ready_queues_core_1_;
    ready_queues[task->get_priority()].push(task);
    
    LOG_DEBUG("Added task {} to ready queue (core {}, priority {})", 
              task->get_name(), static_cast<int>(target_core), task->get_priority());
}

void TaskScheduler::remove_task_from_ready_queue(Task* task) {
    if (!task) return;
    
    std::lock_guard<std::mutex> lock(ready_queues_mutex_);
    
    // Remove from both cores' queues (inefficient but simple)
    for (int priority = 0; priority < configMAX_PRIORITIES; ++priority) {
        // Core 0 queues
        std::queue<Task*> temp_queue;
        while (!ready_queues_core_0_[priority].empty()) {
            Task* current_task = ready_queues_core_0_[priority].front();
            ready_queues_core_0_[priority].pop();
            if (current_task != task) {
                temp_queue.push(current_task);
            }
        }
        ready_queues_core_0_[priority] = temp_queue;
        
        // Core 1 queues
        while (!ready_queues_core_1_[priority].empty()) {
            Task* current_task = ready_queues_core_1_[priority].front();
            ready_queues_core_1_[priority].pop();
            if (current_task != task) {
                temp_queue.push(current_task);
            }
        }
        ready_queues_core_1_[priority] = temp_queue;
    }
}

void TaskScheduler::perform_context_switch(CoreAssignment core, Task* new_task) {
    size_t core_index = core_to_index(core);
    Task* current_task = (core == CoreAssignment::CORE_0) ? current_task_core_0_.load() : current_task_core_1_.load();
    
    LOG_DEBUG("Context switch on core {}: {} -> {}", 
              static_cast<int>(core),
              current_task ? current_task->get_name() : "null",
              new_task ? new_task->get_name() : "null");
    
    // Save current task context
    if (current_task && current_task->get_state() == TaskState::RUNNING) {
        save_task_context(current_task, core);
        current_task->set_state(TaskState::READY);
        
        // Add back to ready queue if not idle task
        if (current_task != idle_task_core_0_.get() && current_task != idle_task_core_1_.get()) {
            add_task_to_ready_queue(current_task, core);
        }
    }
    
    // Restore new task context
    if (new_task) {
        restore_task_context(new_task, core);
        new_task->set_state(TaskState::RUNNING);
    }
    
    // Update current task pointer
    if (core == CoreAssignment::CORE_0) {
        current_task_core_0_ = new_task;
    } else {
        current_task_core_1_ = new_task;
    }
    
    // Mark context switch as complete
    context_switch_pending_[core_index] = false;
    
    // Track statistics
    track_context_switch(core);
}

void TaskScheduler::save_task_context(Task* task, CoreAssignment core) {
    if (!task) return;
    
    // In a real implementation, this would save CPU registers to the task's stack
    // For emulation, we'll save a simplified context
    
    // Get the CPU core for this core assignment
    auto cpu_core_result = cpu_manager_.get_core(
        core == CoreAssignment::CORE_0 ? DualCoreManager::CoreId::CORE_0 : DualCoreManager::CoreId::CORE_1
    );
    
    if (cpu_core_result.has_value()) {
        // Save program counter (simplified)
        // In real implementation, would save all registers to task stack
        LOG_DEBUG("Saved context for task: {} on core {}", task->get_name(), static_cast<int>(core));
    }
}

void TaskScheduler::restore_task_context(Task* task, CoreAssignment core) {
    if (!task) return;
    
    // In a real implementation, this would restore CPU registers from the task's stack
    // For emulation, we'll restore a simplified context
    
    // Get the CPU core for this core assignment  
    auto cpu_core_result = cpu_manager_.get_core(
        core == CoreAssignment::CORE_0 ? DualCoreManager::CoreId::CORE_0 : DualCoreManager::CoreId::CORE_1
    );
    
    if (cpu_core_result.has_value()) {
        // Restore program counter (simplified)
        // In real implementation, would restore all registers from task stack
        LOG_DEBUG("Restored context for task: {} on core {}", task->get_name(), static_cast<int>(core));
    }
}

void TaskScheduler::create_idle_tasks() {
    // Create idle task for Core 0
    Task::TaskParameters idle_params_0;
    idle_params_0.name = "IDLE0";
    idle_params_0.function = [this](void*) { idle_task_function_core_0(this); };
    idle_params_0.priority = tskIDLE_PRIORITY;
    idle_params_0.stack_size = config_.idle_task_stack_size;
    idle_params_0.core_affinity = 0;
    
    idle_task_core_0_ = std::make_unique<Task>(idle_params_0);
    
    // Create idle task for Core 1
    Task::TaskParameters idle_params_1;
    idle_params_1.name = "IDLE1";
    idle_params_1.function = [this](void*) { idle_task_function_core_1(this); };
    idle_params_1.priority = tskIDLE_PRIORITY;
    idle_params_1.stack_size = config_.idle_task_stack_size;
    idle_params_1.core_affinity = 1;
    
    idle_task_core_1_ = std::make_unique<Task>(idle_params_1);
    
    LOG_INFO("Created idle tasks for both cores");
}

void TaskScheduler::idle_task_function_core_0(void* parameters) {
    TaskScheduler* scheduler = static_cast<TaskScheduler*>(parameters);
    
    while (scheduler->scheduler_running_) {
        // Idle task work: garbage collection, power management, etc.
        scheduler->process_delayed_tasks();
        
        // Track idle time
        scheduler->stats_.idle_ticks_core_0++;
        
        // Yield to allow other tasks to run
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void TaskScheduler::idle_task_function_core_1(void* parameters) {
    TaskScheduler* scheduler = static_cast<TaskScheduler*>(parameters);
    
    while (scheduler->scheduler_running_) {
        // Idle task work: load balancing, statistics, etc.
        scheduler->balance_cores();
        
        // Track idle time
        scheduler->stats_.idle_ticks_core_1++;
        
        // Yield to allow other tasks to run
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void TaskScheduler::process_delayed_tasks() {
    std::lock_guard<std::mutex> lock(delayed_tasks_mutex_);
    
    TickType_t current_tick = tick_count_;
    
    while (!delayed_tasks_.empty() && delayed_tasks_.top().wake_time <= current_tick) {
        DelayedTask delayed_task = delayed_tasks_.top();
        delayed_tasks_.pop();
        
        // Move task back to ready state
        Task* task = delayed_task.task;
        if (task && task->get_state() == TaskState::BLOCKED) {
            task->set_state(TaskState::READY);
            add_task_to_ready_queue(task, CoreAssignment::NO_PREFERENCE);
            
            LOG_DEBUG("Task {} woke up from delay", task->get_name());
        }
    }
}

void TaskScheduler::balance_cores() {
    // Simple load balancing: move tasks from overloaded core to underloaded core
    std::lock_guard<std::mutex> lock(ready_queues_mutex_);
    
    // Count ready tasks on each core
    size_t core_0_tasks = 0, core_1_tasks = 0;
    
    for (int priority = 0; priority < configMAX_PRIORITIES; ++priority) {
        core_0_tasks += ready_queues_core_0_[priority].size();
        core_1_tasks += ready_queues_core_1_[priority].size();
    }
    
    // Balance if difference is significant (> 2 tasks)
    if (core_0_tasks > core_1_tasks + 2) {
        // Move task from core 0 to core 1
        for (int priority = configMAX_PRIORITIES - 1; priority >= 0; --priority) {
            if (!ready_queues_core_0_[priority].empty()) {
                Task* task = ready_queues_core_0_[priority].front();
                ready_queues_core_0_[priority].pop();
                ready_queues_core_1_[priority].push(task);
                LOG_DEBUG("Balanced task {} from core 0 to core 1", task->get_name());
                break;
            }
        }
    } else if (core_1_tasks > core_0_tasks + 2) {
        // Move task from core 1 to core 0
        for (int priority = configMAX_PRIORITIES - 1; priority >= 0; --priority) {
            if (!ready_queues_core_1_[priority].empty()) {
                Task* task = ready_queues_core_1_[priority].front();
                ready_queues_core_1_[priority].pop();
                ready_queues_core_0_[priority].push(task);
                LOG_DEBUG("Balanced task {} from core 1 to core 0", task->get_name());
                break;
            }
        }
    }
}

void TaskScheduler::handle_tick_interrupt() {
    tick_count_++;
    stats_.total_ticks++;
    
    // Process delayed tasks
    process_delayed_tasks();
    
    // Update running task runtime
    Task* task_core_0 = current_task_core_0_.load();
    if (task_core_0 && task_core_0->get_state() == TaskState::RUNNING) {
        task_core_0->update_runtime(1);
    }
    
    Task* task_core_1 = current_task_core_1_.load();
    if (task_core_1 && task_core_1->get_state() == TaskState::RUNNING) {
        task_core_1->update_runtime(1);
    }
    
    // Trigger scheduling to handle any newly ready tasks
    schedule();
}

void TaskScheduler::increment_tick() {
    handle_tick_interrupt();
}

void TaskScheduler::delay_task(TaskHandle_t task_handle, TickType_t ticks) {
    if (!is_valid_task_handle(task_handle) || ticks == 0) {
        return;
    }
    
    Task* task = handle_to_task_ptr(task_handle);
    TickType_t wake_time = tick_count_ + ticks;
    
    // Remove from ready queue and set to blocked
    remove_task_from_ready_queue(task);
    task->set_state(TaskState::BLOCKED);
    task->set_wait_timeout(wake_time);
    
    // Add to delayed tasks
    {
        std::lock_guard<std::mutex> lock(delayed_tasks_mutex_);
        delayed_tasks_.push({task, wake_time});
    }
    
    LOG_DEBUG("Task {} delayed for {} ticks (wake at {})", task->get_name(), ticks, wake_time);
    
    // If this is the current task, trigger context switch
    if (current_task_core_0_ == task) {
        context_switch_pending_[0] = true;
        context_switch_cv_[0].notify_one();
    }
    if (current_task_core_1_ == task) {
        context_switch_pending_[1] = true;
        context_switch_cv_[1].notify_one();
    }
}

TaskScheduler::CoreAssignment TaskScheduler::determine_best_core(const Task* task) const {
    if (!task) return CoreAssignment::CORE_0;
    
    // Use task's core affinity preference
    UBaseType_t affinity = task->get_core_affinity();
    if (affinity == 0) return CoreAssignment::CORE_0;
    if (affinity == 1) return CoreAssignment::CORE_1;
    
    // No preference, use load balancing
    // Simple heuristic: choose core with fewer ready tasks
    size_t core_0_load = 0, core_1_load = 0;
    
    for (int priority = 0; priority < configMAX_PRIORITIES; ++priority) {
        core_0_load += ready_queues_core_0_[priority].size();
        core_1_load += ready_queues_core_1_[priority].size();
    }
    
    return (core_0_load <= core_1_load) ? CoreAssignment::CORE_0 : CoreAssignment::CORE_1;
}

// Scheduler Manager Implementation
void SchedulerManager::set_instance(TaskScheduler* scheduler) {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    instance_ = scheduler;
}

TaskScheduler* SchedulerManager::get_instance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_;
}

bool SchedulerManager::has_instance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_ != nullptr;
}

// Utility functions
bool TaskScheduler::is_valid_task_handle(TaskHandle_t handle) const {
    if (!handle) return false;
    
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    for (const auto& task : all_tasks_) {
        if (static_cast<void*>(task.get()) == handle) {
            return task->get_state() != TaskState::DELETED;
        }
    }
    return false;
}

Task* TaskScheduler::handle_to_task_ptr(TaskHandle_t handle) const {
    return static_cast<Task*>(handle);
}

size_t TaskScheduler::core_to_index(CoreAssignment core) const {
    return static_cast<size_t>(core);
}

void TaskScheduler::track_context_switch(CoreAssignment core) {
    stats_.total_context_switches++;
}

void TaskScheduler::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_update_);
    
    if (elapsed.count() >= 1) {
        stats_.task_switches_per_second = stats_.total_context_switches;
        last_stats_update_ = now;
    }
}

void TaskScheduler::reset_statistics() {
    stats_ = {};
    last_stats_update_ = std::chrono::steady_clock::now();
}

UBaseType_t TaskScheduler::get_task_count() const {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    return all_tasks_.size();
}

} // namespace m5tab5::emulator::freertos