#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/freertos/task.hpp"
#include "emulator/freertos/scheduler.hpp"
#include "emulator/utils/logging.hpp"
#include <cstring>
#include <algorithm>

namespace m5tab5::emulator::freertos {

// Global scheduler pointer for TaskAPI
TaskScheduler* TaskAPI::scheduler_ = nullptr;

// Task Implementation
Task::Task(const TaskParameters& params) 
    : name_(params.name), function_(params.function), parameters_(params.parameters),
      priority_(params.priority), state_(TaskState::READY), core_affinity_(params.core_affinity),
      stack_size_(params.stack_size), stack_pointer_(nullptr), stack_used_(0),
      creation_time_(0), total_runtime_(0), last_runtime_update_(0), wait_timeout_(0),
      function_complete_(false) {
    
    // Initialize stack
    initialize_stack();
    
    // Set creation time (simplified)
    creation_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    last_execution_ = std::chrono::steady_clock::now();
    
    LOG_DEBUG("Created task: {} with stack size {} bytes", name_, stack_size_);
}

Task::~Task() {
    LOG_DEBUG("Destroying task: {}", name_);
}

void Task::initialize_stack() {
    // Allocate stack memory
    stack_memory_ = std::make_unique<uint8_t[]>(stack_size_);
    
    // Initialize stack with pattern for overflow detection
    std::memset(stack_memory_.get(), 0xA5, stack_size_);
    
    // Set stack pointer to top of stack (grows downward)
    stack_pointer_ = stack_memory_.get() + stack_size_;
    
    LOG_DEBUG("Initialized stack for task {}: {} bytes at {}", 
              name_, stack_size_, static_cast<void*>(stack_memory_.get()));
}

Result<void> Task::suspend() {
    if (state_ == TaskState::DELETED) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Cannot suspend deleted task"));
    }
    
    LOG_DEBUG("Suspending task: {}", name_);
    state_ = TaskState::SUSPENDED;
    return {};
}

Result<void> Task::resume() {
    if (state_ == TaskState::DELETED) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Cannot resume deleted task"));
    }
    
    if (state_ == TaskState::SUSPENDED) {
        LOG_DEBUG("Resuming task: {}", name_);
        state_ = TaskState::READY;
    }
    
    return {};
}

Result<void> Task::delete_task() {
    LOG_DEBUG("Deleting task: {}", name_);
    state_ = TaskState::DELETED;
    function_complete_ = true;
    return {};
}

Result<void> Task::set_priority(UBaseType_t new_priority) {
    if (new_priority >= configMAX_PRIORITIES) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Priority too high"));
    }
    
    LOG_DEBUG("Setting priority of task {} from {} to {}", name_, priority_, new_priority);
    priority_ = new_priority;
    return {};
}

Result<void> Task::set_core_affinity(UBaseType_t core_id) {
    if (core_id > 2) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid core ID"));
    }
    
    LOG_DEBUG("Setting core affinity of task {} to core {}", name_, core_id);
    core_affinity_ = core_id;
    return {};
}

uint16_t Task::get_stack_high_water_mark() const {
    update_stack_usage();
    return stack_size_ - stack_used_;
}

bool Task::is_stack_overflow() const {
    update_stack_usage();
    
    // Check for stack overflow (less than 10% stack remaining)
    return stack_used_ > (stack_size_ * 9 / 10);
}

void Task::update_stack_usage() const {
    // Scan stack for pattern to determine usage
    // This is a simplified implementation
    const uint8_t* stack_base = stack_memory_.get();
    uint16_t used = 0;
    
    // Scan from top of stack downward looking for untouched pattern (0xA5)
    for (uint16_t i = 0; i < stack_size_; ++i) {
        if (stack_base[stack_size_ - 1 - i] != 0xA5) {
            used = i + 1;
            break;
        }
    }
    
    // Update stack usage (const_cast is safe here)
    const_cast<Task*>(this)->stack_used_ = used;
}

TickType_t Task::get_runtime() const {
    return total_runtime_;
}

void Task::update_runtime(TickType_t delta_ticks) {
    total_runtime_ += delta_ticks;
    last_runtime_update_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Task::execute_function() {
    if (function_ && !function_complete_) {
        LOG_DEBUG("Executing function for task: {}", name_);
        
        try {
            function_(parameters_);
            function_complete_ = true;
            state_ = TaskState::DELETED; // Task function completed
            LOG_DEBUG("Task {} function completed", name_);
        } catch (const std::exception& e) {
            LOG_ERROR("Task {} function threw exception: {}", name_, e.what());
            function_complete_ = true;
            state_ = TaskState::DELETED;
        }
    }
}

// TaskAPI Implementation
BaseType_t TaskAPI::xTaskCreate(
    TaskFunction_t task_function,
    const char* task_name,
    const uint16_t stack_size,
    void* const parameters,
    UBaseType_t priority,
    TaskHandle_t* const created_task
) {
    if (!scheduler_) {
        LOG_ERROR("TaskAPI: No scheduler instance available");
        return pdFAIL;
    }
    
    if (!task_function || !task_name) {
        LOG_ERROR("TaskAPI: Invalid task function or name");
        return pdFAIL;
    }
    
    Task::TaskParameters params;
    params.name = task_name;
    params.function = task_function;
    params.parameters = parameters;
    params.priority = priority;
    params.stack_size = stack_size;
    params.core_affinity = 2; // Any core
    
    auto result = scheduler_->create_task(params);
    if (result.has_value()) {
        if (created_task) {
            *created_task = result.value();
        }
        LOG_INFO("TaskAPI: Created task '{}'", task_name);
        return pdPASS;
    }
    
    LOG_ERROR("TaskAPI: Failed to create task '{}'", task_name);
    return pdFAIL;
}

BaseType_t TaskAPI::xTaskCreatePinnedToCore(
    TaskFunction_t task_function,
    const char* task_name,
    const uint16_t stack_size,
    void* const parameters,
    UBaseType_t priority,
    TaskHandle_t* const created_task,
    const BaseType_t core_id
) {
    if (!scheduler_) {
        LOG_ERROR("TaskAPI: No scheduler instance available");
        return pdFAIL;
    }
    
    if (!task_function || !task_name || core_id < 0 || core_id > 1) {
        LOG_ERROR("TaskAPI: Invalid parameters for xTaskCreatePinnedToCore");
        return pdFAIL;
    }
    
    Task::TaskParameters params;
    params.name = task_name;
    params.function = task_function;
    params.parameters = parameters;
    params.priority = priority;
    params.stack_size = stack_size;
    params.core_affinity = static_cast<UBaseType_t>(core_id);
    
    auto result = scheduler_->create_task(params);
    if (result.has_value()) {
        if (created_task) {
            *created_task = result.value();
        }
        LOG_INFO("TaskAPI: Created task '{}' pinned to core {}", task_name, core_id);
        return pdPASS;
    }
    
    LOG_ERROR("TaskAPI: Failed to create pinned task '{}'", task_name);
    return pdFAIL;
}

void TaskAPI::vTaskDelete(TaskHandle_t task_to_delete) {
    if (!scheduler_) {
        LOG_ERROR("TaskAPI: No scheduler instance available");
        return;
    }
    
    // If task_to_delete is NULL, delete current task
    if (!task_to_delete) {
        task_to_delete = xTaskGetCurrentTaskHandle();
    }
    
    if (task_to_delete) {
        auto result = scheduler_->delete_task(task_to_delete);
        if (result.has_value()) {
            LOG_DEBUG("TaskAPI: Deleted task");
        } else {
            LOG_ERROR("TaskAPI: Failed to delete task");
        }
    }
}

void TaskAPI::vTaskDelay(const TickType_t ticks_to_delay) {
    if (!scheduler_ || ticks_to_delay == 0) {
        return;
    }
    
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    if (current_task) {
        scheduler_->delay_task(current_task, ticks_to_delay);
        LOG_DEBUG("TaskAPI: Task delayed for {} ticks", ticks_to_delay);
    }
}

void TaskAPI::vTaskDelayUntil(TickType_t* const previous_wake_time, const TickType_t time_increment) {
    if (!scheduler_ || !previous_wake_time || time_increment == 0) {
        return;
    }
    
    TickType_t current_tick = scheduler_->get_tick_count();
    TickType_t next_wake = *previous_wake_time + time_increment;
    
    if (next_wake > current_tick) {
        vTaskDelay(next_wake - current_tick);
    }
    
    *previous_wake_time = next_wake;
}

void TaskAPI::vTaskSuspend(TaskHandle_t task_to_suspend) {
    if (!scheduler_) {
        return;
    }
    
    // If task_to_suspend is NULL, suspend current task
    if (!task_to_suspend) {
        task_to_suspend = xTaskGetCurrentTaskHandle();
    }
    
    if (task_to_suspend) {
        scheduler_->suspend_task(task_to_suspend);
        LOG_DEBUG("TaskAPI: Suspended task");
    }
}

void TaskAPI::vTaskResume(TaskHandle_t task_to_resume) {
    if (!scheduler_ || !task_to_resume) {
        return;
    }
    
    scheduler_->resume_task(task_to_resume);
    LOG_DEBUG("TaskAPI: Resumed task");
}

void TaskAPI::vTaskPrioritySet(TaskHandle_t task, UBaseType_t new_priority) {
    if (!scheduler_) {
        return;
    }
    
    // If task is NULL, use current task
    if (!task) {
        task = xTaskGetCurrentTaskHandle();
    }
    
    if (task) {
        scheduler_->set_task_priority(task, new_priority);
        LOG_DEBUG("TaskAPI: Set task priority to {}", new_priority);
    }
}

UBaseType_t TaskAPI::uxTaskPriorityGet(TaskHandle_t task) {
    if (!scheduler_) {
        return 0;
    }
    
    // If task is NULL, use current task
    if (!task) {
        task = xTaskGetCurrentTaskHandle();
    }
    
    if (task) {
        return scheduler_->get_task_priority(task);
    }
    
    return 0;
}

TaskHandle_t TaskAPI::xTaskGetCurrentTaskHandle() {
    if (!scheduler_) {
        return nullptr;
    }
    
    return scheduler_->get_current_task();
}

char* TaskAPI::pcTaskGetName(TaskHandle_t task_handle) {
    if (!task_handle) {
        task_handle = xTaskGetCurrentTaskHandle();
    }
    
    Task* task = handle_to_task(task_handle);
    if (task) {
        // Return non-const char* to match FreeRTOS API
        return const_cast<char*>(task->get_name().c_str());
    }
    
    return nullptr;
}

UBaseType_t TaskAPI::uxTaskGetNumberOfTasks() {
    if (!scheduler_) {
        return 0;
    }
    
    return scheduler_->get_task_count();
}

TickType_t TaskAPI::xTaskGetTickCount() {
    if (!scheduler_) {
        return 0;
    }
    
    return scheduler_->get_tick_count();
}

TickType_t TaskAPI::xTaskGetTickCountFromISR() {
    // Same as xTaskGetTickCount for our emulation
    return xTaskGetTickCount();
}

UBaseType_t TaskAPI::uxTaskGetStackHighWaterMark(TaskHandle_t task) {
    if (!task) {
        task = xTaskGetCurrentTaskHandle();
    }
    
    if (!scheduler_ || !task) {
        return 0;
    }
    
    auto result = scheduler_->get_stack_high_water_mark(task);
    return result.has_value() ? result.value() : 0;
}

void TaskAPI::vTaskStartScheduler() {
    if (scheduler_) {
        scheduler_->start();
        LOG_INFO("TaskAPI: Scheduler started");
    }
}

void TaskAPI::vTaskEndScheduler() {
    if (scheduler_) {
        scheduler_->stop();
        LOG_INFO("TaskAPI: Scheduler stopped");
    }
}

void TaskAPI::vTaskSuspendAll() {
    if (scheduler_) {
        scheduler_->suspend_all();
        LOG_DEBUG("TaskAPI: All tasks suspended");
    }
}

BaseType_t TaskAPI::xTaskResumeAll() {
    if (scheduler_) {
        scheduler_->resume_all();
        LOG_DEBUG("TaskAPI: All tasks resumed");
        return pdTRUE;
    }
    return pdFALSE;
}

// Private helper functions
Task* TaskAPI::handle_to_task(TaskHandle_t handle) {
    if (!handle || !scheduler_) {
        return nullptr;
    }
    
    // In our implementation, TaskHandle_t is directly a Task pointer
    return static_cast<Task*>(handle);
}

TaskHandle_t TaskAPI::task_to_handle(Task* task) {
    return static_cast<TaskHandle_t>(task);
}

} // namespace m5tab5::emulator::freertos