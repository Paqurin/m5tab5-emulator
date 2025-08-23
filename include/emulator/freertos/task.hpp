#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <vector>

namespace m5tab5::emulator::freertos {

// Forward declarations
class TaskScheduler;

// FreeRTOS-compatible type definitions
using TaskHandle_t = void*;
using TaskFunction_t = std::function<void(void*)>;
using StackType_t = uint8_t;
using UBaseType_t = uint32_t;
using BaseType_t = int32_t;
using TickType_t = uint32_t;

// Task priorities (0 = lowest, configMAX_PRIORITIES-1 = highest)
constexpr UBaseType_t configMAX_PRIORITIES = 25;
constexpr TickType_t portMAX_DELAY = 0xFFFFFFFF;

// Task states
enum class TaskState {
    READY = 0,      // Ready to run
    RUNNING = 1,    // Currently executing 
    BLOCKED = 2,    // Blocked waiting for resource
    SUSPENDED = 3,  // Suspended by vTaskSuspend
    DELETED = 4     // Task has been deleted
};

// Task priority definitions
constexpr UBaseType_t tskIDLE_PRIORITY = 0;
constexpr UBaseType_t configKERNEL_INTERRUPT_PRIORITY = 1;

/**
 * @brief Task Control Block (TCB) - Core FreeRTOS task structure
 * 
 * Contains all information needed to manage a FreeRTOS task including
 * stack pointer, priority, state, and timing information.
 */
class Task {
public:
    struct TaskParameters {
        std::string name;
        TaskFunction_t function;
        void* parameters = nullptr;
        UBaseType_t priority = tskIDLE_PRIORITY;
        uint16_t stack_size = 2048; // Stack size in bytes
        UBaseType_t core_affinity = 0; // 0=Core0, 1=Core1, 2=Any
    };

    explicit Task(const TaskParameters& params);
    ~Task();

    // Task control
    Result<void> suspend();
    Result<void> resume();
    Result<void> delete_task();
    
    // State queries
    TaskState get_state() const { return state_; }
    UBaseType_t get_priority() const { return priority_; }
    const std::string& get_name() const { return name_; }
    TickType_t get_creation_time() const { return creation_time_; }
    
    // Priority management
    Result<void> set_priority(UBaseType_t new_priority);
    UBaseType_t get_priority_from_isr() const { return priority_; }
    
    // Stack management
    uint16_t get_stack_size() const { return stack_size_; }
    uint16_t get_stack_high_water_mark() const;
    bool is_stack_overflow() const;
    
    // Core affinity
    UBaseType_t get_core_affinity() const { return core_affinity_; }
    Result<void> set_core_affinity(UBaseType_t core_id);
    
    // Timing
    TickType_t get_runtime() const;
    void update_runtime(TickType_t delta_ticks);
    
    // Task execution context
    void* get_stack_pointer() const { return stack_pointer_; }
    void set_stack_pointer(void* sp) { stack_pointer_ = sp; }
    
    // Internal scheduler access
    void set_state(TaskState new_state) { state_ = new_state; }
    void set_wait_timeout(TickType_t timeout) { wait_timeout_ = timeout; }
    TickType_t get_wait_timeout() const { return wait_timeout_; }
    void decrement_wait_timeout() { if (wait_timeout_ > 0) wait_timeout_--; }
    
    // Task function execution
    void execute_function();
    bool is_function_complete() const { return function_complete_; }

private:
    // Core task data
    std::string name_;
    TaskFunction_t function_;
    void* parameters_;
    UBaseType_t priority_;
    TaskState state_;
    UBaseType_t core_affinity_;
    
    // Stack management  
    std::unique_ptr<uint8_t[]> stack_memory_;
    uint16_t stack_size_;
    void* stack_pointer_;
    uint16_t stack_used_;
    
    // Timing information
    TickType_t creation_time_;
    TickType_t total_runtime_;
    TickType_t last_runtime_update_;
    TickType_t wait_timeout_;
    
    // Execution state
    bool function_complete_;
    std::chrono::steady_clock::time_point last_execution_;
    
    // Internal helpers
    void initialize_stack();
    void update_stack_usage() const;
};

/**
 * @brief FreeRTOS-compatible task creation and management APIs
 * 
 * Provides the standard FreeRTOS task APIs that ESP-IDF applications expect.
 * Maps to the underlying TaskScheduler for actual task management.
 */
class TaskAPI {
public:
    // Task creation/deletion
    static BaseType_t xTaskCreate(
        TaskFunction_t task_function,
        const char* task_name,
        const uint16_t stack_size,
        void* const parameters,
        UBaseType_t priority,
        TaskHandle_t* const created_task
    );
    
    static BaseType_t xTaskCreatePinnedToCore(
        TaskFunction_t task_function,
        const char* task_name,
        const uint16_t stack_size,
        void* const parameters,
        UBaseType_t priority,
        TaskHandle_t* const created_task,
        const BaseType_t core_id
    );
    
    static void vTaskDelete(TaskHandle_t task_to_delete);
    
    // Task control
    static void vTaskDelay(const TickType_t ticks_to_delay);
    static void vTaskDelayUntil(TickType_t* const previous_wake_time, const TickType_t time_increment);
    static void vTaskSuspend(TaskHandle_t task_to_suspend);
    static void vTaskResume(TaskHandle_t task_to_resume);
    static BaseType_t xTaskResumeFromISR(TaskHandle_t task_to_resume);
    
    // Priority management
    static void vTaskPrioritySet(TaskHandle_t task, UBaseType_t new_priority);
    static UBaseType_t uxTaskPriorityGet(TaskHandle_t task);
    static UBaseType_t uxTaskPriorityGetFromISR(TaskHandle_t task);
    
    // Task information
    static TaskHandle_t xTaskGetCurrentTaskHandle();
    static char* pcTaskGetName(TaskHandle_t task_handle);
    static UBaseType_t uxTaskGetNumberOfTasks();
    static TickType_t xTaskGetTickCount();
    static TickType_t xTaskGetTickCountFromISR();
    
    // Stack monitoring
    static UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task);
    static BaseType_t xTaskCheckForStackOverflow(TaskHandle_t task);
    
    // Core affinity (ESP-IDF specific)
    static BaseType_t xTaskGetAffinity(TaskHandle_t task);
    static void vTaskSetAffinity(TaskHandle_t task, UBaseType_t core_id);
    
    // Scheduler control
    static void vTaskStartScheduler();
    static void vTaskEndScheduler();
    static void vTaskSuspendAll();
    static BaseType_t xTaskResumeAll();
    static BaseType_t xTaskAbortDelay(TaskHandle_t task);
    
    // Task utilities
    static void vTaskGetInfo(TaskHandle_t task, void* task_status, BaseType_t include_stack_high_water_mark);
    static void vTaskList(char* write_buffer);
    static void vTaskGetRunTimeStats(char* write_buffer);
    
    // Internal scheduler access
    static void set_scheduler(TaskScheduler* scheduler) { scheduler_ = scheduler; }

private:
    static TaskScheduler* scheduler_;
    static Task* handle_to_task(TaskHandle_t handle);
    static TaskHandle_t task_to_handle(Task* task);
};

} // namespace m5tab5::emulator::freertos