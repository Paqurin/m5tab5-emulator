#pragma once

/**
 * @brief FreeRTOS Kernel Emulation for ESP32-P4
 * 
 * This header provides the main interface for FreeRTOS kernel emulation
 * in the M5Stack Tab5 ESP32-P4 emulator. It includes all necessary 
 * components for running ESP-IDF applications that depend on FreeRTOS.
 */

#include "emulator/freertos/task.hpp"
#include "emulator/freertos/scheduler.hpp"
#include "emulator/freertos/synchronization.hpp"
#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/cpu/dual_core_manager.hpp"

namespace m5tab5::emulator::freertos {

/**
 * @brief FreeRTOS Kernel Manager
 * 
 * Provides high-level interface for initializing and managing the FreeRTOS
 * kernel emulation. This integrates with the EmulatorCore and provides
 * the operating system foundation for ESP32-P4 applications.
 */
class FreeRTOSKernel {
public:
    struct KernelConfig {
        uint32_t tick_frequency_hz = 1000;
        uint32_t idle_task_stack_size = 2048;
        uint32_t timer_task_stack_size = 2048;
        uint32_t timer_queue_length = 10;
        bool enable_task_statistics = true;
        bool enable_stack_overflow_checking = true;
        bool enable_memory_management = true;
        uint32_t total_heap_size = 64 * 1024; // 64KB heap
    };

    explicit FreeRTOSKernel(DualCoreManager& cpu_manager);
    ~FreeRTOSKernel();

    // Kernel lifecycle
    Result<void> initialize(const KernelConfig& config);
    Result<void> start_kernel();
    Result<void> stop_kernel();
    Result<void> reset_kernel();
    
    // Scheduler access
    TaskScheduler* get_scheduler() { return scheduler_.get(); }
    const TaskScheduler* get_scheduler() const { return scheduler_.get(); }
    
    // Kernel state
    bool is_kernel_running() const;
    TickType_t get_kernel_tick_count() const;
    
    // Task management convenience methods
    Result<TaskHandle_t> create_task(
        const std::string& name,
        TaskFunction_t function,
        void* parameters = nullptr,
        uint16_t stack_size = 2048,
        UBaseType_t priority = 5,
        UBaseType_t core_affinity = 2
    );
    
    // Synchronization convenience methods
    SemaphoreHandle_t create_binary_semaphore();
    SemaphoreHandle_t create_counting_semaphore(UBaseType_t max_count, UBaseType_t initial_count = 0);
    SemaphoreHandle_t create_mutex();
    QueueHandle_t create_queue(UBaseType_t length, UBaseType_t item_size);
    
    // Kernel information
    struct KernelInfo {
        uint32_t tick_frequency;
        uint32_t total_tasks;
        uint32_t running_tasks;
        uint32_t blocked_tasks;
        uint32_t total_heap_size;
        uint32_t free_heap_size;
        uint64_t total_runtime_ticks;
        uint32_t idle_percentage;
    };
    
    KernelInfo get_kernel_info() const;
    
    // Performance and debugging
    void dump_task_list() const;
    void dump_kernel_statistics() const;
    void print_stack_high_water_marks() const;

private:
    DualCoreManager& cpu_manager_;
    KernelConfig config_;
    
    std::unique_ptr<TaskScheduler> scheduler_;
    bool kernel_initialized_;
    bool kernel_running_;
    
    // Built-in kernel tasks
    TaskHandle_t timer_task_handle_;
    
    // Heap management (simplified)
    std::unique_ptr<uint8_t[]> heap_memory_;
    uint32_t heap_free_bytes_;
    mutable std::mutex heap_mutex_;
    
    // Internal initialization
    Result<void> initialize_heap();
    Result<void> create_timer_task();
    
    // Timer task implementation
    static void timer_task_function(void* parameters);
};

/**
 * @brief Global FreeRTOS kernel instance management
 * 
 * Provides singleton access to the kernel for ESP-IDF API compatibility.
 */
class FreeRTOSKernelManager {
public:
    static void set_instance(FreeRTOSKernel* kernel);
    static FreeRTOSKernel* get_instance();
    static bool has_instance();
    
    // Convenience methods that delegate to the active kernel instance
    static bool is_kernel_running();
    static TickType_t get_tick_count();
    static TaskHandle_t get_current_task();

private:
    static FreeRTOSKernel* instance_;
    static std::mutex instance_mutex_;
};

} // namespace m5tab5::emulator::freertos

// FreeRTOS API compatibility macros and definitions
// These provide the standard FreeRTOS API that ESP-IDF applications expect

// Include guard to prevent conflicts with real FreeRTOS headers
#ifndef FREERTOS_H
#define FREERTOS_H

// FreeRTOS constants
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define pdFAIL  0

// Include standard task creation macros
#define tskIDLE_PRIORITY                0
#define configMAX_PRIORITIES           25
#define configMINIMAL_STACK_SIZE       512
#define configTIMER_TASK_PRIORITY      (configMAX_PRIORITIES - 1)
#define portMAX_DELAY                  0xFFFFFFFF

// ESP32-specific constants
#define configNUM_CORES                2
#define tskNO_AFFINITY                 INT_MAX
#define portTICK_PERIOD_MS             1

// FreeRTOS API Compatibility - Use direct class access
// For ESP-IDF applications, include specific headers:
// - #include "emulator/freertos/task.hpp" 
// - #include "emulator/freertos/synchronization.hpp"
//
// Use API classes directly to avoid namespace conflicts:
// - TaskAPI::xTaskCreate(...)
// - SynchronizationAPI::xSemaphoreCreateBinary()
//
// Macros commented out due to preprocessor namespace conflicts
/*
#define xTaskCreate                    m5tab5::emulator::freertos::TaskAPI::xTaskCreate
#define xTaskCreatePinnedToCore        m5tab5::emulator::freertos::TaskAPI::xTaskCreatePinnedToCore
#define vTaskDelete                    m5tab5::emulator::freertos::TaskAPI::vTaskDelete
#define vTaskDelay                     m5tab5::emulator::freertos::TaskAPI::vTaskDelay
#define vTaskDelayUntil                m5tab5::emulator::freertos::TaskAPI::vTaskDelayUntil
#define vTaskSuspend                   m5tab5::emulator::freertos::TaskAPI::vTaskSuspend
#define vTaskResume                    m5tab5::emulator::freertos::TaskAPI::vTaskResume
#define vTaskPrioritySet               m5tab5::emulator::freertos::TaskAPI::vTaskPrioritySet
#define uxTaskPriorityGet              m5tab5::emulator::freertos::TaskAPI::uxTaskPriorityGet
#define xTaskGetCurrentTaskHandle      m5tab5::emulator::freertos::TaskAPI::xTaskGetCurrentTaskHandle
#define pcTaskGetTaskName              m5tab5::emulator::freertos::TaskAPI::pcTaskGetName
#define uxTaskGetNumberOfTasks         m5tab5::emulator::freertos::TaskAPI::uxTaskGetNumberOfTasks
#define xTaskGetTickCount              m5tab5::emulator::freertos::TaskAPI::xTaskGetTickCount
#define xTaskGetTickCountFromISR       m5tab5::emulator::freertos::TaskAPI::xTaskGetTickCountFromISR
#define uxTaskGetStackHighWaterMark    m5tab5::emulator::freertos::TaskAPI::uxTaskGetStackHighWaterMark

// Scheduler control macros
#define vTaskStartScheduler            m5tab5::emulator::freertos::TaskAPI::vTaskStartScheduler
#define vTaskEndScheduler              m5tab5::emulator::freertos::TaskAPI::vTaskEndScheduler
#define vTaskSuspendAll                m5tab5::emulator::freertos::TaskAPI::vTaskSuspendAll
#define xTaskResumeAll                 m5tab5::emulator::freertos::TaskAPI::xTaskResumeAll

// Synchronization macros
#define xSemaphoreCreateBinary         m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreCreateBinary
#define xSemaphoreCreateCounting       m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreCreateCounting
#define xSemaphoreCreateMutex          m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreCreateMutex
#define xSemaphoreTake                 m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreTake
#define xSemaphoreGive                 m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreGive
#define xSemaphoreTakeFromISR          m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreTakeFromISR
#define xSemaphoreGiveFromISR          m5tab5::emulator::freertos::SynchronizationAPI::xSemaphoreGiveFromISR

#define xQueueCreate                   m5tab5::emulator::freertos::SynchronizationAPI::xQueueCreate
#define xQueueSend                     m5tab5::emulator::freertos::SynchronizationAPI::xQueueSend
#define xQueueSendFromISR              m5tab5::emulator::freertos::SynchronizationAPI::xQueueSendFromISR
#define xQueueReceive                  m5tab5::emulator::freertos::SynchronizationAPI::xQueueReceive
#define xQueueReceiveFromISR           m5tab5::emulator::freertos::SynchronizationAPI::xQueueReceiveFromISR
#define uxQueueMessagesWaiting         m5tab5::emulator::freertos::SynchronizationAPI::uxQueueMessagesWaiting
*/

// Type definitions for compatibility
using TaskHandle_t = m5tab5::emulator::freertos::TaskHandle_t;
using TaskFunction_t = m5tab5::emulator::freertos::TaskFunction_t;
using SemaphoreHandle_t = m5tab5::emulator::freertos::SemaphoreHandle_t;
using QueueHandle_t = m5tab5::emulator::freertos::QueueHandle_t;
using TickType_t = m5tab5::emulator::freertos::TickType_t;
using BaseType_t = m5tab5::emulator::freertos::BaseType_t;
using UBaseType_t = m5tab5::emulator::freertos::UBaseType_t;

#endif // FREERTOS_H