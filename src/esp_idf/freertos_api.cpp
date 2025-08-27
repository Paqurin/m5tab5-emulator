#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/freertos/task.hpp"
#include "emulator/freertos/synchronization.hpp"
#include "emulator/utils/logging.hpp"

#include <cstring>
#include <thread>
#include <functional>

/**
 * @file freertos_api.cpp
 * @brief ESP-IDF FreeRTOS API compatibility layer
 * 
 * This file provides the standard FreeRTOS API functions that ESP-IDF applications
 * expect, mapping them to our emulated FreeRTOS kernel implementation.
 */

using namespace m5tab5::emulator::freertos;

DECLARE_LOGGER("ESP_IDF_FreeRTOS");

// Task Management Functions
extern "C" {

/**
 * @brief Create a new task
 */
BaseType_t xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint16_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask
) {
    COMPONENT_LOG_DEBUG("xTaskCreate: {} with priority {}", pcName ? pcName : "unnamed", uxPriority);
    
    return TaskAPI::xTaskCreate(
        [pxTaskCode](void* params) { pxTaskCode(params); },
        pcName, 
        usStackDepth, 
        pvParameters, 
        uxPriority, 
        pxCreatedTask
    );
}

/**
 * @brief Create a task pinned to a specific core
 */
BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint16_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask,
    const BaseType_t xCoreID
) {
    COMPONENT_LOG_DEBUG("xTaskCreatePinnedToCore: {} on core {} with priority {}", 
                       pcName ? pcName : "unnamed", xCoreID, uxPriority);
    
    return TaskAPI::xTaskCreatePinnedToCore(
        [pxTaskCode](void* params) { pxTaskCode(params); },
        pcName,
        usStackDepth,
        pvParameters,
        uxPriority,
        pxCreatedTask,
        xCoreID
    );
}

/**
 * @brief Delete a task
 */
void vTaskDelete(TaskHandle_t xTaskToDelete) {
    COMPONENT_LOG_DEBUG("vTaskDelete: {}", static_cast<void*>(xTaskToDelete));
    TaskAPI::vTaskDelete(xTaskToDelete);
}

/**
 * @brief Delay the current task for a specified number of ticks
 */
void vTaskDelay(const TickType_t xTicksToDelay) {
    COMPONENT_LOG_TRACE("vTaskDelay: {} ticks", xTicksToDelay);
    TaskAPI::vTaskDelay(xTicksToDelay);
}

/**
 * @brief Delay a task until a specific time
 */
void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement) {
    COMPONENT_LOG_TRACE("vTaskDelayUntil: increment {} ticks", xTimeIncrement);
    TaskAPI::vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}

/**
 * @brief Suspend a task
 */
void vTaskSuspend(TaskHandle_t xTaskToSuspend) {
    COMPONENT_LOG_DEBUG("vTaskSuspend: {}", static_cast<void*>(xTaskToSuspend));
    TaskAPI::vTaskSuspend(xTaskToSuspend);
}

/**
 * @brief Resume a suspended task
 */
void vTaskResume(TaskHandle_t xTaskToResume) {
    COMPONENT_LOG_DEBUG("vTaskResume: {}", static_cast<void*>(xTaskToResume));
    TaskAPI::vTaskResume(xTaskToResume);
}

/**
 * @brief Resume a task from an ISR
 */
BaseType_t xTaskResumeFromISR(TaskHandle_t xTaskToResume) {
    COMPONENT_LOG_DEBUG("xTaskResumeFromISR: {}", static_cast<void*>(xTaskToResume));
    return TaskAPI::xTaskResumeFromISR(xTaskToResume);
}

/**
 * @brief Set task priority
 */
void vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority) {
    COMPONENT_LOG_DEBUG("vTaskPrioritySet: {} to priority {}", static_cast<void*>(xTask), uxNewPriority);
    TaskAPI::vTaskPrioritySet(xTask, uxNewPriority);
}

/**
 * @brief Get task priority
 */
UBaseType_t uxTaskPriorityGet(TaskHandle_t xTask) {
    auto priority = TaskAPI::uxTaskPriorityGet(xTask);
    COMPONENT_LOG_TRACE("uxTaskPriorityGet: {} has priority {}", static_cast<void*>(xTask), priority);
    return priority;
}

/**
 * @brief Get current task handle
 */
TaskHandle_t xTaskGetCurrentTaskHandle(void) {
    auto handle = TaskAPI::xTaskGetCurrentTaskHandle();
    COMPONENT_LOG_TRACE("xTaskGetCurrentTaskHandle: {}", static_cast<void*>(handle));
    return handle;
}

/**
 * @brief Get task name
 */
char* pcTaskGetName(TaskHandle_t xTaskToQuery) {
    return TaskAPI::pcTaskGetName(xTaskToQuery);
}

/**
 * @brief Get current tick count
 */
TickType_t xTaskGetTickCount(void) {
    return TaskAPI::xTaskGetTickCount();
}

/**
 * @brief Get tick count from ISR
 */
TickType_t xTaskGetTickCountFromISR(void) {
    return TaskAPI::xTaskGetTickCountFromISR();
}

/**
 * @brief Get stack high water mark
 */
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask) {
    return TaskAPI::uxTaskGetStackHighWaterMark(xTask);
}

/**
 * @brief Start the FreeRTOS scheduler
 */
void vTaskStartScheduler(void) {
    COMPONENT_LOG_INFO("vTaskStartScheduler called");
    TaskAPI::vTaskStartScheduler();
}

/**
 * @brief End the FreeRTOS scheduler
 */
void vTaskEndScheduler(void) {
    COMPONENT_LOG_INFO("vTaskEndScheduler called");
    TaskAPI::vTaskEndScheduler();
}

/**
 * @brief Suspend all tasks
 */
void vTaskSuspendAll(void) {
    COMPONENT_LOG_DEBUG("vTaskSuspendAll called");
    TaskAPI::vTaskSuspendAll();
}

/**
 * @brief Resume all tasks
 */
BaseType_t xTaskResumeAll(void) {
    COMPONENT_LOG_DEBUG("xTaskResumeAll called");
    return TaskAPI::xTaskResumeAll();
}

// Synchronization Functions

/**
 * @brief Create a binary semaphore
 */
SemaphoreHandle_t xSemaphoreCreateBinary(void) {
    COMPONENT_LOG_DEBUG("xSemaphoreCreateBinary called");
    return SynchronizationAPI::xSemaphoreCreateBinary();
}

/**
 * @brief Create a counting semaphore
 */
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount) {
    COMPONENT_LOG_DEBUG("xSemaphoreCreateCounting: max={}, initial={}", uxMaxCount, uxInitialCount);
    return SynchronizationAPI::xSemaphoreCreateCounting(uxMaxCount, uxInitialCount);
}

/**
 * @brief Create a mutex
 */
SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    COMPONENT_LOG_DEBUG("xSemaphoreCreateMutex called");
    return SynchronizationAPI::xSemaphoreCreateMutex();
}

/**
 * @brief Take a semaphore
 */
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait) {
    COMPONENT_LOG_TRACE("xSemaphoreTake: {} with timeout {}", static_cast<void*>(xSemaphore), xTicksToWait);
    return SynchronizationAPI::xSemaphoreTake(xSemaphore, xTicksToWait);
}

/**
 * @brief Give a semaphore
 */
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    COMPONENT_LOG_TRACE("xSemaphoreGive: {}", static_cast<void*>(xSemaphore));
    return SynchronizationAPI::xSemaphoreGive(xSemaphore);
}

/**
 * @brief Take a semaphore from ISR
 */
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t xSemaphore, BaseType_t *pxHigherPriorityTaskWoken) {
    COMPONENT_LOG_TRACE("xSemaphoreTakeFromISR: {}", static_cast<void*>(xSemaphore));
    return SynchronizationAPI::xSemaphoreTakeFromISR(xSemaphore, pxHigherPriorityTaskWoken);
}

/**
 * @brief Give a semaphore from ISR
 */
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore, BaseType_t *pxHigherPriorityTaskWoken) {
    COMPONENT_LOG_TRACE("xSemaphoreGiveFromISR: {}", static_cast<void*>(xSemaphore));
    return SynchronizationAPI::xSemaphoreGiveFromISR(xSemaphore, pxHigherPriorityTaskWoken);
}

/**
 * @brief Create a queue
 */
QueueHandle_t xQueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize) {
    COMPONENT_LOG_DEBUG("xQueueCreate: length={}, item_size={}", uxQueueLength, uxItemSize);
    return SynchronizationAPI::xQueueCreate(uxQueueLength, uxItemSize);
}

/**
 * @brief Send to queue
 */
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    COMPONENT_LOG_TRACE("xQueueSend: {} with timeout {}", static_cast<void*>(xQueue), xTicksToWait);
    return SynchronizationAPI::xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

/**
 * @brief Receive from queue
 */
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    COMPONENT_LOG_TRACE("xQueueReceive: {} with timeout {}", static_cast<void*>(xQueue), xTicksToWait);
    return SynchronizationAPI::xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}

/**
 * @brief Send to queue from ISR
 */
BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken) {
    COMPONENT_LOG_TRACE("xQueueSendFromISR: {}", static_cast<void*>(xQueue));
    return SynchronizationAPI::xQueueSendFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

/**
 * @brief Receive from queue from ISR
 */
BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxHigherPriorityTaskWoken) {
    COMPONENT_LOG_TRACE("xQueueReceiveFromISR: {}", static_cast<void*>(xQueue));
    return SynchronizationAPI::xQueueReceiveFromISR(xQueue, pvBuffer, pxHigherPriorityTaskWoken);
}

/**
 * @brief Get number of messages waiting in queue
 */
UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue) {
    return SynchronizationAPI::uxQueueMessagesWaiting(xQueue);
}

// Core Information Functions

/**
 * @brief Get current core ID
 */
BaseType_t xPortGetCoreID(void) {
    // For emulation, we can simulate core detection
    // In a real system, this would read a CPU register
    static thread_local BaseType_t core_id = -1;
    
    if (core_id == -1) {
        // Simple thread-based core assignment for emulation
        auto thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
        core_id = static_cast<BaseType_t>(thread_id % 2); // 0 or 1
    }
    
    COMPONENT_LOG_TRACE("xPortGetCoreID: {}", core_id);
    return core_id;
}

// Memory Management Functions

/**
 * @brief Allocate memory from heap
 */
void* pvPortMalloc(size_t xSize) {
    COMPONENT_LOG_TRACE("pvPortMalloc: {} bytes", xSize);
    // For now, use standard malloc - could be enhanced with FreeRTOS heap management
    return malloc(xSize);
}

/**
 * @brief Free memory to heap
 */
void vPortFree(void *pv) {
    COMPONENT_LOG_TRACE("vPortFree: {}", pv);
    free(pv);
}

/**
 * @brief Get free heap size
 */
size_t xPortGetFreeHeapSize(void) {
    // This would need integration with the emulator's memory management
    // For now, return a reasonable value
    const size_t free_heap = 64 * 1024; // 64KB
    COMPONENT_LOG_TRACE("xPortGetFreeHeapSize: {} bytes", free_heap);
    return free_heap;
}

/**
 * @brief Get minimum ever free heap size
 */
size_t xPortGetMinimumEverFreeHeapSize(void) {
    // This would track the minimum heap size
    const size_t min_free_heap = 32 * 1024; // 32KB
    COMPONENT_LOG_TRACE("xPortGetMinimumEverFreeHeapSize: {} bytes", min_free_heap);
    return min_free_heap;
}

// Critical Section Functions

/**
 * @brief Enter critical section
 */
void vTaskEnterCritical(void) {
    COMPONENT_LOG_TRACE("vTaskEnterCritical called");
    // In a real implementation, this would disable interrupts
    // For emulation, we could use a mutex or atomic operations
}

/**
 * @brief Exit critical section
 */
void vTaskExitCritical(void) {
    COMPONENT_LOG_TRACE("vTaskExitCritical called");
    // In a real implementation, this would re-enable interrupts
}

/**
 * @brief Enter critical section from ISR
 */
UBaseType_t taskENTER_CRITICAL_FROM_ISR(void) {
    COMPONENT_LOG_TRACE("taskENTER_CRITICAL_FROM_ISR called");
    return 0; // Return interrupt status
}

/**
 * @brief Exit critical section from ISR
 */
void taskEXIT_CRITICAL_FROM_ISR(UBaseType_t uxSavedInterruptStatus) {
    COMPONENT_LOG_TRACE("taskEXIT_CRITICAL_FROM_ISR called");
    (void)uxSavedInterruptStatus; // Restore interrupt status
}

/**
 * @brief Yield to higher priority task
 */
void taskYIELD(void) {
    COMPONENT_LOG_TRACE("taskYIELD called");
    // This would trigger a context switch to a higher priority task
    if (auto scheduler = SchedulerManager::get_instance()) {
        scheduler->yield_current_task();
    }
}

} // extern "C"

/**
 * @brief Initialize FreeRTOS API compatibility layer
 */
namespace m5tab5::emulator::esp_idf {

void initialize_freertos_api() {
    COMPONENT_LOG_INFO("ESP-IDF FreeRTOS API compatibility layer initialized");
    
    // Any additional initialization for the API layer would go here
    // For example, setting up interrupt handlers, timers, etc.
}

void shutdown_freertos_api() {
    COMPONENT_LOG_INFO("ESP-IDF FreeRTOS API compatibility layer shutdown");
}

} // namespace m5tab5::emulator::esp_idf