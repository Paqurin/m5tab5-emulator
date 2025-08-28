#pragma once

/**
 * @file freertos_api.hpp
 * @brief ESP-IDF FreeRTOS API compatibility layer header
 * 
 * This header provides the standard FreeRTOS API that ESP-IDF applications
 * expect. It maps FreeRTOS function calls to our emulated kernel implementation.
 */

#include "emulator/core/types.hpp"
#include "freertos/FreeRTOS.h"
#include <cstdint>
#include <cstddef>

// FreeRTOS type definitions are handled by our FreeRTOS.h header
// which defines BaseType_t as long and UBaseType_t as unsigned long
// We don't redefine them here to avoid conflicts

// Add missing types that aren't in our FreeRTOS.h
#ifndef TickType_t
using TickType_t = uint32_t;
#endif
#ifndef StackType_t
using StackType_t = uint8_t;
#endif

// Handle types
using TaskHandle_t = void*;
using SemaphoreHandle_t = void*;
using QueueHandle_t = void*;
using TimerHandle_t = void*;

// Function pointer types
using TaskFunction_t = void(*)(void*);
using TimerCallbackFunction_t = void(*)(TimerHandle_t);

// FreeRTOS constants - only define if not already defined
#ifndef pdTRUE
#define pdTRUE                          1
#endif
#ifndef pdFALSE
#define pdFALSE                         0
#endif
#ifndef pdPASS
#define pdPASS                          1  
#endif
#ifndef pdFAIL
#define pdFAIL                          0
#endif

#define errQUEUE_EMPTY                  0
#define errQUEUE_FULL                   0

// Task priorities
#define tskIDLE_PRIORITY                0
#define configMAX_PRIORITIES           25
#define configMINIMAL_STACK_SIZE      512

// Timing - only define if not already defined
#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS              1
#endif
#ifndef portMAX_DELAY
#define portMAX_DELAY                   0xFFFFFFFF
#endif

// ESP32 specific
#define configNUM_CORES                 2
#ifndef tskNO_AFFINITY
#define tskNO_AFFINITY                  0x7FFFFFFF
#endif
#define portNUM_PROCESSORS              2
#define configTICK_RATE_HZ              1000

// Timing conversion macros - only define if not already defined
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(xTimeInMs)        ((TickType_t)((xTimeInMs) * configTICK_RATE_HZ / 1000))
#endif

// Include guard to prevent conflicts with real FreeRTOS
#ifndef INC_FREERTOS_H
#define INC_FREERTOS_H

extern "C" {

// Task Management
BaseType_t xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint16_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask
);

BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint16_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask,
    const BaseType_t xCoreID
);

void vTaskDelete(TaskHandle_t xTaskToDelete);
void vTaskDelay(const TickType_t xTicksToDelay);
void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);
void vTaskSuspend(TaskHandle_t xTaskToSuspend);
void vTaskResume(TaskHandle_t xTaskToResume);
BaseType_t xTaskResumeFromISR(TaskHandle_t xTaskToResume);

// Priority Management
void vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority);
UBaseType_t uxTaskPriorityGet(TaskHandle_t xTask);

// Task Information
TaskHandle_t xTaskGetCurrentTaskHandle(void);
char* pcTaskGetName(TaskHandle_t xTaskToQuery);
TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);

// Scheduler Control
void vTaskStartScheduler(void);
void vTaskEndScheduler(void);
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);

// Semaphore Management
SemaphoreHandle_t xSemaphoreCreateBinary(void);
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount);
SemaphoreHandle_t xSemaphoreCreateMutex(void);

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
// Only declare ISR functions if not already defined as macros  
#ifndef xSemaphoreTakeFromISR
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t xSemaphore, BaseType_t *pxHigherPriorityTaskWoken);
#endif
#ifndef xSemaphoreGiveFromISR
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore, BaseType_t *pxHigherPriorityTaskWoken);
#endif

// Queue Management
QueueHandle_t xQueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize);
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);
// Only declare ISR functions if not already defined as macros
#ifndef xQueueSendFromISR
BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken);
#endif
#ifndef xQueueReceiveFromISR
BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxHigherPriorityTaskWoken);
#endif
UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue);

// Core Information
BaseType_t xPortGetCoreID(void);

// Memory Management
void* pvPortMalloc(size_t xSize);
void vPortFree(void *pv);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);

// Critical Sections
void vTaskEnterCritical(void);
void vTaskExitCritical(void);
UBaseType_t taskENTER_CRITICAL_FROM_ISR(void);
void taskEXIT_CRITICAL_FROM_ISR(UBaseType_t uxSavedInterruptStatus);

// Task Yield - only declare if not already defined as macro
#ifndef taskYIELD
void taskYIELD(void);
#endif

} // extern "C"

#endif // INC_FREERTOS_H

// Convenience macros for common operations - only define if not already defined
#ifndef taskENTER_CRITICAL
#define taskENTER_CRITICAL()            vTaskEnterCritical()
#endif
#ifndef taskEXIT_CRITICAL
#define taskEXIT_CRITICAL()             vTaskExitCritical()
#endif
#ifndef portYIELD
#define portYIELD()                     taskYIELD()
#endif
#ifndef portYIELD_FROM_ISR
#define portYIELD_FROM_ISR(x)           do { if(x) taskYIELD(); } while(0)
#endif

// Semaphore macros
#define xSemaphoreCreateBinary()        xSemaphoreCreateBinary()
#define xSemaphoreTake(sem, timeout)    xSemaphoreTake(sem, timeout)
#define xSemaphoreGive(sem)             xSemaphoreGive(sem)

// Queue macros  
#define xQueueSend(queue, item, timeout) xQueueSend(queue, item, timeout)
#define xQueueReceive(queue, buffer, timeout) xQueueReceive(queue, buffer, timeout)

namespace m5tab5::emulator::esp_idf {

/**
 * @brief Initialize the ESP-IDF FreeRTOS API compatibility layer
 */
void initialize_freertos_api();

/**
 * @brief Shutdown the ESP-IDF FreeRTOS API compatibility layer
 */
void shutdown_freertos_api();

} // namespace m5tab5::emulator::esp_idf