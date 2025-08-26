/**
 * @file FreeRTOS.h
 * @brief FreeRTOS compatibility header for M5Stack Tab5 Emulator
 * 
 * This header provides basic FreeRTOS type definitions and macros
 * for ESP-IDF compatibility in the emulator environment.
 */

#pragma once

#include "../esp_types.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task priorities and configuration
#ifndef configMAX_PRIORITIES
#define configMAX_PRIORITIES        25
#endif

#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ          1000
#endif

#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE    512
#endif

// Task states
typedef enum {
    eRunning = 0,
    eReady,
    eBlocked,
    eSuspended,
    eDeleted,
    eInvalid
} eTaskState;

// Basic FreeRTOS types already defined in esp_types.h
// typedef void* TaskHandle_t;
// typedef void* QueueHandle_t;
// typedef void* SemaphoreHandle_t;
// typedef uint32_t TickType_t;

// Task function type
typedef void (*TaskFunction_t)(void* pvParameters);

// Priority definitions
// Priority definitions (avoid redefinition)
#ifndef tskIDLE_PRIORITY
#define tskIDLE_PRIORITY            ((UBaseType_t)0U)
#endif
#ifndef ESP_TASK_PRIO_MIN
#define ESP_TASK_PRIO_MIN           tskIDLE_PRIORITY
#endif
#ifndef ESP_TASK_PRIO_MAX
#define ESP_TASK_PRIO_MAX           (configMAX_PRIORITIES - 1)
#endif

// Tick and timing macros
#ifndef portMAX_DELAY
#define portMAX_DELAY               ((TickType_t)0xFFFFFFFFUL)
#endif

#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS          ((TickType_t)(1000 / configTICK_RATE_HZ))
#endif

// Convert between ticks and milliseconds (avoid redefinition)
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(xTimeInMs)    ((TickType_t)(((uint64_t)(xTimeInMs) * (uint64_t)configTICK_RATE_HZ) / (uint64_t)1000))
#endif
#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks)       ((uint32_t)(((uint64_t)(xTicks) * (uint64_t)1000) / (uint64_t)configTICK_RATE_HZ))
#endif

// Basic type definitions
typedef unsigned long UBaseType_t;
typedef long BaseType_t;

// Return values
#define pdTRUE                      ((BaseType_t)1)
#define pdFALSE                     ((BaseType_t)0)
#define pdPASS                      (pdTRUE)
#define pdFAIL                      (pdFALSE)

// Memory management
#define pvPortMalloc                malloc
#define vPortFree                   free

// Critical sections (simplified for emulation)
#define portENTER_CRITICAL()        
#define portEXIT_CRITICAL()         
#define portENTER_CRITICAL_ISR()    0
#define portEXIT_CRITICAL_ISR(x)    (void)(x)

// Interrupt management (no-op in emulation)
#define portDISABLE_INTERRUPTS()    0
#define portENABLE_INTERRUPTS()     
#define portSET_INTERRUPT_MASK_FROM_ISR()   0
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) (void)(x)

// Scheduler control (no-op in emulation)
#define portYIELD()                 
#define taskYIELD()                 
#define portYIELD_FROM_ISR(x)       (void)(x)

// Task creation and management (emulated)
#define ESP_TASK_MAIN_STACK         (4096)
#define ESP_TASK_MAIN_PRIO          1

#ifdef __cplusplus
}
#endif