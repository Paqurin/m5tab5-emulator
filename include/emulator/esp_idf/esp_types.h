/**
 * @file esp_types.h
 * @brief ESP-IDF type definitions for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible type definitions and constants
 * used throughout the ESP-IDF API emulation layer.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP error type definition
 */
typedef int esp_err_t;

/**
 * @brief ESP-IDF error codes
 */
#define ESP_OK          0       ///< esp_err_t value indicating success (no error)
#define ESP_FAIL        -1      ///< Generic esp_err_t code indicating failure

#define ESP_ERR_NO_MEM          0x101   ///< Out of memory
#define ESP_ERR_INVALID_ARG     0x102   ///< Invalid argument
#define ESP_ERR_INVALID_STATE   0x103   ///< Invalid state
#define ESP_ERR_INVALID_SIZE    0x104   ///< Invalid size
#define ESP_ERR_NOT_FOUND       0x105   ///< Requested resource not found
#define ESP_ERR_NOT_SUPPORTED   0x106   ///< Operation or feature not supported
#define ESP_ERR_TIMEOUT         0x107   ///< Operation timed out
#define ESP_ERR_INVALID_RESPONSE 0x108  ///< Received response was invalid
#define ESP_ERR_INVALID_CRC     0x109   ///< CRC or checksum was invalid
#define ESP_ERR_INVALID_VERSION 0x10A   ///< Version was invalid
#define ESP_ERR_INVALID_MAC     0x10B   ///< MAC address was invalid
#define ESP_ERR_NOT_FINISHED    0x10C   ///< Operation has not fully completed

#define ESP_ERR_WIFI_BASE       0x3001  ///< Starting number of WiFi error codes
#define ESP_ERR_MESH_BASE       0x4001  ///< Starting number of MESH error codes
#define ESP_ERR_FLASH_BASE      0x6001  ///< Starting number of flash error codes

/**
 * @brief FreeRTOS tick type (for compatibility)
 */
#ifndef portTICK_TYPE_IS_ATOMIC
typedef uint32_t TickType_t;
#define portMAX_DELAY    ((TickType_t) 0xFFFFFFFFUL)
#define portTICK_PERIOD_MS ((TickType_t) 1000 / configTICK_RATE_HZ)
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY    ((TickType_t) 0xFFFFFFFFUL)
#define portTICK_PERIOD_MS 1  // Simplified for emulation
#endif

/**
 * @brief Task handle type (for compatibility)
 */
typedef void* TaskHandle_t;

/**
 * @brief Queue handle type (for compatibility)
 */
typedef void* QueueHandle_t;

/**
 * @brief Semaphore handle type (for compatibility)
 */
typedef void* SemaphoreHandle_t;

/**
 * @brief Mutex handle type (for compatibility)
 */
typedef SemaphoreHandle_t MutexHandle_t;

/**
 * @brief Event group handle type (for compatibility)
 */
typedef void* EventGroupHandle_t;

/**
 * @brief Timer handle type (for compatibility)
 */
typedef void* TimerHandle_t;

/**
 * @brief Base type definitions
 */
#ifndef __cplusplus
typedef enum {
    false = 0,
    true = 1
} bool;
#endif

/**
 * @brief Attribute to place code in IRAM (ignored in emulation)
 */
#define IRAM_ATTR

/**
 * @brief Attribute to place code in DRAM (ignored in emulation)
 */
#define DRAM_ATTR

/**
 * @brief Attribute for interrupt service routines (ignored in emulation)
 */
#define ISR_ATTR

/**
 * @brief Macro to check if an error occurred and return if so
 */
#define ESP_ERROR_CHECK(x) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        return __err_rc; \
    } \
} while(0)

/**
 * @brief Macro to check if an error occurred and return void if so
 */
#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        return; \
    } \
} while(0)

/**
 * @brief Macro for asserting conditions (simplified for emulation)
 */
#define ESP_ASSERT(condition) assert(condition)

/**
 * @brief Convert milliseconds to FreeRTOS ticks
 */
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

/**
 * @brief Convert FreeRTOS ticks to milliseconds
 */
#define pdTICKS_TO_MS(ticks) ((uint32_t)(ticks))

/**
 * @brief FreeRTOS task priorities (for compatibility)
 */
#define tskIDLE_PRIORITY            0
#define configMAX_PRIORITIES        25
#define configTICK_RATE_HZ          1000

/**
 * @brief FreeRTOS task stack sizes (for compatibility)
 */
#define configMINIMAL_STACK_SIZE    512
#define ESP_TASK_PRIO_MIN           0
#define ESP_TASK_PRIO_MAX           (configMAX_PRIORITIES - 1)

/**
 * @brief FreeRTOS memory allocation functions (for compatibility)
 */
#define pvPortMalloc    malloc
#define vPortFree       free

/**
 * @brief Critical section macros (simplified for emulation)
 */
#define portENTER_CRITICAL()
#define portEXIT_CRITICAL()
#define portENTER_CRITICAL_ISR()    0
#define portEXIT_CRITICAL_ISR(x)    (void)(x)

/**
 * @brief Disable/enable interrupts (no-op in emulation)
 */
#define portDISABLE_INTERRUPTS()    0
#define portENABLE_INTERRUPTS()

/**
 * @brief Yield to scheduler (no-op in emulation)
 */
#define portYIELD()
#define taskYIELD()

/**
 * @brief Attribute for packed structures
 */
#define __packed __attribute__((packed))

/**
 * @brief Alignment attributes
 */
#define __align(x) __attribute__((aligned(x)))

/**
 * @brief Macro to mark unused parameters
 */
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/**
 * @brief Size of various system constants
 */
#define ESP_MAC_ADDR_LEN            6       ///< Length of MAC address in bytes
#define ESP_IP4_ADDR_LEN            4       ///< Length of IPv4 address in bytes
#define ESP_IP6_ADDR_LEN            16      ///< Length of IPv6 address in bytes

/**
 * @brief Maximum lengths for various strings
 */
#define ESP_MAX_SSID_LEN            32      ///< Maximum SSID length
#define ESP_MAX_PASSPHRASE_LEN      64      ///< Maximum passphrase length
#define ESP_MAX_HOSTNAME_LEN        32      ///< Maximum hostname length

/**
 * @brief ESP-IDF version information (emulated)
 */
#define ESP_IDF_VERSION_MAJOR       5
#define ESP_IDF_VERSION_MINOR       1
#define ESP_IDF_VERSION_PATCH       0
#define ESP_IDF_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH)

/**
 * @brief Target chip information (emulated as ESP32-P4)
 */
#define CONFIG_IDF_TARGET_ESP32P4   1
#define ESP_CHIP_ID_ESP32P4         13
#define SOC_CPU_CORES_NUM           2
#define SOC_CPU_HP_CORES_NUM        2
#define SOC_CPU_LP_CORES_NUM        1

/**
 * @brief Memory layout constants (ESP32-P4)
 */
#define SOC_DROM_LOW                0x40000000
#define SOC_DROM_HIGH               0x40FFFFFF
#define SOC_IROM_LOW                0x40000000
#define SOC_IROM_HIGH               0x40FFFFFF
#define SOC_DRAM_LOW                0x4FF00000
#define SOC_DRAM_HIGH               0x4FFBFFFF
#define SOC_IRAM_LOW                0x4FF00000
#define SOC_IRAM_HIGH               0x4FFBFFFF

/**
 * @brief APB and CPU clock frequencies (ESP32-P4)
 */
#define APB_CLK_FREQ                80000000    ///< APB clock frequency in Hz
#define CPU_CLK_FREQ                400000000   ///< CPU clock frequency in Hz
#define XTAL_CLK_FREQ               40000000    ///< Crystal oscillator frequency in Hz

#ifdef __cplusplus
}
#endif