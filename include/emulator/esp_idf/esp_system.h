/**
 * @file esp_system.h
 * @brief ESP-IDF system APIs for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible system initialization, reset, and
 * power management APIs that integrate with the emulated hardware.
 */

#pragma once

#include "esp_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Error codes and types are now defined in esp_types.h

/**
 * @brief Reset reasons
 */
typedef enum {
    ESP_RST_UNKNOWN,    ///< Reset reason cannot be determined
    ESP_RST_POWERON,    ///< Reset due to power-on event
    ESP_RST_EXT,        ///< Reset by external pin (not applicable for ESP32-P4)
    ESP_RST_SW,         ///< Software reset via esp_restart
    ESP_RST_PANIC,      ///< Software reset due to exception/panic
    ESP_RST_INT_WDT,    ///< Reset due to interrupt watchdog
    ESP_RST_TASK_WDT,   ///< Reset due to task watchdog
    ESP_RST_WDT,        ///< Reset due to other watchdogs
    ESP_RST_DEEPSLEEP,  ///< Reset after exiting deep sleep mode
    ESP_RST_BROWNOUT,   ///< Brownout reset (software or hardware)
    ESP_RST_SDIO,       ///< Reset over SDIO
} esp_reset_reason_t;

/**
 * @brief Sleep modes
 */
typedef enum {
    ESP_SLEEP_MODE_LIGHT,    ///< Light sleep mode
    ESP_SLEEP_MODE_DEEP,     ///< Deep sleep mode
    ESP_SLEEP_MODE_HIBERNATE ///< Hibernation mode
} esp_sleep_mode_t;

/**
 * @brief Wake-up sources for sleep modes
 */
typedef enum {
    ESP_SLEEP_WAKEUP_UNDEFINED = 0, ///< Wake-up reason undefined
    ESP_SLEEP_WAKEUP_ALL,           ///< Not a wake-up source, used to disable all sources
    ESP_SLEEP_WAKEUP_EXT0,          ///< Wake-up by external signal (RTC_IO)
    ESP_SLEEP_WAKEUP_EXT1,          ///< Wake-up by external signal (RTC_CNTL)
    ESP_SLEEP_WAKEUP_TIMER,         ///< Wake-up by timer
    ESP_SLEEP_WAKEUP_TOUCHPAD,      ///< Wake-up by touchpad
    ESP_SLEEP_WAKEUP_ULP,           ///< Wake-up by ULP program
    ESP_SLEEP_WAKEUP_GPIO,          ///< Wake-up by GPIO (light sleep only)
    ESP_SLEEP_WAKEUP_UART,          ///< Wake-up by UART (light sleep only)
    ESP_SLEEP_WAKEUP_WIFI,          ///< Wake-up by WiFi (light sleep only)
    ESP_SLEEP_WAKEUP_COCPU,         ///< Wake-up by COCPU int
    ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG, ///< Wake-up by COCPU crash
    ESP_SLEEP_WAKEUP_BT,            ///< Wake-up by BT (light sleep only)
} esp_sleep_wakeup_cause_t;

/**
 * @brief Mac address types
 */
typedef enum {
    ESP_MAC_WIFI_STA,
    ESP_MAC_WIFI_SOFTAP,
    ESP_MAC_BT,
    ESP_MAC_ETH,
    ESP_MAC_IEEE802154
} esp_mac_type_t;

// ============================================================================
// System Initialization and Info
// ============================================================================

/**
 * @brief Initialize ESP-IDF system
 * 
 * This function initializes core system components including memory management,
 * interrupt controller, and basic hardware. Must be called before using any
 * other ESP-IDF APIs.
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_system_init(void);

/**
 * @brief Get system information string
 * 
 * @return String describing the emulated ESP32-P4 system
 */
const char* esp_get_idf_version(void);

/**
 * @brief Get chip information
 * 
 * @param[out] chip_info Structure to store chip information
 * @return ESP_OK on success
 */
typedef struct {
    uint32_t cores;              ///< Number of CPU cores
    uint32_t features;           ///< Chip feature flags
    uint8_t revision;            ///< Chip revision
} esp_chip_info_t;

esp_err_t esp_chip_info(esp_chip_info_t* chip_info);

/**
 * @brief Get free heap size
 * 
 * @return Free heap size in bytes
 */
uint32_t esp_get_free_heap_size(void);

/**
 * @brief Get minimum ever free heap size
 * 
 * @return Minimum free heap size since system start
 */
uint32_t esp_get_minimum_free_heap_size(void);

// ============================================================================
// Reset and Restart
// ============================================================================

/**
 * @brief Restart the system
 * 
 * This function does not return. The emulator will restart after a brief delay.
 */
void esp_restart(void) __attribute__((noreturn));

/**
 * @brief Get reset reason
 * 
 * @return Reset reason for the current boot
 */
esp_reset_reason_t esp_reset_reason(void);

/**
 * @brief Register shutdown handler
 * 
 * @param handler Function to call during shutdown
 * @return ESP_OK on success
 */
typedef void (*shutdown_handler_t)(void);
esp_err_t esp_register_shutdown_handler(shutdown_handler_t handler);

/**
 * @brief Unregister shutdown handler
 * 
 * @param handler Previously registered handler
 * @return ESP_OK on success
 */
esp_err_t esp_unregister_shutdown_handler(shutdown_handler_t handler);

// ============================================================================
// Sleep and Power Management
// ============================================================================

/**
 * @brief Enable wake-up by timer
 * 
 * @param time_in_us Wake-up time in microseconds
 * @return ESP_OK on success
 */
esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us);

/**
 * @brief Enable wake-up by GPIO
 * 
 * @param gpio_num GPIO number
 * @param level Level to trigger wake-up
 * @return ESP_OK on success
 */
esp_err_t esp_sleep_enable_ext0_wakeup(uint32_t gpio_num, int level);

/**
 * @brief Enter light sleep mode
 * 
 * @return Wake-up source that caused the wake-up
 */
esp_sleep_wakeup_cause_t esp_light_sleep_start(void);

/**
 * @brief Enter deep sleep mode
 * 
 * This function does not return. The system will restart when waking up.
 */
void esp_deep_sleep_start(void) __attribute__((noreturn));

/**
 * @brief Get wake-up cause
 * 
 * @return Wake-up source that caused the last wake-up
 */
esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(void);

// ============================================================================
// MAC Address and Unique ID
// ============================================================================

/**
 * @brief Read base MAC address
 * 
 * @param[out] mac Buffer to store MAC address (6 bytes)
 * @return ESP_OK on success
 */
esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type);

/**
 * @brief Derive local MAC address
 * 
 * @param[out] local_mac Buffer to store derived MAC address
 * @param[in] universal_mac Universal MAC address
 * @return ESP_OK on success
 */
esp_err_t esp_derive_local_mac(uint8_t* local_mac, const uint8_t* universal_mac);

/**
 * @brief Get unique chip ID
 * 
 * @return 64-bit unique chip identifier
 */
uint64_t esp_get_chip_id(void);

// ============================================================================
// Miscellaneous
// ============================================================================

/**
 * @brief Get current CPU frequency in MHz
 * 
 * @return CPU frequency in MHz
 */
uint32_t esp_get_cpu_freq_mhz(void);

/**
 * @brief Set CPU frequency
 * 
 * @param freq_mhz Target frequency in MHz
 * @return ESP_OK on success
 */
esp_err_t esp_set_cpu_freq_mhz(uint32_t freq_mhz);

/**
 * @brief Get current time in microseconds since boot
 * 
 * @return Time in microseconds
 */
uint64_t esp_get_time_us(void);

/**
 * @brief Fill buffer with random data
 * 
 * @param buf Buffer to fill
 * @param len Buffer length in bytes
 */
void esp_fill_random(void* buf, size_t len);

/**
 * @brief Get random 32-bit value
 * 
 * @return Random 32-bit value
 */
uint32_t esp_random(void);

#ifdef __cplusplus
}

// C++ specific functions for emulator integration
namespace m5tab5::emulator {
    class EmulatorCore;
    
    /**
     * @brief Register EmulatorCore instance with ESP-IDF API layer
     * 
     * This function must be called during emulator initialization to enable
     * ESP-IDF APIs to access emulator components.
     * 
     * @param core Pointer to the EmulatorCore instance
     */
    void esp_idf_set_emulator_core(EmulatorCore* core);
}

#endif