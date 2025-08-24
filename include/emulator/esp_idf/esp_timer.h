/**
 * @file esp_timer.h
 * @brief ESP-IDF high-resolution timer API for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible timer functions for microsecond-precision
 * timing, periodic callbacks, and time management.
 */

#pragma once

#include "esp_system.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Timer handle type
 */
typedef void* esp_timer_handle_t;

/**
 * @brief Timer callback function type
 * 
 * @param arg User data passed to the timer callback
 */
typedef void (*esp_timer_cb_t)(void* arg);

/**
 * @brief Timer configuration structure
 */
typedef struct {
    esp_timer_cb_t callback;        ///< Callback function
    void* arg;                      ///< Argument for callback
    const char* name;               ///< Timer name (for debugging)
    bool skip_unhandled_events;     ///< Skip missed events if timer is late
} esp_timer_create_args_t;

// ============================================================================
// Timer Management Functions
// ============================================================================

/**
 * @brief Initialize ESP timer library
 * 
 * This function initializes the high-resolution timer system.
 * Must be called before using any other esp_timer functions.
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_timer_init(void);

/**
 * @brief Deinitialize ESP timer library
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_timer_deinit(void);

/**
 * @brief Create a timer
 * 
 * @param create_args Timer configuration
 * @param[out] out_handle Handle for the created timer
 * @return ESP_OK on success
 */
esp_err_t esp_timer_create(const esp_timer_create_args_t* create_args, esp_timer_handle_t* out_handle);

/**
 * @brief Delete a timer
 * 
 * @param timer Timer handle
 * @return ESP_OK on success
 */
esp_err_t esp_timer_delete(esp_timer_handle_t timer);

/**
 * @brief Start a one-shot timer
 * 
 * @param timer Timer handle
 * @param timeout_us Timeout in microseconds
 * @return ESP_OK on success
 */
esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us);

/**
 * @brief Start a periodic timer
 * 
 * @param timer Timer handle
 * @param period_us Period in microseconds
 * @return ESP_OK on success
 */
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us);

/**
 * @brief Stop a timer
 * 
 * @param timer Timer handle
 * @return ESP_OK on success
 */
esp_err_t esp_timer_stop(esp_timer_handle_t timer);

/**
 * @brief Restart a timer
 * 
 * This stops the timer if it's running and starts it again with
 * the same timeout/period value.
 * 
 * @param timer Timer handle
 * @param timeout_us New timeout in microseconds
 * @return ESP_OK on success
 */
esp_err_t esp_timer_restart(esp_timer_handle_t timer, uint64_t timeout_us);

// ============================================================================
// Time Query Functions
// ============================================================================

/**
 * @brief Get time since boot in microseconds
 * 
 * @return Time in microseconds since system boot
 */
uint64_t esp_timer_get_time(void);

/**
 * @brief Get the next alarm time
 * 
 * This function returns the time when the next timer will fire.
 * 
 * @return Next alarm time in microseconds, or 0 if no timers are active
 */
uint64_t esp_timer_get_next_alarm(void);

/**
 * @brief Check if a timer is active
 * 
 * @param timer Timer handle
 * @return true if timer is active (started but not fired/stopped)
 */
bool esp_timer_is_active(esp_timer_handle_t timer);

// ============================================================================
// Timer Information Functions
// ============================================================================

/**
 * @brief Get timer information
 * 
 * @param timer Timer handle (NULL to get info for all timers)
 * @param timer_info Buffer to store timer information
 * @param size Size of the buffer
 * @return ESP_OK on success
 */
typedef struct {
    const char* name;           ///< Timer name
    uint64_t period;           ///< Timer period (0 for one-shot)
    uint64_t next_alarm;       ///< Next alarm time
    bool is_active;            ///< Whether timer is active
    uint32_t callback_count;   ///< Number of times callback was called
} esp_timer_info_t;

esp_err_t esp_timer_dump(esp_timer_info_t* timer_info, size_t size);

/**
 * @brief Print timer information to console
 * 
 * This function prints information about all active timers to the console.
 * Useful for debugging.
 */
void esp_timer_dump_all(void);

// ============================================================================
// Delay Functions
// ============================================================================

/**
 * @brief Delay execution for specified microseconds
 * 
 * This function provides a busy-wait delay with microsecond precision.
 * Use sparingly as it blocks the CPU.
 * 
 * @param us Delay time in microseconds
 */
void esp_timer_delay_us(uint32_t us);

/**
 * @brief Get timer resolution
 * 
 * @return Timer resolution in microseconds
 */
uint32_t esp_timer_get_resolution(void);

// ============================================================================
// Backward Compatibility Macros
// ============================================================================

/**
 * @brief Legacy function name for esp_timer_get_time()
 * 
 * @deprecated Use esp_timer_get_time() instead
 */
#define esp_timer_get_time_us() esp_timer_get_time()

/**
 * @brief Legacy function name for creating one-shot timer
 * 
 * @deprecated Use esp_timer_start_once() instead
 */
#define esp_timer_start_one_shot(timer, timeout) esp_timer_start_once(timer, timeout)

#ifdef __cplusplus
}
#endif