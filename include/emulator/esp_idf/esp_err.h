/**
 * @file esp_err.h
 * @brief ESP-IDF error handling compatibility header for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible error handling definitions
 * for the emulator environment.
 */

#pragma once

#include "esp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check if error occurred and return if so
 * 
 * This macro is already defined in esp_types.h, but we ensure it's available here
 */
#ifndef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        return __err_rc; \
    } \
} while(0)
#endif

/**
 * @brief Check if error occurred and return void if so
 */
#ifndef ESP_ERROR_CHECK_WITHOUT_ABORT
#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        return; \
    } \
} while(0)
#endif

/**
 * @brief Get error description string
 * 
 * @param err Error code
 * @return String describing the error
 */
const char* esp_err_to_name(esp_err_t err);

/**
 * @brief Get error description string (same as esp_err_to_name)
 * 
 * @param err Error code
 * @return String describing the error
 */
const char* esp_err_to_name_r(esp_err_t err, char* buf, size_t buflen);

#ifdef __cplusplus
}
#endif