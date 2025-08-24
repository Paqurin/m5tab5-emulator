/**
 * @file esp_log.h
 * @brief ESP-IDF logging API for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible logging macros and functions that
 * integrate with the emulator's logging system, maintaining full compatibility
 * with ESP-IDF applications.
 */

#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Log levels
 */
typedef enum {
    ESP_LOG_NONE,       ///< No log output
    ESP_LOG_ERROR,      ///< Critical errors, software module can not recover on its own
    ESP_LOG_WARN,       ///< Error conditions from which recovery measures have been taken
    ESP_LOG_INFO,       ///< Information messages which describe normal flow of events
    ESP_LOG_DEBUG,      ///< Extra information which is not necessary for normal use (values, pointers, sizes, etc).
    ESP_LOG_VERBOSE     ///< Bigger chunks of debugging information, or frequent messages which can potentially flood the output.
} esp_log_level_t;

/**
 * @brief Default log level
 */
#ifndef CONFIG_LOG_DEFAULT_LEVEL
#define CONFIG_LOG_DEFAULT_LEVEL ESP_LOG_INFO
#endif

/**
 * @brief Maximum tag length
 */
#define ESP_LOG_MAX_TAG_LEN 23

/**
 * @brief Log timestamp type
 */
typedef uint32_t esp_log_timestamp_t;

/**
 * @brief Log output function type
 */
typedef int (*vprintf_like_t)(const char *, va_list);

// ============================================================================
// Core Logging Functions
// ============================================================================

/**
 * @brief Set log level for given tag
 * 
 * @param tag Tag to set level for (NULL for default level)
 * @param level Log level
 */
void esp_log_level_set(const char* tag, esp_log_level_t level);

/**
 * @brief Get log level for given tag
 * 
 * @param tag Tag to get level for
 * @return Log level
 */
esp_log_level_t esp_log_level_get(const char* tag);

/**
 * @brief Set function used to output log entries
 * 
 * @param func Function pointer (vprintf-like)
 * @return Old function pointer
 */
vprintf_like_t esp_log_set_vprintf(vprintf_like_t func);

/**
 * @brief Get current timestamp for logging
 * 
 * @return Timestamp in milliseconds since boot
 */
esp_log_timestamp_t esp_log_timestamp(void);

/**
 * @brief Write message to log
 * 
 * This function should not be called directly. Use ESP_LOG* macros instead.
 * 
 * @param level Log level
 * @param tag Log tag
 * @param format Format string
 * @param ... Arguments
 */
void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));

/**
 * @brief Write message to log (va_list version)
 * 
 * @param level Log level
 * @param tag Log tag
 * @param format Format string
 * @param args Arguments
 */
void esp_log_writev(esp_log_level_t level, const char* tag, const char* format, va_list args);

// ============================================================================
// Log Level Check Macros
// ============================================================================

/**
 * @brief Macro to check if log level is enabled for given tag and level
 */
#define ESP_LOG_LEVEL(level, tag) (esp_log_level_get(tag) >= (level))

/**
 * @brief Check if ERROR level logging is enabled
 */
#define ESP_LOG_LEVEL_ERROR(tag)   ESP_LOG_LEVEL(ESP_LOG_ERROR, tag)

/**
 * @brief Check if WARN level logging is enabled
 */
#define ESP_LOG_LEVEL_WARN(tag)    ESP_LOG_LEVEL(ESP_LOG_WARN, tag)

/**
 * @brief Check if INFO level logging is enabled
 */
#define ESP_LOG_LEVEL_INFO(tag)    ESP_LOG_LEVEL(ESP_LOG_INFO, tag)

/**
 * @brief Check if DEBUG level logging is enabled
 */
#define ESP_LOG_LEVEL_DEBUG(tag)   ESP_LOG_LEVEL(ESP_LOG_DEBUG, tag)

/**
 * @brief Check if VERBOSE level logging is enabled
 */
#define ESP_LOG_LEVEL_VERBOSE(tag) ESP_LOG_LEVEL(ESP_LOG_VERBOSE, tag)

// ============================================================================
// Logging Macros
// ============================================================================

/**
 * @brief Log an error message
 * 
 * @param tag Tag for the log entry
 * @param format Format string (printf-like)
 * @param ... Arguments for the format string
 */
#define ESP_LOGE(tag, format, ...) do {                     \
        if (ESP_LOG_LEVEL_ERROR(tag)) {                      \
            esp_log_write(ESP_LOG_ERROR, tag, format, ##__VA_ARGS__); \
        }                                                    \
    } while(0)

/**
 * @brief Log a warning message
 * 
 * @param tag Tag for the log entry
 * @param format Format string (printf-like)
 * @param ... Arguments for the format string
 */
#define ESP_LOGW(tag, format, ...) do {                     \
        if (ESP_LOG_LEVEL_WARN(tag)) {                       \
            esp_log_write(ESP_LOG_WARN, tag, format, ##__VA_ARGS__); \
        }                                                    \
    } while(0)

/**
 * @brief Log an info message
 * 
 * @param tag Tag for the log entry
 * @param format Format string (printf-like)
 * @param ... Arguments for the format string
 */
#define ESP_LOGI(tag, format, ...) do {                     \
        if (ESP_LOG_LEVEL_INFO(tag)) {                       \
            esp_log_write(ESP_LOG_INFO, tag, format, ##__VA_ARGS__); \
        }                                                    \
    } while(0)

/**
 * @brief Log a debug message
 * 
 * @param tag Tag for the log entry
 * @param format Format string (printf-like)
 * @param ... Arguments for the format string
 */
#define ESP_LOGD(tag, format, ...) do {                     \
        if (ESP_LOG_LEVEL_DEBUG(tag)) {                      \
            esp_log_write(ESP_LOG_DEBUG, tag, format, ##__VA_ARGS__); \
        }                                                    \
    } while(0)

/**
 * @brief Log a verbose message
 * 
 * @param tag Tag for the log entry
 * @param format Format string (printf-like)
 * @param ... Arguments for the format string
 */
#define ESP_LOGV(tag, format, ...) do {                     \
        if (ESP_LOG_LEVEL_VERBOSE(tag)) {                    \
            esp_log_write(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__); \
        }                                                    \
    } while(0)

// ============================================================================
// Buffer Logging Macros
// ============================================================================

/**
 * @brief Log a buffer of hex bytes at ERROR level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 */
void esp_log_buffer_hex(const char *tag, const void *buffer, uint16_t buff_len);

/**
 * @brief Log a buffer of hex bytes at specified level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 * @param level Log level
 */
void esp_log_buffer_hex_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level);

/**
 * @brief Log a buffer as characters at ERROR level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 */
void esp_log_buffer_char(const char *tag, const void *buffer, uint16_t buff_len);

/**
 * @brief Log a buffer as characters at specified level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 * @param level Log level
 */
void esp_log_buffer_char_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level);

/**
 * @brief Log a buffer in hex and character format at ERROR level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 */
void esp_log_buffer_hexdump(const char *tag, const void *buffer, uint16_t buff_len);

/**
 * @brief Log a buffer in hex and character format at specified level
 * 
 * @param tag Tag for the log entry
 * @param buffer Buffer to log
 * @param buff_len Length of buffer
 * @param level Log level
 */
void esp_log_buffer_hexdump_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level);

// Convenient macros for buffer logging
#define ESP_LOG_BUFFER_HEX(tag, buffer, buff_len) \
    do { \
        if (ESP_LOG_LEVEL_ERROR(tag)) { \
            esp_log_buffer_hex(tag, buffer, buff_len); \
        } \
    } while(0)

#define ESP_LOG_BUFFER_CHAR(tag, buffer, buff_len) \
    do { \
        if (ESP_LOG_LEVEL_ERROR(tag)) { \
            esp_log_buffer_char(tag, buffer, buff_len); \
        } \
    } while(0)

#define ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len) \
    do { \
        if (ESP_LOG_LEVEL_ERROR(tag)) { \
            esp_log_buffer_hexdump(tag, buffer, buff_len); \
        } \
    } while(0)

// Level-specific buffer logging macros
#define ESP_LOG_BUFFER_HEX_LEVEL(tag, buffer, buff_len, level) \
    do { \
        if (ESP_LOG_LEVEL(level, tag)) { \
            esp_log_buffer_hex_level(tag, buffer, buff_len, level); \
        } \
    } while(0)

#define ESP_LOG_BUFFER_CHAR_LEVEL(tag, buffer, buff_len, level) \
    do { \
        if (ESP_LOG_LEVEL(level, tag)) { \
            esp_log_buffer_char_level(tag, buffer, buff_len, level); \
        } \
    } while(0)

#define ESP_LOG_BUFFER_HEXDUMP_LEVEL(tag, buffer, buff_len, level) \
    do { \
        if (ESP_LOG_LEVEL(level, tag)) { \
            esp_log_buffer_hexdump_level(tag, buffer, buff_len, level); \
        } \
    } while(0)

// ============================================================================
// Early Logging (for use before main logging system is ready)
// ============================================================================

/**
 * @brief Early log write function
 * 
 * This can be used before the main logging system is initialized.
 * 
 * @param level Log level
 * @param tag Log tag
 * @param format Format string
 * @param ... Arguments
 */
void esp_early_log_write(esp_log_level_t level, const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));

/**
 * @brief Set early log level
 * 
 * @param level Log level
 */
void esp_log_early_level_set(esp_log_level_t level);

// Early logging macros (for bootloader and early init)
#define ESP_EARLY_LOGE(tag, format, ...) esp_early_log_write(ESP_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGW(tag, format, ...) esp_early_log_write(ESP_LOG_WARN, tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGI(tag, format, ...) esp_early_log_write(ESP_LOG_INFO, tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGD(tag, format, ...) esp_early_log_write(ESP_LOG_DEBUG, tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGV(tag, format, ...) esp_early_log_write(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif