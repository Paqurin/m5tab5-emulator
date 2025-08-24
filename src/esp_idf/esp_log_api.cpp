/**
 * @file esp_log_api.cpp
 * @brief ESP-IDF logging API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible logging functions that integrate
 * with the emulator's logging system, providing seamless compatibility.
 */

#include "emulator/esp_idf/esp_log.h"
#include "emulator/utils/logging.hpp"
#include <unordered_map>
#include <string>
#include <mutex>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace {
    using namespace m5tab5::emulator;
    
    // Global log level settings per tag
    static std::unordered_map<std::string, esp_log_level_t> tag_levels;
    static esp_log_level_t default_log_level = static_cast<esp_log_level_t>(CONFIG_LOG_DEFAULT_LEVEL);
    static std::mutex log_mutex;
    
    // Custom vprintf function pointer
    static vprintf_like_t custom_vprintf = nullptr;
    
    // Early logging level
    static esp_log_level_t early_log_level = ESP_LOG_INFO;
    
    // System start time for timestamps
    static auto system_start = std::chrono::steady_clock::now();
    
    /**
     * @brief Convert ESP log level to emulator log level
     */
    m5tab5::emulator::LogLevel convert_log_level(esp_log_level_t esp_level) {
        switch (esp_level) {
            case ESP_LOG_ERROR:
                return m5tab5::emulator::LogLevel::ERROR;
            case ESP_LOG_WARN:
                return m5tab5::emulator::LogLevel::WARN;
            case ESP_LOG_INFO:
                return m5tab5::emulator::LogLevel::INFO;
            case ESP_LOG_DEBUG:
                return m5tab5::emulator::LogLevel::DEBUG_LEVEL;
            case ESP_LOG_VERBOSE:
                return m5tab5::emulator::LogLevel::TRACE;
            case ESP_LOG_NONE:
            default:
                return m5tab5::emulator::LogLevel::ERROR;
        }
    }
    
    /**
     * @brief Get log level name string
     */
    const char* get_log_level_name(esp_log_level_t level) {
        switch (level) {
            case ESP_LOG_ERROR:   return "E";
            case ESP_LOG_WARN:    return "W";
            case ESP_LOG_INFO:    return "I";
            case ESP_LOG_DEBUG:   return "D";
            case ESP_LOG_VERBOSE: return "V";
            default:              return "?";
        }
    }
    
    /**
     * @brief Format and output log message
     */
    void output_log_message(esp_log_level_t level, const char* tag, const char* format, va_list args) {
        char buffer[512];
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        
        if (len >= static_cast<int>(sizeof(buffer))) {
            // Message was truncated
            buffer[sizeof(buffer) - 4] = '.';
            buffer[sizeof(buffer) - 3] = '.';
            buffer[sizeof(buffer) - 2] = '.';
            buffer[sizeof(buffer) - 1] = '\0';
        }
        
        // Get timestamp
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - system_start);
        uint32_t timestamp = static_cast<uint32_t>(duration.count());
        
        // Use custom vprintf if set
        if (custom_vprintf) {
            char full_message[600];
            snprintf(full_message, sizeof(full_message), "%s (%u) %s: %s\n", 
                    get_log_level_name(level), timestamp, tag, buffer);
            
            va_list dummy_args;
            custom_vprintf(full_message, dummy_args);
        } else {
            // Use emulator's logging system
            auto emulator_level = convert_log_level(level);
            if (emulator_level != m5tab5::emulator::LogLevel::ERROR) {
                // Use the appropriate logging based on level
                switch (emulator_level) {
                    case m5tab5::emulator::LogLevel::TRACE:
                        LOG_TRACE("[{}] {}", tag, buffer);
                        break;
                    case m5tab5::emulator::LogLevel::DEBUG_LEVEL:
                        LOG_DEBUG("[{}] {}", tag, buffer);
                        break;
                    case m5tab5::emulator::LogLevel::INFO:
                        LOG_INFO("[{}] {}", tag, buffer);
                        break;
                    case m5tab5::emulator::LogLevel::WARN:
                        LOG_WARN("[{}] {}", tag, buffer);
                        break;
                    case m5tab5::emulator::LogLevel::ERROR:
                        LOG_ERROR("[{}] {}", tag, buffer);
                        break;
                }
            }
        }
    }
}

// ============================================================================
// ESP-IDF Logging API Implementation
// ============================================================================

extern "C" {

void esp_log_level_set(const char* tag, esp_log_level_t level) {
    std::lock_guard<std::mutex> lock(log_mutex);
    
    if (tag == nullptr) {
        // Set default level
        default_log_level = level;
        LOG_DEBUG("esp_log_level_set: default log level set to {}", static_cast<int>(level));
    } else {
        // Set level for specific tag
        tag_levels[std::string(tag)] = level;
        LOG_DEBUG("esp_log_level_set: tag '{}' log level set to {}", tag, static_cast<int>(level));
    }
}

esp_log_level_t esp_log_level_get(const char* tag) {
    std::lock_guard<std::mutex> lock(log_mutex);
    
    if (tag == nullptr) {
        return default_log_level;
    }
    
    auto it = tag_levels.find(std::string(tag));
    if (it != tag_levels.end()) {
        return it->second;
    }
    
    return default_log_level;
}

vprintf_like_t esp_log_set_vprintf(vprintf_like_t func) {
    std::lock_guard<std::mutex> lock(log_mutex);
    
    vprintf_like_t old_func = custom_vprintf;
    custom_vprintf = func;
    
    LOG_DEBUG("esp_log_set_vprintf: custom vprintf function {}", func ? "set" : "cleared");
    return old_func;
}

esp_log_timestamp_t esp_log_timestamp(void) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - system_start);
    return static_cast<esp_log_timestamp_t>(duration.count());
}

void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...) {
    if (!tag || !format) {
        return;
    }
    
    // Check if this level is enabled for this tag
    if (esp_log_level_get(tag) < level) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    esp_log_writev(level, tag, format, args);
    va_end(args);
}

void esp_log_writev(esp_log_level_t level, const char* tag, const char* format, va_list args) {
    if (!tag || !format) {
        return;
    }
    
    // Check if this level is enabled for this tag
    if (esp_log_level_get(tag) < level) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(log_mutex);
    output_log_message(level, tag, format, args);
}

// ============================================================================
// Buffer Logging Implementation
// ============================================================================

void esp_log_buffer_hex_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level) {
    if (!tag || !buffer || buff_len == 0) {
        return;
    }
    
    if (esp_log_level_get(tag) < level) {
        return;
    }
    
    const uint8_t *bytes = static_cast<const uint8_t*>(buffer);
    std::ostringstream oss;
    
    // Format as hex bytes with spaces
    for (uint16_t i = 0; i < buff_len; i++) {
        if (i > 0) oss << " ";
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]);
    }
    
    std::string hex_str = oss.str();
    esp_log_write(level, tag, "Buffer hex dump (%u bytes): %s", buff_len, hex_str.c_str());
}

void esp_log_buffer_hex(const char *tag, const void *buffer, uint16_t buff_len) {
    esp_log_buffer_hex_level(tag, buffer, buff_len, ESP_LOG_ERROR);
}

void esp_log_buffer_char_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level) {
    if (!tag || !buffer || buff_len == 0) {
        return;
    }
    
    if (esp_log_level_get(tag) < level) {
        return;
    }
    
    const uint8_t *bytes = static_cast<const uint8_t*>(buffer);
    std::ostringstream oss;
    
    // Format as printable characters, replace non-printable with '.'
    for (uint16_t i = 0; i < buff_len; i++) {
        char c = static_cast<char>(bytes[i]);
        if (c >= 32 && c <= 126) {
            oss << c;
        } else {
            oss << '.';
        }
    }
    
    std::string char_str = oss.str();
    esp_log_write(level, tag, "Buffer char dump (%u bytes): %s", buff_len, char_str.c_str());
}

void esp_log_buffer_char(const char *tag, const void *buffer, uint16_t buff_len) {
    esp_log_buffer_char_level(tag, buffer, buff_len, ESP_LOG_ERROR);
}

void esp_log_buffer_hexdump_level(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t level) {
    if (!tag || !buffer || buff_len == 0) {
        return;
    }
    
    if (esp_log_level_get(tag) < level) {
        return;
    }
    
    const uint8_t *bytes = static_cast<const uint8_t*>(buffer);
    const uint16_t bytes_per_line = 16;
    
    esp_log_write(level, tag, "Buffer hexdump (%u bytes):", buff_len);
    
    for (uint16_t i = 0; i < buff_len; i += bytes_per_line) {
        std::ostringstream hex_part, char_part;
        
        // Build hex part
        for (uint16_t j = 0; j < bytes_per_line && (i + j) < buff_len; j++) {
            if (j > 0) hex_part << " ";
            hex_part << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i + j]);
        }
        
        // Pad hex part to consistent width
        while (hex_part.str().length() < bytes_per_line * 3 - 1) {
            hex_part << " ";
        }
        
        // Build char part
        for (uint16_t j = 0; j < bytes_per_line && (i + j) < buff_len; j++) {
            char c = static_cast<char>(bytes[i + j]);
            if (c >= 32 && c <= 126) {
                char_part << c;
            } else {
                char_part << '.';
            }
        }
        
        esp_log_write(level, tag, "%04x: %s |%s|", i, hex_part.str().c_str(), char_part.str().c_str());
    }
}

void esp_log_buffer_hexdump(const char *tag, const void *buffer, uint16_t buff_len) {
    esp_log_buffer_hexdump_level(tag, buffer, buff_len, ESP_LOG_ERROR);
}

// ============================================================================
// Early Logging Implementation
// ============================================================================

void esp_log_early_level_set(esp_log_level_t level) {
    early_log_level = level;
    LOG_DEBUG("esp_log_early_level_set: early log level set to {}", static_cast<int>(level));
}

void esp_early_log_write(esp_log_level_t level, const char* tag, const char* format, ...) {
    if (!tag || !format) {
        return;
    }
    
    if (early_log_level < level) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    
    // Early logging bypasses the tag-based level system and uses printf directly
    char buffer[512];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    
    if (len >= static_cast<int>(sizeof(buffer))) {
        buffer[sizeof(buffer) - 4] = '.';
        buffer[sizeof(buffer) - 3] = '.';
        buffer[sizeof(buffer) - 2] = '.';
        buffer[sizeof(buffer) - 1] = '\0';
    }
    
    // Get timestamp
    uint32_t timestamp = esp_log_timestamp();
    
    // Output early log message directly to stdout
    printf("%s (%u) %s: %s\n", get_log_level_name(level), timestamp, tag, buffer);
    fflush(stdout);
    
    va_end(args);
}

} // extern "C"