/**
 * @file esp_err_api.cpp
 * @brief ESP-IDF error handling API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible error handling functions
 * for the emulation layer.
 */

#include "emulator/esp_idf/esp_err.h"
#include "emulator/utils/logging.hpp"
#include <cstring>
#include <cstdio>

extern "C" {

const char *esp_err_to_name(esp_err_t code) {
    switch (code) {
        case ESP_OK:                    return "ESP_OK";
        case ESP_FAIL:                  return "ESP_FAIL";
        case ESP_ERR_NO_MEM:           return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG:      return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE:    return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_INVALID_SIZE:     return "ESP_ERR_INVALID_SIZE";
        case ESP_ERR_NOT_FOUND:        return "ESP_ERR_NOT_FOUND";
        case ESP_ERR_NOT_SUPPORTED:    return "ESP_ERR_NOT_SUPPORTED";
        case ESP_ERR_TIMEOUT:          return "ESP_ERR_TIMEOUT";
        case ESP_ERR_INVALID_RESPONSE: return "ESP_ERR_INVALID_RESPONSE";
        case ESP_ERR_INVALID_CRC:      return "ESP_ERR_INVALID_CRC";
        case ESP_ERR_INVALID_VERSION:  return "ESP_ERR_INVALID_VERSION";
        case ESP_ERR_INVALID_MAC:      return "ESP_ERR_INVALID_MAC";
        case ESP_ERR_NOT_FINISHED:     return "ESP_ERR_NOT_FINISHED";
        default: {
            // Handle range-based errors
            if (code >= ESP_ERR_WIFI_BASE && code < ESP_ERR_WIFI_BASE + 100) {
                return "ESP_ERR_WIFI_*";
            }
            if (code >= ESP_ERR_MESH_BASE && code < ESP_ERR_MESH_BASE + 100) {
                return "ESP_ERR_MESH_*";
            }
            if (code >= ESP_ERR_FLASH_BASE && code < ESP_ERR_FLASH_BASE + 100) {
                return "ESP_ERR_FLASH_*";
            }
            return "ESP_ERR_UNKNOWN";
        }
    }
}

const char *esp_err_to_name_r(esp_err_t code, char *buf, size_t buflen) {
    if (!buf || buflen == 0) {
        return esp_err_to_name(code);
    }
    
    const char* name = esp_err_to_name(code);
    
    // If it's a known error name, just copy it
    if (strcmp(name, "ESP_ERR_UNKNOWN") != 0) {
        strncpy(buf, name, buflen - 1);
        buf[buflen - 1] = '\0';
        return buf;
    }
    
    // For unknown errors, format with the numeric code
    snprintf(buf, buflen, "ESP_ERR_UNKNOWN(0x%x)", code);
    return buf;
}

} // extern "C"