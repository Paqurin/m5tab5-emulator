/**
 * @file esp_idf_init.cpp
 * @brief ESP-IDF initialization and management for M5Stack Tab5 Emulator
 * 
 * This file provides unified initialization and management of all ESP-IDF
 * compatible subsystems in the emulator.
 */

#include "emulator/esp_idf/esp_idf.h"
#include "emulator/utils/logging.hpp"

using namespace m5tab5::emulator;

// Static state for tracking initialization
static bool esp_idf_initialized = false;

extern "C" {

esp_err_t esp_idf_init_all(void) {
    if (esp_idf_initialized) {
        LOG_WARN("esp_idf_init_all: ESP-IDF already initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_idf_init_all: initializing ESP-IDF emulation layer");
    
    esp_err_t ret;
    
    // Initialize system APIs
    ret = esp_system_init();
    if (ret != ESP_OK) {
        LOG_ERROR("esp_idf_init_all: failed to initialize system APIs ({})", ret);
        return ret;
    }
    
    // Initialize timer system
    ret = esp_timer_init();
    if (ret != ESP_OK) {
        LOG_ERROR("esp_idf_init_all: failed to initialize timer system ({})", ret);
        return ret;
    }
    
    // TODO: Initialize other subsystems as they are implemented
    // - GPIO system (automatic with first use)
    // - I2C drivers (initialized per port)
    // - SPI drivers
    // - UART drivers
    // - WiFi stack
    // - Storage systems
    
    esp_idf_initialized = true;
    
    LOG_INFO("esp_idf_init_all: ESP-IDF emulation layer initialized successfully");
    LOG_INFO("ESP-IDF Emulation Version: {}", esp_idf_get_emulation_version());
    
    return ESP_OK;
}

esp_err_t esp_idf_deinit_all(void) {
    if (!esp_idf_initialized) {
        LOG_WARN("esp_idf_deinit_all: ESP-IDF not initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_idf_deinit_all: shutting down ESP-IDF emulation layer");
    
    // Deinitialize subsystems in reverse order
    
    // Deinitialize timer system
    esp_err_t ret = esp_timer_deinit();
    if (ret != ESP_OK) {
        LOG_ERROR("esp_idf_deinit_all: failed to deinitialize timer system ({})", ret);
    }
    
    // TODO: Deinitialize other subsystems
    
    esp_idf_initialized = false;
    
    LOG_INFO("esp_idf_deinit_all: ESP-IDF emulation layer shutdown completed");
    return ESP_OK;
}

const char* esp_idf_get_emulation_version(void) {
    return "ESP-IDF v5.1 Emulation for M5Stack Tab5 - Version 1.0.0";
}

bool esp_idf_is_emulation(void) {
    return true;
}

} // extern "C"