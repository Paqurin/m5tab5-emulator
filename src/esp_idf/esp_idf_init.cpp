/**
 * @file esp_idf_init.cpp
 * @brief ESP-IDF initialization and management for M5Stack Tab5 Emulator
 * 
 * This file provides unified initialization and management of all ESP-IDF
 * compatible subsystems in the emulator.
 */

#include "emulator/esp_idf/esp_idf.h"
#include "emulator/esp_idf/esp_idf_integration.h"
#include "emulator/utils/logging.hpp"
#include "emulator/core/emulator_core.hpp"

using namespace m5tab5::emulator;

// Static state for tracking initialization and EmulatorCore context
static bool esp_idf_initialized = false;
static EmulatorCore* global_emulator_core = nullptr;

/**
 * @brief Set the EmulatorCore instance for ESP-IDF APIs (internal)
 */
void set_emulator_core(EmulatorCore* core) {
    global_emulator_core = core;
}

/**
 * @brief Get the current EmulatorCore instance for ESP-IDF APIs
 * 
 * @return Pointer to EmulatorCore or nullptr if not initialized
 */
EmulatorCore* esp_idf_get_emulator_core() {
    return global_emulator_core;
}

extern "C" {

esp_err_t esp_idf_init_all_with_core(void* emulator_core) {
    if (esp_idf_initialized) {
        LOG_WARN("esp_idf_init_all: ESP-IDF already initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_idf_init_all: initializing comprehensive ESP-IDF compatibility layer with EmulatorCore context");
    
    // Store EmulatorCore instance for API access
    global_emulator_core = static_cast<EmulatorCore*>(emulator_core);
    
    // Use comprehensive integration layer
    esp_err_t ret = esp_idf_initialize_compatibility_layer();
    if (ret != ESP_OK) {
        LOG_ERROR("esp_idf_init_all: failed to initialize ESP-IDF compatibility layer ({})", ret);
        return ret;
    }
    
    esp_idf_initialized = true;
    
    LOG_INFO("esp_idf_init_all: ESP-IDF compatibility layer initialized successfully");
    LOG_INFO("ESP-IDF Emulation Version: {}", esp_idf_get_emulation_version());
    
    return ESP_OK;
}

esp_err_t esp_idf_init_all(void) {
    if (esp_idf_initialized) {
        LOG_WARN("esp_idf_init_all: ESP-IDF already initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_idf_init_all: initializing comprehensive ESP-IDF compatibility layer");
    
    // Use comprehensive integration layer without specific EmulatorCore context
    esp_err_t ret = esp_idf_initialize_compatibility_layer();
    if (ret != ESP_OK) {
        LOG_ERROR("esp_idf_init_all: failed to initialize ESP-IDF compatibility layer ({})", ret);
        return ret;
    }
    
    esp_idf_initialized = true;
    
    LOG_INFO("esp_idf_init_all: ESP-IDF compatibility layer initialized successfully");
    LOG_INFO("ESP-IDF Emulation Version: {}", esp_idf_get_emulation_version());
    
    return ESP_OK;
}

esp_err_t esp_idf_deinit_all(void) {
    if (!esp_idf_initialized) {
        LOG_WARN("esp_idf_deinit_all: ESP-IDF not initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_idf_deinit_all: shutting down ESP-IDF compatibility layer");
    
    // Use comprehensive shutdown
    esp_idf_shutdown_compatibility_layer();
    
    // Clear emulator core reference
    global_emulator_core = nullptr;
    esp_idf_initialized = false;
    
    LOG_INFO("esp_idf_deinit_all: ESP-IDF compatibility layer shutdown completed");
    return ESP_OK;
}

bool esp_idf_is_ready(void) {
    return esp_idf_initialized && esp_idf_is_compatibility_layer_ready();
}

esp_err_t esp_idf_run_self_test(void) {
    if (!esp_idf_is_ready()) {
        LOG_ERROR("esp_idf_run_self_test: ESP-IDF not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return esp_idf_run_integration_test();
}

const char* esp_idf_get_emulation_version(void) {
    return "ESP-IDF v5.1 Complete Runtime Emulation for M5Stack Tab5 - Version 1.1.0";
}

void esp_idf_print_system_info(void) {
    if (esp_idf_is_ready()) {
        esp_idf_print_system_status();
    } else {
        LOG_WARN("ESP-IDF system not ready for status report");
    }
}

bool esp_idf_is_emulation(void) {
    return true;
}

} // extern "C"