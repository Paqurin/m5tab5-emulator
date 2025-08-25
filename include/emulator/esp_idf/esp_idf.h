/**
 * @file esp_idf.h
 * @brief Main ESP-IDF API header for M5Stack Tab5 Emulator
 * 
 * This header includes all ESP-IDF compatible APIs provided by the emulator,
 * allowing ESP-IDF applications to run without modification.
 */

#pragma once

// Core system APIs
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// Driver APIs
#include "driver/gpio.h"
#include "driver/i2c.h"
// TODO: Add more driver headers as they are implemented
// #include "driver/spi_master.h"
// #include "driver/uart.h"
// #include "driver/ledc.h"

// FreeRTOS integration
// TODO: Add FreeRTOS headers when implemented
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"

// Networking APIs (when implemented)
// #include "esp_wifi.h"
// #include "esp_netif.h"
// #include "lwip/lwip.h"

// Storage APIs (when implemented)  
// #include "nvs_flash.h"
// #include "esp_partition.h"
// #include "esp_vfs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all ESP-IDF subsystems for the emulator with EmulatorCore context
 * 
 * This function initializes all ESP-IDF compatible subsystems provided
 * by the emulator. Call this before using any ESP-IDF APIs.
 * 
 * @param emulator_core Pointer to EmulatorCore instance for component access
 * @return ESP_OK on success
 */
esp_err_t esp_idf_init_all_with_core(void* emulator_core);

/**
 * @brief Initialize all ESP-IDF subsystems for the emulator (stub mode)
 * 
 * Version without EmulatorCore context. APIs will work in stub mode.
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_idf_init_all(void);

/**
 * @brief Shutdown all ESP-IDF subsystems
 * 
 * Clean up all initialized ESP-IDF subsystems.
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_idf_deinit_all(void);

/**
 * @brief Get ESP-IDF emulation version information
 * 
 * @return Version string describing the emulation layer
 */
const char* esp_idf_get_emulation_version(void);

/**
 * @brief Check if running in emulation mode
 * 
 * @return Always returns true for the emulator
 */
bool esp_idf_is_emulation(void);

#ifdef __cplusplus
}

// C++ only functions for internal use
namespace m5tab5::emulator {
    class EmulatorCore;
}

/**
 * @brief Get the current EmulatorCore instance (C++ internal use)
 * 
 * @return Pointer to EmulatorCore or nullptr if not initialized
 */
m5tab5::emulator::EmulatorCore* esp_idf_get_emulator_core();

#endif