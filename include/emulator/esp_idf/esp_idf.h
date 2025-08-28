/**
 * @file esp_idf.h
 * @brief Main ESP-IDF API header for M5Stack Tab5 Emulator
 * 
 * This header includes all ESP-IDF compatible APIs provided by the emulator,
 * allowing ESP-IDF applications to run without modification.
 */

#pragma once

// Core type definitions
#include "esp_types.h"
#include "esp_err.h"

// Core system APIs
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// NVS Storage
#include "nvs.h"

// System integration
#include "esp_idf_integration.h"
#include "esp_system_call_interface.h"

// Driver APIs
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
// TODO: Add more driver headers as they are implemented
// #include "driver/ledc.h"

// FreeRTOS integration
#include "freertos_api.hpp"

// Networking APIs (when implemented)
// #include "esp_wifi.h"
// #include "esp_netif.h"
// #include "lwip/lwip.h"

// Storage APIs
// Additional storage APIs (when implemented)
// #include "esp_partition.h"
// #include "esp_vfs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all ESP-IDF subsystems for the emulator
 * 
 * This is the primary initialization function that sets up the complete
 * ESP-IDF compatibility layer including:
 * - System management and heap
 * - NVS storage
 * - FreeRTOS emulation
 * - Driver APIs (GPIO, I2C, SPI, UART)
 * - System call interface
 * 
 * @return ESP_OK on success, error code if critical components fail
 */
esp_err_t esp_idf_init_all(void);

/**
 * @brief Initialize ESP-IDF with EmulatorCore context
 * 
 * Advanced initialization with access to emulator core for hardware integration.
 * 
 * @param emulator_core Pointer to EmulatorCore instance
 * @return ESP_OK on success
 */
esp_err_t esp_idf_init_all_with_core(void* emulator_core);

/**
 * @brief Shutdown all ESP-IDF subsystems
 * 
 * Cleanly shuts down all ESP-IDF components in proper dependency order.
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_idf_deinit_all(void);

/**
 * @brief Check if ESP-IDF subsystems are ready
 * 
 * @return true if all critical components are initialized and ready
 */
bool esp_idf_is_ready(void);

/**
 * @brief Run comprehensive ESP-IDF integration test
 * 
 * Tests all major APIs to verify compatibility layer is working.
 * 
 * @return ESP_OK if all tests pass
 */
esp_err_t esp_idf_run_self_test(void);

/**
 * @brief Get ESP-IDF emulation version information
 * 
 * @return Version string describing the emulation layer
 */
const char* esp_idf_get_emulation_version(void);

/**
 * @brief Print comprehensive ESP-IDF system status
 * 
 * Outputs status of all components, memory usage, and performance metrics.
 */
void esp_idf_print_system_info(void);

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

/**
 * @brief Set the EmulatorCore instance (C++ internal use)
 * 
 * @param core Pointer to EmulatorCore instance
 */
void set_emulator_core(m5tab5::emulator::EmulatorCore* core);

#endif