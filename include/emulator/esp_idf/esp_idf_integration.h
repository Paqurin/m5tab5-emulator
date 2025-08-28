/**
 * @file esp_idf_integration.h
 * @brief ESP-IDF Integration Coordinator for M5Stack Tab5 Emulator
 * 
 * This header provides the master interface for initializing and managing
 * all ESP-IDF API compatibility components, ensuring they work together
 * to provide a complete ESP32-P4 application runtime environment.
 */

#pragma once

#include "esp_types.h"
#include <vector>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the complete ESP-IDF compatibility layer
 * 
 * This function initializes all ESP-IDF API components in the correct order:
 * - System management (esp_system)
 * - Heap management (heap_caps)
 * - NVS flash storage
 * - FreeRTOS kernel emulation
 * - System call interface
 * - Driver APIs (GPIO, I2C, SPI, UART)
 * 
 * @return ESP_OK on success, error code if critical components fail
 */
esp_err_t esp_idf_initialize_compatibility_layer(void);

/**
 * @brief Shutdown the ESP-IDF compatibility layer
 * 
 * Cleanly shuts down all ESP-IDF components in reverse dependency order.
 */
void esp_idf_shutdown_compatibility_layer(void);

/**
 * @brief Check if the ESP-IDF compatibility layer is ready
 * 
 * @return true if the layer is fully initialized and ready for applications
 */
bool esp_idf_is_compatibility_layer_ready(void);

/**
 * @brief Print status of all ESP-IDF components
 * 
 * Outputs the initialization status and health of all components to the log.
 */
void esp_idf_print_system_status(void);

/**
 * @brief Run integration test for ESP-IDF compatibility
 * 
 * Tests basic functionality of all major ESP-IDF APIs to verify the
 * compatibility layer is working correctly.
 * 
 * @return ESP_OK if all tests pass, error code if any test fails
 */
esp_err_t esp_idf_run_integration_test(void);

#ifdef __cplusplus
}

namespace m5tab5::emulator::esp_idf {

/**
 * @brief Initialize ESP-IDF integration (C++ interface)
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t initialize_esp_idf_integration();

/**
 * @brief Shutdown ESP-IDF integration (C++ interface)
 */
void shutdown_esp_idf_integration();

/**
 * @brief Check if ESP-IDF integration is ready (C++ interface)
 * 
 * @return true if ready, false otherwise
 */
bool is_esp_idf_integration_ready();

/**
 * @brief Print ESP-IDF component status (C++ interface)
 */
void print_esp_idf_status();

/**
 * @brief Get ESP-IDF component status as string vector (C++ interface)
 * 
 * @return Vector of status strings, one per component
 */
std::vector<std::string> get_esp_idf_component_status();

} // namespace m5tab5::emulator::esp_idf

#endif