/**
 * @file esp_system_call_interface.h
 * @brief ESP-IDF System Call Interface for M5Stack Tab5 Emulator
 * 
 * This header provides the interface for routing ESP32-P4 RISC-V ECALL instructions
 * to ESP-IDF API implementations, enabling real ESP-IDF applications to run on the emulator.
 */

#pragma once

#include "esp_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ESP-IDF system call interface
 * 
 * This function sets up the system call registry and integrates with the RISC-V
 * CPU to handle ECALL instructions from running ESP-IDF applications.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_idf_init_system_call_interface(void);

/**
 * @brief Shutdown the ESP-IDF system call interface
 * 
 * Cleans up system call handlers and disconnects from the CPU.
 */
void esp_idf_shutdown_system_call_interface(void);

/**
 * @brief Check if system call interface is ready
 * 
 * @return true if interface is initialized and ready to handle calls
 */
bool esp_idf_is_system_call_interface_ready(void);

#ifdef __cplusplus
}

namespace m5tab5::emulator::esp_idf {

/**
 * @brief Initialize the system call interface (C++ interface)
 */
void initialize_system_call_interface();

/**
 * @brief Shutdown the system call interface (C++ interface)
 */
void shutdown_system_call_interface();

/**
 * @brief Check if system call interface is initialized (C++ interface)
 */
bool is_system_call_interface_initialized();

/**
 * @brief Execute a system call directly (for testing/debugging)
 * 
 * @param call_number System call number
 * @param arg0-arg5 System call arguments
 * @param return_value Pointer to store return value (optional)
 * @param error_code Pointer to store error code (optional)
 * @return ESP_OK on successful execution, error code on failure
 */
esp_err_t execute_system_call(uint32_t call_number, uint32_t arg0, uint32_t arg1, 
                              uint32_t arg2, uint32_t arg3, uint32_t arg4, uint32_t arg5,
                              uint32_t* return_value = nullptr, uint32_t* error_code = nullptr);

} // namespace m5tab5::emulator::esp_idf

#endif