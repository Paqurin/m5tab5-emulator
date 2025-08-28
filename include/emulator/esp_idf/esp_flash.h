/**
 * @file esp_flash.h
 * @brief ESP-IDF compatible flash API for M5Stack Tab5 Emulator
 * 
 * Provides complete esp_flash API implementation that delegates to
 * the FlashController for authentic ESP32-P4 flash behavior.
 */

#pragma once

#include "esp_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for flash chip handle
typedef struct esp_flash_t esp_flash_t;

/**
 * @brief Structure to describe a SPI flash chip connected to the system.
 */
struct esp_flash_t {
    uint32_t chip_id;           // Chip ID
    uint32_t size;              // Flash size in bytes
    uint32_t block_size;        // Block size for erase operations
    uint32_t sector_size;       // Sector size for erase operations
    uint32_t page_size;         // Page size for write operations
    bool read_only;             // True if flash is write-protected
    void* driver_data;          // Driver-specific data
};

// Default flash chip instance (main flash)
extern esp_flash_t* esp_flash_default_chip;

// Flash operation modes
typedef enum {
    ESP_FLASH_READ_MODE_FAST = 0,    // Fast read mode
    ESP_FLASH_READ_MODE_SLOW = 1,    // Slow read mode
    ESP_FLASH_READ_MODE_DIO = 2,     // Dual I/O read mode
    ESP_FLASH_READ_MODE_DOUT = 3,    // Dual output read mode
    ESP_FLASH_READ_MODE_QIO = 4,     // Quad I/O read mode
    ESP_FLASH_READ_MODE_QOUT = 5,    // Quad output read mode
} esp_flash_read_mode_t;

// Flash speed configurations
typedef enum {
    ESP_FLASH_20MHZ = 0,    // 20 MHz
    ESP_FLASH_26MHZ = 1,    // 26 MHz  
    ESP_FLASH_40MHZ = 2,    // 40 MHz
    ESP_FLASH_80MHZ = 3,    // 80 MHz
} esp_flash_speed_t;

/**
 * @brief Initialize the default flash chip
 * 
 * This function initializes the default flash chip instance and
 * connects it to the emulator's flash controller.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_init_default_chip(void);

/**
 * @brief Read data from flash
 * 
 * @param chip Flash chip handle (use esp_flash_default_chip for main flash)
 * @param buffer Buffer to read data into
 * @param address Flash address to read from
 * @param length Number of bytes to read
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_read(esp_flash_t* chip, void* buffer, uint32_t address, uint32_t length);

/**
 * @brief Write data to flash
 * 
 * @param chip Flash chip handle
 * @param address Flash address to write to
 * @param buffer Buffer containing data to write
 * @param length Number of bytes to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_write(esp_flash_t* chip, const void* buffer, uint32_t address, uint32_t length);

/**
 * @brief Erase flash sector
 * 
 * @param chip Flash chip handle
 * @param start_address Address of sector to erase (must be sector-aligned)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_erase_sector(esp_flash_t* chip, uint32_t start_address);

/**
 * @brief Erase flash region
 * 
 * @param chip Flash chip handle
 * @param start_address Start address of region to erase
 * @param length Length of region to erase (will be rounded to sector boundaries)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_erase_range(esp_flash_t* chip, uint32_t start_address, uint32_t length);

/**
 * @brief Get flash chip size
 * 
 * @param chip Flash chip handle
 * @param out_size Output parameter for flash size in bytes
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_get_size(esp_flash_t* chip, uint32_t* out_size);

/**
 * @brief Get flash chip ID
 * 
 * @param chip Flash chip handle  
 * @param out_id Output parameter for chip ID
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_read_id(esp_flash_t* chip, uint32_t* out_id);

/**
 * @brief Check if flash chip is write protected
 * 
 * @param chip Flash chip handle
 * @return true if write protected, false otherwise
 */
bool esp_flash_is_write_protected(esp_flash_t* chip);

/**
 * @brief Wait for flash operation to complete
 * 
 * @param chip Flash chip handle
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return ESP_OK if ready, ESP_ERR_TIMEOUT if timeout exceeded
 */
esp_err_t esp_flash_wait_idle(esp_flash_t* chip, uint32_t timeout_ms);

/**
 * @brief Set flash read mode
 * 
 * @param chip Flash chip handle
 * @param read_mode Read mode to set
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_set_read_mode(esp_flash_t* chip, esp_flash_read_mode_t read_mode);

/**
 * @brief Get flash physical sector size
 * 
 * @param chip Flash chip handle
 * @return Sector size in bytes
 */
uint32_t esp_flash_get_physical_sector_size(esp_flash_t* chip);

/**
 * @brief Get flash physical block size
 * 
 * @param chip Flash chip handle
 * @return Block size in bytes
 */
uint32_t esp_flash_get_physical_block_size(esp_flash_t* chip);

/**
 * @brief Suspend erase operation
 * 
 * @param chip Flash chip handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_suspend(esp_flash_t* chip);

/**
 * @brief Resume erase operation
 * 
 * @param chip Flash chip handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_resume(esp_flash_t* chip);

/**
 * @brief Read flash unique ID
 * 
 * @param chip Flash chip handle
 * @param out_id Buffer to store unique ID (8 bytes)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_read_unique_id(esp_flash_t* chip, uint64_t* out_id);

/**
 * @brief Detect flash size automatically
 * 
 * @param chip Flash chip handle
 * @param out_size Output parameter for detected size
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_flash_detect_size(esp_flash_t* chip, uint32_t* out_size);

#ifdef __cplusplus
}
#endif