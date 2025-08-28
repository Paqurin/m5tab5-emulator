/**
 * @file esp_partition.h
 * @brief ESP-IDF compatible partition API for M5Stack Tab5 Emulator
 * 
 * Provides complete esp_partition API implementation that delegates to
 * the PartitionTable for authentic ESP32-P4 partition management.
 */

#pragma once

#include "esp_types.h"
#include "esp_err.h"
#include "esp_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

// Partition type definitions (matching ESP-IDF)
typedef enum {
    ESP_PARTITION_TYPE_APP = 0x00,          // Application partition
    ESP_PARTITION_TYPE_DATA = 0x01,         // Data partition
    ESP_PARTITION_TYPE_ANY = 0xFF,          // Used to search for all partition types
} esp_partition_type_t;

// Partition subtypes (combined APP and DATA subtypes)
typedef enum {
    // APP subtypes
    ESP_PARTITION_SUBTYPE_APP_FACTORY = 0x00,       // Factory application
    ESP_PARTITION_SUBTYPE_APP_OTA_MIN = 0x10,       // Base for OTA partition numbering
    ESP_PARTITION_SUBTYPE_APP_OTA_0 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 0,
    ESP_PARTITION_SUBTYPE_APP_OTA_1 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 1,
    ESP_PARTITION_SUBTYPE_APP_OTA_2 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 2,
    ESP_PARTITION_SUBTYPE_APP_OTA_3 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 3,
    ESP_PARTITION_SUBTYPE_APP_OTA_4 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 4,
    ESP_PARTITION_SUBTYPE_APP_OTA_5 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 5,
    ESP_PARTITION_SUBTYPE_APP_OTA_6 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 6,
    ESP_PARTITION_SUBTYPE_APP_OTA_7 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 7,
    ESP_PARTITION_SUBTYPE_APP_OTA_8 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 8,
    ESP_PARTITION_SUBTYPE_APP_OTA_9 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 9,
    ESP_PARTITION_SUBTYPE_APP_OTA_10 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 10,
    ESP_PARTITION_SUBTYPE_APP_OTA_11 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 11,
    ESP_PARTITION_SUBTYPE_APP_OTA_12 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 12,
    ESP_PARTITION_SUBTYPE_APP_OTA_13 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 13,
    ESP_PARTITION_SUBTYPE_APP_OTA_14 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 14,
    ESP_PARTITION_SUBTYPE_APP_OTA_15 = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 15,
    ESP_PARTITION_SUBTYPE_APP_OTA_MAX = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + 15,
    ESP_PARTITION_SUBTYPE_APP_TEST = 0x20,          // Test application
    
    // DATA subtypes
    ESP_PARTITION_SUBTYPE_DATA_OTA = 0x00,          // OTA selection partition
    ESP_PARTITION_SUBTYPE_DATA_PHY = 0x01,          // PHY init data partition
    ESP_PARTITION_SUBTYPE_DATA_NVS = 0x02,          // NVS partition
    ESP_PARTITION_SUBTYPE_DATA_COREDUMP = 0x03,     // Core dump partition
    ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS = 0x04,     // NVS keys partition
    ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM = 0x05,     // Emulated eFuse partition
    ESP_PARTITION_SUBTYPE_DATA_UNDEFINED = 0x06,    // Undefined data partition
    ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD = 0x80,     // ESPHTTPD partition
    ESP_PARTITION_SUBTYPE_DATA_FAT = 0x81,          // FAT partition
    ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 0x82,       // SPIFFS partition
    ESP_PARTITION_SUBTYPE_DATA_LITTLEFS = 0x83,     // LittleFS partition
    ESP_PARTITION_SUBTYPE_ANY = 0xFF,               // Used to search for all partition subtypes
} esp_partition_subtype_t;

// Forward declarations
typedef struct esp_partition_iterator_opaque_* esp_partition_iterator_t;
typedef struct esp_partition_s esp_partition_t;

// Memory map handle type
typedef uint32_t esp_partition_mmap_handle_t;

/**
 * @brief Partition information structure
 */
struct esp_partition_s {
    esp_partition_type_t type;          // Partition type
    esp_partition_subtype_t subtype;    // Partition subtype  
    uint32_t address;                   // Starting address of the partition in flash
    uint32_t size;                      // Size of the partition in bytes
    char label[17];                     // Partition label (null-terminated, max 16 chars)
    bool encrypted;                     // Flag indicating if partition is encrypted
    bool readonly;                      // Flag indicating if partition is read-only
};

/**
 * @brief Find partition based on one or more parameters
 * 
 * @param type Partition type, or ESP_PARTITION_TYPE_ANY to match any type
 * @param subtype Partition subtype, or ESP_PARTITION_SUBTYPE_ANY to match any subtype
 * @param label Partition label to match, or NULL to match any label
 * @return Iterator to iterate over partitions found, or NULL if no partitions found.
 *         Iterator must be released using esp_partition_iterator_release.
 */
esp_partition_iterator_t esp_partition_find(esp_partition_type_t type,
                                           esp_partition_subtype_t subtype,
                                           const char* label);

/**
 * @brief Find first partition based on one or more parameters
 * 
 * @param type Partition type, or ESP_PARTITION_TYPE_ANY to match any type
 * @param subtype Partition subtype, or ESP_PARTITION_SUBTYPE_ANY to match any subtype  
 * @param label Partition label to match, or NULL to match any label
 * @return Pointer to esp_partition_t structure, or NULL if no partition found
 */
const esp_partition_t* esp_partition_find_first(esp_partition_type_t type,
                                               esp_partition_subtype_t subtype,
                                               const char* label);

/**
 * @brief Get next partition from iterator
 * 
 * @param iterator Iterator obtained from esp_partition_find()
 * @return Pointer to next partition, or NULL if no more partitions
 */
const esp_partition_t* esp_partition_next(esp_partition_iterator_t iterator);

/**
 * @brief Release partition iterator
 * 
 * @param iterator Iterator to release
 */
void esp_partition_iterator_release(esp_partition_iterator_t iterator);

/**
 * @brief Verify partition data
 * 
 * @param partition Pointer to partition structure
 * @return ESP_OK if partition data is valid
 */
esp_err_t esp_partition_verify(const esp_partition_t* partition);

/**
 * @brief Read data from partition
 * 
 * @param partition Pointer to partition structure
 * @param src_offset Offset within partition where data should be read from
 * @param dst Pointer to buffer where data should be stored
 * @param size Size of data to be read, in bytes
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_read(const esp_partition_t* partition,
                            size_t src_offset,
                            void* dst,
                            size_t size);

/**
 * @brief Write data to partition
 * 
 * @param partition Pointer to partition structure
 * @param dst_offset Offset within partition where data should be written
 * @param src Pointer to source buffer
 * @param size Size of data to be written, in bytes
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_write(const esp_partition_t* partition,
                             size_t dst_offset,
                             const void* src,
                             size_t size);

/**
 * @brief Erase part of partition
 * 
 * @param partition Pointer to partition structure
 * @param offset Offset within partition where erase should start
 * @param size Size of area that should be erased, in bytes
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_erase_range(const esp_partition_t* partition,
                                   size_t offset,
                                   size_t size);

/**
 * @brief Configure memory-mapped reading from partition
 * 
 * @param partition Pointer to partition structure
 * @param offset Offset within partition
 * @param size Size of area to map
 * @param memory_map_mode Memory map cache mode
 * @param out_ptr Output pointer to mapped memory
 * @param out_handle Output handle for unmapping
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_mmap(const esp_partition_t* partition,
                            size_t offset,
                            size_t size,
                            esp_flash_read_mode_t memory_map_mode,
                            const void** out_ptr,
                            esp_partition_mmap_handle_t* out_handle);

/**
 * @brief Unmap memory-mapped partition
 * 
 * @param handle Handle obtained from esp_partition_mmap
 */
void esp_partition_munmap(esp_partition_mmap_handle_t handle);

/**
 * @brief Get SHA-256 digest for partition
 * 
 * @param partition Pointer to partition structure
 * @param sha_256 Buffer to store SHA-256 digest (must be 32 bytes)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_get_sha256(const esp_partition_t* partition, uint8_t* sha_256);

/**
 * @brief Check if partition can be written
 * 
 * @param partition Pointer to partition structure
 * @return true if partition can be written
 */
bool esp_partition_check_identity(const esp_partition_t* partition);

/**
 * @brief Register partition in partition table
 * 
 * This function allows registering a partition which was not present in built-in partition table
 * 
 * @param partition_info Pointer to partition info structure
 * @return Pointer to esp_partition_t structure, or NULL on error
 */
const esp_partition_t* esp_partition_register_external(const esp_partition_t* partition_info);

/**
 * @brief Deregister partition from partition table
 * 
 * @param partition Pointer to partition obtained using esp_partition_register_external
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_partition_deregister_external(const esp_partition_t* partition);

// Memory map handle type (declare before use)
typedef uint32_t esp_partition_mmap_handle_t;

#ifdef __cplusplus
}
#endif