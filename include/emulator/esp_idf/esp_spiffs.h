#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP-IDF SPIFFS API Compatibility Layer
 * 
 * Provides complete ESP-IDF SPIFFS API compatibility for M5Stack Tab5 Emulator.
 * This header matches the ESP-IDF esp_spiffs.h interface exactly.
 */

// ESP error codes for SPIFFS operations
#define ESP_OK                    0      /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL                  -1     /*!< Generic esp_err_t code indicating failure */

#define ESP_ERR_INVALID_ARG       0x102  /*!< Invalid argument */
#define ESP_ERR_INVALID_STATE     0x103  /*!< Invalid state */
#define ESP_ERR_NOT_FOUND         0x105  /*!< Requested resource not found */
#define ESP_ERR_NO_MEM            0x101  /*!< Out of memory */
#define ESP_ERR_TIMEOUT           0x107  /*!< Operation timed out */

// SPIFFS specific error codes
#define ESP_ERR_SPIFFS_BASE       0x10000                              /*!< Starting number of SPIFFS error codes */
#define ESP_ERR_SPIFFS_NOT_MOUNTED (ESP_ERR_SPIFFS_BASE + 1)          /*!< SPIFFS not mounted */
#define ESP_ERR_SPIFFS_ALREADY_MOUNTED (ESP_ERR_SPIFFS_BASE + 2)      /*!< SPIFFS already mounted */
#define ESP_ERR_SPIFFS_FULL       (ESP_ERR_SPIFFS_BASE + 3)            /*!< SPIFFS is full */
#define ESP_ERR_SPIFFS_NOT_FORMATTED (ESP_ERR_SPIFFS_BASE + 4)        /*!< SPIFFS not formatted */
#define ESP_ERR_SPIFFS_CORRUPTED  (ESP_ERR_SPIFFS_BASE + 5)            /*!< SPIFFS is corrupted */

// Type definitions matching ESP-IDF
typedef int esp_err_t;
typedef size_t esp_partition_t;  // Simplified for emulator

/**
 * @brief SPIFFS filesystem configuration structure
 */
typedef struct {
    const char* base_path;          /*!< File path prefix associated with the filesystem. */
    const char* partition_label;    /*!< Optional, label of SPIFFS partition to use. If set to NULL, first partition with subtype=spiffs will be used. */
    size_t max_files;              /*!< Maximum files that could be open at the same time. */
    bool format_if_mount_failed;   /*!< If true, it will format the file system if it fails to mount. */
} esp_vfs_spiffs_conf_t;

/**
 * @brief Register and mount SPIFFS to VFS with given path prefix.
 *
 * @param   conf                      Pointer to esp_vfs_spiffs_conf_t configuration structure
 *
 * @return
 *          - ESP_OK                  if success
 *          - ESP_ERR_NO_MEM          if objects could not be allocated
 *          - ESP_ERR_INVALID_STATE   if already mounted or partition is encrypted
 *          - ESP_ERR_NOT_FOUND       if partition for SPIFFS was not found
 *          - ESP_FAIL                if mount or format fails
 */
esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t* conf);

/**
 * @brief Unregister and unmount SPIFFS from VFS
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register.
 *
 * @return
 *          - ESP_OK if successful
 *          - ESP_ERR_INVALID_STATE already unregistered
 */
esp_err_t esp_vfs_spiffs_unregister(const char* partition_label);

/**
 * @brief Check if SPIFFS is mounted
 *
 * @param partition_label  Optional, label of the partition to check. If not specified, first partition with subtype=spiffs is used.
 *
 * @return
 *          - true    if mounted
 *          - false   if not mounted
 */
bool esp_spiffs_mounted(const char* partition_label);

/**
 * @brief Format the SPIFFS partition
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register.
 *
 * @return
 *          - ESP_OK      if successful
 *          - ESP_FAIL    on error
 */
esp_err_t esp_spiffs_format(const char* partition_label);

/**
 * @brief Get information for SPIFFS
 *
 * @param partition_label           Same label as passed to esp_vfs_spiffs_register
 * @param[out] total_bytes          Size of the file system
 * @param[out] used_bytes           Current used bytes in the file system
 *
 * @return
 *          - ESP_OK                  if success
 *          - ESP_ERR_INVALID_STATE   if not mounted
 */
esp_err_t esp_spiffs_info(const char* partition_label, size_t* total_bytes, size_t* used_bytes);

/**
 * @brief Check integrity of SPIFFS
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 *          - ESP_FAIL                on error
 */
esp_err_t esp_spiffs_check(const char* partition_label);

/**
 * @brief Get total bytes and used bytes for SPIFFS
 *
 * This function is deprecated, please consider using esp_spiffs_info instead.
 *
 * @param partition_label  Optional, label of the partition to get info for. If not specified, label of the first partition with subtype=spiffs is used.
 * @param[out] total_bytes Size of the file system
 * @param[out] used_bytes  Current used bytes in the file system
 *
 * @return
 *          - ESP_OK                  if success
 *          - ESP_ERR_INVALID_STATE   if not mounted
 */
esp_err_t esp_spiffs_get_used_size(const char* partition_label, size_t* total_bytes, size_t* used_bytes) __attribute__((deprecated));

// Additional emulator-specific functions for enhanced functionality
/**
 * @brief Perform garbage collection on SPIFFS
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 *          - ESP_FAIL                on error
 */
esp_err_t esp_spiffs_gc(const char* partition_label);

/**
 * @brief Get SPIFFS fragmentation ratio
 *
 * @param partition_label           Same label as passed to esp_vfs_spiffs_register
 * @param[out] fragmentation_ratio  Current fragmentation ratio (0.0 to 1.0)
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 */
esp_err_t esp_spiffs_get_fragmentation(const char* partition_label, float* fragmentation_ratio);

/**
 * @brief Defragment SPIFFS filesystem
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 *          - ESP_FAIL                on error
 */
esp_err_t esp_spiffs_defragment(const char* partition_label);

/**
 * @brief Get SPIFFS statistics
 *
 * @param partition_label      Same label as passed to esp_vfs_spiffs_register
 * @param[out] total_objects   Total number of objects in filesystem
 * @param[out] used_objects    Currently used objects
 * @param[out] free_objects    Available objects
 * @param[out] total_pages     Total pages in filesystem
 * @param[out] used_pages      Currently used pages
 * @param[out] free_pages      Available pages
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 */
esp_err_t esp_spiffs_get_stats(const char* partition_label,
                               uint32_t* total_objects, uint32_t* used_objects, uint32_t* free_objects,
                               uint32_t* total_pages, uint32_t* used_pages, uint32_t* free_pages);

/**
 * @brief Sync all open files to storage
 *
 * @param partition_label  Same label as passed to esp_vfs_spiffs_register
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if not mounted
 */
esp_err_t esp_spiffs_sync(const char* partition_label);

// VFS integration functions matching esp_vfs_spiffs.h

/**
 * @brief Configuration structure for esp_vfs_spiffs_register_fd
 */
typedef struct {
    const esp_partition_t* partition;  /*!< Pointer to SPIFFS partition */
    const char* base_path;             /*!< File path prefix associated with the filesystem */
    size_t max_files;                  /*!< Maximum files that could be open at the same time */
    bool format_if_mount_failed;       /*!< If true, format filesystem if mount fails */
} esp_vfs_spiffs_conf_fd_t;

/**
 * @brief Register and mount SPIFFS to VFS with file descriptor
 *
 * @param conf  Pointer to esp_vfs_spiffs_conf_fd_t configuration structure
 *
 * @return
 *          - ESP_OK                  if success
 *          - ESP_ERR_NO_MEM          if objects could not be allocated
 *          - ESP_ERR_INVALID_STATE   if already mounted or partition is encrypted
 *          - ESP_ERR_NOT_FOUND       if partition for SPIFFS was not found
 *          - ESP_FAIL                if mount or format fails
 */
esp_err_t esp_vfs_spiffs_register_fd(const esp_vfs_spiffs_conf_fd_t* conf);

/**
 * @brief Unregister and unmount SPIFFS from VFS by partition
 *
 * @param partition  Pointer to partition that was used for registration
 *
 * @return
 *          - ESP_OK                  if successful
 *          - ESP_ERR_INVALID_STATE   if already unregistered
 */
esp_err_t esp_vfs_spiffs_unregister_fd(const esp_partition_t* partition);

#ifdef __cplusplus
}
#endif