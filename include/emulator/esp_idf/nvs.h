#pragma once

/**
 * @file nvs.h
 * @brief ESP-IDF Compatible Non-Volatile Storage API for M5Stack Tab5 Emulator
 * 
 * This header provides full compatibility with ESP-IDF NVS API while using
 * the emulator's NVS controller as the backend.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ESP error type compatibility
typedef int esp_err_t;

// Common ESP-IDF error codes
#define ESP_OK                    0       /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL                  -1      /*!< Generic esp_err_t code indicating failure */

// NVS specific error codes (matching ESP-IDF)
#define ESP_ERR_NVS_BASE                0x1100                     /*!< Starting number of error codes */
#define ESP_ERR_NVS_NOT_INITIALIZED     (ESP_ERR_NVS_BASE + 0x01)  /*!< The storage driver is not initialized */
#define ESP_ERR_NVS_NOT_FOUND           (ESP_ERR_NVS_BASE + 0x02)  /*!< Id namespace doesn't exist yet and mode is NVS_READONLY */
#define ESP_ERR_NVS_TYPE_MISMATCH       (ESP_ERR_NVS_BASE + 0x03)  /*!< The type of set or get operation doesn't match the type of value stored in NVS */
#define ESP_ERR_NVS_READ_ONLY           (ESP_ERR_NVS_BASE + 0x04)  /*!< Storage handle was opened as read only */
#define ESP_ERR_NVS_NOT_ENOUGH_SPACE    (ESP_ERR_NVS_BASE + 0x05)  /*!< There is not enough space in the underlying storage to save the value */
#define ESP_ERR_NVS_INVALID_NAME        (ESP_ERR_NVS_BASE + 0x06)  /*!< Namespace name doesn't satisfy constraints */
#define ESP_ERR_NVS_INVALID_HANDLE      (ESP_ERR_NVS_BASE + 0x07)  /*!< Handle has been closed or is NULL */
#define ESP_ERR_NVS_REMOVE_FAILED       (ESP_ERR_NVS_BASE + 0x08)  /*!< The value wasn't updated because flash write operation has failed. The value was written however, and update will be finished after re-initialization of nvs, provided that flash operation doesn't fail again. */
#define ESP_ERR_NVS_KEY_TOO_LONG        (ESP_ERR_NVS_BASE + 0x09)  /*!< Key name is too long */
#define ESP_ERR_NVS_PAGE_FULL           (ESP_ERR_NVS_BASE + 0x0a)  /*!< Internal error; never returned by nvs API functions */
#define ESP_ERR_NVS_INVALID_STATE       (ESP_ERR_NVS_BASE + 0x0b)  /*!< NVS is in an inconsistent state due to a previous error. Call nvs_flash_init and nvs_open again, then retry. */
#define ESP_ERR_NVS_INVALID_LENGTH      (ESP_ERR_NVS_BASE + 0x0c)  /*!< String or blob length is not sufficient to store data */
#define ESP_ERR_NVS_NO_FREE_PAGES       (ESP_ERR_NVS_BASE + 0x0d)  /*!< NVS partition doesn't contain any empty pages. This may happen if NVS partition was truncated. Erase the whole partition and call nvs_flash_init() again. */
#define ESP_ERR_NVS_VALUE_TOO_LONG      (ESP_ERR_NVS_BASE + 0x0e)  /*!< String or blob length is longer than supported by the implementation */
#define ESP_ERR_NVS_PART_NOT_FOUND      (ESP_ERR_NVS_BASE + 0x0f)  /*!< Partition with specified name is not found in the partition table */

// Maximum lengths (matching ESP-IDF constraints)
#define NVS_KEY_NAME_MAX_SIZE    16   /*!< Maximum length of NVS key name (including null terminator) */
#define NVS_NS_NAME_MAX_SIZE     16   /*!< Maximum length of NVS namespace name (including null terminator) */

/**
 * @brief Mode of opening the non-volatile storage
 */
typedef enum {
    NVS_READONLY,  /*!< Read only */
    NVS_READWRITE  /*!< Read and write */
} nvs_open_mode_t;

/**
 * @brief Types of variables
 */
typedef enum {
    NVS_TYPE_U8    = 0x01,  /*!< Type uint8_t */
    NVS_TYPE_I8    = 0x11,  /*!< Type int8_t */
    NVS_TYPE_U16   = 0x02,  /*!< Type uint16_t */
    NVS_TYPE_I16   = 0x12,  /*!< Type int16_t */
    NVS_TYPE_U32   = 0x04,  /*!< Type uint32_t */
    NVS_TYPE_I32   = 0x14,  /*!< Type int32_t */
    NVS_TYPE_U64   = 0x08,  /*!< Type uint64_t */
    NVS_TYPE_I64   = 0x18,  /*!< Type int64_t */
    NVS_TYPE_STR   = 0x21,  /*!< Type string */
    NVS_TYPE_BLOB  = 0x42,  /*!< Type blob */
    NVS_TYPE_ANY   = 0xff   /*!< Must be last */
} nvs_type_t;

/**
 * @brief Opaque pointer type representing non-volatile storage handle
 */
typedef uint32_t nvs_handle_t;

/**
 * @brief NVS iterator opaque pointer type
 */
typedef struct nvs_iterator* nvs_iterator_t;

/**
 * @brief Information about entry obtained from nvs_entry_info function
 */
typedef struct {
    char namespace_name[NVS_NS_NAME_MAX_SIZE];  /*!< Namespace to which key-value belong */
    char key[NVS_KEY_NAME_MAX_SIZE];            /*!< Key of stored key-value pair */
    nvs_type_t type;                            /*!< Type of stored key-value pair */
} nvs_entry_info_t;

/**
 * @brief NVS statistics structure
 */
typedef struct {
    size_t used_entries;      /*!< Amount of used entries */
    size_t free_entries;      /*!< Amount of free entries */
    size_t total_entries;     /*!< Amount of all available entries */
    char namespace_name[NVS_NS_NAME_MAX_SIZE];  /*!< Namespace name */
} nvs_stats_t;

// =============================================================================
// NVS Flash Partition Management API
// =============================================================================

/**
 * @brief Initialize NVS flash storage for the default NVS partition.
 *
 * @return
 *      - ESP_OK if storage was successfully initialized
 *      - ESP_ERR_NVS_NO_FREE_PAGES if the NVS storage contains no empty pages
 *      - ESP_ERR_NOT_FOUND if no partition with label "nvs" is found in the partition table
 *      - ESP_ERR_INVALID_STATE if NVS partition is not initialized
 */
esp_err_t nvs_flash_init(void);

/**
 * @brief Initialize NVS flash storage for the specified partition.
 *
 * @param[in] partition_label Label of the partition
 *
 * @return
 *      - ESP_OK if storage was successfully initialized
 *      - ESP_ERR_NVS_NO_FREE_PAGES if the NVS storage contains no empty pages
 *      - ESP_ERR_NOT_FOUND if specified partition is not found in the partition table
 *      - ESP_ERR_INVALID_STATE if NVS partition is not initialized
 */
esp_err_t nvs_flash_init_partition(const char *partition_label);

/**
 * @brief Erase default NVS partition
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if there is no NVS partition in partition table
 */
esp_err_t nvs_flash_erase(void);

/**
 * @brief Erase specified NVS partition
 *
 * @param[in] part_name  Name (label) of the partition to be erased
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if there is no NVS partition in partition table
 */
esp_err_t nvs_flash_erase_partition(const char *part_name);

/**
 * @brief Deinitialize NVS storage for the default NVS partition
 *
 * @return
 *      - ESP_OK on success (nvs storage was deinitialized)
 *      - ESP_ERR_NVS_NOT_INITIALIZED if the storage was not initialized prior to this call
 */
esp_err_t nvs_flash_deinit(void);

/**
 * @brief Deinitialize NVS storage for the given NVS partition
 *
 * @param[in] partition_label Label of the partition
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NVS_NOT_INITIALIZED if the storage for given partition was not initialized prior to this call
 */
esp_err_t nvs_flash_deinit_partition(const char *partition_label);

// =============================================================================
// NVS Data Access API
// =============================================================================

/**
 * @brief Open non-volatile storage with a given namespace from the default NVS partition
 *
 * @param[in] namespace_name   Namespace name
 * @param[in] open_mode        NVS_READWRITE or NVS_READONLY
 * @param[out] out_handle      If successful, will return handle for the storage handle
 *
 * @return
 *      - ESP_OK if storage handle was opened successfully
 *      - ESP_ERR_NVS_NOT_INITIALIZED if the storage driver is not initialized
 *      - ESP_ERR_NVS_PART_NOT_FOUND if the partition with given name is not found
 *      - ESP_ERR_NVS_NOT_FOUND if namespace doesn't exist yet and mode is NVS_READONLY
 *      - ESP_ERR_NVS_INVALID_NAME if namespace name doesn't satisfy constraints
 *      - other error codes from the underlying storage driver
 */
esp_err_t nvs_open(const char* namespace_name, nvs_open_mode_t open_mode, nvs_handle_t *out_handle);

/**
 * @brief Open non-volatile storage with a given namespace from specified partition
 *
 * @param[in] part_name        Label (name) of the partition of interest for object read/write/erase
 * @param[in] namespace_name   Namespace name
 * @param[in] open_mode        NVS_READWRITE or NVS_READONLY
 * @param[out] out_handle      If successful, will return handle for the storage handle
 *
 * @return
 *      - ESP_OK if storage handle was opened successfully
 *      - ESP_ERR_NVS_NOT_INITIALIZED if the storage driver is not initialized
 *      - ESP_ERR_NVS_PART_NOT_FOUND if the partition with given name is not found
 *      - ESP_ERR_NVS_NOT_FOUND if namespace doesn't exist yet and mode is NVS_READONLY
 *      - ESP_ERR_NVS_INVALID_NAME if namespace name doesn't satisfy constraints
 *      - other error codes from the underlying storage driver
 */
esp_err_t nvs_open_from_partition(const char *part_name, const char* namespace_name, 
                                  nvs_open_mode_t open_mode, nvs_handle_t *out_handle);

/**
 * @brief Close the storage handle and free any allocated resources
 *
 * @param[in] handle Storage handle obtained with nvs_open function.
 *                   To close the handle this function will be called.
 */
void nvs_close(nvs_handle_t handle);

/**
 * @brief Commit written key-value pairs to non-volatile storage
 *
 * @param[in] handle Storage handle obtained with nvs_open function.
 *
 * @return
 *     - ESP_OK if the changes have been written successfully
 *     - other error codes from the underlying storage driver
 */
esp_err_t nvs_commit(nvs_handle_t handle);

// =============================================================================
// Set Value APIs
// =============================================================================

/**
 * @brief Set value for given key as uint8_t
 *
 * @param[in] handle    Storage handle obtained with nvs_open function
 * @param[in] key       Key name
 * @param[in] value     The value to set
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_READ_ONLY if storage handle was opened as read only
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_NOT_ENOUGH_SPACE if there is not enough space in the underlying storage
 *             - ESP_ERR_NVS_REMOVE_FAILED if the value wasn't updated because flash write operation has failed
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_set_u8(nvs_handle_t handle, const char* key, uint8_t value);

/**
 * @brief Set value for given key as int8_t
 */
esp_err_t nvs_set_i8(nvs_handle_t handle, const char* key, int8_t value);

/**
 * @brief Set value for given key as uint16_t
 */
esp_err_t nvs_set_u16(nvs_handle_t handle, const char* key, uint16_t value);

/**
 * @brief Set value for given key as int16_t
 */
esp_err_t nvs_set_i16(nvs_handle_t handle, const char* key, int16_t value);

/**
 * @brief Set value for given key as uint32_t
 */
esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value);

/**
 * @brief Set value for given key as int32_t
 */
esp_err_t nvs_set_i32(nvs_handle_t handle, const char* key, int32_t value);

/**
 * @brief Set value for given key as uint64_t
 */
esp_err_t nvs_set_u64(nvs_handle_t handle, const char* key, uint64_t value);

/**
 * @brief Set value for given key as int64_t
 */
esp_err_t nvs_set_i64(nvs_handle_t handle, const char* key, int64_t value);

/**
 * @brief Set value for given key as string
 *
 * @param[in] handle    Storage handle obtained with nvs_open function
 * @param[in] key       Key name
 * @param[in] value     The null-terminated string value to set
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_READ_ONLY if storage handle was opened as read only
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_VALUE_TOO_LONG if the string value is too long
 *             - ESP_ERR_NVS_NOT_ENOUGH_SPACE if there is not enough space in the underlying storage
 *             - ESP_ERR_NVS_REMOVE_FAILED if the value wasn't updated because flash write operation has failed
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value);

/**
 * @brief Set value for given key as blob
 *
 * @param[in] handle    Storage handle obtained with nvs_open function
 * @param[in] key       Key name
 * @param[in] value     The blob value to set
 * @param[in] length    Length of binary data in bytes
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_READ_ONLY if storage handle was opened as read only
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_VALUE_TOO_LONG if the blob data is too long
 *             - ESP_ERR_NVS_NOT_ENOUGH_SPACE if there is not enough space in the underlying storage
 *             - ESP_ERR_NVS_REMOVE_FAILED if the value wasn't updated because flash write operation has failed
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);

// =============================================================================
// Get Value APIs
// =============================================================================

/**
 * @brief Get value for given key as uint8_t
 *
 * @param[in] handle     Storage handle obtained with nvs_open function
 * @param[in] key        Key name
 * @param[out] out_value Pointer to the output value
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_NOT_FOUND if the requested key doesn't exist
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_TYPE_MISMATCH if the type of get operation doesn't match the type of value stored in NVS
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_get_u8(nvs_handle_t handle, const char* key, uint8_t* out_value);

/**
 * @brief Get value for given key as int8_t
 */
esp_err_t nvs_get_i8(nvs_handle_t handle, const char* key, int8_t* out_value);

/**
 * @brief Get value for given key as uint16_t
 */
esp_err_t nvs_get_u16(nvs_handle_t handle, const char* key, uint16_t* out_value);

/**
 * @brief Get value for given key as int16_t
 */
esp_err_t nvs_get_i16(nvs_handle_t handle, const char* key, int16_t* out_value);

/**
 * @brief Get value for given key as uint32_t
 */
esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value);

/**
 * @brief Get value for given key as int32_t
 */
esp_err_t nvs_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value);

/**
 * @brief Get value for given key as uint64_t
 */
esp_err_t nvs_get_u64(nvs_handle_t handle, const char* key, uint64_t* out_value);

/**
 * @brief Get value for given key as int64_t
 */
esp_err_t nvs_get_i64(nvs_handle_t handle, const char* key, int64_t* out_value);

/**
 * @brief Get value for given key as string
 *
 * @param[in] handle        Storage handle obtained with nvs_open function
 * @param[in] key           Key name
 * @param[out] out_value    Pointer to output value.
 *                          If length is not sufficient to store the value,
 *                          ESP_ERR_NVS_INVALID_LENGTH is returned and required_size will be set to the actual length.
 * @param[inout] length     A non-zero pointer to the variable holding the length of out_value.
 *                          In case out_value a zero, will be set to the length required to hold the value.
 *                          In case out_value is not zero, will be set to the actual length of the value written.
 *                          For strings, this includes zero terminator.
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_NOT_FOUND if the requested key doesn't exist
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_INVALID_LENGTH if length is not sufficient to store data
 *             - ESP_ERR_NVS_TYPE_MISMATCH if the type of get operation doesn't match the type of value stored in NVS
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value, size_t* length);

/**
 * @brief Get value for given key as blob
 *
 * @param[in] handle        Storage handle obtained with nvs_open function
 * @param[in] key           Key name
 * @param[out] out_value    Pointer to output value.
 *                          If length is not sufficient to store the value,
 *                          ESP_ERR_NVS_INVALID_LENGTH is returned and required_size will be set to the actual length.
 * @param[inout] length     A non-zero pointer to the variable holding the length of out_value.
 *                          In case out_value a zero, will be set to the length required to hold the value.
 *                          In case out_value is not zero, will be set to the actual length of the value written.
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_NOT_FOUND if the requested key doesn't exist
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - ESP_ERR_NVS_INVALID_LENGTH if length is not sufficient to store data
 *             - ESP_ERR_NVS_TYPE_MISMATCH if the type of get operation doesn't match the type of value stored in NVS
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);

// =============================================================================
// Erase APIs
// =============================================================================

/**
 * @brief Erase key-value pair with given key name
 *
 * @param[in] handle    Storage handle obtained with nvs_open function
 * @param[in] key       Key name
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_READ_ONLY if storage handle was opened as read only
 *             - ESP_ERR_NVS_NOT_FOUND if the requested key doesn't exist
 *             - ESP_ERR_NVS_INVALID_NAME if key name doesn't satisfy constraints
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key);

/**
 * @brief Erase all key-value pairs in a namespace
 *
 * @param[in] handle    Storage handle obtained with nvs_open function
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - ESP_ERR_NVS_READ_ONLY if storage handle was opened as read only
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_erase_all(nvs_handle_t handle);

// =============================================================================
// Utility and Statistics APIs
// =============================================================================

/**
 * @brief Fill NVS storage statistics
 *
 * @param[in] part_name         Partition name NVS partition
 * @param[out] nvs_stats        Pointer to nvs_stats_t structure to fill with data
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_NOT_INITIALIZED if the storage driver is not initialized
 *             - ESP_ERR_NVS_PART_NOT_FOUND if the partition with given name is not found
 *             - ESP_ERR_INVALID_ARG if nvs_stats equal to NULL
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_get_stats(const char* part_name, nvs_stats_t* nvs_stats);

/**
 * @brief Get the number of used entries
 *
 * @param[in] handle            Storage handle obtained with nvs_open function
 * @param[out] used_entries     Returns amount of used entries from namespace
 *
 * @return
 *             - ESP_OK on success
 *             - ESP_ERR_NVS_INVALID_HANDLE if handle has been closed or is NULL
 *             - other error codes from the underlying storage driver
 */
esp_err_t nvs_get_used_entry_count(nvs_handle_t handle, size_t* used_entries);

#ifdef __cplusplus
}
#endif