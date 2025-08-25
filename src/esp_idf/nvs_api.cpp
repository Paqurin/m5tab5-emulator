#include "emulator/esp_idf/nvs.h"
#include "emulator/utils/logging.hpp"

#include <unordered_map>
#include <mutex>
#include <string>

// Simple in-memory NVS implementation for ESP-IDF compatibility
// This provides a minimal working implementation while the full NVS system is being developed

namespace {
    // Simple in-memory storage
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<uint8_t>>> g_simple_storage;
    std::mutex g_simple_mutex;
    nvs_handle_t g_next_handle = 1;
    std::unordered_map<nvs_handle_t, std::pair<std::string, bool>> g_handle_map; // handle -> (namespace, writable)
    bool g_initialized = false;
}

extern "C" {

// Initialize NVS partition
esp_err_t nvs_flash_init() {
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    g_initialized = true;
    LOG_INFO("Simple NVS flash initialized");
    return ESP_OK;
}

esp_err_t nvs_flash_init_partition(const char* partition_label) {
    return nvs_flash_init();
}

// Erase NVS partition
esp_err_t nvs_flash_erase() {
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    g_simple_storage.clear();
    LOG_INFO("Simple NVS flash erased");
    return ESP_OK;
}

esp_err_t nvs_flash_erase_partition(const char* partition_label) {
    return nvs_flash_erase();
}

// Open NVS namespace
esp_err_t nvs_open(const char* namespace_name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
    if (!namespace_name || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_initialized) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    nvs_handle_t handle = g_next_handle++;
    g_handle_map[handle] = {std::string(namespace_name), open_mode == NVS_READWRITE};
    *out_handle = handle;
    
    LOG_DEBUG("Opened simple NVS namespace '{}' with handle {}", namespace_name, handle);
    return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* partition_label, const char* namespace_name, 
                                 nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
    return nvs_open(namespace_name, open_mode, out_handle);
}

// Close NVS handle
void nvs_close(nvs_handle_t handle) {
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    g_handle_map.erase(handle);
    LOG_DEBUG("Closed simple NVS handle {}", handle);
}

// Generic set function
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length) {
    if (!key || !value || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    std::vector<uint8_t> data(static_cast<const uint8_t*>(value), 
                             static_cast<const uint8_t*>(value) + length);
    g_simple_storage[ns][key] = std::move(data);
    
    return ESP_OK;
}

// Typed set functions
esp_err_t nvs_set_u8(nvs_handle_t handle, const char* key, uint8_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i8(nvs_handle_t handle, const char* key, int8_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u16(nvs_handle_t handle, const char* key, uint16_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i16(nvs_handle_t handle, const char* key, int16_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i32(nvs_handle_t handle, const char* key, int32_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u64(nvs_handle_t handle, const char* key, uint64_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i64(nvs_handle_t handle, const char* key, int64_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    return nvs_set_blob(handle, key, value, strlen(value) + 1);
}

// Generic get function
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length) {
    if (!key || !length) {
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    const std::string& ns = it->second.first;
    auto ns_it = g_simple_storage.find(ns);
    if (ns_it == g_simple_storage.end()) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    auto key_it = ns_it->second.find(key);
    if (key_it == ns_it->second.end()) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    const auto& data = key_it->second;
    
    if (out_value == nullptr) {
        // Just return size
        *length = data.size();
        return ESP_OK;
    }
    
    if (*length < data.size()) {
        *length = data.size();
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    
    memcpy(out_value, data.data(), data.size());
    *length = data.size();
    return ESP_OK;
}

// Typed get functions
esp_err_t nvs_get_u8(nvs_handle_t handle, const char* key, uint8_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint8_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i8(nvs_handle_t handle, const char* key, int8_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int8_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u16(nvs_handle_t handle, const char* key, uint16_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint16_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i16(nvs_handle_t handle, const char* key, int16_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int16_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint32_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int32_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u64(nvs_handle_t handle, const char* key, uint64_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint64_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i64(nvs_handle_t handle, const char* key, int64_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int64_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value, size_t* length) {
    return nvs_get_blob(handle, key, out_value, length);
}

// Erase operations
esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key) {
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    auto ns_it = g_simple_storage.find(ns);
    if (ns_it != g_simple_storage.end()) {
        ns_it->second.erase(key);
    }
    
    return ESP_OK;
}

esp_err_t nvs_erase_all(nvs_handle_t handle) {
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    g_simple_storage[ns].clear();
    
    return ESP_OK;
}

// Commit operations
esp_err_t nvs_commit(nvs_handle_t handle) {
    // No-op for simple implementation
    return ESP_OK;
}

// Statistics and info functions
esp_err_t nvs_get_stats(const char* part_name, nvs_stats_t* nvs_stats) {
    if (!nvs_stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    
    // Calculate simple statistics
    size_t total_entries = 0;
    for (const auto& ns_pair : g_simple_storage) {
        total_entries += ns_pair.second.size();
    }
    
    nvs_stats->used_entries = total_entries;
    nvs_stats->free_entries = 1000 - total_entries;  // Simulated
    nvs_stats->total_entries = 1000;  // Simulated
    
    return ESP_OK;
}

esp_err_t nvs_get_used_entry_count(nvs_handle_t handle, size_t* used_entries) {
    if (!used_entries) {
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(g_simple_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    const std::string& ns = it->second.first;
    auto ns_it = g_simple_storage.find(ns);
    *used_entries = (ns_it != g_simple_storage.end()) ? ns_it->second.size() : 0;
    
    return ESP_OK;
}

// Iterator functions (basic implementation)
esp_err_t nvs_entry_find(const char* part_name, const char* namespace_name, nvs_type_t type, nvs_iterator_t* output_iterator) {
    return ESP_ERR_NVS_NOT_FOUND;  // Not implemented
}

esp_err_t nvs_entry_next(nvs_iterator_t* iterator) {
    return ESP_ERR_NVS_NOT_FOUND;  // Not implemented
}

void nvs_entry_info(nvs_iterator_t iterator, nvs_entry_info_t* out_info) {
    // Not implemented
}

void nvs_release_iterator(nvs_iterator_t iterator) {
    // Not implemented
}

} // extern "C"