#include "emulator/esp_idf/nvs.h"
#include "emulator/utils/logging.hpp"
#include <iostream>
#include <string>
#include <cstring>

// Test program to demonstrate NVS persistence functionality
int main() {
    LOG_INFO("Testing NVS persistence functionality...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to initialize NVS: {}", static_cast<int>(ret));
        return 1;
    }
    
    nvs_handle_t handle;
    ret = nvs_open("storage", NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to open NVS namespace: {}", static_cast<int>(ret));
        return 1;
    }
    
    // Test writing different data types
    LOG_INFO("Writing test data to NVS...");
    
    // Write string data
    const char* wifi_ssid = "MyTestNetwork";
    ret = nvs_set_str(handle, "wifi_ssid", wifi_ssid);
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to set wifi_ssid: {}", static_cast<int>(ret));
        return 1;
    }
    
    // Write integer data
    uint32_t device_id = 0x12345678;
    ret = nvs_set_u32(handle, "device_id", device_id);
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to set device_id: {}", static_cast<int>(ret));
        return 1;
    }
    
    // Write blob data
    uint8_t config_data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    ret = nvs_set_blob(handle, "config_blob", config_data, sizeof(config_data));
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to set config_blob: {}", static_cast<int>(ret));
        return 1;
    }
    
    // Commit data to persistent storage
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to commit NVS data: {}", static_cast<int>(ret));
        return 1;
    }
    
    LOG_INFO("Data written successfully, now reading back...");
    
    // Read data back to verify
    char read_ssid[64];
    size_t ssid_len = sizeof(read_ssid);
    ret = nvs_get_str(handle, "wifi_ssid", read_ssid, &ssid_len);
    if (ret == ESP_OK) {
        LOG_INFO("Read wifi_ssid: '{}'", read_ssid);
        if (strcmp(read_ssid, wifi_ssid) == 0) {
            LOG_INFO("✓ String data matches!");
        } else {
            LOG_ERROR("✗ String data mismatch!");
        }
    } else {
        LOG_ERROR("Failed to read wifi_ssid: {}", static_cast<int>(ret));
    }
    
    uint32_t read_device_id;
    ret = nvs_get_u32(handle, "device_id", &read_device_id);
    if (ret == ESP_OK) {
        LOG_INFO("Read device_id: 0x{:08X}", read_device_id);
        if (read_device_id == device_id) {
            LOG_INFO("✓ Integer data matches!");
        } else {
            LOG_ERROR("✗ Integer data mismatch!");
        }
    } else {
        LOG_ERROR("Failed to read device_id: {}", static_cast<int>(ret));
    }
    
    uint8_t read_blob[16];
    size_t blob_len = sizeof(read_blob);
    ret = nvs_get_blob(handle, "config_blob", read_blob, &blob_len);
    if (ret == ESP_OK) {
        LOG_INFO("Read config_blob: {} bytes", blob_len);
        if (blob_len == sizeof(config_data) && 
            memcmp(read_blob, config_data, blob_len) == 0) {
            LOG_INFO("✓ Blob data matches!");
        } else {
            LOG_ERROR("✗ Blob data mismatch!");
        }
    } else {
        LOG_ERROR("Failed to read config_blob: {}", static_cast<int>(ret));
    }
    
    // Get statistics
    nvs_stats_t stats;
    ret = nvs_get_stats(nullptr, &stats);
    if (ret == ESP_OK) {
        LOG_INFO("NVS Statistics:");
        LOG_INFO("  Used entries: {}", stats.used_entries);
        LOG_INFO("  Free entries: {}", stats.free_entries);
        LOG_INFO("  Total entries: {}", stats.total_entries);
    }
    
    size_t used_entries;
    ret = nvs_get_used_entry_count(handle, &used_entries);
    if (ret == ESP_OK) {
        LOG_INFO("Namespace 'storage' used entries: {}", used_entries);
    }
    
    nvs_close(handle);
    
    LOG_INFO("Test completed. Data should persist across emulator restarts.");
    LOG_INFO("Check ~/.m5tab5_emulator/nvs.db for the SQLite database file.");
    
    return 0;
}