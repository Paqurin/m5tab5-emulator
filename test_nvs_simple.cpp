#include "emulator/esp_idf/nvs.h"
#include "emulator/utils/logging.hpp"
#include <iostream>

int main() {
    // Initialize logging first
    try {
        m5tab5::emulator::Logger::initialize();
    } catch (...) {
        std::cout << "Logger initialization failed, continuing..." << std::endl;
    }
    
    std::cout << "Testing NVS persistence functionality..." << std::endl;
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        std::cout << "Failed to initialize NVS: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    
    std::cout << "NVS initialized successfully" << std::endl;
    
    nvs_handle_t handle;
    ret = nvs_open("storage", NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        std::cout << "Failed to open NVS namespace: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    
    std::cout << "NVS namespace opened successfully" << std::endl;
    
    // Write a simple string value
    ret = nvs_set_str(handle, "test_key", "Hello, NVS!");
    if (ret != ESP_OK) {
        std::cout << "Failed to set string value: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    
    std::cout << "String value written successfully" << std::endl;
    
    // Commit the data
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        std::cout << "Failed to commit: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    
    std::cout << "Data committed successfully" << std::endl;
    
    // Read it back
    char buffer[64];
    size_t buffer_size = sizeof(buffer);
    ret = nvs_get_str(handle, "test_key", buffer, &buffer_size);
    if (ret == ESP_OK) {
        std::cout << "Read back value: '" << buffer << "'" << std::endl;
        std::cout << "Buffer size: " << buffer_size << std::endl;
        std::cout << "✓ NVS read/write test successful!" << std::endl;
    } else {
        std::cout << "Failed to read string value: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    
    nvs_close(handle);
    std::cout << "NVS handle closed" << std::endl;
    
    std::cout << "Test completed successfully!" << std::endl;
    std::cout << "If SQLite is available, data should persist in ~/.m5tab5_emulator/nvs.db" << std::endl;
    
    return 0;
}