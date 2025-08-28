/**
 * @file test_esp_idf_simple.cpp
 * @brief Simple test for ESP-IDF API compilation
 */

#include "include/emulator/esp_idf/esp_types.h"
#include "include/emulator/esp_idf/esp_system.h"
#include <iostream>

int main() {
    std::cout << "Testing ESP-IDF API compilation..." << std::endl;
    
    // Test basic ESP-IDF functions
    esp_chip_info_t chip_info;
    esp_err_t result = esp_chip_info(&chip_info);
    
    if (result == ESP_OK) {
        std::cout << "ESP-IDF chip info successful" << std::endl;
        std::cout << "Cores: " << chip_info.cores << std::endl;
        std::cout << "Revision: " << chip_info.revision << std::endl;
    } else {
        std::cout << "ESP-IDF chip info failed: " << result << std::endl;
    }
    
    return 0;
}