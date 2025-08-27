/**
 * @file freertos_integration_test.cpp
 * @brief Integration test for FreeRTOS kernel and CPU execution
 * 
 * This example demonstrates how to use the enhanced FreeRTOS kernel
 * with CPU integration for running real multi-tasking applications.
 */

#include "emulator/core/emulator_core.hpp"
#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/esp_idf/freertos_api.hpp"
#include "emulator/esp_idf/esp_log.h"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace m5tab5::emulator;

static const char* TAG = "FREERTOS_INTEGRATION_TEST";

/**
 * @brief Simple task that prints messages
 */
void simple_task(void* pvParameters) {
    const char* task_name = static_cast<const char*>(pvParameters);
    int counter = 0;
    
    ESP_LOGI(TAG, "Task '%s' started", task_name);
    
    while (counter < 10) {
        counter++;
        ESP_LOGI(TAG, "Task '%s' iteration %d", task_name, counter);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
    
    ESP_LOGI(TAG, "Task '%s' completed", task_name);
    vTaskDelete(nullptr);
}

/**
 * @brief Producer-Consumer test
 */
static QueueHandle_t test_queue = nullptr;

void producer_task(void* pvParameters) {
    ESP_LOGI(TAG, "Producer task started");
    
    for (int i = 1; i <= 5; i++) {
        int message = i * 10;
        
        if (xQueueSend(test_queue, &message, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Producer: Sent message %d", message);
        } else {
            ESP_LOGE(TAG, "Producer: Failed to send message %d", message);
        }
        
        vTaskDelay(pdMS_TO_TICKS(800)); // 800ms delay
    }
    
    ESP_LOGI(TAG, "Producer task completed");
    vTaskDelete(nullptr);
}

void consumer_task(void* pvParameters) {
    ESP_LOGI(TAG, "Consumer task started");
    
    int received_message;
    int message_count = 0;
    
    while (message_count < 5) {
        if (xQueueReceive(test_queue, &received_message, pdMS_TO_TICKS(2000)) == pdPASS) {
            message_count++;
            ESP_LOGI(TAG, "Consumer: Received message %d (%d/5)", received_message, message_count);
        } else {
            ESP_LOGW(TAG, "Consumer: Timeout waiting for message");
        }
    }
    
    ESP_LOGI(TAG, "Consumer task completed");
    vTaskDelete(nullptr);
}

/**
 * @brief Run basic FreeRTOS functionality test
 */
void run_basic_test() {
    std::cout << "\n=== Basic FreeRTOS Test ===" << std::endl;
    
    ESP_LOGI(TAG, "Creating simple tasks");
    
    // Create simple tasks
    BaseType_t result1 = xTaskCreatePinnedToCore(
        simple_task,
        "Task1",
        2048,
        (void*)"TaskA",
        5,
        nullptr,
        0  // Core 0
    );
    
    BaseType_t result2 = xTaskCreatePinnedToCore(
        simple_task,
        "Task2", 
        2048,
        (void*)"TaskB",
        5,
        nullptr,
        1  // Core 1
    );
    
    if (result1 == pdPASS && result2 == pdPASS) {
        ESP_LOGI(TAG, "Simple tasks created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create simple tasks");
    }
}

/**
 * @brief Run producer-consumer test
 */
void run_producer_consumer_test() {
    std::cout << "\n=== Producer-Consumer Test ===" << std::endl;
    
    ESP_LOGI(TAG, "Creating producer-consumer test");
    
    // Create queue
    test_queue = xQueueCreate(3, sizeof(int));
    if (test_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create test queue");
        return;
    }
    
    // Create producer task
    BaseType_t result1 = xTaskCreatePinnedToCore(
        producer_task,
        "Producer",
        2048,
        nullptr,
        6,
        nullptr,
        0  // Core 0
    );
    
    // Create consumer task
    BaseType_t result2 = xTaskCreatePinnedToCore(
        consumer_task,
        "Consumer",
        2048,
        nullptr,
        6,
        nullptr,
        1  // Core 1
    );
    
    if (result1 == pdPASS && result2 == pdPASS) {
        ESP_LOGI(TAG, "Producer-consumer tasks created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create producer-consumer tasks");
    }
}

/**
 * @brief Main application function
 */
void run_freertos_app() {
    ESP_LOGI(TAG, "=== FreeRTOS Integration Test Application ===");
    ESP_LOGI(TAG, "System information:");
    ESP_LOGI(TAG, "- CPU Cores: %d", configNUM_CORES);
    ESP_LOGI(TAG, "- Max Priorities: %d", configMAX_PRIORITIES);
    ESP_LOGI(TAG, "- Tick Rate: %d Hz", configTICK_RATE_HZ);
    ESP_LOGI(TAG, "- Free Heap: %zu bytes", xPortGetFreeHeapSize());
    
    // Run tests
    run_basic_test();
    run_producer_consumer_test();
    
    ESP_LOGI(TAG, "All test tasks created, letting FreeRTOS scheduler run...");
    
    // Let the application run for a while to see the results
    std::this_thread::sleep_for(std::chrono::seconds(20));
    
    ESP_LOGI(TAG, "Test completed successfully");
}

int main(int argc, char* argv[]) {
    std::cout << "M5Stack Tab5 Emulator - FreeRTOS Integration Test" << std::endl;
    std::cout << "=================================================" << std::endl;
    
    try {
        // Initialize logging
        LoggerManager::initialize();
        LoggerManager::set_global_log_level(LogLevel::INFO);
        
        // Load configuration
        auto config_result = config::Configuration::load("config/development.json");
        if (!config_result.has_value()) {
            std::cerr << "Failed to load configuration: " << config_result.error().to_string() << std::endl;
            return 1;
        }
        
        // Create emulator core
        auto emulator_result = EmulatorCore::create(config_result.value());
        if (!emulator_result.has_value()) {
            std::cerr << "Failed to create emulator core: " << emulator_result.error().to_string() << std::endl;
            return 1;
        }
        
        auto emulator = std::move(emulator_result.value());
        
        // Initialize emulator
        auto init_result = emulator->initialize();
        if (!init_result.has_value()) {
            std::cerr << "Failed to initialize emulator: " << init_result.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "Emulator initialized successfully" << std::endl;
        
        // Get FreeRTOS kernel
        auto freertos_kernel_result = emulator->get_component<freertos::FreeRTOSKernel>();
        if (!freertos_kernel_result.has_value()) {
            std::cerr << "Failed to get FreeRTOS kernel component" << std::endl;
            return 1;
        }
        
        auto freertos_kernel = freertos_kernel_result.value();
        
        // Configure FreeRTOS kernel
        freertos::FreeRTOSKernel::KernelConfig kernel_config;
        kernel_config.tick_frequency_hz = 1000;
        kernel_config.total_heap_size = 128 * 1024; // 128KB
        kernel_config.enable_task_statistics = true;
        kernel_config.enable_stack_overflow_checking = true;
        
        // Initialize FreeRTOS kernel
        auto kernel_init_result = freertos_kernel->initialize(kernel_config);
        if (!kernel_init_result.has_value()) {
            std::cerr << "Failed to initialize FreeRTOS kernel: " << kernel_init_result.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "FreeRTOS kernel initialized successfully" << std::endl;
        
        // Start FreeRTOS kernel
        auto kernel_start_result = freertos_kernel->start_kernel();
        if (!kernel_start_result.has_value()) {
            std::cerr << "Failed to start FreeRTOS kernel: " << kernel_start_result.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "FreeRTOS kernel started successfully" << std::endl;
        
        // Initialize ESP-IDF FreeRTOS API compatibility layer
        esp_idf::initialize_freertos_api();
        
        std::cout << "ESP-IDF API compatibility layer initialized" << std::endl;
        
        // Run the FreeRTOS application
        run_freertos_app();
        
        // Print final statistics
        std::cout << "\n=== Final System Statistics ===" << std::endl;
        freertos_kernel->dump_kernel_statistics();
        freertos_kernel->print_stack_high_water_marks();
        
        // Stop kernel
        auto kernel_stop_result = freertos_kernel->stop_kernel();
        if (!kernel_stop_result.has_value()) {
            std::cerr << "Warning: Failed to stop FreeRTOS kernel cleanly: " << kernel_stop_result.error().to_string() << std::endl;
        }
        
        // Shutdown ESP-IDF API
        esp_idf::shutdown_freertos_api();
        
        std::cout << "Test completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 1;
    }
}