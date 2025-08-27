/**
 * @file freertos_test_firmware.cpp
 * @brief Simple multi-task test application for FreeRTOS kernel validation
 * 
 * This test firmware demonstrates basic FreeRTOS functionality including:
 * - Multi-task creation and scheduling
 * - Inter-task communication with queues
 * - Task synchronization with semaphores  
 * - Task delays and timing
 * - ESP-IDF API compatibility
 */

#include "emulator/esp_idf/freertos_api.hpp"
#include "emulator/esp_idf/esp_log_api.hpp"
#include "emulator/esp_idf/esp_system_api.hpp"
#include <chrono>
#include <cstring>

static const char* TAG = "FreeRTOS_TEST";

// Test data structures
static QueueHandle_t test_queue = nullptr;
static SemaphoreHandle_t test_semaphore = nullptr;
static TaskHandle_t producer_task_handle = nullptr;
static TaskHandle_t consumer_task_handle = nullptr;
static TaskHandle_t monitor_task_handle = nullptr;

// Test message structure
struct TestMessage {
    uint32_t message_id;
    uint32_t timestamp;
    char data[32];
};

/**
 * @brief Producer task - generates test messages
 */
static void producer_task(void* pvParameters) {
    ESP_LOGI(TAG, "Producer task started on core %d", xPortGetCoreID());
    
    uint32_t message_counter = 0;
    TestMessage message;
    
    while (true) {
        // Create test message
        message.message_id = ++message_counter;
        message.timestamp = xTaskGetTickCount();
        snprintf(message.data, sizeof(message.data), "MSG_%lu", message.message_id);
        
        ESP_LOGI(TAG, "Producer: Creating message %lu", message.message_id);
        
        // Send message to queue
        if (xQueueSend(test_queue, &message, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Producer: Message %lu sent successfully", message.message_id);
        } else {
            ESP_LOGW(TAG, "Producer: Failed to send message %lu", message.message_id);
        }
        
        // Wait before producing next message
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds
    }
}

/**
 * @brief Consumer task - processes test messages
 */
static void consumer_task(void* pvParameters) {
    ESP_LOGI(TAG, "Consumer task started on core %d", xPortGetCoreID());
    
    TestMessage received_message;
    
    while (true) {
        // Wait for message from queue
        if (xQueueReceive(test_queue, &received_message, pdMS_TO_TICKS(5000)) == pdPASS) {
            ESP_LOGI(TAG, "Consumer: Received message %lu: '%s' (timestamp: %lu)", 
                     received_message.message_id, 
                     received_message.data,
                     received_message.timestamp);
            
            // Simulate processing time
            vTaskDelay(pdMS_TO_TICKS(500)); // 500ms processing
            
            // Signal processing complete
            xSemaphoreGive(test_semaphore);
            
            ESP_LOGI(TAG, "Consumer: Message %lu processed", received_message.message_id);
        } else {
            ESP_LOGW(TAG, "Consumer: Timeout waiting for message");
        }
    }
}

/**
 * @brief Monitor task - tracks system statistics
 */
static void monitor_task(void* pvParameters) {
    ESP_LOGI(TAG, "Monitor task started on core %d", xPortGetCoreID());
    
    uint32_t processed_count = 0;
    
    while (true) {
        // Wait for semaphore signal (message processed)
        if (xSemaphoreTake(test_semaphore, pdMS_TO_TICKS(10000)) == pdPASS) {
            processed_count++;
            ESP_LOGI(TAG, "Monitor: Total messages processed: %lu", processed_count);
            
            // Print system statistics
            ESP_LOGI(TAG, "Monitor: Current tick count: %lu", xTaskGetTickCount());
            ESP_LOGI(TAG, "Monitor: Free heap size: %d bytes", xPortGetFreeHeapSize());
            
            // Print task stack high water marks
            UBaseType_t producer_stack = uxTaskGetStackHighWaterMark(producer_task_handle);
            UBaseType_t consumer_stack = uxTaskGetStackHighWaterMark(consumer_task_handle);
            UBaseType_t monitor_stack = uxTaskGetStackHighWaterMark(monitor_task_handle);
            
            ESP_LOGI(TAG, "Monitor: Stack usage - Producer: %lu, Consumer: %lu, Monitor: %lu", 
                     producer_stack, consumer_stack, monitor_stack);
        } else {
            ESP_LOGW(TAG, "Monitor: Timeout waiting for processed message signal");
        }
    }
}

/**
 * @brief High priority periodic task
 */
static void periodic_task(void* pvParameters) {
    ESP_LOGI(TAG, "Periodic task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    
    while (true) {
        cycle_count++;
        ESP_LOGI(TAG, "Periodic: Cycle %lu at tick %lu", cycle_count, xTaskGetTickCount());
        
        // Check queue status
        UBaseType_t queue_messages = uxQueueMessagesWaiting(test_queue);
        ESP_LOGI(TAG, "Periodic: Queue has %lu messages waiting", queue_messages);
        
        // Wait for exact period (1 second)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Test task priorities and core affinity
 */
static void priority_test_task(void* pvParameters) {
    const char* task_name = (const char*)pvParameters;
    UBaseType_t priority = uxTaskPriorityGet(nullptr);
    
    ESP_LOGI(TAG, "Priority test task '%s' started with priority %lu on core %d", 
             task_name, priority, xPortGetCoreID());
    
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Task '%s' iteration %d (priority %lu)", task_name, i + 1, priority);
        vTaskDelay(pdMS_TO_TICKS(1500)); // 1.5 seconds
    }
    
    ESP_LOGI(TAG, "Priority test task '%s' completed", task_name);
    vTaskDelete(nullptr); // Delete self
}

/**
 * @brief Initialize test synchronization objects
 */
static void initialize_test_objects() {
    ESP_LOGI(TAG, "Initializing test synchronization objects");
    
    // Create queue for inter-task communication
    test_queue = xQueueCreate(5, sizeof(TestMessage));
    if (test_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create test queue");
        return;
    }
    ESP_LOGI(TAG, "Test queue created successfully");
    
    // Create binary semaphore for synchronization
    test_semaphore = xSemaphoreCreateBinary();
    if (test_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create test semaphore");
        return;
    }
    ESP_LOGI(TAG, "Test semaphore created successfully");
}

/**
 * @brief Create all test tasks
 */
static void create_test_tasks() {
    ESP_LOGI(TAG, "Creating test tasks");
    
    BaseType_t result;
    
    // Create producer task on core 0
    result = xTaskCreatePinnedToCore(
        producer_task,
        "producer",
        4096,          // Stack size
        nullptr,       // Parameters
        5,             // Priority
        &producer_task_handle,
        0              // Core 0
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create producer task");
        return;
    }
    ESP_LOGI(TAG, "Producer task created successfully");
    
    // Create consumer task on core 1
    result = xTaskCreatePinnedToCore(
        consumer_task,
        "consumer",
        4096,          // Stack size
        nullptr,       // Parameters
        5,             // Priority
        &consumer_task_handle,
        1              // Core 1
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create consumer task");
        return;
    }
    ESP_LOGI(TAG, "Consumer task created successfully");
    
    // Create monitor task with higher priority
    result = xTaskCreatePinnedToCore(
        monitor_task,
        "monitor",
        4096,          // Stack size
        nullptr,       // Parameters
        7,             // Higher priority
        &monitor_task_handle,
        tskNO_AFFINITY // Any core
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return;
    }
    ESP_LOGI(TAG, "Monitor task created successfully");
    
    // Create periodic task with highest priority
    TaskHandle_t periodic_task_handle;
    result = xTaskCreatePinnedToCore(
        periodic_task,
        "periodic",
        3072,          // Stack size
        nullptr,       // Parameters
        8,             // Highest priority
        &periodic_task_handle,
        0              // Core 0
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create periodic task");
        return;
    }
    ESP_LOGI(TAG, "Periodic task created successfully");
    
    // Create priority test tasks
    static const char* task_names[] = {"HighPrio", "MedPrio", "LowPrio"};
    UBaseType_t priorities[] = {6, 4, 2};
    
    for (int i = 0; i < 3; i++) {
        TaskHandle_t priority_task_handle;
        result = xTaskCreatePinnedToCore(
            priority_test_task,
            task_names[i],
            2048,                    // Stack size
            (void*)task_names[i],    // Task name as parameter
            priorities[i],           // Priority
            &priority_task_handle,
            (i % 2)                  // Alternate between cores
        );
        
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create priority test task %s", task_names[i]);
        } else {
            ESP_LOGI(TAG, "Priority test task %s created successfully", task_names[i]);
        }
    }
}

/**
 * @brief Main application entry point
 */
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== FreeRTOS Multi-Task Test Application ===");
    ESP_LOGI(TAG, "ESP32-P4 Emulator - FreeRTOS Kernel Validation");
    ESP_LOGI(TAG, "System tick frequency: %d Hz", configTICK_RATE_HZ);
    ESP_LOGI(TAG, "Number of CPU cores: %d", portNUM_PROCESSORS);
    ESP_LOGI(TAG, "Max task priorities: %d", configMAX_PRIORITIES);
    
    // Print initial system information
    ESP_LOGI(TAG, "Free heap size: %d bytes", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Current tick count: %lu", xTaskGetTickCount());
    
    // Initialize test objects
    initialize_test_objects();
    
    // Create test tasks
    create_test_tasks();
    
    ESP_LOGI(TAG, "All test tasks created, FreeRTOS scheduler will manage execution");
    ESP_LOGI(TAG, "Test will run indefinitely, demonstrating multi-tasking functionality");
    
    // The main task can now exit, FreeRTOS scheduler will continue running other tasks
    ESP_LOGI(TAG, "Main task exiting, other tasks continue running");
    
    // Delete main task
    vTaskDelete(nullptr);
}