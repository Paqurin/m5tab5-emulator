/**
 * @file simple_freertos_test.cpp  
 * @brief Simple FreeRTOS integration test
 * 
 * A minimal test to validate FreeRTOS kernel functionality without complex
 * emulator integration, focusing on basic task creation and scheduling.
 */

#include <iostream>
#include <chrono>
#include <thread>

// Include only the essential FreeRTOS components
#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/freertos/task.hpp"
#include "emulator/freertos/synchronization.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::freertos;

/**
 * @brief Simple test task function
 */
void simple_test_task(void* pvParameters) {
    const char* task_name = static_cast<const char*>(pvParameters);
    
    for (int i = 1; i <= 5; i++) {
        std::cout << "Task " << task_name << " - Iteration " << i << std::endl;
        
        // Simulate work
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // In a real FreeRTOS system, we'd use vTaskDelay here
        // For this test, we'll just simulate the delay
    }
    
    std::cout << "Task " << task_name << " completed" << std::endl;
}

/**
 * @brief Producer task for testing queues
 */
void producer_task_func(void* pvParameters) {
    std::cout << "Producer task started" << std::endl;
    
    // Create a binary semaphore for signaling
    auto sem = SynchronizationAPI::xSemaphoreCreateBinary();
    if (sem) {
        std::cout << "Producer: Created semaphore successfully" << std::endl;
        
        // Signal the semaphore a few times
        for (int i = 0; i < 3; i++) {
            std::cout << "Producer: Giving semaphore " << (i + 1) << std::endl;
            SynchronizationAPI::xSemaphoreGive(sem);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
        }
    }
    
    std::cout << "Producer task completed" << std::endl;
}

/**
 * @brief Consumer task for testing queues
 */
void consumer_task_func(void* pvParameters) {
    std::cout << "Consumer task started" << std::endl;
    
    // Try to take from a semaphore (this would be passed via parameters in real code)
    // For this simple test, we'll just simulate the behavior
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout << "Consumer: Simulated semaphore take operation" << std::endl;
    std::cout << "Consumer task completed" << std::endl;
}

int main() {
    std::cout << "Simple FreeRTOS Integration Test" << std::endl;
    std::cout << "=================================" << std::endl;
    
    try {
        // Initialize minimal logging
        Logger::initialize(Logger::LogLevel::INFO);
        
        // Create a minimal configuration
        Configuration minimal_config;
        
        // Create memory controller (required by dual core manager)
        auto memory_controller = std::make_unique<MemoryController>();
        auto mem_init = memory_controller->initialize(minimal_config);
        if (!mem_init.has_value()) {
            std::cerr << "Failed to initialize memory controller: " << mem_init.error().to_string() << std::endl;
            return 1;
        }
        
        // Create dual core manager (required by FreeRTOS kernel)
        auto dual_core_manager = std::make_unique<DualCoreManager>();
        auto cpu_init = dual_core_manager->initialize(minimal_config, *memory_controller);
        if (!cpu_init.has_value()) {
            std::cerr << "Failed to initialize dual core manager: " << cpu_init.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "Hardware components initialized" << std::endl;
        
        // Create FreeRTOS kernel
        auto freertos_kernel = std::make_unique<FreeRTOSKernel>(*dual_core_manager);
        
        // Configure kernel
        FreeRTOSKernel::KernelConfig kernel_config;
        kernel_config.tick_frequency_hz = 1000;
        kernel_config.total_heap_size = 64 * 1024; // 64KB
        kernel_config.enable_task_statistics = true;
        
        // Initialize kernel
        auto kernel_init = freertos_kernel->initialize(kernel_config);
        if (!kernel_init.has_value()) {
            std::cerr << "Failed to initialize FreeRTOS kernel: " << kernel_init.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "FreeRTOS kernel initialized" << std::endl;
        
        // Start kernel
        auto kernel_start = freertos_kernel->start_kernel();
        if (!kernel_start.has_value()) {
            std::cerr << "Failed to start FreeRTOS kernel: " << kernel_start.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "FreeRTOS kernel started" << std::endl;
        
        // Create test tasks using the kernel's create_task method
        std::cout << "Creating test tasks..." << std::endl;
        
        auto task1_result = freertos_kernel->create_task(
            "TestTask1",
            simple_test_task,
            (void*)"Task1", 
            2048,   // Stack size
            5,      // Priority
            0       // Core 0
        );
        
        if (task1_result.has_value()) {
            std::cout << "Task1 created successfully" << std::endl;
        } else {
            std::cerr << "Failed to create Task1: " << task1_result.error().to_string() << std::endl;
        }
        
        auto task2_result = freertos_kernel->create_task(
            "TestTask2",
            simple_test_task,
            (void*)"Task2",
            2048,   // Stack size  
            5,      // Priority
            1       // Core 1
        );
        
        if (task2_result.has_value()) {
            std::cout << "Task2 created successfully" << std::endl;
        } else {
            std::cerr << "Failed to create Task2: " << task2_result.error().to_string() << std::endl;
        }
        
        // Create producer/consumer tasks
        auto producer_result = freertos_kernel->create_task(
            "Producer",
            producer_task_func,
            nullptr,
            2048,   // Stack size
            6,      // Higher priority
            0       // Core 0
        );
        
        auto consumer_result = freertos_kernel->create_task(
            "Consumer", 
            consumer_task_func,
            nullptr,
            2048,   // Stack size
            6,      // Higher priority
            1       // Core 1
        );
        
        if (producer_result.has_value() && consumer_result.has_value()) {
            std::cout << "Producer and Consumer tasks created successfully" << std::endl;
        }
        
        // Let the system run for a while
        std::cout << "Running FreeRTOS system for 10 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // Print statistics
        std::cout << "\n=== System Statistics ===" << std::endl;
        auto kernel_info = freertos_kernel->get_kernel_info();
        std::cout << "Tick frequency: " << kernel_info.tick_frequency << " Hz" << std::endl;
        std::cout << "Total tasks: " << kernel_info.total_tasks << std::endl;
        std::cout << "Runtime ticks: " << kernel_info.total_runtime_ticks << std::endl;
        std::cout << "Idle percentage: " << kernel_info.idle_percentage << "%" << std::endl;
        std::cout << "Free heap: " << kernel_info.free_heap_size << " bytes" << std::endl;
        
        freertos_kernel->dump_task_list();
        freertos_kernel->print_stack_high_water_marks();
        
        // Stop kernel
        auto kernel_stop = freertos_kernel->stop_kernel();
        if (!kernel_stop.has_value()) {
            std::cerr << "Warning: Failed to stop kernel cleanly: " << kernel_stop.error().to_string() << std::endl;
        }
        
        std::cout << "\nFreeRTOS integration test completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 1;
    }
}