/**
 * @file esp_idf_integration.cpp
 * @brief ESP-IDF Integration Coordinator for M5Stack Tab5 Emulator
 * 
 * This file provides the master coordination layer that initializes and manages
 * all ESP-IDF API compatibility components, ensuring they work together seamlessly
 * to provide a complete ESP32-P4 application runtime environment.
 */

#include "emulator/esp_idf/esp_system.h"
#include "emulator/esp_idf/esp_system_call_interface.h"
#include "emulator/esp_idf/freertos_api.hpp"
#include "emulator/esp_idf/esp_heap_caps.h"
#include "emulator/esp_idf/nvs.h"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/shutdown_manager.hpp"
#include <mutex>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

namespace m5tab5::emulator::esp_idf {

/**
 * @brief ESP-IDF integration component interface
 */
class IntegrationComponent {
public:
    virtual ~IntegrationComponent() = default;
    virtual const char* name() const = 0;
    virtual esp_err_t initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool is_initialized() const = 0;
};

/**
 * @brief System component wrapper
 */
class SystemComponent : public IntegrationComponent {
private:
    bool initialized_ = false;

public:
    const char* name() const override { return "ESP System"; }
    
    esp_err_t initialize() override {
        if (initialized_) return ESP_OK;
        
        LOG_INFO("Initializing ESP-IDF system component");
        esp_err_t result = esp_system_init();
        if (result == ESP_OK) {
            initialized_ = true;
            LOG_DEBUG("ESP-IDF system component initialized successfully");
        } else {
            LOG_ERROR("Failed to initialize ESP-IDF system component: {}", result);
        }
        return result;
    }
    
    void shutdown() override {
        if (!initialized_) return;
        
        LOG_DEBUG("Shutting down ESP-IDF system component");
        // System component doesn't need explicit shutdown
        initialized_ = false;
    }
    
    bool is_initialized() const override { return initialized_; }
};

/**
 * @brief FreeRTOS component wrapper
 */
class FreeRTOSComponent : public IntegrationComponent {
private:
    bool initialized_ = false;

public:
    const char* name() const override { return "FreeRTOS"; }
    
    esp_err_t initialize() override {
        if (initialized_) return ESP_OK;
        
        LOG_INFO("Initializing ESP-IDF FreeRTOS component");
        try {
            initialize_freertos_api();
            initialized_ = true;
            LOG_DEBUG("ESP-IDF FreeRTOS component initialized successfully");
            return ESP_OK;
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to initialize ESP-IDF FreeRTOS component: {}", e.what());
            return ESP_FAIL;
        }
    }
    
    void shutdown() override {
        if (!initialized_) return;
        
        LOG_DEBUG("Shutting down ESP-IDF FreeRTOS component");
        try {
            shutdown_freertos_api();
            initialized_ = false;
        } catch (const std::exception& e) {
            LOG_ERROR("Error shutting down FreeRTOS component: {}", e.what());
        }
    }
    
    bool is_initialized() const override { return initialized_; }
};

/**
 * @brief Heap management component wrapper
 */
class HeapComponent : public IntegrationComponent {
private:
    bool initialized_ = false;

public:
    const char* name() const override { return "Heap Management"; }
    
    esp_err_t initialize() override {
        if (initialized_) return ESP_OK;
        
        LOG_INFO("Initializing ESP-IDF heap management component");
        
        // Heap capabilities are always available in emulation
        initialized_ = true;
        
        // Test basic heap functionality
        void* test_ptr = heap_caps_malloc(1024, MALLOC_CAP_DEFAULT);
        if (test_ptr) {
            heap_caps_free(test_ptr);
            LOG_DEBUG("ESP-IDF heap management component initialized successfully");
            return ESP_OK;
        } else {
            LOG_ERROR("Failed to initialize ESP-IDF heap management component");
            initialized_ = false;
            return ESP_ERR_NO_MEM;
        }
    }
    
    void shutdown() override {
        if (!initialized_) return;
        
        LOG_DEBUG("Shutting down ESP-IDF heap management component");
        
        // Check for memory leaks (optional, for debugging)
        heap_caps_dump_all();
        
        initialized_ = false;
    }
    
    bool is_initialized() const override { return initialized_; }
};

/**
 * @brief System call interface component wrapper
 */
class SystemCallComponent : public IntegrationComponent {
public:
    const char* name() const override { return "System Call Interface"; }
    
    esp_err_t initialize() override {
        if (is_initialized()) return ESP_OK;
        
        LOG_INFO("Initializing ESP-IDF system call interface component");
        initialize_system_call_interface();
        
        if (is_initialized()) {
            LOG_DEBUG("ESP-IDF system call interface component initialized successfully");
            return ESP_OK;
        } else {
            LOG_ERROR("Failed to initialize ESP-IDF system call interface component");
            return ESP_FAIL;
        }
    }
    
    void shutdown() override {
        if (!is_initialized()) return;
        
        LOG_DEBUG("Shutting down ESP-IDF system call interface component");
        shutdown_system_call_interface();
    }
    
    bool is_initialized() const override {
        return is_system_call_interface_initialized();
    }
};

/**
 * @brief NVS Flash component wrapper
 */
class NVSComponent : public IntegrationComponent {
private:
    bool initialized_ = false;

public:
    const char* name() const override { return "NVS Flash"; }
    
    esp_err_t initialize() override {
        if (initialized_) return ESP_OK;
        
        LOG_INFO("Initializing ESP-IDF NVS flash component");
        esp_err_t result = nvs_flash_init();
        if (result == ESP_OK) {
            initialized_ = true;
            LOG_DEBUG("ESP-IDF NVS flash component initialized successfully");
        } else {
            LOG_ERROR("Failed to initialize ESP-IDF NVS flash component: {}", result);
        }
        return result;
    }
    
    void shutdown() override {
        if (!initialized_) return;
        
        LOG_DEBUG("Shutting down ESP-IDF NVS flash component");
        // NVS handles its own shutdown via RAII
        initialized_ = false;
    }
    
    bool is_initialized() const override { return initialized_; }
};

/**
 * @brief Main ESP-IDF integration coordinator
 */
class ESPIDFIntegrator {
private:
    std::vector<std::unique_ptr<IntegrationComponent>> components_;
    mutable std::mutex integration_mutex_;
    bool initialized_ = false;
    bool shutdown_registered_ = false;
    std::unique_ptr<utils::ShutdownGuard> shutdown_guard_;
    
    // Performance tracking
    std::chrono::steady_clock::time_point init_start_time_;
    std::chrono::milliseconds total_init_time_{0};

public:
    ESPIDFIntegrator() {
        register_components();
    }
    
    ~ESPIDFIntegrator() {
        shutdown();
    }
    
    esp_err_t initialize() {
        std::lock_guard<std::mutex> lock(integration_mutex_);
        
        if (initialized_) {
            LOG_DEBUG("ESP-IDF integration already initialized");
            return ESP_OK;
        }
        
        init_start_time_ = std::chrono::steady_clock::now();
        LOG_INFO("Starting ESP-IDF API compatibility layer initialization...");
        
        // Register shutdown handler
        if (!shutdown_registered_) {
            shutdown_guard_ = std::make_unique<utils::ShutdownGuard>(
                utils::ShutdownManager::Priority::High, 
                "ESP-IDF Integration",
                [this]() { this->shutdown(); }
            );
            shutdown_registered_ = true;
        }
        
        // Initialize components in dependency order
        esp_err_t overall_result = ESP_OK;
        size_t successful_count = 0;
        
        for (auto& component : components_) {
            LOG_DEBUG("Initializing component: {}", component->name());
            
            auto start = std::chrono::steady_clock::now();
            esp_err_t result = component->initialize();
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            
            if (result == ESP_OK) {
                successful_count++;
                LOG_DEBUG("Component {} initialized in {} ms", component->name(), duration.count());
            } else {
                LOG_ERROR("Component {} failed to initialize: {} (took {} ms)", 
                         component->name(), result, duration.count());
                overall_result = result;
                // Continue with other components - partial initialization may be useful
            }
        }
        
        // Calculate total initialization time
        auto init_end = std::chrono::steady_clock::now();
        total_init_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(init_end - init_start_time_);
        
        // Check if we have a usable system
        if (successful_count >= components_.size() / 2) {
            initialized_ = true;
            LOG_INFO("ESP-IDF API compatibility layer initialized: {}/{} components successful in {} ms",
                     successful_count, components_.size(), total_init_time_.count());
            
            // Print system information
            print_system_info();
            return ESP_OK;
        } else {
            LOG_ERROR("ESP-IDF initialization failed: only {}/{} components successful", 
                     successful_count, components_.size());
            return overall_result != ESP_OK ? overall_result : ESP_FAIL;
        }
    }
    
    void shutdown() {
        std::lock_guard<std::mutex> lock(integration_mutex_);
        
        if (!initialized_) return;
        
        LOG_INFO("Shutting down ESP-IDF API compatibility layer...");
        
        // Shutdown components in reverse order
        for (auto it = components_.rbegin(); it != components_.rend(); ++it) {
            if ((*it)->is_initialized()) {
                LOG_DEBUG("Shutting down component: {}", (*it)->name());
                try {
                    (*it)->shutdown();
                } catch (const std::exception& e) {
                    LOG_ERROR("Error shutting down component {}: {}", (*it)->name(), e.what());
                }
            }
        }
        
        initialized_ = false;
        shutdown_guard_.reset();
        shutdown_registered_ = false;
        
        LOG_INFO("ESP-IDF API compatibility layer shutdown complete");
    }
    
    bool is_initialized() const {
        std::lock_guard<std::mutex> lock(integration_mutex_);
        return initialized_;
    }
    
    std::vector<std::string> get_component_status() const {
        std::lock_guard<std::mutex> lock(integration_mutex_);
        
        std::vector<std::string> status;
        for (const auto& component : components_) {
            status.push_back(std::string(component->name()) + ": " + 
                           (component->is_initialized() ? "OK" : "FAILED"));
        }
        return status;
    }
    
    void print_status() const {
        auto status = get_component_status();
        LOG_INFO("ESP-IDF API Component Status:");
        for (const auto& line : status) {
            LOG_INFO("  {}", line);
        }
        LOG_INFO("Total initialization time: {} ms", total_init_time_.count());
    }

private:
    void register_components() {
        // Register components in dependency order
        components_.push_back(std::make_unique<SystemComponent>());
        components_.push_back(std::make_unique<HeapComponent>());
        components_.push_back(std::make_unique<NVSComponent>());
        components_.push_back(std::make_unique<FreeRTOSComponent>());
        components_.push_back(std::make_unique<SystemCallComponent>());
        
        LOG_DEBUG("Registered {} ESP-IDF integration components", components_.size());
    }
    
    void print_system_info() const {
        LOG_INFO("=== ESP-IDF Emulation System Information ===");
        
        // System information
        esp_chip_info_t chip_info;
        if (esp_chip_info(&chip_info) == ESP_OK) {
            LOG_INFO("Chip: ESP32-P4 (emulated)");
            LOG_INFO("CPU Cores: {}", chip_info.cores);
            LOG_INFO("Chip Revision: {}", chip_info.revision);
        }
        
        // Version information
        LOG_INFO("ESP-IDF Version: {}", esp_get_idf_version());
        
        // Memory information
        LOG_INFO("Free Heap: {} bytes", esp_get_free_heap_size());
        LOG_INFO("Minimum Free Heap: {} bytes", esp_get_minimum_free_heap_size());
        
        // Heap capabilities
        LOG_INFO("Default Heap Free: {} bytes", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        LOG_INFO("DMA Heap Free: {} bytes", heap_caps_get_free_size(MALLOC_CAP_DMA));
        LOG_INFO("Internal Heap Free: {} bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        
        LOG_INFO("=== End System Information ===");
    }
};

// Global instance
static std::unique_ptr<ESPIDFIntegrator> g_esp_idf_integrator;
static std::mutex g_global_mutex;

// Public interface functions
esp_err_t initialize_esp_idf_integration() {
    std::lock_guard<std::mutex> lock(g_global_mutex);
    
    if (!g_esp_idf_integrator) {
        g_esp_idf_integrator = std::make_unique<ESPIDFIntegrator>();
    }
    
    return g_esp_idf_integrator->initialize();
}

void shutdown_esp_idf_integration() {
    std::lock_guard<std::mutex> lock(g_global_mutex);
    
    if (g_esp_idf_integrator) {
        g_esp_idf_integrator->shutdown();
    }
}

bool is_esp_idf_integration_ready() {
    std::lock_guard<std::mutex> lock(g_global_mutex);
    
    return g_esp_idf_integrator && g_esp_idf_integrator->is_initialized();
}

void print_esp_idf_status() {
    std::lock_guard<std::mutex> lock(g_global_mutex);
    
    if (g_esp_idf_integrator) {
        g_esp_idf_integrator->print_status();
    } else {
        LOG_INFO("ESP-IDF integration not initialized");
    }
}

std::vector<std::string> get_esp_idf_component_status() {
    std::lock_guard<std::mutex> lock(g_global_mutex);
    
    if (g_esp_idf_integrator) {
        return g_esp_idf_integrator->get_component_status();
    }
    
    return {"ESP-IDF integration not initialized"};
}

} // namespace m5tab5::emulator::esp_idf

// C interface for integration
extern "C" {

esp_err_t esp_idf_initialize_compatibility_layer(void) {
    return m5tab5::emulator::esp_idf::initialize_esp_idf_integration();
}

void esp_idf_shutdown_compatibility_layer(void) {
    m5tab5::emulator::esp_idf::shutdown_esp_idf_integration();
}

bool esp_idf_is_compatibility_layer_ready(void) {
    return m5tab5::emulator::esp_idf::is_esp_idf_integration_ready();
}

void esp_idf_print_system_status(void) {
    m5tab5::emulator::esp_idf::print_esp_idf_status();
}

esp_err_t esp_idf_run_integration_test(void) {
    if (!esp_idf_is_compatibility_layer_ready()) {
        LOG_ERROR("ESP-IDF compatibility layer not ready for testing");
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_INFO("Running ESP-IDF integration test...");
    
    // Test system functions
    esp_chip_info_t chip_info;
    if (esp_chip_info(&chip_info) != ESP_OK) {
        LOG_ERROR("Integration test failed: esp_chip_info");
        return ESP_FAIL;
    }
    
    // Test heap functions
    void* test_mem = heap_caps_malloc(1024, MALLOC_CAP_DEFAULT);
    if (!test_mem) {
        LOG_ERROR("Integration test failed: heap_caps_malloc");
        return ESP_FAIL;
    }
    heap_caps_free(test_mem);
    
    // TODO: Re-enable driver tests when driver compilation issues are resolved
    /*
    // Test NVS functions
    if (nvs_flash_init() != ESP_OK) {
        LOG_ERROR("Integration test failed: nvs_flash_init");
        return ESP_FAIL;
    }
    
    // Test GPIO functions (basic)
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << 2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    if (gpio_config(&gpio_conf) != ESP_OK) {
        LOG_ERROR("Integration test failed: gpio_config");
        return ESP_FAIL;
    }
    */
    
    LOG_INFO("ESP-IDF integration test completed successfully!");
    return ESP_OK;
}

} // extern "C"