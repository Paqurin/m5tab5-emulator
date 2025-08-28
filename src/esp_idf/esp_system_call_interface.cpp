/**
 * @file esp_system_call_interface.cpp
 * @brief ESP-IDF System Call Interface for M5Stack Tab5 Emulator
 * 
 * This file implements the system call interface that routes ESP32-P4 ECALL instructions
 * from running applications to the appropriate ESP-IDF API implementations. This enables
 * real ESP-IDF applications to run seamlessly on the emulator.
 */

#include "emulator/esp_idf/esp_system.h"
#include "emulator/esp_idf/driver/gpio.h" 
#include "emulator/esp_idf/driver/i2c.h"
#include "emulator/esp_idf/driver/spi_master.h"
#include "emulator/esp_idf/esp_heap_caps.h"
#include "emulator/esp_idf/nvs.h"
#include "emulator/esp_idf/freertos_api.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include <unordered_map>
#include <mutex>
#include <functional>
#include <cstring>

namespace m5tab5::emulator::esp_idf {

// System call numbers (ESP-IDF specific)
enum class SystemCall : uint32_t {
    // System management
    SYS_RESTART = 0x1000,
    SYS_GET_TIME = 0x1001,
    SYS_GET_CHIP_INFO = 0x1002,
    SYS_GET_FREE_HEAP = 0x1003,
    SYS_RANDOM = 0x1004,
    
    // Memory management
    SYS_HEAP_MALLOC = 0x2000,
    SYS_HEAP_FREE = 0x2001,
    SYS_HEAP_REALLOC = 0x2002,
    SYS_HEAP_CALLOC = 0x2003,
    SYS_HEAP_GET_SIZE = 0x2004,
    SYS_HEAP_CHECK = 0x2005,
    
    // GPIO operations
    SYS_GPIO_CONFIG = 0x3000,
    SYS_GPIO_SET_LEVEL = 0x3001,
    SYS_GPIO_GET_LEVEL = 0x3002,
    SYS_GPIO_SET_DIRECTION = 0x3003,
    SYS_GPIO_SET_PULL = 0x3004,
    SYS_GPIO_INSTALL_ISR = 0x3005,
    
    // I2C operations
    SYS_I2C_DRIVER_INSTALL = 0x4000,
    SYS_I2C_PARAM_CONFIG = 0x4001,
    SYS_I2C_MASTER_WRITE = 0x4002,
    SYS_I2C_MASTER_READ = 0x4003,
    SYS_I2C_CMD_BEGIN = 0x4004,
    SYS_I2C_DRIVER_DELETE = 0x4005,
    
    // SPI operations
    SYS_SPI_BUS_INIT = 0x5000,
    SYS_SPI_BUS_FREE = 0x5001,
    SYS_SPI_ADD_DEVICE = 0x5002,
    SYS_SPI_REMOVE_DEVICE = 0x5003,
    SYS_SPI_TRANSMIT = 0x5004,
    
    // NVS operations
    SYS_NVS_FLASH_INIT = 0x6000,
    SYS_NVS_OPEN = 0x6001,
    SYS_NVS_CLOSE = 0x6002,
    SYS_NVS_SET_BLOB = 0x6003,
    SYS_NVS_GET_BLOB = 0x6004,
    SYS_NVS_COMMIT = 0x6005,
    SYS_NVS_ERASE_KEY = 0x6006,
    
    // FreeRTOS operations
    SYS_TASK_CREATE = 0x7000,
    SYS_TASK_DELETE = 0x7001,
    SYS_TASK_DELAY = 0x7002,
    SYS_TASK_YIELD = 0x7003,
    SYS_SEMAPHORE_CREATE = 0x7004,
    SYS_SEMAPHORE_TAKE = 0x7005,
    SYS_SEMAPHORE_GIVE = 0x7006,
    SYS_QUEUE_CREATE = 0x7007,
    SYS_QUEUE_SEND = 0x7008,
    SYS_QUEUE_RECEIVE = 0x7009,
    
    // System call bounds
    SYS_CALL_MAX = 0x7FFF
};

/**
 * @brief System call context structure passed from CPU
 */
struct SystemCallContext {
    uint32_t call_number;      // System call number
    uint32_t arg0, arg1, arg2, arg3, arg4, arg5;  // Arguments
    uint32_t return_value;     // Return value (set by handler)
    uint32_t error_code;       // Error code (set by handler)
};

/**
 * @brief System call handler function type
 */
using SystemCallHandler = std::function<esp_err_t(SystemCallContext*)>;

/**
 * @brief System call registry
 */
class SystemCallRegistry {
private:
    std::unordered_map<uint32_t, SystemCallHandler> handlers_;
    std::mutex registry_mutex_;
    bool initialized_ = false;

public:
    static SystemCallRegistry& instance() {
        static SystemCallRegistry registry;
        return registry;
    }
    
    void initialize() {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        if (initialized_) return;
        
        register_system_handlers();
        register_memory_handlers();
        register_gpio_handlers();
        register_i2c_handlers();
        register_spi_handlers();
        register_nvs_handlers();
        register_freertos_handlers();
        
        initialized_ = true;
        LOG_INFO("ESP-IDF system call registry initialized with {} handlers", handlers_.size());
    }
    
    esp_err_t handle_system_call(uint32_t call_number, SystemCallContext* context) {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        
        auto it = handlers_.find(call_number);
        if (it == handlers_.end()) {
            LOG_ERROR("Unknown system call: 0x{:x}", call_number);
            return ESP_ERR_NOT_SUPPORTED;
        }
        
        LOG_DEBUG("Handling system call: 0x{:x}", call_number);
        return it->second(context);
    }

private:
    void register_handler(SystemCall call, SystemCallHandler handler) {
        handlers_[static_cast<uint32_t>(call)] = std::move(handler);
    }
    
    // System management handlers
    void register_system_handlers() {
        register_handler(SystemCall::SYS_RESTART, [](SystemCallContext* ctx) -> esp_err_t {
            (void)ctx;
            esp_restart();
            return ESP_OK;  // Never reached
        });
        
        register_handler(SystemCall::SYS_GET_TIME, [](SystemCallContext* ctx) -> esp_err_t {
            uint64_t time_us = esp_get_time_us();
            ctx->return_value = static_cast<uint32_t>(time_us);
            ctx->arg0 = static_cast<uint32_t>(time_us >> 32);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_GET_CHIP_INFO, [](SystemCallContext* ctx) -> esp_err_t {
            esp_chip_info_t* chip_info = reinterpret_cast<esp_chip_info_t*>(ctx->arg0);
            return esp_chip_info(chip_info);
        });
        
        register_handler(SystemCall::SYS_GET_FREE_HEAP, [](SystemCallContext* ctx) -> esp_err_t {
            ctx->return_value = esp_get_free_heap_size();
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_RANDOM, [](SystemCallContext* ctx) -> esp_err_t {
            ctx->return_value = esp_random();
            return ESP_OK;
        });
    }
    
    // Memory management handlers
    void register_memory_handlers() {
        register_handler(SystemCall::SYS_HEAP_MALLOC, [](SystemCallContext* ctx) -> esp_err_t {
            size_t size = ctx->arg0;
            uint32_t caps = ctx->arg1;
            void* ptr = heap_caps_malloc(size, caps);
            ctx->return_value = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ptr));
            return ptr ? ESP_OK : ESP_ERR_NO_MEM;
        });
        
        register_handler(SystemCall::SYS_HEAP_FREE, [](SystemCallContext* ctx) -> esp_err_t {
            void* ptr = reinterpret_cast<void*>(ctx->arg0);
            heap_caps_free(ptr);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_HEAP_REALLOC, [](SystemCallContext* ctx) -> esp_err_t {
            void* ptr = reinterpret_cast<void*>(ctx->arg0);
            size_t size = ctx->arg1;
            uint32_t caps = ctx->arg2;
            void* new_ptr = heap_caps_realloc(ptr, size, caps);
            ctx->return_value = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(new_ptr));
            return new_ptr ? ESP_OK : ESP_ERR_NO_MEM;
        });
        
        register_handler(SystemCall::SYS_HEAP_CALLOC, [](SystemCallContext* ctx) -> esp_err_t {
            size_t n = ctx->arg0;
            size_t size = ctx->arg1;
            uint32_t caps = ctx->arg2;
            void* ptr = heap_caps_calloc(n, size, caps);
            ctx->return_value = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ptr));
            return ptr ? ESP_OK : ESP_ERR_NO_MEM;
        });
        
        register_handler(SystemCall::SYS_HEAP_GET_SIZE, [](SystemCallContext* ctx) -> esp_err_t {
            uint32_t caps = ctx->arg0;
            ctx->return_value = heap_caps_get_free_size(caps);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_HEAP_CHECK, [](SystemCallContext* ctx) -> esp_err_t {
            uint32_t caps = ctx->arg0;
            bool print_errors = ctx->arg1 != 0;
            bool result = heap_caps_check_integrity(caps, print_errors);
            ctx->return_value = result ? 1 : 0;
            return ESP_OK;
        });
    }
    
    // GPIO handlers
    void register_gpio_handlers() {
        register_handler(SystemCall::SYS_GPIO_CONFIG, [](SystemCallContext* ctx) -> esp_err_t {
            const gpio_config_t* config = reinterpret_cast<const gpio_config_t*>(ctx->arg0);
            int result = gpio_config(config);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_GPIO_SET_LEVEL, [](SystemCallContext* ctx) -> esp_err_t {
            gpio_num_t gpio_num = static_cast<gpio_num_t>(ctx->arg0);
            uint32_t level = ctx->arg1;
            int result = gpio_set_level(gpio_num, level);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_GPIO_GET_LEVEL, [](SystemCallContext* ctx) -> esp_err_t {
            gpio_num_t gpio_num = static_cast<gpio_num_t>(ctx->arg0);
            int result = gpio_get_level(gpio_num);
            ctx->return_value = result;
            return result >= 0 ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_GPIO_SET_DIRECTION, [](SystemCallContext* ctx) -> esp_err_t {
            gpio_num_t gpio_num = static_cast<gpio_num_t>(ctx->arg0);
            gpio_mode_t mode = static_cast<gpio_mode_t>(ctx->arg1);
            int result = gpio_set_direction(gpio_num, mode);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_GPIO_SET_PULL, [](SystemCallContext* ctx) -> esp_err_t {
            gpio_num_t gpio_num = static_cast<gpio_num_t>(ctx->arg0);
            gpio_pull_mode_t pull = static_cast<gpio_pull_mode_t>(ctx->arg1);
            int result = gpio_set_pull_mode(gpio_num, pull);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_GPIO_INSTALL_ISR, [](SystemCallContext* ctx) -> esp_err_t {
            int intr_alloc_flags = static_cast<int>(ctx->arg0);
            int result = gpio_install_isr_service(intr_alloc_flags);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
    }
    
    // I2C handlers
    void register_i2c_handlers() {
        register_handler(SystemCall::SYS_I2C_DRIVER_INSTALL, [](SystemCallContext* ctx) -> esp_err_t {
            i2c_port_t i2c_num = static_cast<i2c_port_t>(ctx->arg0);
            i2c_mode_t mode = static_cast<i2c_mode_t>(ctx->arg1);
            size_t slv_rx_buf_len = ctx->arg2;
            size_t slv_tx_buf_len = ctx->arg3;
            int intr_alloc_flags = static_cast<int>(ctx->arg4);
            int result = i2c_driver_install(i2c_num, mode, slv_rx_buf_len, slv_tx_buf_len, intr_alloc_flags);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_I2C_PARAM_CONFIG, [](SystemCallContext* ctx) -> esp_err_t {
            i2c_port_t i2c_num = static_cast<i2c_port_t>(ctx->arg0);
            const i2c_config_t* config = reinterpret_cast<const i2c_config_t*>(ctx->arg1);
            int result = i2c_param_config(i2c_num, config);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_I2C_CMD_BEGIN, [](SystemCallContext* ctx) -> esp_err_t {
            i2c_port_t i2c_num = static_cast<i2c_port_t>(ctx->arg0);
            i2c_cmd_handle_t cmd_handle = reinterpret_cast<i2c_cmd_handle_t>(ctx->arg1);
            uint32_t ticks_to_wait = ctx->arg2;
            int result = i2c_master_cmd_begin(i2c_num, cmd_handle, ticks_to_wait);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_I2C_DRIVER_DELETE, [](SystemCallContext* ctx) -> esp_err_t {
            i2c_port_t i2c_num = static_cast<i2c_port_t>(ctx->arg0);
            int result = i2c_driver_delete(i2c_num);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
    }
    
    // SPI handlers
    void register_spi_handlers() {
        register_handler(SystemCall::SYS_SPI_BUS_INIT, [](SystemCallContext* ctx) -> esp_err_t {
            spi_host_device_t host_id = static_cast<spi_host_device_t>(ctx->arg0);
            const spi_bus_config_t* bus_config = reinterpret_cast<const spi_bus_config_t*>(ctx->arg1);
            int dma_chan = static_cast<int>(ctx->arg2);
            int result = spi_bus_initialize(host_id, bus_config, dma_chan);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_SPI_BUS_FREE, [](SystemCallContext* ctx) -> esp_err_t {
            spi_host_device_t host_id = static_cast<spi_host_device_t>(ctx->arg0);
            int result = spi_bus_free(host_id);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_SPI_ADD_DEVICE, [](SystemCallContext* ctx) -> esp_err_t {
            spi_host_device_t host_id = static_cast<spi_host_device_t>(ctx->arg0);
            const spi_device_interface_config_t* dev_config = reinterpret_cast<const spi_device_interface_config_t*>(ctx->arg1);
            spi_device_handle_t* handle = reinterpret_cast<spi_device_handle_t*>(ctx->arg2);
            int result = spi_bus_add_device(host_id, dev_config, handle);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_SPI_TRANSMIT, [](SystemCallContext* ctx) -> esp_err_t {
            spi_device_handle_t handle = reinterpret_cast<spi_device_handle_t>(ctx->arg0);
            spi_transaction_t* trans_desc = reinterpret_cast<spi_transaction_t*>(ctx->arg1);
            int result = spi_device_transmit(handle, trans_desc);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
    }
    
    // NVS handlers
    void register_nvs_handlers() {
        register_handler(SystemCall::SYS_NVS_FLASH_INIT, [](SystemCallContext* ctx) -> esp_err_t {
            (void)ctx;
            int result = nvs_flash_init();
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_NVS_OPEN, [](SystemCallContext* ctx) -> esp_err_t {
            const char* namespace_name = reinterpret_cast<const char*>(ctx->arg0);
            nvs_open_mode_t open_mode = static_cast<nvs_open_mode_t>(ctx->arg1);
            nvs_handle_t* out_handle = reinterpret_cast<nvs_handle_t*>(ctx->arg2);
            int result = nvs_open(namespace_name, open_mode, out_handle);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_NVS_CLOSE, [](SystemCallContext* ctx) -> esp_err_t {
            nvs_handle_t handle = static_cast<nvs_handle_t>(ctx->arg0);
            nvs_close(handle);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_NVS_SET_BLOB, [](SystemCallContext* ctx) -> esp_err_t {
            nvs_handle_t handle = static_cast<nvs_handle_t>(ctx->arg0);
            const char* key = reinterpret_cast<const char*>(ctx->arg1);
            const void* value = reinterpret_cast<const void*>(ctx->arg2);
            size_t length = ctx->arg3;
            int result = nvs_set_blob(handle, key, value, length);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_NVS_GET_BLOB, [](SystemCallContext* ctx) -> esp_err_t {
            nvs_handle_t handle = static_cast<nvs_handle_t>(ctx->arg0);
            const char* key = reinterpret_cast<const char*>(ctx->arg1);
            void* out_value = reinterpret_cast<void*>(ctx->arg2);
            size_t* length = reinterpret_cast<size_t*>(ctx->arg3);
            int result = nvs_get_blob(handle, key, out_value, length);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_NVS_COMMIT, [](SystemCallContext* ctx) -> esp_err_t {
            nvs_handle_t handle = static_cast<nvs_handle_t>(ctx->arg0);
            int result = nvs_commit(handle);
            ctx->return_value = result;
            return result == ESP_OK ? ESP_OK : ESP_FAIL;
        });
    }
    
    // FreeRTOS handlers
    void register_freertos_handlers() {
        register_handler(SystemCall::SYS_TASK_CREATE, [](SystemCallContext* ctx) -> esp_err_t {
            TaskFunction_t task_func = reinterpret_cast<TaskFunction_t>(ctx->arg0);
            const char* name = reinterpret_cast<const char*>(ctx->arg1);
            uint16_t stack_depth = static_cast<uint16_t>(ctx->arg2);
            void* parameters = reinterpret_cast<void*>(ctx->arg3);
            UBaseType_t priority = static_cast<UBaseType_t>(ctx->arg4);
            TaskHandle_t* handle = reinterpret_cast<TaskHandle_t*>(ctx->arg5);
            BaseType_t result = xTaskCreate(task_func, name, stack_depth, parameters, priority, handle);
            ctx->return_value = result;
            return result == pdPASS ? ESP_OK : ESP_FAIL;
        });
        
        register_handler(SystemCall::SYS_TASK_DELETE, [](SystemCallContext* ctx) -> esp_err_t {
            TaskHandle_t task = reinterpret_cast<TaskHandle_t>(ctx->arg0);
            vTaskDelete(task);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_TASK_DELAY, [](SystemCallContext* ctx) -> esp_err_t {
            TickType_t ticks = static_cast<TickType_t>(ctx->arg0);
            vTaskDelay(ticks);
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_TASK_YIELD, [](SystemCallContext* ctx) -> esp_err_t {
            (void)ctx;
            taskYIELD();
            return ESP_OK;
        });
        
        register_handler(SystemCall::SYS_SEMAPHORE_CREATE, [](SystemCallContext* ctx) -> esp_err_t {
            uint32_t type = ctx->arg0;  // 0=binary, 1=counting, 2=mutex
            SemaphoreHandle_t handle = nullptr;
            
            switch (type) {
                case 0: handle = xSemaphoreCreateBinary(); break;
                case 1: {
                    UBaseType_t max_count = static_cast<UBaseType_t>(ctx->arg1);
                    UBaseType_t initial_count = static_cast<UBaseType_t>(ctx->arg2);
                    handle = xSemaphoreCreateCounting(max_count, initial_count);
                    break;
                }
                case 2: handle = xSemaphoreCreateMutex(); break;
            }
            
            ctx->return_value = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(handle));
            return handle ? ESP_OK : ESP_ERR_NO_MEM;
        });
        
        register_handler(SystemCall::SYS_SEMAPHORE_TAKE, [](SystemCallContext* ctx) -> esp_err_t {
            SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(ctx->arg0);
            TickType_t ticks_to_wait = static_cast<TickType_t>(ctx->arg1);
            BaseType_t result = xSemaphoreTake(handle, ticks_to_wait);
            ctx->return_value = result;
            return result == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
        });
        
        register_handler(SystemCall::SYS_SEMAPHORE_GIVE, [](SystemCallContext* ctx) -> esp_err_t {
            SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(ctx->arg0);
            BaseType_t result = xSemaphoreGive(handle);
            ctx->return_value = result;
            return result == pdTRUE ? ESP_OK : ESP_FAIL;
        });
    }
};

/**
 * @brief RISC-V ECALL handler integration
 */
class ECCallHandler {
private:
    DualCoreManager* cpu_manager_ = nullptr;
    bool integrated_ = false;

public:
    void integrate_with_cpu(DualCoreManager* cpu_manager) {
        if (!cpu_manager) {
            LOG_ERROR("ECCallHandler: null CPU manager");
            return;
        }
        
        cpu_manager_ = cpu_manager;
        
        // Register ECALL handler with CPU
        auto ecall_callback = [this](uint32_t core_id, uint32_t call_number, 
                                    uint32_t* registers) -> bool {
            return this->handle_ecall(core_id, call_number, registers);
        };
        
        // TODO: Register callback with CPU manager when CPU interface is available
        // cpu_manager_->register_ecall_handler(ecall_callback);
        
        integrated_ = true;
        LOG_INFO("ECALL handler integrated with dual-core CPU manager");
    }
    
private:
    bool handle_ecall(uint32_t core_id, uint32_t call_number, uint32_t* registers) {
        LOG_DEBUG("ECALL handler: core={}, call=0x{:x}", core_id, call_number);
        
        // Build system call context from CPU registers
        SystemCallContext context;
        context.call_number = call_number;
        context.arg0 = registers[10];  // a0
        context.arg1 = registers[11];  // a1
        context.arg2 = registers[12];  // a2
        context.arg3 = registers[13];  // a3
        context.arg4 = registers[14];  // a4
        context.arg5 = registers[15];  // a5
        context.return_value = 0;
        context.error_code = 0;
        
        // Handle system call
        esp_err_t result = SystemCallRegistry::instance().handle_system_call(call_number, &context);
        
        // Write results back to CPU registers
        registers[10] = context.return_value;  // a0 = return value
        registers[11] = context.error_code;    // a1 = error code
        
        return result == ESP_OK;
    }
};

// Global instances
static std::unique_ptr<ECCallHandler> g_ecall_handler;
static bool g_system_call_interface_initialized = false;

// Public initialization functions
void initialize_system_call_interface() {
    if (g_system_call_interface_initialized) {
        LOG_WARN("System call interface already initialized");
        return;
    }
    
    // Initialize system call registry
    SystemCallRegistry::instance().initialize();
    
    // Create ECALL handler
    g_ecall_handler = std::make_unique<ECCallHandler>();
    
    // TODO: Get CPU manager from emulator core and integrate
    // This would be done when the emulator is fully initialized
    // auto emulator = EmulatorCore::get_instance();
    // if (emulator) {
    //     auto cpu_manager = emulator->getComponent<DualCoreManager>();
    //     if (cpu_manager) {
    //         g_ecall_handler->integrate_with_cpu(cpu_manager.get());
    //     }
    // }
    
    g_system_call_interface_initialized = true;
    LOG_INFO("ESP-IDF system call interface initialized successfully");
}

void shutdown_system_call_interface() {
    if (!g_system_call_interface_initialized) {
        return;
    }
    
    g_ecall_handler.reset();
    g_system_call_interface_initialized = false;
    
    LOG_INFO("ESP-IDF system call interface shut down");
}

bool is_system_call_interface_initialized() {
    return g_system_call_interface_initialized;
}

// Direct system call interface for testing/debugging
esp_err_t execute_system_call(uint32_t call_number, uint32_t arg0, uint32_t arg1, 
                              uint32_t arg2, uint32_t arg3, uint32_t arg4, uint32_t arg5,
                              uint32_t* return_value, uint32_t* error_code) {
    if (!g_system_call_interface_initialized) {
        LOG_ERROR("System call interface not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    SystemCallContext context;
    context.call_number = call_number;
    context.arg0 = arg0;
    context.arg1 = arg1;
    context.arg2 = arg2;
    context.arg3 = arg3;
    context.arg4 = arg4;
    context.arg5 = arg5;
    context.return_value = 0;
    context.error_code = 0;
    
    esp_err_t result = SystemCallRegistry::instance().handle_system_call(call_number, &context);
    
    if (return_value) *return_value = context.return_value;
    if (error_code) *error_code = context.error_code;
    
    return result;
}

} // namespace m5tab5::emulator::esp_idf

// C interface for system call interface
extern "C" {

esp_err_t esp_idf_init_system_call_interface(void) {
    m5tab5::emulator::esp_idf::initialize_system_call_interface();
    return ESP_OK;
}

void esp_idf_shutdown_system_call_interface(void) {
    m5tab5::emulator::esp_idf::shutdown_system_call_interface();
}

bool esp_idf_is_system_call_interface_ready(void) {
    return m5tab5::emulator::esp_idf::is_system_call_interface_initialized();
}

}