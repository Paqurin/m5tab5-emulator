#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/logging.hpp"

#include <memory>
#include <unordered_map>
#include <functional>
#include <vector>
#include <string>
#include <chrono>

namespace m5tab5::emulator {

// Forward declarations
class EmulatorCore;
class MemoryController;
namespace freertos { class FreeRTOSKernel; }

/**
 * @brief System Call Interface for RISC-V ESP32-P4 Emulation
 * 
 * Implements the bridge between RISC-V ECALL instructions and ESP-IDF APIs.
 * Provides complete ESP-IDF compatibility by translating system calls to
 * emulator component operations.
 * 
 * Features:
 * - RISC-V ECALL instruction handling
 * - ESP-IDF system call compatibility
 * - Memory management system calls
 * - Task management and synchronization
 * - File I/O and VFS operations
 * - Timer and interrupt management
 * - ESP-IDF specific operations
 */
class SystemCallInterface {
public:
    // System call context for parameter passing
    struct SystemCallContext {
        uint32_t syscall_number;        // From a7 register
        uint64_t args[7];              // From a0-a6 registers
        Address return_pc;             // PC after ECALL instruction
        uint32_t caller_privilege;     // Privilege level of caller
        uint32_t calling_core_id;      // Which core made the call
        
        SystemCallContext() : syscall_number(0), return_pc(0), caller_privilege(0), calling_core_id(0) {
            for (int i = 0; i < 7; i++) args[i] = 0;
        }
    };

    // System call handler function type
    using SystemCallHandler = std::function<int32_t(const SystemCallContext&)>;

    // System call descriptor
    struct SystemCall {
        uint32_t number;
        std::string name;
        SystemCallHandler handler;
        uint8_t arg_count;
        std::string description;
        bool requires_kernel_mode;
        
        SystemCall() : number(0), arg_count(0), requires_kernel_mode(false) {}
    };

    // System call error codes
    enum class SystemCallError : int32_t {
        SUCCESS = 0,
        INVALID_SYSCALL = -1,
        INVALID_ARGS = -2,
        PERMISSION_DENIED = -3,
        RESOURCE_EXHAUSTED = -4,
        TIMEOUT = -5,
        KERNEL_ERROR = -6,
        NOT_IMPLEMENTED = -7,
        MEMORY_ERROR = -8
    };

    explicit SystemCallInterface(EmulatorCore& core);
    ~SystemCallInterface();

    // Lifecycle management
    Result<void> initialize();
    void shutdown();
    bool is_initialized() const { return initialized_; }

    // Main system call interface
    inline int32_t handle_ecall(const SystemCallContext& context) {
        if (!initialized_) {
            LOG_ERROR("SystemCallInterface not initialized");
            return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Update statistics
        stats_.total_calls++;
        
        // Find the system call handler
        auto it = syscall_registry_.find(context.syscall_number);
        if (it == syscall_registry_.end()) {
            LOG_WARN("Unknown system call: {}", context.syscall_number);
            stats_.failed_calls++;
            return static_cast<int32_t>(SystemCallError::INVALID_SYSCALL);
        }
        
        const SystemCall& syscall = it->second;
        
        // Check privilege level if required
        if (syscall.requires_kernel_mode && context.caller_privilege != 0) {
            LOG_WARN("System call {} requires kernel mode, but called from privilege level {}", 
                        syscall.name, context.caller_privilege);
            stats_.failed_calls++;
            return static_cast<int32_t>(SystemCallError::PERMISSION_DENIED);
        }
        
        LOG_TRACE("Executing system call: {} ({})", syscall.name, context.syscall_number);
        
        // Execute the system call
        int32_t result = static_cast<int32_t>(SystemCallError::NOT_IMPLEMENTED);
        try {
            result = syscall.handler(context);
            stats_.successful_calls++;
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in system call {}: {}", syscall.name, e.what());
            result = static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
            stats_.failed_calls++;
        } catch (...) {
            LOG_ERROR("Unknown exception in system call {}", syscall.name);
            result = static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
            stats_.failed_calls++;
        }
        
        // Update execution time statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        update_statistics(context.syscall_number, result >= 0, duration.count());
        
        LOG_TRACE("System call {} completed with result: {}", syscall.name, result);
        
        return result;
    }
    
    // System call registration
    Result<void> register_syscall(uint32_t number, const std::string& name, 
                                 SystemCallHandler handler, uint8_t arg_count,
                                 const std::string& description = "",
                                 bool requires_kernel_mode = false);
    
    // System call information
    bool is_syscall_registered(uint32_t number) const;
    std::vector<uint32_t> get_registered_syscalls() const;
    std::string get_syscall_name(uint32_t number) const;
    std::string get_syscall_description(uint32_t number) const;

    // Statistics and monitoring
    struct Statistics {
        uint64_t total_calls;
        uint64_t successful_calls;
        uint64_t failed_calls;
        std::unordered_map<uint32_t, uint64_t> call_counts;
        uint64_t avg_execution_time_us;
    };
    
    Statistics get_statistics() const;
    void reset_statistics();

    // ESP-IDF system call numbers (matching real ESP-IDF)
    enum class ESPSystemCall : uint32_t {
        // Memory management (1-10)
        MALLOC = 1,
        FREE = 2,
        CALLOC = 3,
        REALLOC = 4,
        HEAP_CAPS_MALLOC = 5,
        HEAP_CAPS_FREE = 6,
        HEAP_CAPS_CALLOC = 7,
        HEAP_CAPS_REALLOC = 8,
        GET_FREE_HEAP_SIZE = 9,
        GET_MINIMUM_FREE_HEAP_SIZE = 10,
        
        // Task management (11-30)
        TASK_CREATE = 11,
        TASK_DELETE = 12,
        TASK_DELAY = 13,
        TASK_DELAY_UNTIL = 14,
        TASK_SUSPEND = 15,
        TASK_RESUME = 16,
        TASK_PRIORITY_GET = 17,
        TASK_PRIORITY_SET = 18,
        TASK_GET_CURRENT_HANDLE = 19,
        TASK_GET_NAME = 20,
        TASK_GET_STATE = 21,
        TASK_LIST = 22,
        YIELD = 23,
        
        // Synchronization (31-50)
        QUEUE_CREATE = 31,
        QUEUE_DELETE = 32,
        QUEUE_SEND = 33,
        QUEUE_SEND_FROM_ISR = 34,
        QUEUE_RECEIVE = 35,
        QUEUE_RECEIVE_FROM_ISR = 36,
        SEMAPHORE_CREATE_BINARY = 37,
        SEMAPHORE_CREATE_COUNTING = 38,
        SEMAPHORE_DELETE = 39,
        SEMAPHORE_TAKE = 40,
        SEMAPHORE_GIVE = 41,
        MUTEX_CREATE = 42,
        MUTEX_DELETE = 43,
        MUTEX_TAKE = 44,
        MUTEX_GIVE = 45,
        
        // File I/O and VFS (51-70)
        OPEN = 51,
        CLOSE = 52,
        READ = 53,
        WRITE = 54,
        LSEEK = 55,
        STAT = 56,
        FSTAT = 57,
        FSYNC = 58,
        MKDIR = 59,
        RMDIR = 60,
        UNLINK = 61,
        RENAME = 62,
        
        // Timer management (71-85)
        TIMER_CREATE = 71,
        TIMER_DELETE = 72,
        TIMER_START_ONCE = 73,
        TIMER_START_PERIODIC = 74,
        TIMER_STOP = 75,
        GET_TIME = 76,
        
        // ESP-IDF specific (86-120)
        ESP_LOG_WRITE = 86,
        ESP_RESTART = 87,
        ESP_GET_CHIP_INFO = 88,
        ESP_GET_IDF_VERSION = 89,
        ESP_GET_FREE_HEAP_SIZE = 90,
        ESP_RANDOM = 91,
        
        // GPIO operations (121-140)
        GPIO_SET_DIRECTION = 121,
        GPIO_GET_LEVEL = 122,
        GPIO_SET_LEVEL = 123,
        GPIO_CONFIG = 124,
        GPIO_INSTALL_ISR_SERVICE = 125,
        GPIO_ISR_HANDLER_ADD = 126,
        GPIO_ISR_HANDLER_REMOVE = 127,
        
        // I2C operations (141-155)
        I2C_DRIVER_INSTALL = 141,
        I2C_DRIVER_DELETE = 142,
        I2C_MASTER_WRITE_TO_DEVICE = 143,
        I2C_MASTER_READ_FROM_DEVICE = 144,
        I2C_MASTER_WRITE_READ_DEVICE = 145,
        
        // Debug and system (200+)
        SYSTEM_HALT = 200,
        SYSTEM_RESET = 201,
        GET_SYSTEM_INFO = 202
    };

private:
    // Component references
    EmulatorCore& emulator_core_;
    bool initialized_ = false;
    
    // System call registry
    std::unordered_map<uint32_t, SystemCall> syscall_registry_;
    
    // Statistics
    mutable Statistics stats_;
    
    // Initialization helpers
    Result<void> register_memory_syscalls();
    Result<void> register_task_syscalls();
    Result<void> register_sync_syscalls();
    Result<void> register_file_syscalls();
    Result<void> register_timer_syscalls();
    Result<void> register_esp_syscalls();
    Result<void> register_gpio_syscalls();
    Result<void> register_i2c_syscalls();
    
    // System call implementations
    // Memory management
    int32_t syscall_malloc(const SystemCallContext& ctx);
    int32_t syscall_free(const SystemCallContext& ctx);
    int32_t syscall_calloc(const SystemCallContext& ctx);
    int32_t syscall_realloc(const SystemCallContext& ctx);
    int32_t syscall_heap_caps_malloc(const SystemCallContext& ctx);
    int32_t syscall_heap_caps_free(const SystemCallContext& ctx);
    int32_t syscall_get_free_heap_size(const SystemCallContext& ctx);
    
    // Task management
    int32_t syscall_task_create(const SystemCallContext& ctx);
    int32_t syscall_task_delete(const SystemCallContext& ctx);
    int32_t syscall_task_delay(const SystemCallContext& ctx);
    int32_t syscall_task_suspend(const SystemCallContext& ctx);
    int32_t syscall_task_resume(const SystemCallContext& ctx);
    int32_t syscall_yield(const SystemCallContext& ctx);
    
    // Synchronization
    int32_t syscall_queue_create(const SystemCallContext& ctx);
    int32_t syscall_queue_delete(const SystemCallContext& ctx);
    int32_t syscall_queue_send(const SystemCallContext& ctx);
    int32_t syscall_queue_receive(const SystemCallContext& ctx);
    int32_t syscall_semaphore_create_binary(const SystemCallContext& ctx);
    int32_t syscall_semaphore_take(const SystemCallContext& ctx);
    int32_t syscall_semaphore_give(const SystemCallContext& ctx);
    
    // File I/O
    int32_t syscall_open(const SystemCallContext& ctx);
    int32_t syscall_close(const SystemCallContext& ctx);
    int32_t syscall_read(const SystemCallContext& ctx);
    int32_t syscall_write(const SystemCallContext& ctx);
    
    // ESP-IDF specific
    int32_t syscall_esp_log_write(const SystemCallContext& ctx);
    int32_t syscall_esp_restart(const SystemCallContext& ctx);
    int32_t syscall_esp_get_chip_info(const SystemCallContext& ctx);
    int32_t syscall_esp_random(const SystemCallContext& ctx);
    
    // GPIO operations
    int32_t syscall_gpio_set_direction(const SystemCallContext& ctx);
    int32_t syscall_gpio_get_level(const SystemCallContext& ctx);
    int32_t syscall_gpio_set_level(const SystemCallContext& ctx);
    
    // I2C operations
    int32_t syscall_i2c_master_write_to_device(const SystemCallContext& ctx);
    int32_t syscall_i2c_master_read_from_device(const SystemCallContext& ctx);
    
    // Utility functions
    int32_t map_esp_error_to_syscall_error(int esp_err);
    inline void update_statistics(uint32_t syscall_number, bool success, uint64_t execution_time_us) {
        if (success) {
            stats_.successful_calls++;
        } else {
            stats_.failed_calls++;
        }
        stats_.call_counts[syscall_number]++;
        
        // Update average execution time (simple moving average)
        const uint64_t total_calls = stats_.successful_calls + stats_.failed_calls;
        if (total_calls > 0) {
            stats_.avg_execution_time_us = 
                ((stats_.avg_execution_time_us * (total_calls - 1)) + execution_time_us) / total_calls;
        }
    }
    
    // Component access helpers
    std::shared_ptr<MemoryController> get_memory_controller();
    std::shared_ptr<freertos::FreeRTOSKernel> get_freertos_kernel();
};

} // namespace m5tab5::emulator