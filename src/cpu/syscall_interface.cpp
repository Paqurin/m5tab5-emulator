#include "emulator/cpu/syscall_interface.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/utils/logging.hpp"

#include <chrono>
#include <sstream>

namespace m5tab5::emulator {

SystemCallInterface::SystemCallInterface(EmulatorCore& core)
    : emulator_core_(core) {
    LOG_DEBUG("SystemCallInterface created for EmulatorCore");
}

SystemCallInterface::~SystemCallInterface() {
    if (initialized_) {
        shutdown();
    }
    LOG_DEBUG("SystemCallInterface destroyed");
}

Result<void> SystemCallInterface::initialize() {
    if (initialized_) {
        LOG_WARN("SystemCallInterface already initialized");
        return {};
    }
    
    LOG_INFO("Initializing RISC-V System Call Interface");
    
    // Clear statistics
    stats_ = Statistics{};
    
    // Register all ESP-IDF compatible system calls
    auto result = register_memory_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register memory system calls");
        return result;
    }
    
    result = register_task_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register task system calls");
        return result;
    }
    
    result = register_sync_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register synchronization system calls");
        return result;
    }
    
    result = register_file_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register file I/O system calls: {}", result.error().to_string());
        return result;
    }
    
    result = register_timer_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register timer system calls: {}", result.error().to_string());
        return result;
    }
    
    result = register_esp_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register ESP-IDF system calls: {}", result.error().to_string());
        return result;
    }
    
    result = register_gpio_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register GPIO system calls: {}", result.error().to_string());
        return result;
    }
    
    result = register_i2c_syscalls();
    if (!result.has_value()) {
        LOG_ERROR("Failed to register I2C system calls: {}", result.error().to_string());
        return result;
    }
    
    initialized_ = true;
    LOG_INFO("System Call Interface initialized with {} system calls", syscall_registry_.size());
    
    return {};
}

void SystemCallInterface::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down System Call Interface");
    LOG_INFO("System call statistics - Total: {}, Success: {}, Failed: {}", 
             stats_.total_calls, stats_.successful_calls, stats_.failed_calls);
    
    syscall_registry_.clear();
    initialized_ = false;
}


Result<void> SystemCallInterface::register_syscall(uint32_t number, const std::string& name,
                                                  SystemCallHandler handler, uint8_t arg_count,
                                                  const std::string& description,
                                                  bool requires_kernel_mode) {
    if (syscall_registry_.find(number) != syscall_registry_.end()) {
        LOG_WARN("System call {} already registered ({})", number, name);
        return unexpected(Error(ErrorCode::SYSTEM_BUSY, "System call already registered"));
    }
    
    SystemCall syscall;
    syscall.number = number;
    syscall.name = name;
    syscall.handler = std::move(handler);
    syscall.arg_count = arg_count;
    syscall.description = description;
    syscall.requires_kernel_mode = requires_kernel_mode;
    
    syscall_registry_[number] = std::move(syscall);
    
    LOG_DEBUG("Registered system call: {} ({}) - {}", name, number, description);
    return {};
}

bool SystemCallInterface::is_syscall_registered(uint32_t number) const {
    return syscall_registry_.find(number) != syscall_registry_.end();
}

std::vector<uint32_t> SystemCallInterface::get_registered_syscalls() const {
    std::vector<uint32_t> numbers;
    numbers.reserve(syscall_registry_.size());
    
    for (const auto& [number, syscall] : syscall_registry_) {
        numbers.push_back(number);
    }
    
    std::sort(numbers.begin(), numbers.end());
    return numbers;
}

std::string SystemCallInterface::get_syscall_name(uint32_t number) const {
    auto it = syscall_registry_.find(number);
    if (it != syscall_registry_.end()) {
        return it->second.name;
    }
    return "UNKNOWN";
}

std::string SystemCallInterface::get_syscall_description(uint32_t number) const {
    auto it = syscall_registry_.find(number);
    if (it != syscall_registry_.end()) {
        return it->second.description;
    }
    return "Unknown system call";
}

SystemCallInterface::Statistics SystemCallInterface::get_statistics() const {
    return stats_;
}

void SystemCallInterface::reset_statistics() {
    stats_ = Statistics{};
    LOG_DEBUG("System call statistics reset");
}

// Helper functions for component access
std::shared_ptr<MemoryController> SystemCallInterface::get_memory_controller() {
    auto result = emulator_core_.getComponent<MemoryController>();
    if (!result) {
        LOG_ERROR("Failed to get MemoryController component");
        return nullptr;
    }
    return result;
}

std::shared_ptr<freertos::FreeRTOSKernel> SystemCallInterface::get_freertos_kernel() {
    auto result = emulator_core_.getComponent<freertos::FreeRTOSKernel>();
    if (!result) {
        LOG_ERROR("Failed to get FreeRTOSKernel component");
        return nullptr;
    }
    return result;
}

// System call registration methods
Result<void> SystemCallInterface::register_memory_syscalls() {
    LOG_DEBUG("Registering memory management system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::MALLOC), "malloc",
                                  [this](const SystemCallContext& ctx) { return syscall_malloc(ctx); },
                                  1, "Allocate memory");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::FREE), "free",
                             [this](const SystemCallContext& ctx) { return syscall_free(ctx); },
                             1, "Free memory");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::CALLOC), "calloc",
                             [this](const SystemCallContext& ctx) { return syscall_calloc(ctx); },
                             2, "Allocate and clear memory");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::REALLOC), "realloc",
                             [this](const SystemCallContext& ctx) { return syscall_realloc(ctx); },
                             2, "Reallocate memory");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::HEAP_CAPS_MALLOC), "heap_caps_malloc",
                             [this](const SystemCallContext& ctx) { return syscall_heap_caps_malloc(ctx); },
                             2, "Allocate memory with capabilities");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::HEAP_CAPS_FREE), "heap_caps_free",
                             [this](const SystemCallContext& ctx) { return syscall_heap_caps_free(ctx); },
                             1, "Free memory allocated with capabilities");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::GET_FREE_HEAP_SIZE), "get_free_heap_size",
                             [this](const SystemCallContext& ctx) { return syscall_get_free_heap_size(ctx); },
                             0, "Get available heap memory");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("Memory management system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_task_syscalls() {
    LOG_DEBUG("Registering task management system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::TASK_CREATE), "xTaskCreate",
                                  [this](const SystemCallContext& ctx) { return syscall_task_create(ctx); },
                                  6, "Create a new task");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::TASK_DELETE), "vTaskDelete",
                             [this](const SystemCallContext& ctx) { return syscall_task_delete(ctx); },
                             1, "Delete a task");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::TASK_DELAY), "vTaskDelay",
                             [this](const SystemCallContext& ctx) { return syscall_task_delay(ctx); },
                             1, "Delay task execution");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::TASK_SUSPEND), "vTaskSuspend",
                             [this](const SystemCallContext& ctx) { return syscall_task_suspend(ctx); },
                             1, "Suspend a task");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::TASK_RESUME), "vTaskResume",
                             [this](const SystemCallContext& ctx) { return syscall_task_resume(ctx); },
                             1, "Resume a suspended task");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::YIELD), "taskYIELD",
                             [this](const SystemCallContext& ctx) { return syscall_yield(ctx); },
                             0, "Yield CPU to other tasks");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("Task management system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_sync_syscalls() {
    LOG_DEBUG("Registering synchronization system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::QUEUE_CREATE), "xQueueCreate",
                                  [this](const SystemCallContext& ctx) { return syscall_queue_create(ctx); },
                                  2, "Create a queue");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::QUEUE_DELETE), "vQueueDelete",
                             [this](const SystemCallContext& ctx) { return syscall_queue_delete(ctx); },
                             1, "Delete a queue");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::QUEUE_SEND), "xQueueSend",
                             [this](const SystemCallContext& ctx) { return syscall_queue_send(ctx); },
                             3, "Send data to queue");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::QUEUE_RECEIVE), "xQueueReceive",
                             [this](const SystemCallContext& ctx) { return syscall_queue_receive(ctx); },
                             3, "Receive data from queue");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::SEMAPHORE_CREATE_BINARY), "xSemaphoreCreateBinary",
                             [this](const SystemCallContext& ctx) { return syscall_semaphore_create_binary(ctx); },
                             0, "Create binary semaphore");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::SEMAPHORE_TAKE), "xSemaphoreTake",
                             [this](const SystemCallContext& ctx) { return syscall_semaphore_take(ctx); },
                             2, "Take semaphore");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::SEMAPHORE_GIVE), "xSemaphoreGive",
                             [this](const SystemCallContext& ctx) { return syscall_semaphore_give(ctx); },
                             1, "Give semaphore");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("Synchronization system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_file_syscalls() {
    LOG_DEBUG("Registering file I/O system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::OPEN), "open",
                                  [this](const SystemCallContext& ctx) { return syscall_open(ctx); },
                                  2, "Open file");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::CLOSE), "close",
                             [this](const SystemCallContext& ctx) { return syscall_close(ctx); },
                             1, "Close file");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::READ), "read",
                             [this](const SystemCallContext& ctx) { return syscall_read(ctx); },
                             3, "Read from file");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::WRITE), "write",
                             [this](const SystemCallContext& ctx) { return syscall_write(ctx); },
                             3, "Write to file");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("File I/O system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_timer_syscalls() {
    LOG_DEBUG("Registering timer system calls");
    
    // Timer system calls will be implemented when needed
    LOG_DEBUG("Timer system calls registered successfully (placeholder)");
    return {};
}

Result<void> SystemCallInterface::register_esp_syscalls() {
    LOG_DEBUG("Registering ESP-IDF specific system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::ESP_LOG_WRITE), "esp_log_write",
                                  [this](const SystemCallContext& ctx) { return syscall_esp_log_write(ctx); },
                                  4, "Write log message");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::ESP_RESTART), "esp_restart",
                             [this](const SystemCallContext& ctx) { return syscall_esp_restart(ctx); },
                             0, "Restart ESP32");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::ESP_GET_CHIP_INFO), "esp_get_chip_info",
                             [this](const SystemCallContext& ctx) { return syscall_esp_get_chip_info(ctx); },
                             1, "Get chip information");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::ESP_RANDOM), "esp_random",
                             [this](const SystemCallContext& ctx) { return syscall_esp_random(ctx); },
                             0, "Generate random number");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("ESP-IDF specific system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_gpio_syscalls() {
    LOG_DEBUG("Registering GPIO system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::GPIO_SET_DIRECTION), "gpio_set_direction",
                                  [this](const SystemCallContext& ctx) { return syscall_gpio_set_direction(ctx); },
                                  2, "Set GPIO direction");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::GPIO_GET_LEVEL), "gpio_get_level",
                             [this](const SystemCallContext& ctx) { return syscall_gpio_get_level(ctx); },
                             1, "Get GPIO level");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::GPIO_SET_LEVEL), "gpio_set_level",
                             [this](const SystemCallContext& ctx) { return syscall_gpio_set_level(ctx); },
                             2, "Set GPIO level");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("GPIO system calls registered successfully");
    return {};
}

Result<void> SystemCallInterface::register_i2c_syscalls() {
    LOG_DEBUG("Registering I2C system calls");
    
    auto result = register_syscall(static_cast<uint32_t>(ESPSystemCall::I2C_MASTER_WRITE_TO_DEVICE), "i2c_master_write_to_device",
                                  [this](const SystemCallContext& ctx) { return syscall_i2c_master_write_to_device(ctx); },
                                  5, "Write data to I2C device");
    if (!result.has_value()) return result;
    
    result = register_syscall(static_cast<uint32_t>(ESPSystemCall::I2C_MASTER_READ_FROM_DEVICE), "i2c_master_read_from_device",
                             [this](const SystemCallContext& ctx) { return syscall_i2c_master_read_from_device(ctx); },
                             5, "Read data from I2C device");
    if (!result.has_value()) return result;
    
    LOG_DEBUG("I2C system calls registered successfully");
    return {};
}

// System call implementations - Memory management
int32_t SystemCallInterface::syscall_malloc(const SystemCallContext& ctx) {
    size_t size = static_cast<size_t>(ctx.args[0]);
    
    auto memory_controller = get_memory_controller();
    if (!memory_controller) {
        return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
    }
    
    // Simple malloc implementation - allocate memory and return address
    void* ptr = std::malloc(size);
    if (!ptr) {
        return static_cast<int32_t>(SystemCallError::MEMORY_ERROR);
    }
    
    LOG_TRACE("malloc({}): allocated at {}", size, ptr);
    return reinterpret_cast<intptr_t>(ptr);
}

int32_t SystemCallInterface::syscall_free(const SystemCallContext& ctx) {
    void* ptr = reinterpret_cast<void*>(ctx.args[0]);
    
    if (!ptr) {
        return static_cast<int32_t>(SystemCallError::INVALID_ARGS);
    }
    
    std::free(ptr);
    LOG_TRACE("free({}): freed", ptr);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_calloc(const SystemCallContext& ctx) {
    size_t count = static_cast<size_t>(ctx.args[0]);
    size_t size = static_cast<size_t>(ctx.args[1]);
    
    void* ptr = std::calloc(count, size);
    if (!ptr) {
        return static_cast<int32_t>(SystemCallError::MEMORY_ERROR);
    }
    
    LOG_TRACE("calloc({}, {}): allocated at {}", count, size, ptr);
    return reinterpret_cast<intptr_t>(ptr);
}

int32_t SystemCallInterface::syscall_realloc(const SystemCallContext& ctx) {
    void* old_ptr = reinterpret_cast<void*>(ctx.args[0]);
    size_t new_size = static_cast<size_t>(ctx.args[1]);
    
    void* new_ptr = std::realloc(old_ptr, new_size);
    if (!new_ptr && new_size > 0) {
        return static_cast<int32_t>(SystemCallError::MEMORY_ERROR);
    }
    
    LOG_TRACE("realloc({}, {}): reallocated to {}", old_ptr, new_size, new_ptr);
    return reinterpret_cast<intptr_t>(new_ptr);
}

int32_t SystemCallInterface::syscall_heap_caps_malloc(const SystemCallContext& ctx) {
    size_t size = static_cast<size_t>(ctx.args[0]);
    uint32_t caps = static_cast<uint32_t>(ctx.args[1]);
    
    // For now, ignore capabilities and use standard malloc
    void* ptr = std::malloc(size);
    if (!ptr) {
        return static_cast<int32_t>(SystemCallError::MEMORY_ERROR);
    }
    
    LOG_TRACE("heap_caps_malloc({}, 0x{:x}): allocated at {}", size, caps, ptr);
    return reinterpret_cast<intptr_t>(ptr);
}

int32_t SystemCallInterface::syscall_heap_caps_free(const SystemCallContext& ctx) {
    void* ptr = reinterpret_cast<void*>(ctx.args[0]);
    
    if (!ptr) {
        return static_cast<int32_t>(SystemCallError::INVALID_ARGS);
    }
    
    std::free(ptr);
    LOG_TRACE("heap_caps_free({}): freed", ptr);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_get_free_heap_size(const SystemCallContext& ctx) {
    // Return a reasonable simulated heap size
    return 1024 * 1024; // 1MB
}

// System call implementations - Task management
int32_t SystemCallInterface::syscall_task_create(const SystemCallContext& ctx) {
    auto freertos = get_freertos_kernel();
    if (!freertos) {
        return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
    }
    
    // For now, return success without actually creating tasks
    LOG_TRACE("xTaskCreate: simulated task creation");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_task_delete(const SystemCallContext& ctx) {
    auto freertos = get_freertos_kernel();
    if (!freertos) {
        return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
    }
    
    LOG_TRACE("vTaskDelete: simulated task deletion");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_task_delay(const SystemCallContext& ctx) {
    uint32_t delay_ms = static_cast<uint32_t>(ctx.args[0]);
    
    auto freertos = get_freertos_kernel();
    if (!freertos) {
        return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
    }
    
    LOG_TRACE("vTaskDelay({}): simulated delay", delay_ms);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_task_suspend(const SystemCallContext& ctx) {
    LOG_TRACE("vTaskSuspend: simulated task suspend");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_task_resume(const SystemCallContext& ctx) {
    LOG_TRACE("vTaskResume: simulated task resume");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_yield(const SystemCallContext& ctx) {
    LOG_TRACE("taskYIELD: simulated yield");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

// System call implementations - Synchronization
int32_t SystemCallInterface::syscall_queue_create(const SystemCallContext& ctx) {
    LOG_TRACE("xQueueCreate: simulated queue creation");
    return 0x1000; // Return fake queue handle
}

int32_t SystemCallInterface::syscall_queue_delete(const SystemCallContext& ctx) {
    LOG_TRACE("vQueueDelete: simulated queue deletion");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_queue_send(const SystemCallContext& ctx) {
    LOG_TRACE("xQueueSend: simulated queue send");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_queue_receive(const SystemCallContext& ctx) {
    LOG_TRACE("xQueueReceive: simulated queue receive");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_semaphore_create_binary(const SystemCallContext& ctx) {
    LOG_TRACE("xSemaphoreCreateBinary: simulated semaphore creation");
    return 0x2000; // Return fake semaphore handle
}

int32_t SystemCallInterface::syscall_semaphore_take(const SystemCallContext& ctx) {
    LOG_TRACE("xSemaphoreTake: simulated semaphore take");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_semaphore_give(const SystemCallContext& ctx) {
    LOG_TRACE("xSemaphoreGive: simulated semaphore give");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

// System call implementations - File I/O
int32_t SystemCallInterface::syscall_open(const SystemCallContext& ctx) {
    LOG_TRACE("open: simulated file open");
    return 3; // Return fake file descriptor
}

int32_t SystemCallInterface::syscall_close(const SystemCallContext& ctx) {
    LOG_TRACE("close: simulated file close");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_read(const SystemCallContext& ctx) {
    LOG_TRACE("read: simulated file read");
    return 0; // Return 0 bytes read
}

int32_t SystemCallInterface::syscall_write(const SystemCallContext& ctx) {
    uint32_t size = static_cast<uint32_t>(ctx.args[2]);
    LOG_TRACE("write: simulated file write");
    return size; // Return bytes written
}

// System call implementations - ESP-IDF specific
int32_t SystemCallInterface::syscall_esp_log_write(const SystemCallContext& ctx) {
    // For now, just acknowledge the log write
    LOG_TRACE("esp_log_write: simulated log write");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_esp_restart(const SystemCallContext& ctx) {
    LOG_INFO("esp_restart: simulated ESP32 restart");
    // In a real implementation, this would trigger a system restart
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_esp_get_chip_info(const SystemCallContext& ctx) {
    LOG_TRACE("esp_get_chip_info: simulated chip info");
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_esp_random(const SystemCallContext& ctx) {
    // Return a simple pseudo-random number
    static uint32_t seed = 12345;
    seed = seed * 1103515245 + 12345;
    return seed & 0x7FFFFFFF;
}

// System call implementations - GPIO operations
int32_t SystemCallInterface::syscall_gpio_set_direction(const SystemCallContext& ctx) {
    uint32_t gpio_num = static_cast<uint32_t>(ctx.args[0]);
    uint32_t direction = static_cast<uint32_t>(ctx.args[1]);
    
    LOG_TRACE("gpio_set_direction({}, {}): simulated GPIO direction", gpio_num, direction);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_gpio_get_level(const SystemCallContext& ctx) {
    uint32_t gpio_num = static_cast<uint32_t>(ctx.args[0]);
    
    LOG_TRACE("gpio_get_level({}): simulated GPIO read", gpio_num);
    return 0; // Return low level
}

int32_t SystemCallInterface::syscall_gpio_set_level(const SystemCallContext& ctx) {
    uint32_t gpio_num = static_cast<uint32_t>(ctx.args[0]);
    uint32_t level = static_cast<uint32_t>(ctx.args[1]);
    
    LOG_TRACE("gpio_set_level({}, {}): simulated GPIO write", gpio_num, level);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

// System call implementations - I2C operations
int32_t SystemCallInterface::syscall_i2c_master_write_to_device(const SystemCallContext& ctx) {
    uint32_t port = static_cast<uint32_t>(ctx.args[0]);
    uint32_t device_addr = static_cast<uint32_t>(ctx.args[1]);
    
    LOG_TRACE("i2c_master_write_to_device: port={}, addr=0x{:02x}", port, device_addr);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

int32_t SystemCallInterface::syscall_i2c_master_read_from_device(const SystemCallContext& ctx) {
    uint32_t port = static_cast<uint32_t>(ctx.args[0]);
    uint32_t device_addr = static_cast<uint32_t>(ctx.args[1]);
    
    LOG_TRACE("i2c_master_read_from_device: port={}, addr=0x{:02x}", port, device_addr);
    return static_cast<int32_t>(SystemCallError::SUCCESS);
}

// Utility functions
int32_t SystemCallInterface::map_esp_error_to_syscall_error(int esp_err) {
    switch (esp_err) {
        case 0: // ESP_OK
            return static_cast<int32_t>(SystemCallError::SUCCESS);
        case -1: // ESP_FAIL
            return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
        case -2: // ESP_ERR_NO_MEM
            return static_cast<int32_t>(SystemCallError::MEMORY_ERROR);
        case -3: // ESP_ERR_INVALID_ARG
            return static_cast<int32_t>(SystemCallError::INVALID_ARGS);
        case -4: // ESP_ERR_INVALID_STATE
            return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
        case -5: // ESP_ERR_INVALID_SIZE
            return static_cast<int32_t>(SystemCallError::INVALID_ARGS);
        case -6: // ESP_ERR_NOT_FOUND
            return static_cast<int32_t>(SystemCallError::INVALID_SYSCALL);
        case -7: // ESP_ERR_NOT_SUPPORTED
            return static_cast<int32_t>(SystemCallError::NOT_IMPLEMENTED);
        case -8: // ESP_ERR_TIMEOUT
            return static_cast<int32_t>(SystemCallError::TIMEOUT);
        default:
            return static_cast<int32_t>(SystemCallError::KERNEL_ERROR);
    }
}


} // namespace m5tab5::emulator