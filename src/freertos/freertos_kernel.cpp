#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/freertos/cpu_integration.hpp"
#include "emulator/utils/logging.hpp"
#include <thread>

namespace m5tab5::emulator::freertos {

// Global kernel instance
FreeRTOSKernel* FreeRTOSKernelManager::instance_ = nullptr;
std::mutex FreeRTOSKernelManager::instance_mutex_;

FreeRTOSKernel::FreeRTOSKernel(DualCoreManager& cpu_manager)
    : cpu_manager_(cpu_manager), kernel_initialized_(false), kernel_running_(false),
      timer_task_handle_(nullptr), heap_free_bytes_(0) {
    
    LOG_INFO("Creating FreeRTOS kernel emulation");
}

FreeRTOSKernel::~FreeRTOSKernel() {
    if (kernel_running_) {
        stop_kernel();
    }
    LOG_INFO("FreeRTOS kernel destroyed");
}

Result<void> FreeRTOSKernel::initialize(const KernelConfig& config) {
    if (kernel_initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Kernel already initialized"));
    }
    
    LOG_INFO("Initializing FreeRTOS kernel emulation");
    config_ = config;
    
    // Initialize heap
    auto heap_result = initialize_heap();
    if (!heap_result.has_value()) {
        return heap_result;
    }
    
    // Create scheduler with configuration
    TaskScheduler::SchedulerConfig scheduler_config;
    scheduler_config.tick_frequency_hz = config_.tick_frequency_hz;
    scheduler_config.idle_task_stack_size = config_.idle_task_stack_size;
    scheduler_config.enable_task_statistics = config_.enable_task_statistics;
    scheduler_config.enable_stack_overflow_checking = config_.enable_stack_overflow_checking;
    
    scheduler_ = std::make_unique<TaskScheduler>(scheduler_config, cpu_manager_);
    auto init_result = scheduler_->initialize();
    if (!init_result.has_value()) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to initialize task scheduler"));
    }
    
    // Set as global kernel instance
    FreeRTOSKernelManager::set_instance(this);
    
    kernel_initialized_ = true;
    LOG_INFO("FreeRTOS kernel initialized successfully");
    return {};
}

Result<void> FreeRTOSKernel::start_kernel() {
    if (!kernel_initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Kernel not initialized"));
    }
    
    if (kernel_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Kernel already running"));
    }
    
    LOG_INFO("Starting FreeRTOS kernel");
    
    // Create timer task for software timers
    auto timer_result = create_timer_task();
    if (!timer_result.has_value()) {
        LOG_WARN("Failed to create timer task, continuing without software timers");
    }
    
    // Initialize CPU integration
    auto cpu_integration_result = initialize_cpu_integration(cpu_manager_, *scheduler_);
    if (!cpu_integration_result.has_value()) {
        LOG_ERROR("Failed to initialize CPU integration: {}", cpu_integration_result.error().to_string());
        return cpu_integration_result;
    }
    
    // Start the scheduler
    auto start_result = scheduler_->start();
    if (!start_result.has_value()) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to start task scheduler"));
    }
    
    // Start CPU integration to enable task execution
    auto cpu_start_result = start_cpu_integration();
    if (!cpu_start_result.has_value()) {
        LOG_ERROR("Failed to start CPU integration: {}", cpu_start_result.error().to_string());
        scheduler_->stop();
        return cpu_start_result;
    }
    
    kernel_running_ = true;
    LOG_INFO("FreeRTOS kernel started successfully with CPU integration");
    return {};
}

Result<void> FreeRTOSKernel::stop_kernel() {
    if (!kernel_running_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Kernel not running"));
    }
    
    LOG_INFO("Stopping FreeRTOS kernel");
    
    // Stop CPU integration first
    auto cpu_stop_result = stop_cpu_integration();
    if (!cpu_stop_result.has_value()) {
        LOG_WARN("Failed to stop CPU integration cleanly: {}", cpu_stop_result.error().to_string());
    }
    
    // Stop the scheduler
    if (scheduler_) {
        scheduler_->stop();
    }
    
    // Clean up timer task
    if (timer_task_handle_) {
        scheduler_->delete_task(timer_task_handle_);
        timer_task_handle_ = nullptr;
    }
    
    kernel_running_ = false;
    LOG_INFO("FreeRTOS kernel stopped");
    return {};
}

Result<void> FreeRTOSKernel::reset_kernel() {
    LOG_INFO("Resetting FreeRTOS kernel");
    
    if (kernel_running_) {
        auto stop_result = stop_kernel();
        if (!stop_result.has_value()) {
            return stop_result;
        }
    }
    
    // Reinitialize everything
    kernel_initialized_ = false;
    scheduler_.reset();
    timer_task_handle_ = nullptr;
    
    // Shutdown CPU integration
    shutdown_cpu_integration();
    
    // Restart if needed
    return initialize(config_);
}

Result<void> FreeRTOSKernel::initialize_heap() {
    LOG_DEBUG("Initializing FreeRTOS heap: {} bytes", config_.total_heap_size);
    
    if (config_.enable_memory_management) {
        heap_memory_ = std::make_unique<uint8_t[]>(config_.total_heap_size);
        heap_free_bytes_ = config_.total_heap_size;
        
        // Initialize heap with pattern
        std::memset(heap_memory_.get(), 0x5A, config_.total_heap_size);
    }
    
    LOG_DEBUG("FreeRTOS heap initialized");
    return {};
}

Result<void> FreeRTOSKernel::create_timer_task() {
    LOG_DEBUG("Creating FreeRTOS timer task");
    
    if (!scheduler_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler not initialized"));
    }
    
    Task::TaskParameters params;
    params.name = "tmr";
    params.function = [this](void* p) { timer_task_function(p); };
    params.parameters = this;
    params.priority = configMAX_PRIORITIES - 1; // High priority
    params.stack_size = config_.timer_task_stack_size;
    params.core_affinity = 0; // Run on core 0
    
    auto result = scheduler_->create_task(params);
    if (result.has_value()) {
        timer_task_handle_ = result.value();
        LOG_DEBUG("Created timer task successfully");
        return {};
    }
    
    return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to create timer task"));
}

void FreeRTOSKernel::timer_task_function(void* parameters) {
    FreeRTOSKernel* kernel = static_cast<FreeRTOSKernel*>(parameters);
    LOG_DEBUG("Timer task started");
    
    while (kernel->kernel_running_) {
        // Process software timers (simplified implementation)
        // In a full implementation, this would handle timer callbacks
        
        // Sleep for 10ms (timer resolution)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Yield to other tasks periodically
        if (kernel->scheduler_) {
            kernel->scheduler_->yield_current_task();
        }
    }
    
    LOG_DEBUG("Timer task ended");
}

bool FreeRTOSKernel::is_kernel_running() const {
    return kernel_running_ && scheduler_ && scheduler_->is_running();
}

TickType_t FreeRTOSKernel::get_kernel_tick_count() const {
    if (scheduler_) {
        return scheduler_->get_tick_count();
    }
    return 0;
}

Result<TaskHandle_t> FreeRTOSKernel::create_task(
    const std::string& name,
    TaskFunction_t function,
    void* parameters,
    uint16_t stack_size,
    UBaseType_t priority,
    UBaseType_t core_affinity
) {
    if (!scheduler_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Scheduler not initialized"));
    }
    
    Task::TaskParameters params;
    params.name = name;
    params.function = function;
    params.parameters = parameters;
    params.stack_size = stack_size;
    params.priority = priority;
    params.core_affinity = core_affinity;
    
    return scheduler_->create_task(params);
}

SemaphoreHandle_t FreeRTOSKernel::create_binary_semaphore() {
    return SynchronizationAPI::xSemaphoreCreateBinary();
}

SemaphoreHandle_t FreeRTOSKernel::create_counting_semaphore(UBaseType_t max_count, UBaseType_t initial_count) {
    return SynchronizationAPI::xSemaphoreCreateCounting(max_count, initial_count);
}

SemaphoreHandle_t FreeRTOSKernel::create_mutex() {
    return SynchronizationAPI::xSemaphoreCreateMutex();
}

QueueHandle_t FreeRTOSKernel::create_queue(UBaseType_t length, UBaseType_t item_size) {
    return SynchronizationAPI::xQueueCreate(length, item_size);
}

FreeRTOSKernel::KernelInfo FreeRTOSKernel::get_kernel_info() const {
    KernelInfo info = {};
    
    info.tick_frequency = config_.tick_frequency_hz;
    info.total_heap_size = config_.total_heap_size;
    info.free_heap_size = heap_free_bytes_;
    
    if (scheduler_) {
        info.total_tasks = scheduler_->get_task_count();
        info.total_runtime_ticks = scheduler_->get_tick_count();
        
        const auto& stats = scheduler_->get_statistics();
        uint64_t total_ticks = stats.total_ticks;
        uint64_t idle_ticks = stats.idle_ticks_core_0 + stats.idle_ticks_core_1;
        
        if (total_ticks > 0) {
            info.idle_percentage = static_cast<uint32_t>((idle_ticks * 100) / total_ticks);
        }
    }
    
    return info;
}

void FreeRTOSKernel::dump_task_list() const {
    LOG_INFO("=== FreeRTOS Task List ===");
    
    if (!scheduler_) {
        LOG_INFO("Scheduler not initialized");
        return;
    }
    
    auto tasks = scheduler_->get_all_tasks();
    LOG_INFO("Total tasks: {}", tasks.size());
    
    for (size_t i = 0; i < tasks.size(); ++i) {
        // In a full implementation, would display detailed task information
        LOG_INFO("Task {}: {}", i, static_cast<void*>(tasks[i]));
    }
    
    LOG_INFO("=========================");
}

void FreeRTOSKernel::dump_kernel_statistics() const {
    LOG_INFO("=== FreeRTOS Kernel Statistics ===");
    
    auto info = get_kernel_info();
    LOG_INFO("Tick frequency: {} Hz", info.tick_frequency);
    LOG_INFO("Total tasks: {}", info.total_tasks);
    LOG_INFO("Running tasks: {}", info.running_tasks);
    LOG_INFO("Blocked tasks: {}", info.blocked_tasks);
    LOG_INFO("Heap total: {} bytes", info.total_heap_size);
    LOG_INFO("Heap free: {} bytes", info.free_heap_size);
    LOG_INFO("Runtime ticks: {}", info.total_runtime_ticks);
    LOG_INFO("Idle percentage: {}%", info.idle_percentage);
    
    if (scheduler_) {
        const auto& stats = scheduler_->get_statistics();
        LOG_INFO("Context switches: {}", stats.total_context_switches);
        LOG_INFO("Context switches/sec: {}", stats.task_switches_per_second);
    }
    
    LOG_INFO("===================================");
}

void FreeRTOSKernel::print_stack_high_water_marks() const {
    LOG_INFO("=== Task Stack Usage ===");
    
    if (!scheduler_) {
        LOG_INFO("Scheduler not initialized");
        return;
    }
    
    auto tasks = scheduler_->get_all_tasks();
    for (auto task_handle : tasks) {
        auto mark_result = scheduler_->get_stack_high_water_mark(task_handle);
        if (mark_result.has_value()) {
            LOG_INFO("Task {}: {} bytes free", 
                     static_cast<void*>(task_handle), 
                     mark_result.value());
        }
    }
    
    LOG_INFO("========================");
}

// FreeRTOSKernelManager Implementation
void FreeRTOSKernelManager::set_instance(FreeRTOSKernel* kernel) {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    instance_ = kernel;
}

FreeRTOSKernel* FreeRTOSKernelManager::get_instance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_;
}

bool FreeRTOSKernelManager::has_instance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_ != nullptr;
}

bool FreeRTOSKernelManager::is_kernel_running() {
    auto instance = get_instance();
    return instance ? instance->is_kernel_running() : false;
}

TickType_t FreeRTOSKernelManager::get_tick_count() {
    auto instance = get_instance();
    return instance ? instance->get_kernel_tick_count() : 0;
}

TaskHandle_t FreeRTOSKernelManager::get_current_task() {
    return TaskAPI::xTaskGetCurrentTaskHandle();
}

} // namespace m5tab5::emulator::freertos