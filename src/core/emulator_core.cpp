#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include <thread>
#include <chrono>

namespace m5tab5::emulator {

DECLARE_LOGGER("EmulatorCore");

EmulatorCore::EmulatorCore(const Configuration& config) 
    : config_(config),
      state_(EmulatorState::UNINITIALIZED),
      running_(false),
      target_frequency_(400000000),  // 400MHz default
      cycles_executed_(0) {
    COMPONENT_LOG_DEBUG("EmulatorCore created");
}

EmulatorCore::~EmulatorCore() {
    if (state_ != EmulatorState::UNINITIALIZED && state_ != EmulatorState::SHUTDOWN) {
        auto result = shutdown();
        if (!result) {
            auto error_msg = result.error().to_string();
            COMPONENT_LOG_ERROR("Failed to shutdown emulator in destructor: {}", error_msg);
        }
    }
    COMPONENT_LOG_DEBUG("EmulatorCore destroyed");
}

Result<void> EmulatorCore::initialize(const Configuration& config) {
    if (state_ != EmulatorState::UNINITIALIZED) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Emulator already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing M5Stack Tab5 emulator");
    
    try {
        // Store configuration
        config_ = config;
        
        // Get debug configuration for logging setup
        auto debug_config = config.getDebugConfig();
        
        // Initialize logging with configuration (if not already initialized)
        if (Logger::get_logger() == nullptr) {
            auto log_level_enum = Logger::from_string(debug_config.log_level);
            auto log_result = Logger::initialize(
                log_level_enum,
                debug_config.log_file,
                debug_config.enable_logging
            );
            if (!log_result) {
                return unexpected(log_result.error());
            }
        } else {
            // Logger already initialized, just update level if needed
            auto log_level_enum = Logger::from_string(debug_config.log_level);
            Logger::set_level(log_level_enum);
        }
        
        // Set target frequency from CPU configuration
        auto cpu_config = config.getCPUConfig();
        target_frequency_ = cpu_config.main_core_freq;
        
        // Initialize memory controller
        COMPONENT_LOG_DEBUG("Initializing memory controller");
        memory_controller_ = std::make_unique<MemoryController>();
        RETURN_IF_ERROR(memory_controller_->initialize(config));
        
        // Initialize CPU cores
        COMPONENT_LOG_DEBUG("Initializing CPU cores");
        cpu_manager_ = std::make_unique<DualCoreManager>();
        RETURN_IF_ERROR(cpu_manager_->initialize(config, *memory_controller_));
        
        // Initialize peripheral manager
        COMPONENT_LOG_DEBUG("Initializing peripheral manager");
        auto interrupt_callback = [this](uint32_t interrupt_id, bool level) {
            // Handle interrupt routing to CPU (TODO: implement proper interrupt routing)
            if (cpu_manager_ && level) {
                // For now, just signal interrupt to main core
                cpu_manager_->send_inter_core_interrupt(DualCoreManager::CoreId::CORE_0, 
                                                       DualCoreManager::CoreId::CORE_0, 
                                                       interrupt_id);
            }
        };
        peripheral_manager_ = std::make_unique<PeripheralManager>(interrupt_callback);
        auto peripheral_init_result = peripheral_manager_->initialize();
        if (peripheral_init_result != EmulatorError::Success) {
            return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Failed to initialize peripheral manager"));
        }
        
        // Initialize graphics engine if display is enabled
        auto display_config = config.getDisplayConfig();
        if (display_config.width > 0 && display_config.height > 0) {
            COMPONENT_LOG_DEBUG("Initializing graphics engine");
            graphics_engine_ = std::make_unique<GraphicsEngine>();
            RETURN_IF_ERROR(graphics_engine_->initialize(config));
        }
        
        // Initialize plugin manager
        COMPONENT_LOG_DEBUG("Initializing plugin manager");
        plugin_manager_ = std::make_unique<PluginManager>();
        RETURN_IF_ERROR(plugin_manager_->initialize(config));
        
        // Load and initialize enabled plugins
        // TODO: Add plugin configuration section to Configuration class
        // For now, skip plugin loading
        if (false) { // Placeholder to avoid compilation error
            const std::vector<std::string> plugin_names; // Empty for now
            for (const auto& plugin_name : plugin_names) {
                auto load_result = plugin_manager_->load_plugin(plugin_name);
                if (!load_result) {
                    COMPONENT_LOG_WARN("Failed to load plugin '{}': {}", 
                                      plugin_name, load_result.error().to_string());
                } else {
                    COMPONENT_LOG_INFO("Loaded plugin: {}", plugin_name);
                }
            }
        }
        
        // Initialize debugger if enabled
        if (debug_config.enable_debugger) {
            COMPONENT_LOG_DEBUG("Initializing debugger");
            Debugger::DebugConfig debugger_config;
            debugger_config.enable_gdb_server = debug_config.enable_debugger;
            debugger_config.gdb_port = debug_config.debugger_port;
            debugger_config.enable_profiler = debug_config.enable_profiling;
            debugger_ = std::make_unique<Debugger>(*this, debugger_config);
            auto debugger_init_result = debugger_->initialize();
            if (debugger_init_result != EmulatorError::Success) {
                return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Failed to initialize debugger"));
            }
        }
        
        // Register all components in the registry for external access
        RETURN_IF_ERROR(initializeComponents());
        
        state_ = EmulatorState::INITIALIZED;
        COMPONENT_LOG_INFO("Emulator initialization completed successfully");
        
        return {};
        
    } catch (const std::exception& e) {
        state_ = EmulatorState::ERROR;
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during initialization: " + std::string(e.what())));
    }
}

Result<void> EmulatorCore::start() {
    if (state_ != EmulatorState::INITIALIZED && state_ != EmulatorState::PAUSED) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Emulator must be initialized before starting"));
    }
    
    COMPONENT_LOG_INFO("Starting emulator execution");
    
    running_ = true;
    state_ = EmulatorState::RUNNING;
    
    // Start main execution thread
    execution_thread_ = std::thread(&EmulatorCore::execution_loop, this);
    
    // Start graphics rendering thread if graphics engine is available
    if (graphics_engine_) {
        graphics_thread_ = std::thread(&EmulatorCore::graphics_loop, this);
    }
    
    COMPONENT_LOG_INFO("Emulator started successfully");
    return {};
}

Result<void> EmulatorCore::pause() {
    if (state_ != EmulatorState::RUNNING) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Emulator is not running"));
    }
    
    COMPONENT_LOG_INFO("Pausing emulator");
    
    state_ = EmulatorState::PAUSED;
    
    // CPU cores will check state and pause automatically
    if (cpu_manager_) {
        cpu_manager_->pause();
    }
    
    COMPONENT_LOG_INFO("Emulator paused");
    return {};
}

Result<void> EmulatorCore::resume() {
    if (state_ != EmulatorState::PAUSED) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Emulator is not paused"));
    }
    
    COMPONENT_LOG_INFO("Resuming emulator");
    
    state_ = EmulatorState::RUNNING;
    
    if (cpu_manager_) {
        cpu_manager_->resume();
    }
    
    COMPONENT_LOG_INFO("Emulator resumed");
    return {};
}

Result<void> EmulatorCore::stop() {
    if (state_ != EmulatorState::RUNNING && state_ != EmulatorState::PAUSED) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Emulator is not running or paused"));
    }
    
    COMPONENT_LOG_INFO("Stopping emulator");
    
    running_ = false;
    state_ = EmulatorState::STOPPING;
    
    // Stop CPU cores
    if (cpu_manager_) {
        cpu_manager_->stop();
    }
    
    // Wait for threads to finish
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
    
    if (graphics_thread_.joinable()) {
        graphics_thread_.join();
    }
    
    state_ = EmulatorState::STOPPED;
    COMPONENT_LOG_INFO("Emulator stopped");
    return {};
}

Result<void> EmulatorCore::shutdown() {
    if (state_ == EmulatorState::UNINITIALIZED || state_ == EmulatorState::SHUTDOWN) {
        return {};  // Already shutdown
    }
    
    COMPONENT_LOG_INFO("Shutting down emulator");
    
    // Stop if still running
    if (state_ == EmulatorState::RUNNING || state_ == EmulatorState::PAUSED) {
        RETURN_IF_ERROR(stop());
    }
    
    // Clear component registry first
    shutdownComponents();
    
    // Shutdown components in reverse order of initialization
    if (debugger_) {
        debugger_->shutdown();
        debugger_.reset();
    }
    
    if (plugin_manager_) {
        plugin_manager_->shutdown();
        plugin_manager_.reset();
    }
    
    if (graphics_engine_) {
        graphics_engine_->shutdown();
        graphics_engine_.reset();
    }
    
    if (peripheral_manager_) {
        peripheral_manager_->shutdown();
        peripheral_manager_.reset();
    }
    
    if (cpu_manager_) {
        cpu_manager_->shutdown();
        cpu_manager_.reset();
    }
    
    if (memory_controller_) {
        // MemoryController doesn't have explicit shutdown - destructor handles cleanup
        memory_controller_.reset();
    }
    
    // config_ is by value, no reset needed
    
    state_ = EmulatorState::SHUTDOWN;
    COMPONENT_LOG_INFO("Emulator shutdown completed");
    
    return {};
}

Result<void> EmulatorCore::reset() {
    COMPONENT_LOG_INFO("Resetting emulator");
    
    bool was_running = (state_ == EmulatorState::RUNNING);
    
    // Stop if running
    if (was_running) {
        RETURN_IF_ERROR(stop());
    }
    
    // Reset components
    if (cpu_manager_) {
        RETURN_IF_ERROR(cpu_manager_->reset());
    }
    
    if (memory_controller_) {
        // MemoryController doesn't have reset method - would need to be recreated
        // For now, skip memory controller reset
    }
    
    if (peripheral_manager_) {
        auto peripheral_reset_result = peripheral_manager_->reset();
        if (peripheral_reset_result != EmulatorError::Success) {
            return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to reset peripheral manager"));
        }
    }
    
    cycles_executed_ = 0;
    
    // Restart if it was running before
    if (was_running) {
        RETURN_IF_ERROR(start());
    } else {
        state_ = EmulatorState::INITIALIZED;
    }
    
    COMPONENT_LOG_INFO("Emulator reset completed");
    return {};
}

EmulatorState EmulatorCore::get_state() const {
    return state_;
}

Cycles EmulatorCore::get_cycles_executed() const {
    return cycles_executed_;
}

double EmulatorCore::get_execution_speed() const {
    if (target_frequency_ == 0) return 0.0;
    
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        current_time - start_time_).count();
    
    if (elapsed == 0) return 0.0;
    
    double actual_frequency = (static_cast<double>(cycles_executed_) * 1e9) / elapsed;
    return actual_frequency / target_frequency_;
}

void EmulatorCore::execution_loop() {
    COMPONENT_LOG_DEBUG("Execution loop started");
    
    start_time_ = std::chrono::steady_clock::now();
    auto last_time = start_time_;
    
    const auto target_frame_time = std::chrono::nanoseconds(1000000000 / 60);  // 60 FPS
    Cycles cycles_per_frame = target_frequency_ / 60;
    
    while (running_ && state_ == EmulatorState::RUNNING) {
        auto frame_start = std::chrono::steady_clock::now();
        
        // Execute CPU for one frame worth of cycles
        if (cpu_manager_ && state_ == EmulatorState::RUNNING) {
            auto exec_result = cpu_manager_->execute_cycles(cycles_per_frame);
            if (exec_result) {
                cycles_executed_ += exec_result.value();
            } else {
                COMPONENT_LOG_ERROR("CPU execution error: {}", exec_result.error().to_string());
                if (exec_result.error().code() == ErrorCode::CPU_HALT) {
                    COMPONENT_LOG_INFO("CPU halted, stopping emulator");
                    running_ = false;
                    break;
                }
            }
        }
        
        // Update peripherals  
        if (peripheral_manager_ && state_ == EmulatorState::RUNNING) {
            // Use tick method with current cycle count
            peripheral_manager_->tick(cycles_executed_);
        }
        
        // Sleep to maintain target frequency
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = frame_end - frame_start;
        
        if (frame_duration < target_frame_time) {
            std::this_thread::sleep_for(target_frame_time - frame_duration);
        }
        
        // Log performance statistics periodically
        static int frame_count = 0;
        if (++frame_count % (60 * 10) == 0) {  // Every 10 seconds
            COMPONENT_LOG_DEBUG("Execution speed: {:.2f}x, Cycles: {}", 
                              get_execution_speed(), cycles_executed_);
        }
    }
    
    COMPONENT_LOG_DEBUG("Execution loop ended");
}

void EmulatorCore::graphics_loop() {
    COMPONENT_LOG_DEBUG("Graphics loop started");
    
    const auto target_frame_time = std::chrono::nanoseconds(1000000000 / 60);  // 60 FPS
    
    while (running_) {
        auto frame_start = std::chrono::steady_clock::now();
        
        if (graphics_engine_ && (state_ == EmulatorState::RUNNING || state_ == EmulatorState::PAUSED)) {
            auto render_result = graphics_engine_->render_frame();
            if (!render_result) {
                COMPONENT_LOG_ERROR("Graphics rendering error: {}", 
                                   render_result.error().to_string());
            }
        }
        
        // Maintain 60 FPS
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = frame_end - frame_start;
        
        if (frame_duration < target_frame_time) {
            std::this_thread::sleep_for(target_frame_time - frame_duration);
        }
    }
    
    COMPONENT_LOG_DEBUG("Graphics loop ended");
}

//
// Component access interface implementations
//

std::shared_ptr<MemoryController> EmulatorCore::getMemoryController() const {
    // Convert unique_ptr to shared_ptr for safe external access
    // Note: This creates a shared_ptr that doesn't own the resource
    // The EmulatorCore retains ownership via unique_ptr
    if (!memory_controller_) {
        return nullptr;
    }
    
    // Create a shared_ptr that shares ownership with the EmulatorCore
    // We use a custom deleter that does nothing since EmulatorCore owns the resource
    return std::shared_ptr<MemoryController>(
        memory_controller_.get(),
        [](MemoryController*) { /* EmulatorCore owns this resource */ }
    );
}

Cycles EmulatorCore::getCurrentCycle() const {
    return cycles_executed_;
}

std::shared_ptr<void> EmulatorCore::getComponent(const std::string& name) const {
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    auto it = component_registry_.find(name);
    if (it == component_registry_.end()) {
        return nullptr;
    }
    
    return it->second;
}

Result<void> EmulatorCore::initializeComponents() {
    // Register components in the registry after they are created
    // This is called during initialize() after components are created
    
    if (memory_controller_) {
        registerComponent<MemoryController>("memory", getMemoryController());
    }
    
    if (cpu_manager_) {
        // Convert unique_ptr to shared_ptr with custom deleter
        auto shared_cpu = std::shared_ptr<DualCoreManager>(
            cpu_manager_.get(),
            [](DualCoreManager*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<DualCoreManager>("cpu", shared_cpu);
        registerComponent<DualCoreManager>("dual_core_manager", shared_cpu);
    }
    
    if (peripheral_manager_) {
        auto shared_peripheral = std::shared_ptr<PeripheralManager>(
            peripheral_manager_.get(),
            [](PeripheralManager*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<PeripheralManager>("peripherals", shared_peripheral);
        registerComponent<PeripheralManager>("peripheral_manager", shared_peripheral);
    }
    
    if (graphics_engine_) {
        auto shared_graphics = std::shared_ptr<GraphicsEngine>(
            graphics_engine_.get(),
            [](GraphicsEngine*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<GraphicsEngine>("graphics", shared_graphics);
        registerComponent<GraphicsEngine>("display", shared_graphics);
    }
    
    if (plugin_manager_) {
        auto shared_plugins = std::shared_ptr<PluginManager>(
            plugin_manager_.get(),
            [](PluginManager*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<PluginManager>("plugins", shared_plugins);
    }
    
    if (debugger_) {
        auto shared_debugger = std::shared_ptr<Debugger>(
            debugger_.get(),
            [](Debugger*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<Debugger>("debugger", shared_debugger);
    }
    
    COMPONENT_LOG_DEBUG("Component registry initialized with {} components", 
                       component_registry_.size());
    
    return {};
}

void EmulatorCore::shutdownComponents() {
    std::lock_guard<std::mutex> lock(component_registry_mutex_);
    
    // Clear all registered components
    component_registry_.clear();
    type_registry_.clear();
    
    COMPONENT_LOG_DEBUG("Component registry cleared");
}

}  // namespace m5tab5::emulator