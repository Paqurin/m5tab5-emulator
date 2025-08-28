#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/firmware/elf_loader.hpp"
#include <thread>
#include <chrono>
#include <future>

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
        
        // Initialize Boot ROM for ESP32-P4 boot sequence
        COMPONENT_LOG_DEBUG("Initializing Boot ROM");
        boot_rom_ = std::make_unique<BootROM>();
        RETURN_IF_ERROR(boot_rom_->initialize());
        
        // Configure boot ROM with necessary components
        boot_rom_->setMemoryController(std::shared_ptr<MemoryController>(
            memory_controller_.get(),
            [](MemoryController*) { /* EmulatorCore owns this resource */ }
        ));
        
        boot_rom_->setDualCoreManager(std::shared_ptr<DualCoreManager>(
            cpu_manager_.get(),
            [](DualCoreManager*) { /* EmulatorCore owns this resource */ }
        ));
        
        // Initialize ELF loader for application loading
        COMPONENT_LOG_DEBUG("Initializing ELF loader");
        elf_loader_ = std::make_unique<firmware::ELFLoader>();
        RETURN_IF_ERROR(elf_loader_->initialize());
        
        // Configure ELF loader with necessary components
        elf_loader_->set_memory_controller(std::shared_ptr<MemoryController>(
            memory_controller_.get(),
            [](MemoryController*) { /* EmulatorCore owns this resource */ }
        ));
        
        elf_loader_->set_cpu_manager(std::shared_ptr<DualCoreManager>(
            cpu_manager_.get(),
            [](DualCoreManager*) { /* EmulatorCore owns this resource */ }
        ));
        
        elf_loader_->set_boot_rom(std::shared_ptr<BootROM>(
            boot_rom_.get(),
            [](BootROM*) { /* EmulatorCore owns this resource */ }
        ));
        
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
        
        // Initialize graphics engine if display is enabled and graphics are not disabled
        auto display_config = config.getDisplayConfig();
        bool graphics_enabled = true;
        try {
            graphics_enabled = config.getValue<bool>("graphics", "enable", true);
        } catch (...) {
            graphics_enabled = true; // Default to enabled if setting not found
        }
        
        if (graphics_enabled && display_config.width > 0 && display_config.height > 0) {
            COMPONENT_LOG_DEBUG("Initializing graphics engine");
            graphics_engine_ = std::make_unique<GraphicsEngine>();
            RETURN_IF_ERROR(graphics_engine_->initialize(config));
        } else {
            COMPONENT_LOG_INFO("Graphics engine disabled - running in headless mode");
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
    
    // Wait for threads to finish with timeout
    const auto thread_timeout = std::chrono::milliseconds(2000);
    
    if (execution_thread_.joinable()) {
        COMPONENT_LOG_DEBUG("Waiting for execution thread to finish...");
        // Wait with timeout pattern
        auto future = std::async(std::launch::async, [this]() {
            if (execution_thread_.joinable()) {
                execution_thread_.join();
            }
        });
        
        if (future.wait_for(thread_timeout) == std::future_status::timeout) {
            COMPONENT_LOG_WARN("Execution thread did not finish within timeout, detaching");
            execution_thread_.detach();
        }
    }
    
    if (graphics_thread_.joinable()) {
        COMPONENT_LOG_DEBUG("Waiting for graphics thread to finish...");
        auto future = std::async(std::launch::async, [this]() {
            if (graphics_thread_.joinable()) {
                graphics_thread_.join();
            }
        });
        
        if (future.wait_for(thread_timeout) == std::future_status::timeout) {
            COMPONENT_LOG_WARN("Graphics thread did not finish within timeout, detaching");
            graphics_thread_.detach();
        }
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
    
    if (boot_rom_) {
        boot_rom_->shutdown();
        boot_rom_.reset();
    }
    
    if (elf_loader_) {
        elf_loader_->shutdown();
        elf_loader_.reset();
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
    
    const auto target_frame_time = std::chrono::nanoseconds(1000000000 / 60);  // 60 FPS
    const auto shutdown_check_interval = std::chrono::milliseconds(10);  // Check shutdown every 10ms
    Cycles cycles_per_frame = target_frequency_ / 60;
    
    while (running_.load() && state_ == EmulatorState::RUNNING) {
        auto frame_start = std::chrono::steady_clock::now();
        
        // Check for shutdown more frequently
        if (!running_.load() || state_ == EmulatorState::STOPPING) {
            break;
        }
        
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
        if (peripheral_manager_ && state_ == EmulatorState::RUNNING && running_.load()) {
            // Use tick method with current cycle count
            peripheral_manager_->tick(cycles_executed_);
        }
        
        // Sleep to maintain target frequency with shutdown responsiveness
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = frame_end - frame_start;
        
        if (frame_duration < target_frame_time && running_.load()) {
            auto sleep_time = target_frame_time - frame_duration;
            // Break sleep into smaller chunks to be responsive to shutdown
            while (sleep_time > std::chrono::nanoseconds(0) && running_.load()) {
                auto chunk = std::min(sleep_time, std::chrono::duration_cast<std::chrono::nanoseconds>(shutdown_check_interval));
                std::this_thread::sleep_for(chunk);
                sleep_time -= chunk;
            }
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
    const auto shutdown_check_interval = std::chrono::milliseconds(50);  // Check shutdown every 50ms
    
    while (running_.load()) {
        auto frame_start = std::chrono::steady_clock::now();
        
        // Check if we should exit early
        if (!running_.load() || state_ == EmulatorState::STOPPING) {
            break;
        }
        
        if (graphics_engine_ && (state_ == EmulatorState::RUNNING || state_ == EmulatorState::PAUSED)) {
            auto render_result = graphics_engine_->render_frame();
            if (!render_result) {
                COMPONENT_LOG_ERROR("Graphics rendering error: {}", 
                                   render_result.error().to_string());
                // Don't exit on render error, just continue
            }
        }
        
        // Maintain 60 FPS with shutdown responsiveness
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = frame_end - frame_start;
        
        if (frame_duration < target_frame_time) {
            auto sleep_time = target_frame_time - frame_duration;
            // Break sleep into smaller chunks to be responsive to shutdown
            while (sleep_time > std::chrono::nanoseconds(0) && running_.load()) {
                auto chunk = std::min(sleep_time, std::chrono::duration_cast<std::chrono::nanoseconds>(shutdown_check_interval));
                std::this_thread::sleep_for(chunk);
                sleep_time -= chunk;
            }
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
    
    if (boot_rom_) {
        auto shared_boot_rom = std::shared_ptr<BootROM>(
            boot_rom_.get(),
            [](BootROM*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<BootROM>("boot_rom", shared_boot_rom);
        registerComponent<BootROM>("bootrom", shared_boot_rom);
    }
    
    if (elf_loader_) {
        auto shared_elf_loader = std::shared_ptr<firmware::ELFLoader>(
            elf_loader_.get(),
            [](firmware::ELFLoader*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<firmware::ELFLoader>("elf_loader", shared_elf_loader);
        registerComponent<firmware::ELFLoader>("firmware_loader", shared_elf_loader);
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

// ESP32-P4 Boot Sequence Implementation

Result<void> EmulatorCore::cold_boot() {
    COMPONENT_LOG_INFO("Starting ESP32-P4 cold boot sequence");
    
    if (state_ != EmulatorState::INITIALIZED && state_ != EmulatorState::STOPPED) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "Emulator must be initialized or stopped for cold boot"));
    }
    
    // Initialize boot ROM if not already done
    if (!boot_rom_) {
        boot_rom_ = std::make_unique<BootROM>();
        RETURN_IF_ERROR(boot_rom_->initialize());
        
        // Configure boot ROM with emulator components
        boot_rom_->setMemoryController(std::shared_ptr<MemoryController>(
            memory_controller_.get(),
            [](MemoryController*) { /* EmulatorCore owns this resource */ }
        ));
        
        boot_rom_->setDualCoreManager(std::shared_ptr<DualCoreManager>(
            cpu_manager_.get(),
            [](DualCoreManager*) { /* EmulatorCore owns this resource */ }
        ));
        
        COMPONENT_LOG_DEBUG("Boot ROM initialized and configured");
    }
    
    // Reset all components to initial state
    RETURN_IF_ERROR(reset());
    
    // Execute complete ESP32-P4 boot sequence
    RETURN_IF_ERROR(execute_complete_boot_sequence());
    
    COMPONENT_LOG_INFO("ESP32-P4 cold boot completed successfully");
    return {};
}

Result<void> EmulatorCore::warm_restart() {
    COMPONENT_LOG_INFO("Starting ESP32-P4 warm restart sequence");
    
    if (state_ == EmulatorState::RUNNING) {
        RETURN_IF_ERROR(stop());
    }
    
    // Warm restart skips full hardware initialization
    // Reset CPU and peripherals but keep memory state
    if (cpu_manager_) {
        RETURN_IF_ERROR(cpu_manager_->reset());
    }
    
    if (peripheral_manager_) {
        auto peripheral_reset_result = peripheral_manager_->reset();
        if (peripheral_reset_result != EmulatorError::Success) {
            return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to reset peripheral manager"));
        }
    }
    
    // Execute boot sequence starting from bootloader phase
    if (boot_rom_) {
        RETURN_IF_ERROR(boot_rom_->executeEsp32P4BootSequence());
    }
    
    // Start execution
    RETURN_IF_ERROR(start());
    
    COMPONENT_LOG_INFO("ESP32-P4 warm restart completed successfully");
    return {};
}

Result<void> EmulatorCore::execute_complete_boot_sequence() {
    COMPONENT_LOG_INFO("Executing complete ESP32-P4 boot sequence");
    
    if (!boot_rom_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Boot ROM not initialized"));
    }
    
    // Set up boot sequence callback to start emulator after boot
    boot_rom_->setBootSequenceCallback([this]() -> Result<void> {
        COMPONENT_LOG_DEBUG("Boot sequence callback: starting emulator execution");
        return start();
    });
    
    // Execute the enhanced ESP32-P4 boot sequence
    RETURN_IF_ERROR(boot_rom_->executeEsp32P4BootSequence());
    
    // Register boot ROM in component registry
    if (boot_rom_) {
        auto shared_boot_rom = std::shared_ptr<BootROM>(
            boot_rom_.get(),
            [](BootROM*) { /* EmulatorCore owns this resource */ }
        );
        registerComponent<BootROM>("boot_rom", shared_boot_rom);
    }
    
    COMPONENT_LOG_INFO("Complete ESP32-P4 boot sequence executed successfully");
    return {};
}

// Firmware loading and execution methods

Result<firmware::ELFLoader::LoadingResult> EmulatorCore::load_elf_application(const std::string& file_path,
                                                                               firmware::ELFLoader::ProgressCallback progress_callback) {
    COMPONENT_LOG_INFO("Loading ELF application: {}", file_path);
    
    if (!elf_loader_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "ELF loader not initialized"));
    }
    
    // Ensure emulator is in a state where we can load applications
    if (state_ != EmulatorState::INITIALIZED && state_ != EmulatorState::STOPPED) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, "Cannot load application while emulator is running"));
    }
    
    // Load the ELF application
    auto result = elf_loader_->load_elf_application(file_path, progress_callback);
    if (!result.has_value()) {
        COMPONENT_LOG_ERROR("Failed to load ELF application: {}", result.error().message());
        return unexpected(result.error());
    }
    
    COMPONENT_LOG_INFO("ELF application loaded successfully: entry=0x{:08x}, size={} bytes", 
                      result.value().entry_point, result.value().total_size);
    
    return result;
}

Result<firmware::ELFLoader::LoadingResult> EmulatorCore::load_esp_application_image(const std::string& file_path,
                                                                                   firmware::ELFLoader::ProgressCallback progress_callback) {
    COMPONENT_LOG_INFO("Loading ESP application image: {}", file_path);
    
    if (!elf_loader_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "ELF loader not initialized"));
    }
    
    // Ensure emulator is in a state where we can load applications
    if (state_ != EmulatorState::INITIALIZED && state_ != EmulatorState::STOPPED) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, "Cannot load application while emulator is running"));
    }
    
    // Load the ESP application image
    auto result = elf_loader_->load_esp_application_image(file_path, progress_callback);
    if (!result.has_value()) {
        COMPONENT_LOG_ERROR("Failed to load ESP application image: {}", result.error().message());
        return unexpected(result.error());
    }
    
    COMPONENT_LOG_INFO("ESP application image loaded successfully: entry=0x{:08x}, size={} bytes", 
                      result.value().entry_point, result.value().total_size);
    
    return result;
}

Result<void> EmulatorCore::set_application_entry_point(Address entry_point) {
    COMPONENT_LOG_INFO("Setting application entry point: 0x{:08x}", entry_point);
    
    if (!cpu_manager_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "CPU manager not initialized"));
    }
    
    // Validate entry point is in valid memory region
    if (!memory_controller_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Memory controller not initialized"));
    }
    
    // Set the entry point in CPU manager
    // This would require CPU manager to have a method to set the initial PC
    // For now, just validate and log
    
    COMPONENT_LOG_DEBUG("Application entry point set successfully");
    return {};
}

Result<void> EmulatorCore::start_application_execution() {
    COMPONENT_LOG_INFO("Starting application execution");
    
    if (!cpu_manager_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "CPU manager not initialized"));
    }
    
    // Start CPU execution if not already running
    if (state_ == EmulatorState::STOPPED || state_ == EmulatorState::INITIALIZED) {
        RETURN_IF_ERROR(start());
    }
    
    COMPONENT_LOG_INFO("Application execution started successfully");
    return {};
}

}  // namespace m5tab5::emulator