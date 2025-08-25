#include "emulator/firmware/firmware_integration.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"

#include <thread>
#include <chrono>

using m5tab5::emulator::unexpected;

namespace m5tab5::emulator::firmware {

//
// FirmwareIntegration Implementation
//

FirmwareIntegration::FirmwareIntegration(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core)
    : emulator_core_(emulator_core) {
    
    // Initialize sub-components
    firmware_loader_ = std::make_unique<FirmwareLoader>();
    boot_sequencer_ = std::make_unique<BootSequencer>();
    
    // Set default configuration
    boot_config_ = BootConfiguration{};
    
    LOG_DEBUG("FirmwareIntegration created with emulator core");
}

FirmwareIntegration::~FirmwareIntegration() {
    shutdown();
}

Result<void> FirmwareIntegration::initialize() {
    LOG_INFO("Initializing Firmware Integration system...");
    
    // Initialize firmware loader
    auto loader_result = firmware_loader_->initialize();
    if (!loader_result.has_value()) {
        add_error("Failed to initialize firmware loader: " + loader_result.error().message());
        return unexpected(loader_result.error());
    }
    
    // Set dependencies for firmware loader
    if (emulator_core_) {
        auto memory_controller = emulator_core_->getComponent<::m5tab5::emulator::MemoryController>();
        if (memory_controller) {
            firmware_loader_->set_memory_controller(memory_controller);
        }
        
        auto cpu_manager = emulator_core_->getComponent<::m5tab5::emulator::DualCoreManager>();
        if (cpu_manager) {
            firmware_loader_->set_cpu_manager(cpu_manager);
        }
        
        firmware_loader_->set_emulator_core(emulator_core_);
    }
    
    // Initialize boot sequencer
    auto boot_result = boot_sequencer_->initialize(boot_config_);
    if (!boot_result.has_value()) {
        add_error("Failed to initialize boot sequencer: " + boot_result.error().message());
        return unexpected(boot_result.error());
    }
    
    // Set dependencies for boot sequencer  
    if (emulator_core_) {
        boot_sequencer_->set_emulator_core(emulator_core_);
        
        auto memory_controller = emulator_core_->getComponent<::m5tab5::emulator::MemoryController>();
        if (memory_controller) {
            boot_sequencer_->set_memory_controller(memory_controller);
        }
        
        auto cpu_manager = emulator_core_->getComponent<::m5tab5::emulator::DualCoreManager>();
        if (cpu_manager) {
            boot_sequencer_->set_cpu_manager(cpu_manager);
        }
    }
    
    update_status(FirmwareStatus::UNLOADED);
    
    LOG_INFO("Firmware Integration system initialized successfully");
    return {};
}

void FirmwareIntegration::shutdown() {
    LOG_INFO("Shutting down Firmware Integration system...");
    
    // Cancel any ongoing operations
    cancel_operation();
    
    // Wait for operation thread to complete
    if (operation_thread_ && operation_thread_->joinable()) {
        operation_thread_->join();
    }
    
    // Shutdown components
    if (boot_sequencer_) {
        boot_sequencer_->shutdown();
    }
    
    if (firmware_loader_) {
        firmware_loader_->shutdown();
    }
    
    update_status(FirmwareStatus::UNLOADED);
    cleanup_operation_state();
    
    LOG_INFO("Firmware Integration system shut down");
}

Result<FirmwareOperationResult> FirmwareIntegration::load_and_boot_firmware(const std::string& file_path) {
    // LOG_INFO("Loading and booting firmware: {}", file_path); // Temporarily disabled
    
    if (!validate_firmware_path(file_path)) {
        return unexpected(Error{ErrorCode::CONFIG_FILE_NOT_FOUND, "Invalid firmware file path"});
    }
    
    if (operation_in_progress_.load()) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Another operation is in progress"});
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Execute load operation
    auto load_result = execute_load_operation(file_path);
    if (!load_result.has_value()) {
        return unexpected(load_result.error());
    }
    
    // Execute boot operation
    auto boot_result = execute_boot_operation();
    if (!boot_result.has_value()) {
        return unexpected(boot_result.error());
    }
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    return create_success_result("load_and_boot", duration);
}

Result<FirmwareOperationResult> FirmwareIntegration::load_firmware(const std::string& file_path) {
    // LOG_INFO("Loading firmware: {}", file_path); // Temporarily disabled
    
    if (!validate_firmware_path(file_path)) {
        return unexpected(Error{ErrorCode::CONFIG_FILE_NOT_FOUND, "Invalid firmware file path"});
    }
    
    if (operation_in_progress_.load()) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Another operation is in progress"});
    }
    
    return execute_load_operation(file_path);
}

Result<FirmwareOperationResult> FirmwareIntegration::boot_firmware() {
    LOG_INFO("Booting loaded firmware");
    
    if (!validate_operation_preconditions(FirmwareStatus::LOADED)) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No firmware loaded to boot"});
    }
    
    return execute_boot_operation();
}

Result<FirmwareOperationResult> FirmwareIntegration::reset_firmware(bool hard_reset) {
    // LOG_INFO("Resetting firmware (hard_reset: {})", hard_reset); // Temporarily disabled
    
    if (!validate_operation_preconditions(FirmwareStatus::LOADED)) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No firmware loaded to reset"});
    }
    
    return execute_reset_operation(hard_reset);
}

Result<FirmwareOperationResult> FirmwareIntegration::unload_firmware() {
    LOG_INFO("Unloading firmware");
    
    if (current_status_ == FirmwareStatus::UNLOADED) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No firmware loaded to unload"});
    }
    
    return execute_unload_operation();
}

// Async operations
Result<void> FirmwareIntegration::load_and_boot_firmware_async(const std::string& file_path, FirmwareEventCallback callback) {
    if (operation_in_progress_.load()) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Another operation is in progress"});
    }
    
    operation_in_progress_.store(true);
    cancel_requested_.store(false);
    
    auto operation = [this, file_path]() -> Result<FirmwareOperationResult> {
        return load_and_boot_firmware(file_path);
    };
    
    execute_async_operation(operation, "load_and_boot", callback);
    return {};
}

Result<void> FirmwareIntegration::load_firmware_async(const std::string& file_path, FirmwareEventCallback callback) {
    if (operation_in_progress_.load()) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Another operation is in progress"});
    }
    
    operation_in_progress_.store(true);
    cancel_requested_.store(false);
    
    auto operation = [this, file_path]() -> Result<FirmwareOperationResult> {
        return load_firmware(file_path);
    };
    
    execute_async_operation(operation, "load", callback);
    return {};
}

void FirmwareIntegration::cancel_operation() {
    cancel_requested_.store(true);
    
    // Cancel sub-component operations
    if (firmware_loader_) {
        firmware_loader_->cancel_loading();
    }
    
    if (boot_sequencer_) {
        boot_sequencer_->abort_boot();
    }
}

// State management
Result<void> FirmwareIntegration::pause_execution() {
    if (current_status_ != FirmwareStatus::RUNNING) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware is not running"});
    }
    
    // TODO: Implement CPU pause functionality
    update_status(FirmwareStatus::PAUSED);
    emit_event(FirmwareStatus::PAUSED, 1.0f, "Firmware execution paused");
    
    return {};
}

Result<void> FirmwareIntegration::resume_execution() {
    if (current_status_ != FirmwareStatus::PAUSED) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware is not paused"});
    }
    
    // TODO: Implement CPU resume functionality
    update_status(FirmwareStatus::RUNNING);
    emit_event(FirmwareStatus::RUNNING, 1.0f, "Firmware execution resumed");
    
    return {};
}

Result<void> FirmwareIntegration::step_execution(u32 instruction_count) {
    if (current_status_ != FirmwareStatus::PAUSED) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware must be paused for single-step execution"});
    }
    
    // TODO: Implement single-step execution
    emit_event(FirmwareStatus::PAUSED, 1.0f, 
              "Executed " + std::to_string(instruction_count) + " instructions");
    
    return {};
}

bool FirmwareIntegration::is_operation_in_progress() const {
    return operation_in_progress_.load();
}

// Information access
Result<ValidationResult> FirmwareIntegration::get_firmware_info() const {
    if (!firmware_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware loader not initialized"});
    }
    
    return ValidationResult{firmware_loader_->get_firmware_info()};
}

Result<BootStatistics> FirmwareIntegration::get_boot_statistics() const {
    if (!boot_sequencer_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Boot sequencer not initialized"});
    }
    
    return boot_sequencer_->get_boot_statistics();
}

Result<std::string> FirmwareIntegration::get_loaded_firmware_path() const {
    if (!firmware_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware loader not initialized"});
    }
    
    return firmware_loader_->get_loaded_firmware_path();
}

std::vector<std::string> FirmwareIntegration::get_recent_warnings() const {
    return recent_warnings_;
}

std::vector<std::string> FirmwareIntegration::get_recent_errors() const {
    return recent_errors_;
}

// Configuration
void FirmwareIntegration::set_boot_configuration(const BootConfiguration& config) {
    boot_config_ = config;
    
    if (boot_sequencer_) {
        boot_sequencer_->set_boot_configuration(config);
    }
}

BootConfiguration FirmwareIntegration::get_boot_configuration() const {
    return boot_config_;
}

void FirmwareIntegration::set_loading_configuration(bool strict_validation, bool security_checks, bool dual_core) {
    strict_validation_ = strict_validation;
    security_checks_enabled_ = security_checks;
    dual_core_loading_ = dual_core;
    
    if (firmware_loader_) {
        firmware_loader_->set_validation_strict(strict_validation);
        firmware_loader_->set_security_checks_enabled(security_checks);
        firmware_loader_->set_dual_core_loading(dual_core);
    }
}

// Validation and analysis
Result<ValidationResult> FirmwareIntegration::validate_firmware_file(const std::string& file_path) {
    if (!firmware_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware loader not initialized"});
    }
    
    return firmware_loader_->validate_firmware(file_path);
}

Result<std::vector<std::string>> FirmwareIntegration::analyze_firmware_dependencies(const std::string& file_path) {
    // TODO: Implement dependency analysis
    std::vector<std::string> dependencies;
    dependencies.push_back("esp_system");
    dependencies.push_back("freertos");
    dependencies.push_back("driver");
    dependencies.push_back("esp_common");
    
    return dependencies;
}

Result<size_t> FirmwareIntegration::estimate_memory_requirements(const std::string& file_path) {
    auto validation_result = validate_firmware_file(file_path);
    if (!validation_result.has_value()) {
        return unexpected(validation_result.error());
    }
    
    const auto& info = validation_result.value();
    
    // Estimate total memory requirements
    size_t total_requirements = info.code_size + info.data_size + info.bss_size;
    
    // Add overhead for runtime stack, heap, etc.
    total_requirements += 64 * 1024; // 64KB overhead
    
    return total_requirements;
}

// Private implementation methods

Result<FirmwareOperationResult> FirmwareIntegration::execute_load_operation(const std::string& file_path) {
    auto start_time = std::chrono::steady_clock::now();
    
    update_status(FirmwareStatus::LOADING);
    
    // Load firmware using firmware loader
    auto load_result = firmware_loader_->load_firmware(file_path, 
        [this](const std::string& stage, float progress, const std::string& message, bool success) {
            emit_progress_event(progress * 0.5f, "Loading: " + message);
        });
    
    if (!load_result.has_value()) {
        auto error = load_result.error();
        handle_operation_error("load", error);
        return unexpected(error);
    }
    
    current_firmware_path_ = file_path;
    update_status(FirmwareStatus::LOADED);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    return create_success_result("load", duration);
}

Result<FirmwareOperationResult> FirmwareIntegration::execute_boot_operation() {
    auto start_time = std::chrono::steady_clock::now();
    
    update_status(FirmwareStatus::BOOTING);
    
    // Get firmware info to determine entry point
    auto firmware_info = get_firmware_info();
    if (!firmware_info.has_value()) {
        return unexpected(firmware_info.error());
    }
    
    // Execute boot sequence
    auto boot_result = boot_sequencer_->execute_boot_sequence(firmware_info.value().entry_point,
        [this](BootStage stage, float progress, const std::string& message, bool success) {
            emit_progress_event(0.5f + progress * 0.5f, "Booting: " + message);
        });
    
    if (!boot_result.has_value()) {
        auto error = boot_result.error();
        handle_operation_error("boot", error);
        return unexpected(error);
    }
    
    update_status(FirmwareStatus::RUNNING);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    return create_success_result("boot", duration);
}

Result<FirmwareOperationResult> FirmwareIntegration::execute_reset_operation(bool hard_reset) {
    auto start_time = std::chrono::steady_clock::now();
    
    update_status(FirmwareStatus::RESETTING);
    
    Result<void> reset_result;
    if (hard_reset) {
        reset_result = boot_sequencer_->execute_hard_reset();
    } else {
        reset_result = boot_sequencer_->execute_soft_reset();
    }
    
    if (!reset_result.has_value()) {
        auto error = reset_result.error();
        handle_operation_error("reset", error);
        return unexpected(error);
    }
    
    update_status(FirmwareStatus::LOADED);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    return create_success_result("reset", duration);
}

Result<FirmwareOperationResult> FirmwareIntegration::execute_unload_operation() {
    auto start_time = std::chrono::steady_clock::now();
    
    // Unload firmware
    auto unload_result = firmware_loader_->unload_firmware();
    if (!unload_result.has_value()) {
        auto error = unload_result.error();
        handle_operation_error("unload", error);
        return unexpected(error);
    }
    
    current_firmware_path_.clear();
    update_status(FirmwareStatus::UNLOADED);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    return create_success_result("unload", duration);
}

// Async operation management
void FirmwareIntegration::execute_async_operation(std::function<Result<FirmwareOperationResult>()> operation,
                                                  const std::string& operation_name,
                                                  FirmwareEventCallback callback) {
    
    operation_thread_ = std::make_unique<std::thread>([this, operation, operation_name, callback]() {
        emit_event(FirmwareStatus::LOADING, 0.0f, "Starting " + operation_name + " operation");
        
        auto result = operation();
        
        if (result.has_value()) {
            emit_completion_event(result.value());
        } else {
            auto error_result = create_error_result(operation_name, result.error());
            emit_completion_event(error_result);
        }
        
        cleanup_operation_state();
    });
}

// Progress and event management
void FirmwareIntegration::update_status(FirmwareStatus new_status) {
    current_status_.store(new_status);
}

void FirmwareIntegration::emit_event(FirmwareStatus status, float progress, const std::string& message,
                                   const FirmwareOperationResult* result) {
    if (event_callback_) {
        event_callback_(status, progress, message, result);
    }
}

void FirmwareIntegration::emit_progress_event(float progress, const std::string& message) {
    emit_event(current_status_.load(), progress, message);
}

void FirmwareIntegration::emit_completion_event(const FirmwareOperationResult& result) {
    emit_event(result.final_status, 1.0f, result.message, &result);
}

// History and logging
void FirmwareIntegration::add_operation_to_history(const FirmwareOperationResult& result) {
    operation_history_.push_back(result);
    
    // Limit history size
    if (operation_history_.size() > MAX_HISTORY_SIZE) {
        operation_history_.erase(operation_history_.begin());
    }
}

void FirmwareIntegration::add_warning(const std::string& warning) {
    recent_warnings_.push_back(warning);
    cleanup_message_history();
    // LOG_WARN("Firmware Integration: {}", warning); // Temporarily disabled
}

void FirmwareIntegration::add_error(const std::string& error) {
    recent_errors_.push_back(error);
    cleanup_message_history();
    // LOG_ERROR("Firmware Integration: {}", error); // Temporarily disabled
}

void FirmwareIntegration::cleanup_message_history() {
    if (recent_warnings_.size() > MAX_MESSAGE_HISTORY) {
        recent_warnings_.erase(recent_warnings_.begin());
    }
    if (recent_errors_.size() > MAX_MESSAGE_HISTORY) {
        recent_errors_.erase(recent_errors_.begin());
    }
}

// Status conversion helpers
std::string FirmwareIntegration::status_to_string(FirmwareStatus status) {
    switch (status) {
        case FirmwareStatus::UNLOADED: return "unloaded";
        case FirmwareStatus::LOADING: return "loading";
        case FirmwareStatus::LOADED: return "loaded";
        case FirmwareStatus::BOOTING: return "booting";
        case FirmwareStatus::RUNNING: return "running";
        case FirmwareStatus::PAUSED: return "paused";
        case FirmwareStatus::ERROR: return "error";
        case FirmwareStatus::RESETTING: return "resetting";
        default: return "unknown";
    }
}

// Validation helpers
bool FirmwareIntegration::validate_firmware_path(const std::string& file_path) const {
    return !file_path.empty() && file_path.size() > 4;
}

bool FirmwareIntegration::validate_operation_preconditions(FirmwareStatus required_status) const {
    return current_status_.load() >= required_status;
}

// Cleanup and error handling
void FirmwareIntegration::cleanup_operation_state() {
    operation_in_progress_.store(false);
    cancel_requested_.store(false);
}

void FirmwareIntegration::handle_operation_error(const std::string& operation, const Error& error) {
    add_error("Operation '" + operation + "' failed: " + error.message());
    update_status(FirmwareStatus::ERROR);
}

FirmwareOperationResult FirmwareIntegration::create_error_result(const std::string& operation, const Error& error) {
    return FirmwareOperationResult{
        .success = false,
        .operation = operation,
        .message = error.message(),
        .final_status = FirmwareStatus::ERROR,
        .duration = std::chrono::milliseconds(0),
        .validation_info = ValidationResult{},
        .boot_stats = BootStatistics{},
        .warnings = recent_warnings_,
        .errors = recent_errors_
    };
}

FirmwareOperationResult FirmwareIntegration::create_success_result(const std::string& operation, 
                                                                  std::chrono::milliseconds duration) {
    return FirmwareOperationResult{
        .success = true,
        .operation = operation,
        .message = "Operation completed successfully",
        .final_status = current_status_.load(),
        .duration = duration,
        .validation_info = get_firmware_info().has_value() ? get_firmware_info().value() : ValidationResult{},
        .boot_stats = get_boot_statistics().has_value() ? get_boot_statistics().value() : BootStatistics{},
        .warnings = recent_warnings_,
        .errors = recent_errors_
    };
}

//
// FirmwareManagerFactory Implementation
//

Result<std::unique_ptr<FirmwareIntegration>> 
FirmwareManagerFactory::create_firmware_manager(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core) {
    
    auto integration = std::make_unique<FirmwareIntegration>(emulator_core);
    
    auto init_result = integration->initialize();
    if (!init_result.has_value()) {
        return unexpected(init_result.error());
    }
    
    return integration;
}

BootConfiguration FirmwareManagerFactory::create_default_boot_config() {
    return BootConfiguration{
        .dual_core_enabled = true,
        .cpu_frequency_mhz = 400,
        .enable_psram = true,
        .flash_size_mb = 16,
        .psram_size_mb = 32,
        .boot_delay = std::chrono::milliseconds(100),
        .watchdog_timeout = std::chrono::seconds(30),
        .enable_boot_logging = true
    };
}

BootConfiguration FirmwareManagerFactory::create_fast_boot_config() {
    auto config = create_default_boot_config();
    config.boot_delay = std::chrono::milliseconds(10);
    config.enable_boot_logging = false;
    return config;
}

BootConfiguration FirmwareManagerFactory::create_debug_boot_config() {
    auto config = create_default_boot_config();
    config.boot_delay = std::chrono::milliseconds(500);
    config.enable_boot_logging = true;
    config.watchdog_timeout = std::chrono::minutes(5);
    return config;
}

//
// FirmwareSession Implementation
//

FirmwareSession::FirmwareSession(std::unique_ptr<FirmwareIntegration> integration)
    : integration_(std::move(integration)) {
}

FirmwareSession::~FirmwareSession() {
    if (session_active_ && integration_) {
        integration_->shutdown();
    }
}

Result<void> FirmwareSession::load_and_run(const std::string& file_path) {
    if (!integration_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Integration not available"});
    }
    
    auto result = integration_->load_and_boot_firmware(file_path);
    return result.has_value() ? Result<void>{} : unexpected(result.error());
}

void FirmwareSession::wait_for_completion() {
    if (!integration_) return;
    
    // Wait for any ongoing operation to complete
    while (integration_->is_operation_in_progress()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool FirmwareSession::is_successful() const {
    return integration_ && integration_->is_firmware_running();
}

} // namespace m5tab5::emulator::firmware