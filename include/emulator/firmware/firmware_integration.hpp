#pragma once

#include "emulator/firmware/firmware_loader.hpp"
#include "emulator/firmware/boot_sequencer.hpp"
#include "emulator/firmware/elf_parser.hpp"
#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <vector>

// Forward declarations
namespace m5tab5::emulator {
    class EmulatorCore;
}

namespace m5tab5::emulator::firmware {

/**
 * @brief Complete firmware lifecycle status
 */
enum class FirmwareStatus {
    UNLOADED,           // No firmware loaded
    LOADING,            // Firmware loading in progress
    LOADED,             // Firmware loaded but not booted
    BOOTING,            // Boot sequence in progress
    RUNNING,            // Firmware running successfully
    PAUSED,             // Execution paused
    ERROR,              // Error state
    RESETTING           // Reset in progress
};

/**
 * @brief Firmware operation result with detailed information
 */
struct FirmwareOperationResult {
    bool success;
    std::string operation;
    std::string message;
    FirmwareStatus final_status;
    std::chrono::milliseconds duration;
    
    // Optional detailed information
    ValidationResult validation_info;
    BootStatistics boot_stats;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
};

/**
 * @brief Firmware lifecycle event callback
 * 
 * @param status Current firmware status
 * @param progress Operation progress (0.0 - 1.0)
 * @param message Human-readable status message
 * @param operation_result Optional detailed result for completed operations
 */
using FirmwareEventCallback = std::function<void(
    FirmwareStatus status, 
    float progress, 
    const std::string& message,
    const FirmwareOperationResult* operation_result
)>;

/**
 * @brief Professional firmware integration layer
 * 
 * Provides a unified, high-level API for firmware management:
 * - Complete firmware lifecycle management
 * - Integrated loading, validation, and boot sequence
 * - Comprehensive error handling and recovery
 * - Event-driven status updates
 * - Performance monitoring and profiling
 * - Thread-safe operations
 */
class FirmwareIntegration {
public:
    explicit FirmwareIntegration(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core);
    ~FirmwareIntegration();

    // Lifecycle management
    Result<void> initialize();
    void shutdown();

    // Firmware operations
    Result<FirmwareOperationResult> load_and_boot_firmware(const std::string& file_path);
    Result<FirmwareOperationResult> load_firmware(const std::string& file_path);
    Result<FirmwareOperationResult> boot_firmware();
    Result<FirmwareOperationResult> reset_firmware(bool hard_reset = false);
    Result<FirmwareOperationResult> unload_firmware();
    
    // Async operations
    Result<void> load_and_boot_firmware_async(const std::string& file_path, FirmwareEventCallback callback);
    Result<void> load_firmware_async(const std::string& file_path, FirmwareEventCallback callback);
    void cancel_operation();
    
    // State management
    Result<void> pause_execution();
    Result<void> resume_execution();
    Result<void> step_execution(u32 instruction_count = 1);
    
    // Status and monitoring
    FirmwareStatus get_status() const { return current_status_; }
    bool is_firmware_loaded() const { return current_status_ >= FirmwareStatus::LOADED; }
    bool is_firmware_running() const { return current_status_ == FirmwareStatus::RUNNING; }
    bool is_operation_in_progress() const;
    
    // Information access
    Result<ValidationResult> get_firmware_info() const;
    Result<BootStatistics> get_boot_statistics() const;
    Result<std::string> get_loaded_firmware_path() const;
    std::vector<std::string> get_recent_warnings() const;
    std::vector<std::string> get_recent_errors() const;
    
    // Configuration
    void set_boot_configuration(const BootConfiguration& config);
    BootConfiguration get_boot_configuration() const;
    
    void set_loading_configuration(bool strict_validation, bool security_checks, bool dual_core);
    void set_event_callback(FirmwareEventCallback callback) { event_callback_ = callback; }
    
    // Validation and analysis
    Result<ValidationResult> validate_firmware_file(const std::string& file_path);
    Result<std::vector<std::string>> analyze_firmware_dependencies(const std::string& file_path);
    Result<size_t> estimate_memory_requirements(const std::string& file_path);

private:
    // Core components
    std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core_;
    std::unique_ptr<FirmwareLoader> firmware_loader_;
    std::unique_ptr<BootSequencer> boot_sequencer_;
    
    // State management
    std::atomic<FirmwareStatus> current_status_{FirmwareStatus::UNLOADED};
    std::mutex operation_mutex_;
    std::atomic<bool> operation_in_progress_{false};
    std::atomic<bool> cancel_requested_{false};
    
    // Current operation tracking
    std::unique_ptr<std::thread> operation_thread_;
    FirmwareEventCallback event_callback_;
    std::string current_firmware_path_;
    
    // Statistics and history
    std::vector<FirmwareOperationResult> operation_history_;
    std::vector<std::string> recent_warnings_;
    std::vector<std::string> recent_errors_;
    static constexpr size_t MAX_HISTORY_SIZE = 10;
    static constexpr size_t MAX_MESSAGE_HISTORY = 50;
    
    // Configuration
    BootConfiguration boot_config_;
    bool strict_validation_ = true;
    bool security_checks_enabled_ = true;
    bool dual_core_loading_ = true;
    
    // Internal operation methods
    Result<FirmwareOperationResult> execute_load_operation(const std::string& file_path);
    Result<FirmwareOperationResult> execute_boot_operation();
    Result<FirmwareOperationResult> execute_reset_operation(bool hard_reset);
    Result<FirmwareOperationResult> execute_unload_operation();
    
    // Async operation management
    void execute_async_operation(std::function<Result<FirmwareOperationResult>()> operation,
                                const std::string& operation_name,
                                FirmwareEventCallback callback);
    
    // Progress and event management
    void update_status(FirmwareStatus new_status);
    void emit_event(FirmwareStatus status, float progress, const std::string& message,
                   const FirmwareOperationResult* result = nullptr);
    void emit_progress_event(float progress, const std::string& message);
    void emit_completion_event(const FirmwareOperationResult& result);
    
    // History and logging
    void add_operation_to_history(const FirmwareOperationResult& result);
    void add_warning(const std::string& warning);
    void add_error(const std::string& error);
    void cleanup_message_history();
    
    // Progress callback bridges
    void handle_loader_progress(const std::string& stage, float progress, 
                               const std::string& message, bool success);
    void handle_boot_progress(BootStage stage, float progress, 
                             const std::string& message, bool success);
    
    // Status conversion helpers
    static std::string status_to_string(FirmwareStatus status);
    static float normalize_progress(float loader_progress, float boot_progress, 
                                   bool is_loading_phase);
    
    // Validation helpers
    bool validate_firmware_path(const std::string& file_path) const;
    bool validate_operation_preconditions(FirmwareStatus required_status) const;
    
    // Cleanup and error handling
    void cleanup_operation_state();
    void handle_operation_error(const std::string& operation, const Error& error);
    FirmwareOperationResult create_error_result(const std::string& operation, const Error& error);
    FirmwareOperationResult create_success_result(const std::string& operation, 
                                                  std::chrono::milliseconds duration);
};

/**
 * @brief Firmware management factory for easy integration
 */
class FirmwareManagerFactory {
public:
    static Result<std::unique_ptr<FirmwareIntegration>> 
    create_firmware_manager(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core);
    
    static BootConfiguration create_default_boot_config();
    static BootConfiguration create_fast_boot_config();
    static BootConfiguration create_debug_boot_config();
};

/**
 * @brief RAII firmware session for automatic cleanup
 */
class FirmwareSession {
public:
    explicit FirmwareSession(std::unique_ptr<FirmwareIntegration> integration);
    ~FirmwareSession();
    
    // Non-copyable, movable
    FirmwareSession(const FirmwareSession&) = delete;
    FirmwareSession& operator=(const FirmwareSession&) = delete;
    FirmwareSession(FirmwareSession&&) = default;
    FirmwareSession& operator=(FirmwareSession&&) = default;
    
    // Access to firmware integration
    FirmwareIntegration& get_integration() { return *integration_; }
    const FirmwareIntegration& get_integration() const { return *integration_; }
    
    // Convenience methods
    Result<void> load_and_run(const std::string& file_path);
    void wait_for_completion();
    bool is_successful() const;

private:
    std::unique_ptr<FirmwareIntegration> integration_;
    bool session_active_ = true;
};

} // namespace m5tab5::emulator::firmware