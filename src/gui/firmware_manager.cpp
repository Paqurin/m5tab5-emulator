#include "emulator/gui/firmware_manager.hpp"
#include "emulator/gui/emulator_gui.hpp"
#include "emulator/gui/main_window.hpp"
#include "emulator/firmware/firmware_integration.hpp"

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <chrono>

using m5tab5::emulator::unexpected;

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

//
// FirmwareManager Implementation
//

FirmwareManager::FirmwareManager(EmulatorGUI& gui)
    : gui_(gui)
    , show_progress_dialog_(false)
    , current_loading_progress_(0.0f)
{
}

FirmwareManager::~FirmwareManager() {
    shutdown();
}

Result<void> FirmwareManager::initialize() {
    // Initialize firmware integration with emulator core dependencies
    auto emulator_core = gui_.get_emulator_core();
    if (!emulator_core) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "EmulatorCore not available"});
    }

    // Create firmware integration system using factory
    auto emulator_core_shared = std::shared_ptr<EmulatorCore>(emulator_core, [](EmulatorCore*){});
    auto integration_result = firmware::FirmwareManagerFactory::create_firmware_manager(emulator_core_shared);
    if (!integration_result.has_value()) {
        return unexpected(integration_result.error());
    }
    
    firmware_integration_ = std::move(integration_result.value());

    // Set up event callbacks for GUI updates
    firmware_integration_->set_event_callback(
        [this](firmware::FirmwareStatus status, float progress, const std::string& message, 
               const firmware::FirmwareOperationResult* result) {
            handle_firmware_event(status, progress, message, result);
        }
    );

    // Initialize UI components
    progress_dialog_ = std::make_unique<ProgressDialog>(*this);
    info_dialog_ = std::make_unique<FirmwareInfoDialog>(*this);

    // Load recent files and configuration
    load_recent_files();

    return {};
}

void FirmwareManager::update() {
    // Update loading state
    if (is_loading()) {
        // Check firmware integration status
        auto status = firmware_integration_->get_status();
        
        // Check if loading completed
        if (status == firmware::FirmwareStatus::LOADED) {
            handle_firmware_load_complete(true, "");
        } else if (status == firmware::FirmwareStatus::ERROR) {
            handle_firmware_load_complete(false, "Firmware loading failed");
        }
    }

    // Update UI components
    if (progress_dialog_ && progress_dialog_->is_visible()) {
        progress_dialog_->render();
        
        // Handle cancel request
        if (progress_dialog_->is_cancel_requested() && is_loading()) {
            firmware_integration_->cancel_operation();
        }
    }
}

void FirmwareManager::render() {
    // Render overlay dialogs
    if (progress_dialog_ && progress_dialog_->is_visible()) {
        progress_dialog_->render();
    }
    
    if (info_dialog_ && info_dialog_->is_visible()) {
        info_dialog_->render();
    }
}

void FirmwareManager::shutdown() {
    if (firmware_integration_) {
        firmware_integration_->shutdown();
    }
    
    save_recent_files();
    
    progress_dialog_.reset();
    info_dialog_.reset();
}

void FirmwareManager::handle_drop_event(int event_type, const char* file_path) {
    // SDL drop event handling - simplified interface for build compatibility
    if (file_path && is_valid_firmware_file(std::string(file_path))) {
        handle_firmware_load_request(std::string(file_path));
    } else if (file_path) {
        show_error_dialog("Invalid File", 
                        "The dropped file is not a valid firmware file.",
                        {"Supported formats: .elf, .bin"});
    }
}

void FirmwareManager::open_firmware_dialog() {
    // For now, implement a simple file path dialog
    // In a real implementation, this would use platform-specific file dialogs
    
    // Show a simple input dialog for file path
    // This is a placeholder - real implementation would use native file dialogs
    std::string file_path;
    
    // Example: Use environment variable or config for test firmware path
    const char* test_firmware = std::getenv("M5TAB5_TEST_FIRMWARE");
    if (test_firmware) {
        file_path = test_firmware;
    }
    
    if (!file_path.empty() && is_valid_firmware_file(file_path)) {
        handle_firmware_load_request(file_path);
    }
}

void FirmwareManager::reload_current_firmware() {
    if (!current_state_.loaded) {
        show_error_dialog("No Firmware Loaded", "No firmware is currently loaded to reload.");
        return;
    }
    
    handle_firmware_load_request(current_state_.file_path);
}

void FirmwareManager::unload_firmware() {
    if (!current_state_.loaded) {
        return;
    }
    
    auto result = firmware_integration_->unload_firmware();
    if (result.has_value()) {
        handle_firmware_unload_complete();
        show_success_notification("Firmware unloaded successfully");
    } else {
        show_error_dialog("Unload Failed", "Failed to unload firmware");
    }
}

void FirmwareManager::show_firmware_info() {
    if (!current_state_.loaded) {
        show_error_dialog("No Firmware Loaded", "No firmware is currently loaded to show information for.");
        return;
    }
    
    if (info_dialog_) {
        info_dialog_->show(current_state_.info);
    }
}

bool FirmwareManager::is_loading() const {
    return firmware_integration_ && firmware_integration_->is_operation_in_progress();
}

bool FirmwareManager::can_load_firmware() const {
    return firmware_integration_ && !is_loading() && gui_.get_emulator_core();
}

void FirmwareManager::handle_firmware_load_request(const std::string& file_path) {
    if (!can_load_firmware()) {
        show_error_dialog("Cannot Load Firmware", "Emulator is not ready or another operation is in progress.");
        return;
    }
    
    // Show progress dialog
    if (progress_dialog_) {
        progress_dialog_->show("Loading Firmware...");
        progress_dialog_->set_cancellable(true);
        progress_dialog_->set_auto_close_on_success(true);
    }
    
    // Start async loading
    auto result = firmware_integration_->load_firmware_async(file_path, 
        [this](firmware::FirmwareStatus status, float progress, const std::string& message, 
               const firmware::FirmwareOperationResult* result) {
            handle_firmware_event(status, progress, message, result);
        }
    );
    if (!result.has_value()) {
        if (progress_dialog_) {
            progress_dialog_->hide();
        }
        std::vector<std::string> details = {result.error().message()};
        show_error_dialog("Load Failed", "Failed to start firmware loading", details);
    }
}

void FirmwareManager::handle_firmware_load_complete(bool success, const std::string& error_message) {
    if (progress_dialog_) {
        if (success) {
            progress_dialog_->set_progress(1.0f);
            progress_dialog_->set_message("Firmware loaded successfully!");
            progress_dialog_->set_success(true);
        } else {
            progress_dialog_->set_error(error_message);
        }
    }
    
    if (success) {
        // Update current state
        current_state_.loaded = true;
        auto file_path_result = firmware_integration_->get_loaded_firmware_path();
        if (file_path_result.has_value()) {
            current_state_.file_path = file_path_result.value();
            current_state_.file_name = extract_filename(current_state_.file_path);
        }
        
        auto info_result = firmware_integration_->get_firmware_info();
        if (info_result.has_value()) {
            current_state_.info = info_result.value();
        }
        current_state_.load_time = std::chrono::steady_clock::now();
        
        // Add to recent files
        add_to_recent_files(current_state_.file_path);
        
        // Update UI
        update_ui_state();
        
        // Trigger celebration
        gui_.unlock_achievement(EmulatorGUI::Achievement::FIRST_FIRMWARE_LOAD);
        show_success_notification("Firmware loaded: " + current_state_.file_name);
    } else {
        current_state_.loaded = false;
        show_error_dialog("Firmware Load Failed", error_message);
    }
}

void FirmwareManager::handle_firmware_unload_complete() {
    current_state_ = FirmwareState{}; // Reset to default state
    update_ui_state();
}

void FirmwareManager::firmware_progress_callback(const std::string& stage, float progress, const std::string& message, bool success) {
    current_loading_stage_ = stage;
    current_loading_progress_ = progress;
    current_loading_message_ = message;
    
    if (progress_dialog_) {
        progress_dialog_->set_stage(stage);
        progress_dialog_->set_progress(progress);
        progress_dialog_->set_message(message);
        
        if (!success) {
            progress_dialog_->set_error(message);
        }
    }
}

void FirmwareManager::update_ui_state() {
    update_window_title();
    update_status_bar();
}

void FirmwareManager::update_window_title() {
    // Update main window title with firmware info
    std::string title = "M5Stack Tab5 Emulator";
    if (current_state_.loaded) {
        title += " - " + current_state_.get_display_name();
    }
    // Title update would be implemented through MainWindow interface
}

void FirmwareManager::update_status_bar() {
    // Update status bar with firmware info
    // This would be implemented through MainWindow::StatusBar interface
}

void FirmwareManager::add_to_recent_files(const std::string& file_path) {
    // Remove if already exists
    auto it = std::find(recent_firmware_files_.begin(), recent_firmware_files_.end(), file_path);
    if (it != recent_firmware_files_.end()) {
        recent_firmware_files_.erase(it);
    }
    
    // Add to front
    recent_firmware_files_.insert(recent_firmware_files_.begin(), file_path);
    
    // Limit size
    if (recent_firmware_files_.size() > MAX_RECENT_FILES) {
        recent_firmware_files_.resize(MAX_RECENT_FILES);
    }
}

void FirmwareManager::remove_from_recent_files(const std::string& file_path) {
    auto it = std::find(recent_firmware_files_.begin(), recent_firmware_files_.end(), file_path);
    if (it != recent_firmware_files_.end()) {
        recent_firmware_files_.erase(it);
    }
}

bool FirmwareManager::is_valid_firmware_file(const std::string& file_path) const {
    if (!std::filesystem::exists(file_path)) {
        return false;
    }
    
    // Check file extension
    std::filesystem::path path(file_path);
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    return ext == ".elf" || ext == ".bin";
}

std::string FirmwareManager::extract_filename(const std::string& file_path) const {
    std::filesystem::path path(file_path);
    return path.filename().string();
}

void FirmwareManager::save_recent_files() {
    // Save recent files to configuration
    // This would integrate with the GUI configuration system
}

void FirmwareManager::load_recent_files() {
    // Load recent files from configuration
    // This would integrate with the GUI configuration system
}

void FirmwareManager::show_error_dialog(const std::string& title, const std::string& message, const std::vector<std::string>& details) {
    // Show error dialog - would integrate with GUI dialog system
    // For now, just log the error
    printf("ERROR: %s - %s\n", title.c_str(), message.c_str());
    for (const auto& detail : details) {
        printf("  Detail: %s\n", detail.c_str());
    }
}

void FirmwareManager::show_success_notification(const std::string& message) {
    // Show success notification - would integrate with GUI notification system
    printf("SUCCESS: %s\n", message.c_str());
}

//
// ProgressDialog Implementation
//

ProgressDialog::ProgressDialog(FirmwareManager& parent)
    : parent_(parent)
    , visible_(false)
    , cancel_requested_(false)
    , cancellable_(true)
    , auto_close_on_success_(false)
    , progress_value_(0.0f)
    , has_error_(false)
{
}

ProgressDialog::~ProgressDialog() = default;

void ProgressDialog::show(const std::string& title) {
    title_ = title;
    visible_ = true;
    cancel_requested_ = false;
    has_error_ = false;
    progress_value_ = 0.0f;
    current_stage_ = "";
    status_message_ = "";
    error_message_ = "";
}

void ProgressDialog::hide() {
    visible_ = false;
}

void ProgressDialog::render() {
    if (!visible_) return;
    
    // Render progress dialog
    // This would use ImGui or custom SDL2 rendering
    // For now, just print progress to console
    printf("Progress: %s - %.1f%% - %s - %s\n", 
           current_stage_.c_str(), 
           progress_value_ * 100.0f, 
           status_message_.c_str(),
           has_error_ ? error_message_.c_str() : "OK");
    
    // Auto-close on success
    if (auto_close_on_success_ && progress_value_ >= 1.0f && !has_error_) {
        auto now = std::chrono::steady_clock::now();
        if (completion_time_ == std::chrono::steady_clock::time_point{}) {
            completion_time_ = now;
        } else if (now - completion_time_ >= AUTO_CLOSE_DELAY) {
            hide();
        }
    }
}

void ProgressDialog::set_stage(const std::string& stage) {
    current_stage_ = stage;
}

void ProgressDialog::set_progress(float progress) {
    progress_value_ = std::clamp(progress, 0.0f, 1.0f);
}

void ProgressDialog::set_message(const std::string& message) {
    status_message_ = message;
}

void ProgressDialog::set_success(bool success) {
    if (success && progress_value_ >= 1.0f) {
        completion_time_ = std::chrono::steady_clock::now();
    }
}

void ProgressDialog::set_error(const std::string& error_message) {
    has_error_ = true;
    error_message_ = error_message;
}

//
// FirmwareInfoDialog Implementation
//

FirmwareInfoDialog::FirmwareInfoDialog(FirmwareManager& parent)
    : parent_(parent)
    , visible_(false)
{
}

FirmwareInfoDialog::~FirmwareInfoDialog() = default;

void FirmwareInfoDialog::show(const firmware::ValidationResult& firmware_info) {
    firmware_info_ = firmware_info;
    visible_ = true;
}

void FirmwareInfoDialog::hide() {
    visible_ = false;
}

void FirmwareInfoDialog::render() {
    if (!visible_) return;
    
    // Render firmware info dialog
    // This would use ImGui or custom SDL2 rendering
    printf("Firmware Info:\n");
    printf("  Valid: %s\n", firmware_info_.valid ? "Yes" : "No");
    printf("  Architecture: %s\n", firmware_info_.architecture.c_str());
    printf("  Target Chip: %s\n", firmware_info_.target_chip.c_str());
    printf("  Entry Point: 0x%08X\n", firmware_info_.entry_point);
    printf("  Code Size: %zu bytes\n", firmware_info_.code_size);
    printf("  Data Size: %zu bytes\n", firmware_info_.data_size);
    printf("  BSS Size: %zu bytes\n", firmware_info_.bss_size);
    printf("  Dual Core: %s\n", firmware_info_.has_dual_core_support ? "Yes" : "No");
    printf("  Uses PSRAM: %s\n", firmware_info_.uses_psram ? "Yes" : "No");
}

std::string FirmwareInfoDialog::format_address(Address address) const {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "0x%08X", address);
    return std::string(buffer);
}

std::string FirmwareInfoDialog::format_size(size_t size) const {
    if (size >= 1024 * 1024) {
        return std::to_string(size / (1024 * 1024)) + " MB";
    } else if (size >= 1024) {
        return std::to_string(size / 1024) + " KB";
    } else {
        return std::to_string(size) + " bytes";
    }
}

std::string FirmwareInfoDialog::format_architecture(const std::string& arch) const {
    if (arch == "riscv") return "RISC-V";
    if (arch == "riscv32") return "RISC-V 32-bit";
    if (arch == "riscv64") return "RISC-V 64-bit";
    return arch;
}

u32 FirmwareInfoDialog::get_info_color(bool is_error, bool is_warning) const {
    if (is_error) return 0xFF5722;    // Error red
    if (is_warning) return 0xFFC107;  // Warning yellow
    return 0x4CAF50;                  // Success green
}

} // namespace m5tab5::emulator::gui