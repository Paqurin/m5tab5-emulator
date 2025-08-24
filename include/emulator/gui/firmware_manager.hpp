#pragma once

#include "emulator/firmware/firmware_loader.hpp"
#include "emulator/gui/emulator_gui.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

// Forward declarations
class MainWindow;
class ProgressDialog;
class FirmwareInfoDialog;

/**
 * @brief GUI integration layer for firmware loading system
 * 
 * Provides:
 * - Drag-and-drop ELF file handling
 * - Visual progress feedback during loading
 * - Firmware validation error display
 * - Real-time loading status updates
 * - User interaction for firmware switching
 * - Integration with EmulatorGUI lifecycle
 */
class FirmwareManager {
public:
    struct FirmwareState {
        bool loaded = false;
        std::string file_path;
        std::string file_name;
        firmware::ValidationResult info;
        std::chrono::steady_clock::time_point load_time;
        
        bool is_valid() const {
            return loaded && info.valid;
        }
        
        std::string get_display_name() const {
            if (file_name.empty()) return "No firmware loaded";
            return file_name;
        }
    };

    explicit FirmwareManager(EmulatorGUI& gui);
    ~FirmwareManager();

    // Lifecycle
    Result<void> initialize();
    void update();
    void render();
    void shutdown();

    // Drag-and-drop interface
#ifndef NO_GRAPHICS
    void handle_file_dialog_result(const std::string& file_path);
    void handle_drop_event(int event_type, const char* file_path = nullptr);
#endif

    // User actions
    void open_firmware_dialog();
    void reload_current_firmware();
    void unload_firmware();
    void show_firmware_info();

    // State queries
    const FirmwareState& get_current_state() const { return current_state_; }
    bool is_loading() const;
    bool can_load_firmware() const;

    // Integration with main window
    void register_menu_actions();
    void register_toolbar_actions();
    void update_ui_state();

    // Progress and feedback
    void show_loading_progress(bool show);
    void update_loading_progress(const std::string& stage, float progress, const std::string& message, bool success);
    void show_error_dialog(const std::string& title, const std::string& message, const std::vector<std::string>& details = {});
    void show_success_notification(const std::string& message);

private:
    EmulatorGUI& gui_;
    std::shared_ptr<firmware::FirmwareLoader> firmware_loader_;
    
    // Current state
    FirmwareState current_state_;
    
    // UI components
    std::unique_ptr<ProgressDialog> progress_dialog_;
    std::unique_ptr<FirmwareInfoDialog> info_dialog_;
    
    // Loading state
    bool show_progress_dialog_;
    std::string current_loading_stage_;
    float current_loading_progress_;
    std::string current_loading_message_;
    
    // Recent files list
    std::vector<std::string> recent_firmware_files_;
    static constexpr size_t MAX_RECENT_FILES = 10;

    // Internal methods
    void handle_firmware_load_request(const std::string& file_path);
    void handle_firmware_validation_result(const firmware::ValidationResult& result);
    void handle_firmware_load_complete(bool success, const std::string& error_message);
    void handle_firmware_unload_complete();

    // Progress callback for firmware loader
    void firmware_progress_callback(const std::string& stage, float progress, const std::string& message, bool success);

    // UI helpers
    void update_window_title();
    void update_status_bar();
    void add_to_recent_files(const std::string& file_path);
    void remove_from_recent_files(const std::string& file_path);

    // File validation
    bool is_valid_firmware_file(const std::string& file_path) const;
    std::string extract_filename(const std::string& file_path) const;

    // Configuration persistence
    void save_recent_files();
    void load_recent_files();
};

/**
 * @brief Progress dialog for firmware loading operations
 * 
 * Shows:
 * - Current loading stage
 * - Progress bar with percentage
 * - Detailed status messages
 * - Cancel button for async operations
 * - Error display on failure
 */
class ProgressDialog {
public:
    explicit ProgressDialog(FirmwareManager& parent);
    ~ProgressDialog();

    void show(const std::string& title = "Loading Firmware...");
    void hide();
    void render();
    
    bool is_visible() const { return visible_; }
    bool is_cancel_requested() const { return cancel_requested_; }
    
    // Progress updates
    void set_stage(const std::string& stage);
    void set_progress(float progress); // 0.0 - 1.0
    void set_message(const std::string& message);
    void set_success(bool success);
    void set_error(const std::string& error_message);

    // Dialog configuration
    void set_cancellable(bool cancellable) { cancellable_ = cancellable; }
    void set_auto_close_on_success(bool auto_close) { auto_close_on_success_ = auto_close; }

private:
    FirmwareManager& parent_;
    
    // Dialog state
    bool visible_;
    bool cancel_requested_;
    bool cancellable_;
    bool auto_close_on_success_;
    
    // Progress information
    std::string title_;
    std::string current_stage_;
    float progress_value_;
    std::string status_message_;
    bool has_error_;
    std::string error_message_;
    
    // Timing for auto-close
    std::chrono::steady_clock::time_point completion_time_;
    static constexpr std::chrono::milliseconds AUTO_CLOSE_DELAY{2000};

    void render_progress_bar();
    void render_stage_info();
    void render_buttons();
    void handle_cancel_button();
    void handle_close_button();
};

/**
 * @brief Firmware information dialog
 * 
 * Displays comprehensive firmware details:
 * - Binary information (architecture, size, entry point)
 * - Memory layout and requirements  
 * - ESP32-P4 compatibility information
 * - Validation warnings and errors
 * - Loading statistics and performance
 */
class FirmwareInfoDialog {
public:
    explicit FirmwareInfoDialog(FirmwareManager& parent);
    ~FirmwareInfoDialog();

    void show(const firmware::ValidationResult& firmware_info);
    void hide();
    void render();
    
    bool is_visible() const { return visible_; }

private:
    FirmwareManager& parent_;
    bool visible_;
    firmware::ValidationResult firmware_info_;

    void render_basic_info();
    void render_memory_layout();
    void render_compatibility_info();
    void render_warnings_and_errors();
    void render_close_button();
    
    // Helper methods
    std::string format_address(Address address) const;
    std::string format_size(size_t size) const;
    std::string format_architecture(const std::string& arch) const;
    u32 get_info_color(bool is_error, bool is_warning) const;
};

/**
 * @brief Drag-and-drop overlay for firmware loading
 * 
 * Visual feedback during drag-and-drop operations:
 * - Overlay highlight when dragging files over window
 * - File type validation feedback
 * - Drop zone indication
 * - Animated feedback on successful drop
 */
class DragDropOverlay {
public:
    explicit DragDropOverlay(FirmwareManager& parent);
    ~DragDropOverlay();

    void show(bool is_valid_file);
    void hide();
    void render();
    
    bool is_visible() const { return visible_; }
    
    // Animation control
    void animate_success();
    void animate_error();

private:
    FirmwareManager& parent_;
    bool visible_;
    bool is_valid_drop_;
    
    // Animation state
    enum class AnimationState {
        NONE,
        SUCCESS,
        ERROR
    };
    AnimationState animation_state_;
    float animation_progress_;
    std::chrono::steady_clock::time_point animation_start_time_;
    
    static constexpr std::chrono::milliseconds ANIMATION_DURATION{1000};

    void update_animation();
    void render_drop_zone();
    void render_file_validation_feedback();
    void render_animation_effect();
    
    u32 get_overlay_color() const;
    std::string get_overlay_message() const;
};

/**
 * @brief File dialog wrapper for firmware selection
 * 
 * Platform-specific file dialog integration:
 * - Native file browser on supported platforms
 * - Filter for ELF and binary files
 * - Recent locations memory
 * - Multiple file format support
 */
class FirmwareFileDialog {
public:
    explicit FirmwareFileDialog(FirmwareManager& parent);
    ~FirmwareFileDialog();

    // File selection
    void show_open_dialog();
    bool is_dialog_open() const { return dialog_open_; }
    
    // Configuration
    void set_initial_directory(const std::string& directory);
    void add_file_filter(const std::string& description, const std::vector<std::string>& extensions);

    // Result handling
    using FileSelectedCallback = std::function<void(const std::string& file_path)>;
    void set_file_selected_callback(FileSelectedCallback callback) { file_selected_callback_ = callback; }

private:
    FirmwareManager& parent_;
    bool dialog_open_;
    std::string initial_directory_;
    
    struct FileFilter {
        std::string description;
        std::vector<std::string> extensions;
    };
    std::vector<FileFilter> file_filters_;
    
    FileSelectedCallback file_selected_callback_;

    void setup_default_filters();
    void handle_file_selection(const std::string& file_path);
    
    // Platform-specific implementations
#ifdef _WIN32
    void show_windows_dialog();
#elif defined(__linux__)
    void show_linux_dialog();
#elif defined(__APPLE__)
    void show_macos_dialog();
#else
    void show_fallback_dialog();
#endif
};

} // namespace m5tab5::emulator::gui