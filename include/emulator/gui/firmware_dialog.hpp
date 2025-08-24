#pragma once

#include "emulator/gui/emulator_gui.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <filesystem>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

class MainWindow;

/**
 * @brief Professional firmware loading dialog with ESP32-P4 ELF support
 * 
 * Features:
 * - Native file browser for .elf files
 * - ELF metadata parsing and validation
 * - Progress indication for loading operations
 * - Recent firmware history management
 * - Firmware profile system for development workflows
 * - ESP32-P4 compatibility verification
 */
class FirmwareDialog {
public:
    enum class LoadingStage {
        Idle,
        Validating,
        Parsing,
        LoadingToMemory,
        Initializing,
        Complete,
        Failed
    };

    struct ElfMetadata {
        std::string filepath;
        std::string filename;
        u64 file_size = 0;
        u32 entry_point = 0;
        std::string architecture;
        std::string build_id;
        std::chrono::system_clock::time_point build_time;
        
        struct Section {
            std::string name;
            u32 virtual_address = 0;
            u32 size = 0;
            std::string type;
            bool loadable = false;
        };
        
        std::vector<Section> sections;
        u64 total_code_size = 0;
        u64 total_data_size = 0;
        bool esp32_p4_compatible = false;
        std::string compatibility_notes;
    };

    struct FirmwareProfile {
        std::string name;
        std::string filepath;
        std::string description;
        std::chrono::system_clock::time_point last_used;
        u32 usage_count = 0;
        ElfMetadata metadata;
    };

    struct LoadingProgress {
        LoadingStage stage = LoadingStage::Idle;
        float progress = 0.0f;
        std::string status_text;
        std::string error_message;
        bool can_cancel = true;
        std::chrono::steady_clock::time_point start_time;
    };

    explicit FirmwareDialog(MainWindow& parent);
    ~FirmwareDialog();

    // Dialog management
    Result<void> initialize();
    void show();
    void hide();
    bool is_visible() const { return visible_; }
    
    // Main interface
    void render();
    void update();
    
    // Firmware operations
    Result<void> load_firmware(const std::string& filepath);
    void cancel_loading();
    bool is_loading() const { return loading_progress_.stage != LoadingStage::Idle && 
                                     loading_progress_.stage != LoadingStage::Complete && 
                                     loading_progress_.stage != LoadingStage::Failed; }
    
    // Profile management
    void save_profile(const std::string& name, const std::string& description = "");
    void delete_profile(const std::string& name);
    void load_profile(const std::string& name);
    const std::vector<FirmwareProfile>& get_profiles() const { return firmware_profiles_; }
    
    // Recent files
    const std::vector<std::string>& get_recent_files() const { return recent_files_; }
    void clear_recent_files();
    
    // Current firmware info
    const ElfMetadata& get_current_metadata() const { return current_metadata_; }
    bool has_firmware_loaded() const { return firmware_loaded_; }
    
    // Callbacks for parent window
    void set_load_callback(std::function<void(const std::string&)> callback) { 
        load_callback_ = callback; 
    }
    void set_error_callback(std::function<void(const std::string&)> callback) { 
        error_callback_ = callback; 
    }

private:
    MainWindow& parent_;
    bool visible_ = false;
    bool initialized_ = false;
    
    // Dialog state
    bool show_file_browser_ = false;
    bool show_metadata_details_ = false;
    bool show_profile_dialog_ = false;
    std::string current_directory_ = ".";
    std::string selected_file_;
    std::string file_filter_ = ".elf";
    
    // Firmware data
    ElfMetadata current_metadata_;
    bool firmware_loaded_ = false;
    LoadingProgress loading_progress_;
    
    // Profile and history management
    std::vector<FirmwareProfile> firmware_profiles_;
    std::vector<std::string> recent_files_;
    static constexpr size_t MAX_RECENT_FILES = 10;
    std::string profile_name_buffer_[256] = {};
    std::string profile_desc_buffer_[512] = {};
    
    // Callbacks
    std::function<void(const std::string&)> load_callback_;
    std::function<void(const std::string&)> error_callback_;
    
    // UI rendering methods
    void render_main_dialog();
    void render_file_browser();
    void render_firmware_info();
    void render_loading_progress();
    void render_recent_files();
    void render_firmware_profiles();
    void render_profile_dialog();
    
    // File operations
    void browse_for_file();
    void refresh_directory_listing();
    std::vector<std::filesystem::directory_entry> get_directory_contents();
    void select_file(const std::string& filepath);
    void navigate_to_directory(const std::string& path);
    
    // ELF processing
    Result<ElfMetadata> parse_elf_file(const std::string& filepath);
    bool validate_esp32_p4_compatibility(const ElfMetadata& metadata);
    std::string get_architecture_from_elf(const std::string& filepath);
    std::vector<ElfMetadata::Section> parse_elf_sections(const std::string& filepath);
    
    // Profile management
    void load_profiles_from_config();
    void save_profiles_to_config();
    void add_to_recent_files(const std::string& filepath);
    void sort_profiles_by_usage();
    
    // Progress management
    void start_loading_operation(const std::string& filepath);
    void update_loading_progress(LoadingStage stage, float progress, const std::string& status);
    void complete_loading_operation(bool success, const std::string& error = "");
    
    // Utility methods
    std::string format_file_size(u64 size) const;
    std::string format_address(u32 address) const;
    std::string format_time(const std::chrono::system_clock::time_point& time) const;
    u32 get_stage_color(LoadingStage stage) const;
    const char* get_stage_text(LoadingStage stage) const;
    bool is_elf_file(const std::string& filepath) const;
    
    // Configuration persistence
    std::string get_config_file_path() const;
    void save_dialog_state();
    void load_dialog_state();
};

/**
 * @brief ELF file parser for ESP32-P4 firmware analysis
 * 
 * This class handles the low-level parsing of ELF files specifically
 * for ESP32-P4 firmware, extracting metadata and validating compatibility.
 */
class ElfParser {
public:
    static Result<FirmwareDialog::ElfMetadata> parse(const std::string& filepath);
    static bool validate_esp32_p4_compatibility(const FirmwareDialog::ElfMetadata& metadata);
    static std::string get_compatibility_notes(const FirmwareDialog::ElfMetadata& metadata);
    
private:
    // ELF header parsing
    static Result<void> parse_elf_header(const std::string& filepath, FirmwareDialog::ElfMetadata& metadata);
    static Result<void> parse_program_headers(const std::string& filepath, FirmwareDialog::ElfMetadata& metadata);
    static Result<void> parse_section_headers(const std::string& filepath, FirmwareDialog::ElfMetadata& metadata);
    
    // ESP32-P4 specific validation
    static bool is_risc_v_architecture(const std::string& arch);
    static bool is_valid_entry_point(u32 entry_point);
    static bool are_sections_valid(const std::vector<FirmwareDialog::ElfMetadata::Section>& sections);
    
    // Utility methods
    static std::string read_string_from_file(const std::string& filepath, size_t offset, size_t max_length = 256);
    static u32 read_u32_from_file(const std::string& filepath, size_t offset);
    static u16 read_u16_from_file(const std::string& filepath, size_t offset);
};

/**
 * @brief Firmware profile manager for saving and loading development configurations
 */
class FirmwareProfileManager {
public:
    static std::vector<FirmwareDialog::FirmwareProfile> load_profiles();
    static void save_profiles(const std::vector<FirmwareDialog::FirmwareProfile>& profiles);
    static void add_profile(const FirmwareDialog::FirmwareProfile& profile);
    static void remove_profile(const std::string& name);
    static void update_profile_usage(const std::string& name);
    
private:
    static std::string get_profiles_config_path();
    static void ensure_config_directory_exists();
};

} // namespace m5tab5::emulator::gui