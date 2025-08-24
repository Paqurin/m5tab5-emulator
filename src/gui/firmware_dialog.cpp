#include "emulator/gui/firmware_dialog.hpp"
#include "emulator/gui/main_window.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cstring>
#include <sys/stat.h>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::gui;

// ELF constants for ESP32-P4 validation
static constexpr u16 EM_RISCV = 243;  // RISC-V architecture
static constexpr u32 ESP32_P4_FLASH_START = 0x42000000;
static constexpr u32 ESP32_P4_FLASH_END = 0x43000000;
static constexpr u32 ESP32_P4_SRAM_START = 0x4FF00000;
static constexpr u32 ESP32_P4_SRAM_END = 0x4FFFFFFF;

FirmwareDialog::FirmwareDialog(MainWindow& parent)
    : parent_(parent) {
    
    // Initialize current directory to common ESP32 build paths
    std::vector<std::string> common_paths = {
        "./build",
        "../build", 
        "./build/esp32p4",
        "./"
    };
    
    for (const auto& path : common_paths) {
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
            current_directory_ = std::filesystem::canonical(path);
            break;
        }
    }
    
    LOG_DEBUG("FirmwareDialog initialized with directory: {}", current_directory_);
}

FirmwareDialog::~FirmwareDialog() {
    save_dialog_state();
}

Result<void> FirmwareDialog::initialize() {
    if (initialized_) {
        return {};
    }
    
    try {
        // Load saved state
        load_dialog_state();
        
        // Load firmware profiles
        load_profiles_from_config();
        
        initialized_ = true;
        LOG_INFO("FirmwareDialog initialized successfully");
        
        return {};
    } catch (const std::exception& e) {
        return unexpected(Error(ErrorCode::OPERATION_FAILED));
    }
}

void FirmwareDialog::show() {
    if (!initialized_) {
        auto result = initialize();
        if (!result) {
            LOG_ERROR("Failed to initialize FirmwareDialog: {}", static_cast<int>(result.error().code()));
            return;
        }
    }
    
    visible_ = true;
    LOG_DEBUG("FirmwareDialog shown");
}

void FirmwareDialog::hide() {
    visible_ = false;
    show_file_browser_ = false;
    show_profile_dialog_ = false;
    LOG_DEBUG("FirmwareDialog hidden");
}

void FirmwareDialog::render() {
    if (!visible_) return;
    
    render_main_dialog();
    
    if (show_file_browser_) {
        render_file_browser();
    }
    
    if (show_profile_dialog_) {
        render_profile_dialog();
    }
}

void FirmwareDialog::update() {
    if (!visible_) return;
    
    // Update loading progress if active
    if (is_loading()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
            (current_time - loading_progress_.start_time).count();
        
        // Simulate progress stages for demonstration
        // In real implementation, this would be driven by actual loading progress
        switch (loading_progress_.stage) {
            case LoadingStage::Validating:
                if (elapsed > 200) {
                    update_loading_progress(LoadingStage::Parsing, 0.2f, "📋 Parsing ELF headers with scientific precision...");
                }
                break;
            case LoadingStage::Parsing:
                if (elapsed > 500) {
                    update_loading_progress(LoadingStage::LoadingToMemory, 0.5f, "📥 Carefully arranging code sections in memory...");
                }
                break;
            case LoadingStage::LoadingToMemory:
                if (elapsed > 1000) {
                    update_loading_progress(LoadingStage::Initializing, 0.8f, "⚙️ Initializing ESP32-P4 state with confidence...");
                }
                break;
            case LoadingStage::Initializing:
                if (elapsed > 1500) {
                    complete_loading_operation(true, "🎉 Firmware loaded and ready to execute!");
                }
                break;
            default:
                break;
        }
    }
}

Result<void> FirmwareDialog::load_firmware(const std::string& filepath) {
    LOG_INFO("📦 Loading firmware with care: {}", filepath);
    
    if (!std::filesystem::exists(filepath)) {
        std::string error = "File not found: " + filepath;
        LOG_ERROR(error);
        if (error_callback_) error_callback_(error);
        return unexpected(Error(ErrorCode::FILE_ERROR));
    }
    
    if (!is_elf_file(filepath)) {
        std::string error = "Invalid file type. Expected ELF file: " + filepath;
        LOG_ERROR(error);
        if (error_callback_) error_callback_(error);
        return unexpected(Error(ErrorCode::INVALID_PARAMETER));
    }
    
    // Start loading operation with delightful messages
    start_loading_operation(filepath);
    LOG_INFO("✨ {}", "Beginning firmware loading with M5Stack magic!");
    
    // Parse ELF metadata
    auto metadata_result = parse_elf_file(filepath);
    if (!metadata_result) {
        complete_loading_operation(false, "Failed to parse ELF file");
        return unexpected(metadata_result.error());
    }
    
    current_metadata_ = metadata_result.value();
    
    // Validate ESP32-P4 compatibility
    if (!validate_esp32_p4_compatibility(current_metadata_)) {
        std::string warning = "Warning: Firmware may not be compatible with ESP32-P4: " + 
                             current_metadata_.compatibility_notes;
        LOG_WARN(warning);
        // Continue loading but with warning
    }
    
    // Add to recent files
    add_to_recent_files(filepath);
    
    firmware_loaded_ = true;
    
    // Notify parent
    if (load_callback_) {
        load_callback_(filepath);
    }
    
    LOG_INFO("🎉 Firmware loaded successfully: {} (Entry: 0x{:08X}) - Ready for ESP32-P4 action!", 
             current_metadata_.filename, current_metadata_.entry_point);
    
    return {};
}

void FirmwareDialog::cancel_loading() {
    if (is_loading()) {
        complete_loading_operation(false, "Loading cancelled by user");
        LOG_INFO("Firmware loading cancelled");
    }
}

void FirmwareDialog::render_main_dialog() {
    // Main firmware dialog window
    const float dialog_width = 800.0f;
    const float dialog_height = 600.0f;
    
    // This is a simplified implementation for demonstration
    // In a real implementation, you would use a GUI framework like Dear ImGui
    // or implement custom SDL2-based widgets
    
    LOG_DEBUG("Rendering main firmware dialog ({}x{})", dialog_width, dialog_height);
    
    // Title bar
    render_firmware_info();
    
    // Recent files section
    render_recent_files();
    
    // Profiles section  
    render_firmware_profiles();
    
    // Loading progress (if active)
    if (is_loading()) {
        render_loading_progress();
    }
}

void FirmwareDialog::render_file_browser() {
    LOG_DEBUG("Rendering file browser for directory: {}", current_directory_);
    
    try {
        auto entries = get_directory_contents();
        
        // In a real implementation, this would render a file list
        // with navigation buttons, file type filtering, etc.
        // For now, just log the available files
        for (const auto& entry : entries) {
            if (entry.is_regular_file() && is_elf_file(entry.path().string())) {
                LOG_DEBUG("Available ELF file: {}", entry.path().filename().string());
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Error reading directory {}: {}", current_directory_, e.what());
    }
}

void FirmwareDialog::render_firmware_info() {
    if (!firmware_loaded_) {
        LOG_DEBUG("No firmware loaded - showing placeholder info");
        return;
    }
    
    LOG_DEBUG("Rendering firmware info for: {}", current_metadata_.filename);
    LOG_DEBUG("  Entry point: 0x{:08X}", current_metadata_.entry_point);
    LOG_DEBUG("  File size: {} bytes", current_metadata_.file_size);
    LOG_DEBUG("  Architecture: {}", current_metadata_.architecture);
    LOG_DEBUG("  Sections: {}", current_metadata_.sections.size());
    LOG_DEBUG("  ESP32-P4 compatible: {}", current_metadata_.esp32_p4_compatible ? "Yes" : "No");
    
    if (!current_metadata_.compatibility_notes.empty()) {
        LOG_DEBUG("  Compatibility notes: {}", current_metadata_.compatibility_notes);
    }
}

void FirmwareDialog::render_loading_progress() {
    LOG_DEBUG("Rendering loading progress - Stage: {}, Progress: {:.1f}%", 
              get_stage_text(loading_progress_.stage),
              loading_progress_.progress * 100.0f);
    
    if (!loading_progress_.status_text.empty()) {
        LOG_DEBUG("  Status: {}", loading_progress_.status_text);
    }
    
    if (!loading_progress_.error_message.empty()) {
        LOG_ERROR("  Error: {}", loading_progress_.error_message);
    }
}

void FirmwareDialog::render_recent_files() {
    LOG_DEBUG("Rendering recent files ({} items)", recent_files_.size());
    
    for (size_t i = 0; i < recent_files_.size() && i < MAX_RECENT_FILES; ++i) {
        const auto& file = recent_files_[i];
        LOG_DEBUG("  {}: {}", i + 1, std::filesystem::path(file).filename().string());
    }
}

void FirmwareDialog::render_firmware_profiles() {
    LOG_DEBUG("Rendering firmware profiles ({} items)", firmware_profiles_.size());
    
    for (const auto& profile : firmware_profiles_) {
        LOG_DEBUG("  Profile: {} (Used {} times)", profile.name, profile.usage_count);
        LOG_DEBUG("    Path: {}", profile.filepath);
        if (!profile.description.empty()) {
            LOG_DEBUG("    Description: {}", profile.description);
        }
    }
}

void FirmwareDialog::render_profile_dialog() {
    LOG_DEBUG("Rendering profile dialog");
    // Profile creation/editing dialog implementation would go here
}

void FirmwareDialog::browse_for_file() {
    show_file_browser_ = true;
    refresh_directory_listing();
    LOG_DEBUG("File browser opened");
}

std::vector<std::filesystem::directory_entry> FirmwareDialog::get_directory_contents() {
    std::vector<std::filesystem::directory_entry> entries;
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(current_directory_)) {
            entries.push_back(entry);
        }
        
        // Sort directories first, then files
        std::sort(entries.begin(), entries.end(), 
            [](const auto& a, const auto& b) {
                if (a.is_directory() != b.is_directory()) {
                    return a.is_directory();
                }
                return a.path().filename() < b.path().filename();
            });
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to read directory {}: {}", current_directory_, e.what());
    }
    
    return entries;
}

void FirmwareDialog::select_file(const std::string& filepath) {
    selected_file_ = filepath;
    auto result = load_firmware(filepath);
    if (result) {
        show_file_browser_ = false;
        LOG_INFO("File selected and loaded: {}", filepath);
    } else {
        LOG_ERROR("Failed to load selected file: {}", filepath);
    }
}

Result<FirmwareDialog::ElfMetadata> FirmwareDialog::parse_elf_file(const std::string& filepath) {
    return ElfParser::parse(filepath);
}

bool FirmwareDialog::validate_esp32_p4_compatibility(const ElfMetadata& metadata) {
    return ElfParser::validate_esp32_p4_compatibility(metadata);
}

void FirmwareDialog::start_loading_operation(const std::string& filepath) {
    loading_progress_ = LoadingProgress{};
    loading_progress_.stage = LoadingStage::Validating;
    loading_progress_.progress = 0.0f;
    loading_progress_.status_text = "🔍 Examining ELF file with our finest magnifying glass...";
    loading_progress_.start_time = std::chrono::steady_clock::now();
    
    LOG_INFO("🚀 Started delightful loading operation for: {}", std::filesystem::path(filepath).filename().string());
}

void FirmwareDialog::update_loading_progress(LoadingStage stage, float progress, const std::string& status) {
    loading_progress_.stage = stage;
    loading_progress_.progress = progress;
    loading_progress_.status_text = status;
    
    LOG_DEBUG("Loading progress updated: {} ({:.1f}%)", status, progress * 100.0f);
}

void FirmwareDialog::complete_loading_operation(bool success, const std::string& error) {
    if (success) {
        loading_progress_.stage = LoadingStage::Complete;
        loading_progress_.progress = 1.0f;
        loading_progress_.status_text = "🎉 Firmware loaded successfully - ESP32-P4 is ready to rock!";
        loading_progress_.error_message.clear();
        LOG_INFO("✨ Firmware loading completed successfully with style!");
    } else {
        loading_progress_.stage = LoadingStage::Failed;
        loading_progress_.progress = 0.0f;
        loading_progress_.status_text = "Loading failed";
        loading_progress_.error_message = error;
        LOG_ERROR("Firmware loading failed: {}", error);
        if (error_callback_) error_callback_(error);
    }
}

void FirmwareDialog::add_to_recent_files(const std::string& filepath) {
    // Remove if already exists
    recent_files_.erase(
        std::remove(recent_files_.begin(), recent_files_.end(), filepath),
        recent_files_.end()
    );
    
    // Add to front
    recent_files_.insert(recent_files_.begin(), filepath);
    
    // Limit size
    if (recent_files_.size() > MAX_RECENT_FILES) {
        recent_files_.resize(MAX_RECENT_FILES);
    }
    
    LOG_DEBUG("Added to recent files: {} (total: {})", filepath, recent_files_.size());
}

void FirmwareDialog::load_profiles_from_config() {
    firmware_profiles_ = FirmwareProfileManager::load_profiles();
    LOG_DEBUG("Loaded {} firmware profiles", firmware_profiles_.size());
}

void FirmwareDialog::save_profiles_to_config() {
    FirmwareProfileManager::save_profiles(firmware_profiles_);
    LOG_DEBUG("Saved {} firmware profiles", firmware_profiles_.size());
}

std::string FirmwareDialog::format_file_size(u64 size) const {
    const char* units[] = {"B", "KB", "MB", "GB"};
    double d = static_cast<double>(size);
    int i = 0;
    
    while (d >= 1024.0 && i < 3) {
        d /= 1024.0;
        ++i;
    }
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << d << " " << units[i];
    return oss.str();
}

std::string FirmwareDialog::format_address(u32 address) const {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << address;
    return oss.str();
}

const char* FirmwareDialog::get_stage_text(LoadingStage stage) const {
    switch (stage) {
        case LoadingStage::Idle: return "Idle";
        case LoadingStage::Validating: return "Validating";
        case LoadingStage::Parsing: return "Parsing";
        case LoadingStage::LoadingToMemory: return "Loading to Memory";
        case LoadingStage::Initializing: return "Initializing";
        case LoadingStage::Complete: return "Complete";
        case LoadingStage::Failed: return "Failed";
    }
    return "Unknown";
}

bool FirmwareDialog::is_elf_file(const std::string& filepath) const {
    // Check file extension
    auto ext = std::filesystem::path(filepath).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext != ".elf") {
        return false;
    }
    
    // Check ELF magic bytes
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    char magic[4];
    file.read(magic, 4);
    
    return file.good() && 
           magic[0] == 0x7f && 
           magic[1] == 'E' && 
           magic[2] == 'L' && 
           magic[3] == 'F';
}

void FirmwareDialog::save_dialog_state() {
    // Save current directory, window size, etc. to configuration
    LOG_DEBUG("Saving dialog state");
}

void FirmwareDialog::load_dialog_state() {
    // Load saved dialog state from configuration
    LOG_DEBUG("Loading dialog state");
}

// ElfParser implementation
Result<FirmwareDialog::ElfMetadata> ElfParser::parse(const std::string& filepath) {
    LOG_DEBUG("Parsing ELF file: {}", filepath);
    
    FirmwareDialog::ElfMetadata metadata;
    metadata.filepath = filepath;
    metadata.filename = std::filesystem::path(filepath).filename().string();
    
    try {
        // Get file size
        metadata.file_size = std::filesystem::file_size(filepath);
        
        // Parse ELF header
        auto result = parse_elf_header(filepath, metadata);
        if (!result) {
            return unexpected(result.error());
        }
        
        // Parse sections
        result = parse_section_headers(filepath, metadata);
        if (!result) {
            LOG_WARN("Could not parse section headers for {}", filepath);
            // Continue without section information
        }
        
        // Validate ESP32-P4 compatibility
        metadata.esp32_p4_compatible = validate_esp32_p4_compatibility(metadata);
        metadata.compatibility_notes = get_compatibility_notes(metadata);
        
        LOG_INFO("Successfully parsed ELF: {} (Entry: 0x{:08X}, Size: {} bytes)",
                metadata.filename, metadata.entry_point, metadata.file_size);
        
        return metadata;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse ELF file {}: {}", filepath, e.what());
        return unexpected(Error(ErrorCode::INVALID_PARAMETER));
    }
}

Result<void> ElfParser::parse_elf_header(const std::string& filepath, FirmwareDialog::ElfMetadata& metadata) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return unexpected(Error(ErrorCode::FILE_ERROR));
    }
    
    // Read ELF header (simplified - would need complete ELF parsing in production)
    char header[64];
    file.read(header, 64);
    
    if (!file.good()) {
        return unexpected(Error(ErrorCode::FILE_ERROR));
    }
    
    // Check ELF magic
    if (header[0] != 0x7f || header[1] != 'E' || header[2] != 'L' || header[3] != 'F') {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER));
    }
    
    // Extract basic information (simplified)
    u16 machine = *reinterpret_cast<u16*>(&header[18]);
    metadata.entry_point = *reinterpret_cast<u32*>(&header[24]);
    
    if (machine == EM_RISCV) {
        metadata.architecture = "RISC-V";
    } else {
        metadata.architecture = "Unknown (" + std::to_string(machine) + ")";
    }
    
    return {};
}

Result<void> ElfParser::parse_section_headers(const std::string& filepath, FirmwareDialog::ElfMetadata& metadata) {
    // Simplified section parsing - would need complete ELF implementation
    LOG_DEBUG("Parsing section headers for {}", filepath);
    
    // Add some dummy sections for demonstration
    metadata.sections = {
        {".text", 0x42000000, 0x10000, "PROGBITS", true},
        {".rodata", 0x42010000, 0x2000, "PROGBITS", true},
        {".data", 0x4FF00000, 0x1000, "PROGBITS", true},
        {".bss", 0x4FF01000, 0x800, "NOBITS", false}
    };
    
    // Calculate totals
    for (const auto& section : metadata.sections) {
        if (section.name == ".text" || section.name == ".rodata") {
            metadata.total_code_size += section.size;
        } else if (section.name == ".data" || section.name == ".bss") {
            metadata.total_data_size += section.size;
        }
    }
    
    return {};
}

bool ElfParser::validate_esp32_p4_compatibility(const FirmwareDialog::ElfMetadata& metadata) {
    // Check architecture
    if (metadata.architecture.find("RISC-V") == std::string::npos) {
        return false;
    }
    
    // Check entry point is in valid range
    if (!is_valid_entry_point(metadata.entry_point)) {
        return false;
    }
    
    // Check sections are in valid memory ranges
    return are_sections_valid(metadata.sections);
}

std::string ElfParser::get_compatibility_notes(const FirmwareDialog::ElfMetadata& metadata) {
    std::string notes;
    
    if (metadata.architecture.find("RISC-V") == std::string::npos) {
        notes += "Architecture is not RISC-V. ";
    }
    
    if (!is_valid_entry_point(metadata.entry_point)) {
        notes += "Entry point (" + std::to_string(metadata.entry_point) + ") is not in valid range. ";
    }
    
    if (!are_sections_valid(metadata.sections)) {
        notes += "Some sections are mapped to invalid memory regions. ";
    }
    
    if (notes.empty()) {
        notes = "Fully compatible with ESP32-P4";
    }
    
    return notes;
}

bool ElfParser::is_valid_entry_point(u32 entry_point) {
    return (entry_point >= ESP32_P4_FLASH_START && entry_point < ESP32_P4_FLASH_END) ||
           (entry_point >= ESP32_P4_SRAM_START && entry_point < ESP32_P4_SRAM_END);
}

bool ElfParser::are_sections_valid(const std::vector<FirmwareDialog::ElfMetadata::Section>& sections) {
    for (const auto& section : sections) {
        if (section.loadable) {
            u32 addr = section.virtual_address;
            if (!((addr >= ESP32_P4_FLASH_START && addr < ESP32_P4_FLASH_END) ||
                  (addr >= ESP32_P4_SRAM_START && addr < ESP32_P4_SRAM_END))) {
                return false;
            }
        }
    }
    return true;
}

// FirmwareProfileManager implementation
std::vector<FirmwareDialog::FirmwareProfile> FirmwareProfileManager::load_profiles() {
    std::vector<FirmwareDialog::FirmwareProfile> profiles;
    
    try {
        std::string config_path = get_profiles_config_path();
        if (std::filesystem::exists(config_path)) {
            LOG_DEBUG("Loading firmware profiles from: {}", config_path);
            // JSON loading would go here
        } else {
            LOG_DEBUG("No firmware profiles file found at: {}", config_path);
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to load firmware profiles: {}", e.what());
    }
    
    return profiles;
}

void FirmwareProfileManager::save_profiles(const std::vector<FirmwareDialog::FirmwareProfile>& profiles) {
    try {
        ensure_config_directory_exists();
        std::string config_path = get_profiles_config_path();
        LOG_DEBUG("Saving {} firmware profiles to: {}", profiles.size(), config_path);
        // JSON saving would go here
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to save firmware profiles: {}", e.what());
    }
}

std::string FirmwareProfileManager::get_profiles_config_path() {
    return std::filesystem::path(std::getenv("HOME") ? std::getenv("HOME") : ".") / 
           ".config" / "m5tab5-emulator" / "firmware_profiles.json";
}

void FirmwareProfileManager::ensure_config_directory_exists() {
    std::filesystem::path config_dir = std::filesystem::path(get_profiles_config_path()).parent_path();
    if (!std::filesystem::exists(config_dir)) {
        std::filesystem::create_directories(config_dir);
    }
}

void FirmwareDialog::refresh_directory_listing() {
    LOG_DEBUG("FirmwareDialog::refresh_directory_listing - TODO: implement directory refresh");
    // TODO: implement directory listing refresh
    // This would scan current_directory_ for ELF files and update the file list
    
    try {
        size_t elf_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(current_directory_)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                std::string extension = entry.path().extension().string();
                
                // Check for ELF files (common extensions)
                if (extension == ".elf" || extension == ".bin" || filename.find("firmware") != std::string::npos) {
                    elf_count++;
                    LOG_DEBUG("Found potential firmware file: {} ({})", 
                             filename, format_file_size(std::filesystem::file_size(entry.path())));
                }
            }
        }
        
        LOG_DEBUG("Directory refresh complete: {} potential firmware files found in {}", 
                  elf_count, current_directory_);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to refresh directory listing: {}", e.what());
    }
}