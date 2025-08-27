#include "emulator/firmware/firmware_loader.hpp"
#include "emulator/firmware/elf_parser.hpp"
#include "emulator/firmware/elf_loader.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/memory/boot_rom.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"

using m5tab5::emulator::unexpected;

#include <fstream>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstring>

namespace m5tab5::emulator::firmware {

//
// FirmwareLoader Implementation
//

FirmwareLoader::FirmwareLoader() {
    // Initialize ELF loader
    elf_loader_ = std::make_unique<ELFLoader>();
}

FirmwareLoader::~FirmwareLoader() {
    shutdown();
}

Result<void> FirmwareLoader::initialize() {
    // Initialize internal components
    boot_loader_ = std::make_unique<BootLoader>();
    auto boot_result = boot_loader_->initialize();
    if (!boot_result.has_value()) {
        return unexpected(boot_result.error());
    }

    // Initialize ELF loader
    if (!elf_loader_) {
        elf_loader_ = std::make_unique<ELFLoader>();
    }
    auto elf_result = elf_loader_->initialize();
    if (!elf_result.has_value()) {
        return unexpected(elf_result.error());
    }

    // Configure boot loader
    boot_loader_->set_dual_core_enabled(dual_core_loading_);

    loading_state_.store(LoadingState::IDLE);
    cancel_requested_.store(false);

    return {};
}

void FirmwareLoader::shutdown() {
    // Cancel any ongoing loading
    cancel_loading();
    
    // Wait for loading thread to complete
    if (loading_thread_ && loading_thread_->joinable()) {
        loading_thread_->join();
    }
    
    // Cleanup
    parsed_elf_data_.reset();
    boot_loader_.reset();
    backup_state_.reset();
    loading_thread_.reset();
    segment_mappings_.clear();
    
    // Shutdown ELF loader
    if (elf_loader_) {
        elf_loader_->shutdown();
        elf_loader_.reset();
    }
}

Result<ValidationResult> FirmwareLoader::validate_firmware(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) {
        return unexpected(Error{ErrorCode::CONFIG_FILE_NOT_FOUND, "Firmware file not found: " + file_path});
    }

    return validate_elf_binary(file_path);
}

Result<void> FirmwareLoader::load_firmware(const std::string& file_path, ProgressCallback progress_callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (loading_state_.load() != LoadingState::IDLE) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware loading already in progress"});
    }

    return load_firmware_internal(file_path, progress_callback);
}

Result<void> FirmwareLoader::load_firmware_async(const std::string& file_path, ProgressCallback progress_callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (loading_state_.load() != LoadingState::IDLE) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware loading already in progress"});
    }

    // Start loading in background thread
    loading_thread_ = std::make_unique<std::thread>([this, file_path, progress_callback]() {
        auto result = load_firmware_internal(file_path, progress_callback);
        if (!result.has_value()) {
            loading_state_.store(LoadingState::FAILED);
        }
    });

    return {};
}

void FirmwareLoader::cancel_loading() {
    cancel_requested_.store(true);
    
    // Wait for cancellation to complete
    if (loading_thread_ && loading_thread_->joinable()) {
        loading_thread_->join();
        loading_thread_.reset();
    }
    
    loading_state_.store(LoadingState::IDLE);
    cancel_requested_.store(false);
}

Result<void> FirmwareLoader::unload_firmware() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!firmware_loaded_) {
        return {}; // Already unloaded
    }

    // Clear memory regions
    auto clear_result = clear_memory_regions();
    if (!clear_result.has_value()) {
        return unexpected(clear_result.error());
    }

    // Reset state
    parsed_elf_data_.reset();
    segment_mappings_.clear();
    firmware_loaded_ = false;
    loaded_firmware_path_.clear();
    current_firmware_info_ = ValidationResult{};

    return {};
}

Result<void> FirmwareLoader::switch_firmware(const std::string& file_path, ProgressCallback progress_callback) {
    // Backup current state before switching
    auto backup_result = backup_current_state();
    if (!backup_result.has_value()) {
        return unexpected(backup_result.error());
    }

    // Unload current firmware
    auto unload_result = unload_firmware();
    if (!unload_result.has_value()) {
        // Restore backup on failure
        restore_backup_state();
        return unexpected(unload_result.error());
    }

    // Load new firmware
    auto load_result = load_firmware(file_path, progress_callback);
    if (!load_result.has_value()) {
        // Restore backup on failure
        restore_backup_state();
        return unexpected(load_result.error());
    }

    return {};
}

Result<void> FirmwareLoader::trigger_boot_sequence() {
    if (!firmware_loaded_ || !boot_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No firmware loaded or boot loader not initialized"});
    }

    auto boot_progress = [](BootLoader::BootStage stage, const std::string& message) {
        // Boot progress callback
        printf("Boot Stage: %d - %s\n", static_cast<int>(stage), message.c_str());
    };

    return boot_loader_->execute_boot_sequence(current_firmware_info_.entry_point, boot_progress);
}

Result<void> FirmwareLoader::trigger_soft_reset() {
    if (!boot_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Boot loader not initialized"});
    }

    return boot_loader_->execute_soft_reset();
}

Result<void> FirmwareLoader::trigger_hard_reset() {
    if (!boot_loader_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Boot loader not initialized"});
    }

    return boot_loader_->execute_hard_reset();
}

void FirmwareLoader::set_memory_controller(std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller) {
    memory_controller_ = memory_controller;
    
    // Pass to ELF loader
    if (elf_loader_) {
        elf_loader_->set_memory_controller(memory_controller);
    }
}

void FirmwareLoader::set_cpu_manager(std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager) {
    cpu_manager_ = cpu_manager;
    
    // Pass to ELF loader
    if (elf_loader_) {
        elf_loader_->set_cpu_manager(cpu_manager);
    }
}

void FirmwareLoader::set_emulator_core(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core) {
    emulator_core_ = emulator_core;
}

void FirmwareLoader::set_boot_rom(std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom) {
    boot_rom_ = boot_rom;
    
    // Pass to ELF loader
    if (elf_loader_) {
        elf_loader_->set_boot_rom(boot_rom);
    }
}

Result<void> FirmwareLoader::load_firmware_internal(const std::string& file_path, ProgressCallback progress_callback) {
    loading_state_.store(LoadingState::VALIDATING);
    progress_update(progress_callback, "Validating", 0.1f, "Validating ELF binary...");

    if (!elf_loader_) {
        loading_state_.store(LoadingState::FAILED);
        return unexpected(Error{ErrorCode::INVALID_STATE, "ELF loader not initialized"});
    }

    // Use new ELF loader for complete application loading
    auto elf_progress_callback = [progress_callback, this](const std::string& stage, float progress, const std::string& message) {
        if (cancel_requested_.load()) {
            return;
        }
        progress_update(progress_callback, stage, progress, message);
    };

    // Load ELF application with comprehensive pipeline
    auto load_result = elf_loader_->load_elf_application(file_path, elf_progress_callback);
    if (!load_result.has_value()) {
        loading_state_.store(LoadingState::FAILED);
        return unexpected(load_result.error());
    }

    if (cancel_requested_.load()) {
        loading_state_.store(LoadingState::IDLE);
        return unexpected(Error{ErrorCode::OPERATION_ABORTED, "Loading cancelled by user"});
    }

    const auto& result = load_result.value();
    
    // Update firmware info from ELF loader result
    current_firmware_info_.valid = result.success;
    current_firmware_info_.entry_point = result.entry_point;
    current_firmware_info_.architecture = "RISC-V";
    current_firmware_info_.target_chip = "ESP32-P4";
    current_firmware_info_.warnings = result.warnings;
    
    // Integrate with Boot ROM if available
    if (boot_rom_) {
        loading_state_.store(LoadingState::SETTING_UP_BOOT);
        progress_update(progress_callback, "Boot ROM", 0.95f, "Integrating with Boot ROM...");
        
        auto boot_integration_result = elf_loader_->integrate_with_boot_rom(result.entry_point);
        if (!boot_integration_result.has_value()) {
            // Non-fatal error - log warning and continue
            LOG_WARN("Boot ROM integration failed: {}", boot_integration_result.error().message());
        }
    }

    // Success!
    firmware_loaded_ = true;
    loaded_firmware_path_ = file_path;
    loading_state_.store(LoadingState::COMPLETED);
    progress_update(progress_callback, "Complete", 1.0f, "Firmware loaded successfully with ELF loader!", true);

    LOG_INFO("Firmware loaded successfully: entry=0x{:08x}, size={} bytes, warnings={}",
             result.entry_point, result.total_size, result.warnings.size());

    return {};
}

Result<ValidationResult> FirmwareLoader::validate_elf_binary(const std::string& file_path) {
    // Create new ELF parser
    auto elf_parser = std::make_unique<ELFParser>(file_path);
    
    // Parse the ELF file
    auto parse_result = elf_parser->parse();
    if (!parse_result.has_value()) {
        LOG_ERROR("ELF parsing failed: {}", parse_result.error().message());
        return unexpected(parse_result.error());
    }

    // Store parsed ELF information
    parsed_elf_data_ = std::make_unique<ParsedELF>(std::move(parse_result.value()));
    
    // Validate for ESP32-P4 compatibility
    auto validation_result = elf_parser->validate_for_esp32p4();
    if (!validation_result.has_value()) {
        LOG_ERROR("ELF validation failed: {}", validation_result.error().message());
        return unexpected(validation_result.error());
    }

    LOG_INFO("Firmware validation completed: {} segments, entry=0x{:08x}", 
             parsed_elf_data_->segments.size(), parsed_elf_data_->entry_point);

    return validation_result.value();
}

Result<void> FirmwareLoader::parse_elf_segments() {
    if (!parsed_elf_data_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No ELF data available"});
    }

    // ELF parsing is already completed in validation phase
    LOG_DEBUG("ELF segments already parsed: {} segments available", 
              parsed_elf_data_->segments.size());
    
    return {};
}

Result<void> FirmwareLoader::validate_memory_layout() {
    if (!parsed_elf_data_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No ELF data available"});
    }

    // Create memory mapper for segment validation
    MemoryMapper mapper;
    mapper.set_strict_mapping(strict_validation_);
    mapper.set_allow_psram(true);
    
    // Validate all segments can be mapped
    auto mapping_result = mapper.map_all_segments(parsed_elf_data_->segments);
    if (!mapping_result.has_value()) {
        LOG_ERROR("Memory layout validation failed: {}", mapping_result.error().message());
        return unexpected(mapping_result.error());
    }

    // Store mapping results for loading phase
    segment_mappings_ = std::move(mapping_result.value());
    
    LOG_INFO("Memory layout validated: {} segments mapped successfully", 
             segment_mappings_.size());

    return {};
}

Result<void> FirmwareLoader::load_segments_to_memory(ProgressCallback progress_callback) {
    if (!parsed_elf_data_ || !memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "ELF data or memory controller not available"});
    }

    const auto& segments = parsed_elf_data_->segments;
    size_t total_segments = segments.size();
    size_t loaded_segments = 0;
    
    LOG_INFO("Loading {} segments to memory...", total_segments);
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        if (segment.memory_size == 0) {
            LOG_DEBUG("Skipping empty segment {}", i);
            continue; // Skip empty segments
        }
        
        // Update progress
        float segment_progress = 0.6f + (0.2f * i / total_segments);
        std::string message = "Loading segment " + std::to_string(i + 1) + "/" + 
                             std::to_string(total_segments) + " (" + 
                             segment.get_permissions() + ", " + 
                             std::to_string(segment.memory_size) + " bytes)";
        progress_update(progress_callback, "Loading", segment_progress, message);
        
        // Write segment data to memory
        if (!segment.data.empty()) {
            auto write_result = write_memory_segment(segment.virtual_address, segment.data);
            if (!write_result.has_value()) {
                LOG_ERROR("Failed to load segment {} at 0x{:08x}: {}", 
                         i, segment.virtual_address, write_result.error().message());
                return unexpected(write_result.error());
            }
            loaded_segments++;
        }
        
        // Zero-fill BSS sections
        if (segment.memory_size > segment.file_size) {
            size_t bss_size = segment.memory_size - segment.file_size;
            Address bss_start = segment.virtual_address + segment.file_size;
            
            LOG_DEBUG("Zero-filling BSS section: 0x{:08x}, {} bytes", bss_start, bss_size);
            auto clear_result = clear_memory_region(bss_start, bss_size);
            if (!clear_result.has_value()) {
                LOG_ERROR("Failed to clear BSS section at 0x{:08x}", bss_start);
                return unexpected(clear_result.error());
            }
        }
        
        // Check for cancellation
        if (cancel_requested_.load()) {
            LOG_INFO("Loading cancelled by user after {} segments", loaded_segments);
            return unexpected(Error{ErrorCode::OPERATION_ABORTED, "Loading cancelled by user"});
        }
        
        // Small delay to allow cancellation checks and progress updates
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LOG_INFO("Successfully loaded {} segments to memory", loaded_segments);
    return {};
}

Result<void> FirmwareLoader::initialize_bss_sections() {
    // BSS initialization is now handled in load_segments_to_memory for efficiency
    LOG_DEBUG("BSS sections already initialized during segment loading");
    return {};
}

Result<void> FirmwareLoader::setup_stack_and_heap() {
    // Setup stack pointer and heap regions for ESP32-P4
    // This would typically be done by the runtime, but we need to prepare the memory layout
    
    // For now, just validate that stack and heap regions are available
    // Real implementation would setup initial stack pointer and heap metadata
    
    return {};
}

Result<void> FirmwareLoader::configure_memory_protection() {
    // Configure memory protection units (if available)
    // ESP32-P4 has memory protection features that need configuration
    
    return {};
}

Result<void> FirmwareLoader::setup_interrupt_vectors() {
    if (!memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Memory controller not available"});
    }

    // Setup interrupt vector table at the appropriate location
    // For RISC-V, this is typically at a fixed address
    
    return {};
}

Result<void> FirmwareLoader::initialize_cores() {
    if (!cpu_manager_ || !parsed_elf_data_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "CPU manager or ELF data not available"});
    }

    // Initialize CPU cores with entry point
    LOG_INFO("Initializing CPU cores with entry point: 0x{:08x}", parsed_elf_data_->entry_point);
    
    // TODO: Set initial register state and prepare cores for execution
    // This would call cpu_manager_->initialize_core(core_id, entry_point, initial_stack_pointer)
    
    if (dual_core_loading_ && parsed_elf_data_->uses_dual_core) {
        LOG_INFO("Setting up dual-core execution mode");
        // TODO: Initialize both cores
    } else {
        LOG_INFO("Setting up single-core execution mode");
        // TODO: Initialize primary core only
    }
    
    return {};
}

Result<void> FirmwareLoader::validate_boot_readiness() {
    // Final validation that everything is ready for boot
    if (!firmware_loaded_ || current_firmware_info_.entry_point == 0) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Firmware not properly loaded"});
    }

    return {};
}

bool FirmwareLoader::is_address_in_valid_region(Address address, size_t size) const {
    Address end_address = address + size - 1;

    // Check against ESP32-P4 memory regions
    if (address >= ESP32P4MemoryLayout::IROM_BASE && 
        end_address < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) {
        return true; // Flash ROM
    }
    
    if (address >= ESP32P4MemoryLayout::DRAM_BASE && 
        end_address < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) {
        return true; // SRAM
    }
    
    if (address >= ESP32P4MemoryLayout::PSRAM_BASE && 
        end_address < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE) {
        return true; // PSRAM
    }
    
    return false;
}

Result<void> FirmwareLoader::clear_memory_region(Address start, size_t size) {
    if (!memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Memory controller not available"});
    }

    // Clear memory region by writing zeros
    std::vector<u8> zeros(size, 0);
    return write_memory_segment(start, zeros);
}

Result<void> FirmwareLoader::write_memory_segment(Address address, const std::vector<u8>& data) {
    if (!memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Memory controller not available"});
    }

    auto result = memory_controller_->write_bytes(address, data.data(), data.size());
    if (!result.has_value()) {
        return unexpected(Error{ErrorCode::MEMORY_ACCESS_ERROR, "Failed to write memory segment at 0x" + std::to_string(address)});
    }

    return {};
}

void FirmwareLoader::progress_update(ProgressCallback callback, const std::string& stage, float progress, const std::string& message, bool success) {
    if (callback) {
        callback(stage, progress, message, success);
    }
}

Result<void> FirmwareLoader::clear_memory_regions() {
    if (!memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Memory controller not available"});
    }

    // Clear main memory regions
    auto result = memory_controller_->reset();
    if (!result.has_value()) {
        return unexpected(result.error());
    }

    return {};
}

Result<void> FirmwareLoader::backup_current_state() {
    if (!memory_controller_) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "Memory controller not available"});
    }

    backup_state_ = std::make_unique<BackupState>();
    
    // Backup would read current memory contents
    // For now, just mark as valid
    backup_state_->valid = true;

    return {};
}

Result<void> FirmwareLoader::restore_backup_state() {
    if (!backup_state_ || !backup_state_->valid) {
        return unexpected(Error{ErrorCode::INVALID_STATE, "No valid backup state available"});
    }

    // Restore would write backup data back to memory
    // For now, just clear current state
    firmware_loaded_ = false;
    loaded_firmware_path_.clear();
    current_firmware_info_ = ValidationResult{};

    return {};
}

// ELFBinary class removed - replaced by ELFParser with better functionality

//
// BootLoader Implementation
//

BootLoader::BootLoader() = default;

BootLoader::~BootLoader() = default;

Result<void> BootLoader::initialize() {
    return {};
}

Result<void> BootLoader::execute_boot_sequence(Address entry_point, BootProgressCallback progress_callback) {
    progress_update(progress_callback, BootStage::ROM_BOOT, "Starting ROM bootloader...");
    auto rom_result = execute_rom_boot(progress_callback);
    if (!rom_result.has_value()) return unexpected(rom_result.error());

    progress_update(progress_callback, BootStage::APP_BOOT, "Starting application bootloader...");
    auto app_result = execute_app_boot(progress_callback);
    if (!app_result.has_value()) return unexpected(app_result.error());

    progress_update(progress_callback, BootStage::PARTITION_INIT, "Initializing memory subsystem...");
    auto mem_result = initialize_memory_subsystem(progress_callback);
    if (!mem_result.has_value()) return unexpected(mem_result.error());

    progress_update(progress_callback, BootStage::HEAP_INIT, "Setting up interrupt system...");
    auto int_result = setup_interrupt_system(progress_callback);
    if (!int_result.has_value()) return unexpected(int_result.error());

    progress_update(progress_callback, BootStage::APP_START, "Starting application cores...");
    auto core_result = start_application_cores(entry_point, progress_callback);
    if (!core_result.has_value()) return unexpected(core_result.error());

    progress_update(progress_callback, BootStage::COMPLETE, "Boot sequence completed successfully");
    return {};
}

Result<void> BootLoader::execute_soft_reset() {
    // Implement soft reset logic
    return {};
}

Result<void> BootLoader::execute_hard_reset() {
    // Implement hard reset logic
    return {};
}

Result<void> BootLoader::execute_rom_boot(BootProgressCallback callback) {
    // Simulate ROM boot process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return {};
}

Result<void> BootLoader::execute_app_boot(BootProgressCallback callback) {
    // Simulate app bootloader
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return {};
}

Result<void> BootLoader::initialize_memory_subsystem(BootProgressCallback callback) {
    // Initialize memory subsystem
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    return {};
}

Result<void> BootLoader::setup_interrupt_system(BootProgressCallback callback) {
    // Setup interrupt system
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return {};
}

Result<void> BootLoader::start_application_cores(Address entry_point, BootProgressCallback callback) {
    // Start application cores
    if (dual_core_enabled_) {
        // Start both cores
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } else {
        // Start single core
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return {};
}

Result<void> BootLoader::verify_boot_completion(BootProgressCallback callback) {
    // Verify boot completion
    return {};
}

void BootLoader::progress_update(BootProgressCallback callback, BootStage stage, const std::string& message) {
    if (callback) {
        callback(stage, message);
    }
}

Result<void> BootLoader::wait_for_core_ready(u32 core_id, std::chrono::milliseconds timeout) {
    // Wait for core to be ready
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return {};
}

} // namespace m5tab5::emulator::firmware