#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"

// Forward declarations
namespace m5tab5::emulator {
    class MemoryController;
    class DualCoreManager;
    class EmulatorCore;
    class BootROM;
}

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

namespace m5tab5::emulator::firmware {

// Forward declarations
class ELFParser;
class BootLoader;
class ELFLoader;
struct ParsedELF;
struct ParsedSegment;

// Forward declarations and types
struct MappingResult {
    Address target_address;
    size_t size;
    std::string region_name;
    bool success;
    std::string error_message;
};

class MemoryMapper;

/**
 * @brief Firmware loading progress callback
 * 
 * @param stage Current loading stage name
 * @param progress Progress percentage (0.0 - 1.0)
 * @param message Detailed progress message
 * @param success True if stage completed successfully, false if error
 */
using ProgressCallback = std::function<void(const std::string& stage, float progress, const std::string& message, bool success)>;

/**
 * @brief Firmware validation result
 */
struct ValidationResult {
    bool valid;
    std::string error_message;
    std::vector<std::string> warnings;
    
    // Binary information
    std::string architecture;
    std::string target_chip;
    Address entry_point;
    size_t code_size;
    size_t data_size;
    size_t bss_size;
    
    // ESP32-P4 specific
    bool has_dual_core_support;
    bool uses_psram;
    std::vector<Address> memory_regions;
};

/**
 * @brief Firmware loading state
 */
enum class LoadingState {
    IDLE,
    VALIDATING,
    LOADING_SEGMENTS,
    INITIALIZING_MEMORY,
    SETTING_UP_BOOT,
    STARTING_CORES,
    COMPLETED,
    FAILED
};

/**
 * @brief ESP32-P4 memory regions for firmware loading
 */
struct ESP32P4MemoryLayout {
    static constexpr Address IROM_BASE = 0x40000000;    // Flash ROM base
    static constexpr Address IROM_SIZE = 0x01000000;    // 16MB Flash
    static constexpr Address DROM_BASE = 0x3C000000;    // Flash data base
    static constexpr Address DRAM_BASE = 0x4FF00000;    // SRAM base
    static constexpr Address DRAM_SIZE = 0x000C0000;    // 768KB SRAM
    static constexpr Address PSRAM_BASE = 0x48000000;   // PSRAM base
    static constexpr Address PSRAM_SIZE = 0x02000000;   // 32MB PSRAM
    static constexpr Address RTCRAM_BASE = 0x50108000;  // RTC RAM base
    static constexpr Address RTCRAM_SIZE = 0x00002000;  // 8KB RTC RAM
    
    // Boot and exception vectors
    static constexpr Address RESET_VECTOR = 0x40000000;
    static constexpr Address EXCEPTION_VECTOR = 0x40000400;
    
    // Memory protection boundaries
    static constexpr Address EXECUTABLE_START = IROM_BASE;
    static constexpr Address EXECUTABLE_END = IROM_BASE + IROM_SIZE;
    static constexpr Address WRITABLE_START = DRAM_BASE;
    static constexpr Address WRITABLE_END = PSRAM_BASE + PSRAM_SIZE;
};

/**
 * @brief Comprehensive firmware loader for ESP32-P4 ELF binaries
 * 
 * Features:
 * - ELF binary validation and parsing
 * - Memory layout verification for ESP32-P4
 * - Progressive loading with callback support
 * - Dual-core initialization support
 * - Boot sequence management
 * - Firmware unloading and switching
 * - Error recovery and rollback
 * - Security validation (optional)
 */
class FirmwareLoader {
public:
    FirmwareLoader();
    ~FirmwareLoader();

    // Core lifecycle
    Result<void> initialize();
    void shutdown();

    // Firmware loading operations
    Result<ValidationResult> validate_firmware(const std::string& file_path);
    Result<void> load_firmware(const std::string& file_path, ProgressCallback progress_callback = nullptr);
    Result<void> unload_firmware();
    
    // Async loading support
    Result<void> load_firmware_async(const std::string& file_path, ProgressCallback progress_callback = nullptr);
    void cancel_loading();
    bool is_loading() const { return loading_state_.load() != LoadingState::IDLE; }
    LoadingState get_loading_state() const { return loading_state_.load(); }

    // Firmware switching
    Result<void> switch_firmware(const std::string& file_path, ProgressCallback progress_callback = nullptr);
    
    // State queries
    bool is_firmware_loaded() const { return firmware_loaded_; }
    std::string get_loaded_firmware_path() const { return loaded_firmware_path_; }
    ValidationResult get_firmware_info() const { return current_firmware_info_; }

    // Boot control
    Result<void> trigger_boot_sequence();
    Result<void> trigger_soft_reset();
    Result<void> trigger_hard_reset();

    // Memory management
    Result<void> clear_memory_regions();
    Result<void> backup_current_state();
    Result<void> restore_backup_state();

    // Configuration
    void set_validation_strict(bool strict) { strict_validation_ = strict; }
    void set_security_checks_enabled(bool enabled) { security_checks_enabled_ = enabled; }
    void set_dual_core_loading(bool enabled) { dual_core_loading_ = enabled; }

    // Integration points
    void set_memory_controller(std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller);
    void set_cpu_manager(std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager);
    void set_emulator_core(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core);
    void set_boot_rom(std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom);

private:
    // Core components (injected dependencies)
    std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller_;
    std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager_;
    std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core_;

    // Internal components
    std::unique_ptr<ParsedELF> parsed_elf_data_;
    std::unique_ptr<BootLoader> boot_loader_;
    std::unique_ptr<ELFLoader> elf_loader_;
    std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom_;
    std::vector<MappingResult> segment_mappings_;

    // State management
    std::atomic<LoadingState> loading_state_{LoadingState::IDLE};
    std::atomic<bool> cancel_requested_{false};
    bool firmware_loaded_{false};
    std::string loaded_firmware_path_;
    ValidationResult current_firmware_info_;

    // Configuration
    bool strict_validation_{true};
    bool security_checks_enabled_{true};
    bool dual_core_loading_{true};

    // Threading for async operations
    std::unique_ptr<std::thread> loading_thread_;
    mutable std::mutex state_mutex_;

    // Backup/restore state
    struct BackupState {
        std::vector<u8> flash_backup;
        std::vector<u8> sram_backup;
        std::vector<u8> psram_backup;
        bool valid = false;
    };
    std::unique_ptr<BackupState> backup_state_;

    // Internal loading pipeline
    Result<void> load_firmware_internal(const std::string& file_path, ProgressCallback progress_callback);
    Result<ValidationResult> validate_elf_binary(const std::string& file_path);
    Result<void> parse_elf_segments();
    Result<void> validate_memory_layout();
    Result<void> load_segments_to_memory(ProgressCallback progress_callback);
    Result<void> initialize_bss_sections();
    Result<void> setup_stack_and_heap();
    Result<void> configure_memory_protection();
    Result<void> setup_interrupt_vectors();
    Result<void> initialize_cores();
    Result<void> validate_boot_readiness();

    // Utility methods
    bool is_address_in_valid_region(Address address, size_t size) const;
    Result<void> clear_memory_region(Address start, size_t size);
    Result<void> write_memory_segment(Address address, const std::vector<u8>& data);
    
    void progress_update(ProgressCallback callback, const std::string& stage, float progress, const std::string& message, bool success = true);
    void cleanup_loading_state();

    // Security validation
    Result<void> perform_security_checks();
    bool validate_code_signatures();
    bool check_memory_bounds();
    bool validate_system_calls();

    // Error recovery
    void handle_loading_error(const std::string& error_message);
    Result<void> rollback_partial_load();
};

/**
 * @brief ELF binary parser and analyzer
 * 
 * Handles ESP32-P4 specific ELF format requirements:
 * - RISC-V architecture validation
 * - Segment parsing and validation
 * - Symbol table extraction
 * - Memory layout analysis
 */
class ELFBinary {
public:
    struct ELFSegment {
        u32 type;           // PT_LOAD, PT_DYNAMIC, etc.
        Address virtual_addr;
        Address physical_addr;
        size_t file_size;
        size_t memory_size;
        u32 flags;          // PF_X, PF_W, PF_R
        std::vector<u8> data;
        
        bool is_executable() const { return (flags & 0x1) != 0; }
        bool is_writable() const { return (flags & 0x2) != 0; }
        bool is_readable() const { return (flags & 0x4) != 0; }
    };

    struct ELFHeader {
        std::string architecture;
        std::string abi_version;
        Address entry_point;
        bool is_64bit;
        bool is_little_endian;
        u16 machine_type;
    };

    explicit ELFBinary(const std::string& file_path);
    ~ELFBinary();

    Result<void> parse();
    
    // Accessors
    const ELFHeader& get_header() const { return header_; }
    const std::vector<ELFSegment>& get_segments() const { return segments_; }
    Address get_entry_point() const { return header_.entry_point; }
    
    // Validation
    Result<ValidationResult> validate_for_esp32p4();
    bool is_dual_core_firmware() const;
    std::vector<Address> get_required_memory_regions() const;

private:
    std::string file_path_;
    ELFHeader header_;
    std::vector<ELFSegment> segments_;
    std::vector<u8> file_data_;

    Result<void> parse_header();
    Result<void> parse_segments();
    Result<void> load_file_data();
    bool validate_elf_magic() const;
    bool validate_architecture() const;
};

/**
 * @brief ESP32-P4 boot sequence manager
 * 
 * Manages the complex boot process:
 * - ROM bootloader simulation
 * - App bootloader execution
 * - Partition table parsing
 * - Application startup
 * - Dual-core initialization
 */
class BootLoader {
public:
    enum class BootStage {
        ROM_BOOT,
        APP_BOOT,
        PARTITION_INIT,
        HEAP_INIT,
        APP_START,
        DUAL_CORE_START,
        COMPLETE
    };

    using BootProgressCallback = std::function<void(BootStage stage, const std::string& message)>;

    BootLoader();
    ~BootLoader();

    Result<void> initialize();
    
    // Boot sequence execution
    Result<void> execute_boot_sequence(Address entry_point, BootProgressCallback progress_callback = nullptr);
    Result<void> execute_soft_reset();
    Result<void> execute_hard_reset();

    // Configuration
    void set_dual_core_enabled(bool enabled) { dual_core_enabled_ = enabled; }
    void set_boot_timeout(std::chrono::milliseconds timeout) { boot_timeout_ = timeout; }

private:
    bool dual_core_enabled_{true};
    std::chrono::milliseconds boot_timeout_{std::chrono::seconds(30)};

    // Boot sequence stages
    Result<void> execute_rom_boot(BootProgressCallback callback);
    Result<void> execute_app_boot(BootProgressCallback callback);
    Result<void> initialize_memory_subsystem(BootProgressCallback callback);
    Result<void> setup_interrupt_system(BootProgressCallback callback);
    Result<void> start_application_cores(Address entry_point, BootProgressCallback callback);
    Result<void> verify_boot_completion(BootProgressCallback callback);

    // Utility methods
    void progress_update(BootProgressCallback callback, BootStage stage, const std::string& message);
    Result<void> wait_for_core_ready(u32 core_id, std::chrono::milliseconds timeout);
};

} // namespace m5tab5::emulator::firmware