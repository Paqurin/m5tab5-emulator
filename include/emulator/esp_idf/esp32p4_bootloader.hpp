#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/esp_idf/esp_system.h"

#include <memory>
#include <string>
#include <vector>
#include <map>

namespace m5tab5::emulator::esp_idf {

// Forward declarations
class PartitionTable;
class ApplicationLoader;

/**
 * @brief ESP32-P4 Second-Stage Bootloader
 * 
 * Emulates the ESP32-P4's second-stage bootloader functionality including:
 * - System clock configuration to 400MHz
 * - PSRAM controller initialization  
 * - Cache configuration and MMU setup
 * - Partition table parsing
 * - Application loading from flash
 * - Heap regions setup
 * - ESP-IDF component initialization
 * - FreeRTOS scheduler startup
 * - Transition to app_main()
 */
class ESP32P4Bootloader {
public:
    /**
     * @brief Boot modes supported by ESP32-P4
     */
    enum class BootMode {
        NORMAL_BOOT,        // Normal application boot
        DOWNLOAD_BOOT,      // UART/USB download mode
        DEEP_SLEEP_WAKE,    // Wake from deep sleep
        SOFTWARE_RESTART    // Software-triggered restart
    };

    /**
     * @brief System configuration settings
     */
    struct SystemConfig {
        u32 cpu_freq_mhz = 400;         // CPU frequency in MHz
        bool dual_core_enabled = true;   // Enable dual-core operation
        bool psram_enabled = true;       // Enable PSRAM support
        u32 psram_size_mb = 32;         // PSRAM size in MB
        u32 flash_size_mb = 16;         // Flash size in MB
        bool cache_enabled = true;       // Enable instruction/data cache
        u32 heap_caps_init = 0x3F;      // Heap capabilities initialization
        bool wdt_enabled = true;         // Enable watchdog timer
        u32 wdt_timeout_ms = 30000;     // Watchdog timeout in ms
    };

    /**
     * @brief Memory region configuration
     */
    struct MemoryLayout {
        Address flash_base = 0x42000000;        // Flash memory base (XIP)
        Address sram_base = 0x4FF00000;         // L2 SRAM base
        Address psram_base = 0x48000000;        // PSRAM base
        size_t flash_size = 16 * 1024 * 1024;  // 16MB flash
        size_t sram_size = 768 * 1024;         // 768KB SRAM
        size_t psram_size = 32 * 1024 * 1024;  // 32MB PSRAM
        
        // Application layout
        Address app_entry = 0x42010000;         // Default app entry
        Address app_stack = 0x4FFBFF00;         // Initial stack pointer
        size_t stack_size = 8192;              // Default stack size
    };

    ESP32P4Bootloader();
    ~ESP32P4Bootloader();

    // Bootloader lifecycle
    Result<void> initialize(std::shared_ptr<MemoryController> memory_controller,
                           std::shared_ptr<DualCoreManager> cpu_manager);
    void shutdown();

    // Main bootloader execution
    Result<void> execute_bootloader_main(BootMode boot_mode = BootMode::NORMAL_BOOT);
    
    // Boot phase execution
    Result<void> configure_system_clocks();
    Result<void> initialize_psram_controller();
    Result<void> setup_cache_configuration();
    Result<void> parse_partition_table();
    Result<void> validate_application_partition();
    Result<void> load_application_from_flash();
    Result<void> setup_application_memory_layout();
    Result<void> initialize_heap_regions();
    Result<void> initialize_esp_idf_components();
    Result<void> start_freertos_scheduler();
    Result<void> call_application_main();

    // Configuration and state
    void set_system_config(const SystemConfig& config) { system_config_ = config; }
    const SystemConfig& get_system_config() const { return system_config_; }
    
    void set_memory_layout(const MemoryLayout& layout) { memory_layout_ = layout; }
    const MemoryLayout& get_memory_layout() const { return memory_layout_; }
    
    BootMode get_boot_mode() const { return current_boot_mode_; }
    bool is_bootloader_running() const { return bootloader_running_; }

    // Application information
    Address get_application_entry_point() const { return application_entry_point_; }
    size_t get_application_size() const { return application_size_; }
    
    // Performance metrics
    struct BootMetrics {
        std::chrono::milliseconds total_boot_time{0};
        std::chrono::milliseconds clock_config_time{0};
        std::chrono::milliseconds psram_init_time{0};
        std::chrono::milliseconds cache_setup_time{0};
        std::chrono::milliseconds partition_parse_time{0};
        std::chrono::milliseconds app_load_time{0};
        std::chrono::milliseconds esp_idf_init_time{0};
        std::chrono::milliseconds freertos_start_time{0};
    };
    
    const BootMetrics& get_boot_metrics() const { return boot_metrics_; }

private:
    // Component dependencies
    std::shared_ptr<MemoryController> memory_controller_;
    std::shared_ptr<DualCoreManager> cpu_manager_;
    
    // Bootloader state
    SystemConfig system_config_;
    MemoryLayout memory_layout_;
    BootMode current_boot_mode_;
    bool bootloader_running_;
    bool initialized_;
    
    // Application information
    Address application_entry_point_;
    size_t application_size_;
    std::vector<u8> application_data_;
    
    // Partition table and application
    std::unique_ptr<PartitionTable> partition_table_;
    std::unique_ptr<ApplicationLoader> app_loader_;
    
    // Boot metrics and timing
    BootMetrics boot_metrics_;
    std::chrono::steady_clock::time_point boot_start_time_;
    
    // Internal boot phase helpers
    Result<void> detect_and_configure_flash();
    Result<void> setup_mmu_mappings();
    Result<void> configure_interrupt_matrix();
    Result<void> enable_dual_core_operation();
    Result<void> setup_watchdog_timer();
    Result<void> initialize_peripheral_clocks();
    
    // Memory management helpers
    Result<void> setup_heap_region(Address start, size_t size, u32 capabilities);
    Result<void> configure_stack_guard_pages();
    Result<void> setup_memory_protection_unit();
    
    // Application validation and loading
    Result<void> validate_elf_header(const u8* elf_data, size_t size);
    Result<void> load_elf_segments(const u8* elf_data, size_t size);
    Result<Address> parse_elf_entry_point(const u8* elf_data, size_t size);
    
    // ESP-IDF initialization helpers
    Result<void> init_newlib_locks();
    Result<void> init_pthread_system();
    Result<void> init_vfs_system();
    Result<void> init_nvs_flash();
    Result<void> init_wifi_netif();
    Result<void> init_event_loops();
    
    // FreeRTOS integration
    Result<void> create_main_task();
    Result<void> create_idle_tasks();
    Result<void> setup_tick_timer();
    Result<void> enable_scheduler_interrupts();
    
    // Timing and metrics
    void start_phase_timer();
    void end_phase_timer(std::chrono::milliseconds& phase_time);
    void log_boot_metrics();
    
    // Error handling
    void handle_bootloader_error(const std::string& phase, const Error& error);
    void cleanup_bootloader_state();
};

/**
 * @brief Partition table parser for ESP32-P4
 */
class PartitionTable {
public:
    struct Partition {
        std::string name;
        u8 type;
        u8 subtype;
        Address offset;
        size_t size;
        u32 flags;
        bool encrypted;
        
        bool is_app_partition() const { return type == 0x00; }  // APP type
        bool is_data_partition() const { return type == 0x01; } // DATA type
    };
    
    PartitionTable();
    ~PartitionTable();
    
    Result<void> parse_from_flash(std::shared_ptr<MemoryController> memory, Address table_offset);
    Result<Partition> find_factory_app() const;
    Result<Partition> find_partition_by_name(const std::string& name) const;
    Result<std::vector<Partition>> get_app_partitions() const;
    
    size_t get_partition_count() const { return partitions_.size(); }
    const std::vector<Partition>& get_all_partitions() const { return partitions_; }

private:
    std::vector<Partition> partitions_;
    bool table_parsed_;
    
    Result<void> parse_partition_entry(const u8* entry_data);
    bool validate_partition_table_md5(const u8* table_data, size_t size) const;
    u16 calculate_partition_crc(const u8* entry_data) const;
};

/**
 * @brief Application loader for ESP32-P4 ELF binaries
 */
class ApplicationLoader {
public:
    struct LoadedApplication {
        Address entry_point;
        Address stack_pointer;
        size_t total_size;
        std::map<std::string, Address> symbol_table;
        bool valid;
    };
    
    ApplicationLoader();
    ~ApplicationLoader();
    
    Result<LoadedApplication> load_application(std::shared_ptr<MemoryController> memory,
                                             const PartitionTable::Partition& app_partition);
    
    Result<void> verify_application_signature(const u8* app_data, size_t size);
    Result<void> decrypt_application_if_needed(u8* app_data, size_t size);
    
    const LoadedApplication& get_loaded_app() const { return loaded_app_; }

private:
    LoadedApplication loaded_app_;
    
    // ELF parsing helpers
    Result<void> parse_elf_header(const u8* elf_data, size_t size);
    Result<void> load_program_segments(const u8* elf_data, size_t size,
                                     std::shared_ptr<MemoryController> memory);
    Result<void> process_section_headers(const u8* elf_data, size_t size);
    Result<void> build_symbol_table(const u8* elf_data, size_t size);
    
    // Security and validation
    bool verify_elf_magic(const u8* elf_data) const;
    bool verify_target_architecture(const u8* elf_data) const;
    Result<void> check_memory_boundaries(Address addr, size_t size) const;
};

/**
 * @brief ESP32-P4 Boot Coordinator
 * 
 * Manages the complete boot sequence from reset to application execution
 */
class ESP32P4BootSequence {
public:
    enum class BootPhase {
        RESET,
        BOOT_ROM,
        BOOTLOADER,
        ESP_IDF_INIT,
        APPLICATION_RUNNING,
        ERROR
    };
    
    ESP32P4BootSequence();
    ~ESP32P4BootSequence();
    
    Result<void> execute_complete_boot_sequence();
    Result<void> execute_warm_restart();
    Result<void> execute_cold_boot();
    
    BootPhase get_current_phase() const { return current_phase_; }
    
    // Integration with emulator core
    void set_memory_controller(std::shared_ptr<MemoryController> memory_controller);
    void set_cpu_manager(std::shared_ptr<DualCoreManager> cpu_manager);
    
    // Boot sequence control
    void abort_boot_sequence();
    bool is_boot_complete() const { return current_phase_ == BootPhase::APPLICATION_RUNNING; }
    
    // Boot configuration
    void configure_boot_parameters(const ESP32P4Bootloader::SystemConfig& config);

private:
    std::shared_ptr<MemoryController> memory_controller_;
    std::shared_ptr<DualCoreManager> cpu_manager_;
    std::unique_ptr<ESP32P4Bootloader> bootloader_;
    
    BootPhase current_phase_;
    bool boot_aborted_;
    
    Result<void> execute_boot_rom_phase();
    Result<void> execute_bootloader_phase();
    Result<void> execute_esp_idf_initialization();
    Result<void> start_application_execution();
    
    void advance_to_phase(BootPhase phase);
    void handle_phase_error(BootPhase phase, const Error& error);
};

} // namespace m5tab5::emulator::esp_idf