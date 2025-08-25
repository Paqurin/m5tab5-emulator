#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <map>
#include <atomic>
#include <thread>

// Forward declarations
namespace m5tab5::emulator {
    class EmulatorCore;
    class MemoryController;
    class DualCoreManager;
}

namespace m5tab5::emulator::firmware {

/**
 * @brief ESP32-P4 boot sequence stages
 */
enum class BootStage {
    POWER_ON_RESET,     // Initial hardware reset
    ROM_BOOTLOADER,     // ROM bootloader execution
    FLASH_DETECTION,    // Detect and configure flash
    BOOTLOADER_LOAD,    // Load second-stage bootloader
    PARTITION_TABLE,    // Read partition table
    APP_SELECTION,      // Select application partition
    APP_LOAD,          // Load application
    MEMORY_INIT,       // Initialize memory subsystem
    PERIPHERAL_INIT,   // Initialize peripherals
    RTOS_INIT,         // Initialize FreeRTOS
    DUAL_CORE_START,   // Start second CPU core (if enabled)
    APP_MAIN,          // Jump to app_main()
    RUNNING,           // Application running
    ERROR,             // Boot error occurred
    SHUTDOWN           // System shutdown
};

/**
 * @brief Boot progress callback
 * 
 * @param stage Current boot stage
 * @param progress Progress within stage (0.0 - 1.0)
 * @param message Human-readable progress message
 * @param success True if stage completed successfully
 */
using BootProgressCallback = std::function<void(BootStage stage, float progress, const std::string& message, bool success)>;

/**
 * @brief Boot configuration for ESP32-P4
 */
struct BootConfiguration {
    // Core configuration
    bool dual_core_enabled = true;
    u32 cpu_frequency_mhz = 400;
    
    // Memory configuration
    bool enable_psram = true;
    u32 flash_size_mb = 16;
    u32 psram_size_mb = 32;
    
    // Boot behavior
    std::chrono::milliseconds boot_delay = std::chrono::milliseconds(100);
    std::chrono::milliseconds watchdog_timeout = std::chrono::seconds(30);
    bool enable_boot_logging = true;
    
    // Application configuration
    Address app_entry_point = 0x40000000;
    Address app_stack_pointer = 0x4FFBFF00;  // Top of SRAM
    size_t app_stack_size = 8192;
    
    // Peripheral initialization
    bool init_gpio = true;
    bool init_uart = true;
    bool init_i2c = true;
    bool init_spi = true;
    bool init_display = true;
    bool init_audio = true;
};

/**
 * @brief Boot statistics and timing information
 */
struct BootStatistics {
    std::chrono::steady_clock::time_point boot_start_time;
    std::chrono::steady_clock::time_point boot_complete_time;
    std::chrono::milliseconds total_boot_time{0};
    
    // Stage timing
    std::map<BootStage, std::chrono::milliseconds> stage_durations;
    
    // Memory usage
    size_t flash_used_bytes = 0;
    size_t sram_used_bytes = 0;
    size_t psram_used_bytes = 0;
    
    // Error information
    bool boot_successful = false;
    BootStage failed_stage = BootStage::POWER_ON_RESET;
    std::string failure_reason;
    
    double get_boot_time_seconds() const {
        return static_cast<double>(total_boot_time.count()) / 1000.0;
    }
};

/**
 * @brief Professional ESP32-P4 boot sequencer
 * 
 * Provides realistic boot sequence emulation with:
 * - Accurate boot stage progression
 * - Configurable timing and delays
 * - Peripheral initialization
 * - Dual-core startup coordination
 * - Error handling and recovery
 * - Boot statistics and profiling
 */
class BootSequencer {
public:
    BootSequencer();
    ~BootSequencer();

    // Lifecycle management
    Result<void> initialize(const BootConfiguration& config);
    void shutdown();

    // Boot sequence execution
    Result<void> execute_boot_sequence(Address entry_point, 
                                     BootProgressCallback progress_callback = nullptr);
    Result<void> execute_soft_reset();
    Result<void> execute_hard_reset();
    
    // Boot control
    void abort_boot();
    bool is_boot_in_progress() const { return current_stage_ != BootStage::POWER_ON_RESET && 
                                               current_stage_ != BootStage::RUNNING; }
    bool is_boot_successful() const { return boot_stats_.boot_successful; }
    
    // State queries
    BootStage get_current_stage() const { return current_stage_; }
    float get_boot_progress() const;
    const BootStatistics& get_boot_statistics() const { return boot_stats_; }
    
    // Configuration
    void set_boot_configuration(const BootConfiguration& config) { boot_config_ = config; }
    const BootConfiguration& get_boot_configuration() const { return boot_config_; }
    
    // Integration with emulator components
    void set_emulator_core(std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core);
    void set_memory_controller(std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller);
    void set_cpu_manager(std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager);

private:
    // Component dependencies
    std::shared_ptr<::m5tab5::emulator::EmulatorCore> emulator_core_;
    std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller_;
    std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager_;
    
    // Configuration and state
    BootConfiguration boot_config_;
    BootStage current_stage_;
    BootStatistics boot_stats_;
    std::atomic<bool> abort_requested_{false};
    
    // Boot sequence implementation
    Result<void> execute_power_on_reset(BootProgressCallback callback);
    Result<void> execute_rom_bootloader(BootProgressCallback callback);
    Result<void> execute_flash_detection(BootProgressCallback callback);
    Result<void> execute_bootloader_load(BootProgressCallback callback);
    Result<void> execute_partition_table_read(BootProgressCallback callback);
    Result<void> execute_app_selection(BootProgressCallback callback);
    Result<void> execute_app_load(BootProgressCallback callback);
    Result<void> execute_memory_initialization(BootProgressCallback callback);
    Result<void> execute_peripheral_initialization(BootProgressCallback callback);
    Result<void> execute_rtos_initialization(BootProgressCallback callback);
    Result<void> execute_dual_core_startup(BootProgressCallback callback);
    Result<void> execute_app_main_jump(BootProgressCallback callback);
    
    // Peripheral initialization helpers
    Result<void> initialize_gpio_subsystem();
    Result<void> initialize_uart_subsystem();
    Result<void> initialize_i2c_subsystem();
    Result<void> initialize_spi_subsystem();
    Result<void> initialize_display_subsystem();
    Result<void> initialize_audio_subsystem();
    
    // Memory configuration helpers
    Result<void> configure_memory_regions();
    Result<void> setup_memory_protection();
    Result<void> initialize_heap_allocator();
    Result<void> setup_stack_regions();
    
    // CPU configuration helpers
    Result<void> configure_primary_cpu_core();
    Result<void> configure_secondary_cpu_core();
    Result<void> setup_interrupt_vectors();
    Result<void> configure_cpu_frequency();
    
    // Boot timing and progress
    void start_stage_timer(BootStage stage);
    void end_stage_timer(BootStage stage);
    void update_progress(BootProgressCallback callback, BootStage stage, 
                        float progress, const std::string& message, bool success = true);
    
    // Error handling
    void handle_boot_error(BootStage stage, const std::string& error_message);
    void cleanup_partial_boot();
    
    // Validation helpers
    bool validate_flash_configuration() const;
    bool validate_memory_configuration() const;
    bool validate_application_binary() const;
    bool validate_peripheral_availability() const;
    
    // Boot stage mapping
    static std::string get_stage_name(BootStage stage);
    static float get_stage_base_progress(BootStage stage);
};

/**
 * @brief Boot watchdog timer for timeout protection
 */
class BootWatchdog {
public:
    explicit BootWatchdog(std::chrono::milliseconds timeout);
    ~BootWatchdog();
    
    void start();
    void stop();
    void feed();
    
    bool has_timed_out() const { return timed_out_.load(); }
    
    using TimeoutCallback = std::function<void()>;
    void set_timeout_callback(TimeoutCallback callback) { timeout_callback_ = callback; }

private:
    std::chrono::milliseconds timeout_;
    std::atomic<bool> active_{false};
    std::atomic<bool> timed_out_{false};
    std::thread watchdog_thread_;
    TimeoutCallback timeout_callback_;
    
    void watchdog_loop();
};

/**
 * @brief ESP32-P4 partition table parser
 */
class PartitionTable {
public:
    struct Partition {
        std::string name;
        std::string type;
        std::string subtype;
        Address offset;
        size_t size;
        bool encrypted = false;
    };
    
    PartitionTable();
    ~PartitionTable();
    
    Result<void> parse_from_memory(Address partition_table_address);
    Result<Partition> find_app_partition() const;
    Result<std::vector<Partition>> get_all_partitions() const;
    
    bool has_partition(const std::string& name) const;
    Result<Partition> get_partition(const std::string& name) const;

private:
    std::vector<Partition> partitions_;
    bool parsed_ = false;
    
    Result<void> parse_partition_entry(const u8* entry_data);
    bool validate_partition_table_checksum(const u8* table_data, size_t size) const;
};

} // namespace m5tab5::emulator::firmware