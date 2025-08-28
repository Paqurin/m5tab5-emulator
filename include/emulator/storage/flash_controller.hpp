#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/storage/partition_table.hpp"
#include "emulator/memory/memory_interface.hpp"

using namespace m5tab5::emulator;

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <unordered_map>
#include <functional>
#include <thread>
#include <chrono>

namespace m5tab5::emulator::storage {

/**
 * @brief ESP32-P4 Flash Memory Controller with SPI Flash Simulation
 * 
 * Provides authentic ESP32-P4 flash memory behavior including:
 * - Standard SPI flash interface (esp_flash APIs)
 * - Memory-mapped flash access (XIP mode) 
 * - Flash timing and wait states
 * - Sector/block erase operations
 * - Persistent flash image storage
 * - Partition table management
 * - Wear leveling simulation
 */
class FlashController {
public:
    // ESP32-P4 Flash characteristics
    static constexpr size_t FLASH_SIZE = 16 * 1024 * 1024;  // 16MB
    static constexpr size_t SECTOR_SIZE = 4096;              // 4KB sectors
    static constexpr size_t BLOCK_SIZE = 65536;              // 64KB blocks
    static constexpr size_t PAGE_SIZE = 256;                 // 256B pages
    static constexpr Address FLASH_BASE_ADDRESS = 0x40000000;
    static constexpr Address FLASH_END_ADDRESS = 0x40FFFFFF;
    
    // SPI Flash command set
    enum class FlashCommand : uint8_t {
        READ_DATA = 0x03,           // Standard read
        FAST_READ = 0x0B,           // Fast read with dummy
        READ_STATUS = 0x05,         // Read status register
        WRITE_ENABLE = 0x06,        // Write enable
        WRITE_DISABLE = 0x04,       // Write disable
        PAGE_PROGRAM = 0x02,        // Page program
        SECTOR_ERASE = 0x20,        // 4KB sector erase
        BLOCK_ERASE_32K = 0x52,     // 32KB block erase
        BLOCK_ERASE_64K = 0xD8,     // 64KB block erase
        CHIP_ERASE = 0xC7,          // Full chip erase
        READ_ID = 0x9F,             // Read manufacturer/device ID
        RESET_DEVICE = 0x99,        // Software reset
    };
    
    // Flash status register bits
    enum class StatusBit : uint8_t {
        BUSY = 0x01,               // Write/erase in progress
        WEL = 0x02,                // Write enable latch
        BP0 = 0x04,                // Block protect 0
        BP1 = 0x08,                // Block protect 1
        BP2 = 0x10,                // Block protect 2
        TB = 0x20,                 // Top/bottom protect
        SEC = 0x40,                // Sector protect
        SRP0 = 0x80,               // Status register protect 0
    };
    
    // Flash timing parameters (microseconds)
    struct FlashTiming {
        uint32_t page_program_us = 300;      // Page program time
        uint32_t sector_erase_us = 60000;    // Sector erase time  
        uint32_t block_erase_us = 200000;    // Block erase time
        uint32_t chip_erase_us = 20000000;   // Chip erase time
        uint32_t reset_us = 30;              // Reset time
        uint32_t write_enable_us = 10;       // Write enable time
    };
    
    // Flash operation statistics
    struct FlashStats {
        uint64_t read_operations = 0;
        uint64_t write_operations = 0;
        uint64_t erase_operations = 0;
        uint64_t bytes_read = 0;
        uint64_t bytes_written = 0;
        uint64_t sectors_erased = 0;
        uint64_t blocks_erased = 0;
        uint32_t chip_erases = 0;
        double average_write_time_us = 0.0;
        double average_erase_time_us = 0.0;
        std::chrono::steady_clock::time_point last_operation_time;
    };
    
    // Configuration options
    struct Config {
        std::string flash_image_path = "~/.m5tab5_emulator/flash.bin";
        bool enable_persistence = true;
        bool enable_wear_leveling = false;
        bool enable_write_protection = false;
        bool simulate_timing = true;
        bool enable_statistics = true;
        FlashTiming timing;
        uint32_t max_erase_cycles = 100000;  // Per sector
        bool auto_save_on_write = true;
        uint32_t save_interval_ms = 5000;
    };
    
    FlashController();
    explicit FlashController(const Config& config);
    ~FlashController();
    
    // Lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    bool is_initialized() const { return initialized_; }
    
    // Primary flash operations (ESP-IDF esp_flash API compatible)
    Result<void> read(Address address, void* buffer, size_t size);
    Result<void> write(Address address, const void* data, size_t size);
    Result<void> erase_sector(Address address);
    Result<void> erase_block(Address address);
    Result<void> erase_chip();
    Result<void> erase_range(Address start_address, size_t size);
    
    // Memory-mapped access (XIP mode)
    Result<uint8_t> read_byte_mapped(Address address);
    Result<uint16_t> read_word_mapped(Address address);
    Result<uint32_t> read_dword_mapped(Address address);
    Result<std::vector<uint8_t>> read_bytes_mapped(Address address, size_t size);
    
    // SPI flash command interface
    Result<void> execute_command(FlashCommand cmd, Address address = 0, 
                               const void* write_data = nullptr, size_t write_size = 0,
                               void* read_data = nullptr, size_t read_size = 0);
    
    // Flash information and status
    Result<uint32_t> get_flash_id();
    Result<uint8_t> get_status_register();
    Result<void> set_status_register(uint8_t status);
    Result<bool> is_busy();
    Result<void> wait_for_ready(uint32_t timeout_ms = 5000);
    
    // Write protection
    Result<void> write_enable();
    Result<void> write_disable();
    Result<bool> is_write_enabled();
    Result<void> set_write_protection(Address start, size_t size, bool is_protected);
    
    // Partition table integration
    Result<void> load_partition_table();
    Result<void> save_partition_table(const PartitionTable& table);
    PartitionTable* get_partition_table() const { return partition_table_.get(); }
    
    // Flash image management
    Result<void> load_flash_image();
    Result<void> save_flash_image();
    Result<void> set_flash_image_path(const std::string& path);
    std::string get_flash_image_path() const { return config_.flash_image_path; }
    
    // Address validation and translation
    Result<bool> is_valid_flash_address(Address address) const;
    Result<Address> physical_to_mapped_address(Address physical_addr) const;
    Result<Address> mapped_to_physical_address(Address mapped_addr) const;
    Result<size_t> get_sector_number(Address address) const;
    Result<size_t> get_block_number(Address address) const;
    
    // Wear leveling simulation
    Result<void> enable_wear_leveling(bool enable);
    Result<uint32_t> get_erase_count(size_t sector);
    Result<double> get_wear_level() const;
    Result<std::vector<uint32_t>> get_wear_histogram() const;
    
    // Statistics and monitoring
    const FlashStats& get_statistics() const { return stats_; }
    Result<void> reset_statistics();
    Result<void> export_statistics(const std::string& filename) const;
    
    // Debug and diagnostics  
    Result<void> dump_flash_content(Address start, size_t size) const;
    Result<void> dump_sector_info(size_t sector) const;
    Result<void> verify_flash_integrity() const;
    Result<std::string> get_flash_info() const;
    
    // Advanced operations
    Result<void> suspend_erase();
    Result<void> resume_erase();
    Result<bool> is_erase_suspended();
    Result<void> reset_device();
    
    // Background operations
    Result<void> start_background_save();
    Result<void> stop_background_save();
    
private:
    // Internal flash data management
    struct FlashData {
        std::vector<uint8_t> memory;
        std::vector<uint32_t> erase_counts;  // Per sector
        std::vector<bool> write_protected;   // Per sector
        mutable std::shared_mutex mutex;
        
        FlashData() : memory(FLASH_SIZE, 0xFF), 
                     erase_counts(FLASH_SIZE / SECTOR_SIZE, 0),
                     write_protected(FLASH_SIZE / SECTOR_SIZE, false) {}
    };
    
    // Flash state management
    struct FlashState {
        std::atomic<uint8_t> status_register{0x00};
        std::atomic<bool> write_enabled{false};
        std::atomic<bool> busy{false};
        std::atomic<bool> erase_suspended{false};
        std::atomic<bool> initialized{false};
        mutable std::mutex operation_mutex;
    };
    
    // Internal operations
    Result<void> validate_address_range(Address address, size_t size) const;
    Result<void> validate_write_operation(Address address, size_t size) const;
    Result<void> validate_erase_operation(Address address) const;
    Result<void> perform_timing_delay(uint32_t delay_us) const;
    
    // Flash memory operations
    Result<void> internal_read(Address address, void* buffer, size_t size);
    Result<void> internal_write(Address address, const void* data, size_t size);
    Result<void> internal_erase_sector(size_t sector);
    Result<void> internal_erase_block(size_t block);
    Result<void> internal_erase_chip();
    
    // Wear leveling
    Result<void> update_erase_count(size_t sector);
    Result<void> balance_wear_leveling();
    
    // Background operations
    void background_save_thread();
    Result<void> auto_save_if_needed();
    
    // File I/O operations
    Result<void> ensure_flash_directory();
    Result<void> load_from_file();
    Result<void> save_to_file();
    std::string expand_path(const std::string& path) const;
    
    // Statistics tracking
    void update_read_stats(size_t bytes);
    void update_write_stats(size_t bytes, double time_us);
    void update_erase_stats(double time_us);
    
    // Configuration and state
    Config config_;
    std::unique_ptr<FlashData> flash_data_;
    std::unique_ptr<FlashState> flash_state_;
    std::unique_ptr<PartitionTable> partition_table_;
    mutable FlashStats stats_;
    mutable std::mutex stats_mutex_;
    
    // Background save thread
    std::unique_ptr<std::thread> save_thread_;
    std::atomic<bool> save_thread_running_{false};
    std::chrono::steady_clock::time_point last_save_time_;
    
    // Initialization state
    std::atomic<bool> initialized_{false};
};

/**
 * @brief Flash memory region for integration with MemoryController
 * 
 * Provides MemoryInterface implementation for flash memory access
 * through the standard memory controller interface.
 */
class FlashMemoryRegion : public MemoryInterface {
public:
    explicit FlashMemoryRegion(FlashController* controller);
    ~FlashMemoryRegion() override = default;
    
    // MemoryInterface implementation
    EmulatorError read8(Address address, uint8_t& value) override;
    EmulatorError read16(Address address, uint16_t& value) override;
    EmulatorError read32(Address address, uint32_t& value) override;
    EmulatorError write8(Address address, uint8_t value) override;
    EmulatorError write16(Address address, uint16_t value) override;
    EmulatorError write32(Address address, uint32_t value) override;
    
    bool isValidAddress(Address address) const override;
    bool isWritableAddress(Address address) const override;
    bool isExecutableAddress(Address address) const override;
    
    // Bulk operations
    EmulatorError readBlock(Address address, void* buffer, size_t size) override;
    EmulatorError writeBlock(Address address, const void* data, size_t size) override;
    
    // Flash-specific methods
    FlashController* get_flash_controller() const { return controller_; }
    
private:
    FlashController* controller_;
};

} // namespace m5tab5::emulator::storage