#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/storage/flash_controller.hpp"
#include "emulator/storage/partition_table.hpp"

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <atomic>
#include <mutex>

namespace m5tab5::emulator::storage {

/**
 * @brief OTA (Over-The-Air) Update Manager for ESP32-P4
 * 
 * Provides authentic ESP32-P4 OTA functionality including:
 * - OTA partition management (OTA_0, OTA_1, factory)
 * - Boot partition selection and validation
 * - Secure OTA updates with SHA256 checksums
 * - OTA rollback mechanism for failed boots
 * - Boot counter and watchdog integration
 * - ESP32-P4 compatible OTA data structures
 * - Support for encrypted OTA images (foundation)
 */
class OTAManager {
public:
    // ESP32-P4 OTA partition types
    enum class PartitionType : uint8_t {
        APP = 0x00,         // Application partition
        DATA = 0x01         // Data partition (OTA data)
    };
    
    enum class PartitionSubtype : uint8_t {
        FACTORY = 0x00,     // Factory app partition
        OTA_0 = 0x10,       // OTA_0 partition
        OTA_1 = 0x11,       // OTA_1 partition
        OTA_2 = 0x12,       // OTA_2 partition (future)
        OTA_3 = 0x13,       // OTA_3 partition (future)
        OTA_4 = 0x14,       // OTA_4 partition (future)
        OTA_5 = 0x15,       // OTA_5 partition (future)
        OTA_6 = 0x16,       // OTA_6 partition (future)
        OTA_7 = 0x17,       // OTA_7 partition (future)
        OTA_DATA = 0x00     // OTA data partition
    };
    
    // OTA state values
    enum class OTAState : uint32_t {
        NEW = 0x00000000,           // Partition is new (never booted)
        PENDING_VERIFY = 0x01234567, // Partition is pending verification
        VALID = 0xABCDEF01,         // Partition is valid and bootable
        INVALID = 0x12345678,       // Partition is invalid (boot failed)
        ABORTED = 0x87654321        // OTA was aborted
    };
    
    // Boot partition selection
    enum class BootPartition {
        FACTORY,    // Boot from factory partition
        OTA_0,      // Boot from OTA_0 partition
        OTA_1,      // Boot from OTA_1 partition
        UNKNOWN     // Unknown/invalid partition
    };
    
    // OTA data structure (compatible with ESP-IDF)
    struct OTAData {
        uint32_t seq;               // Sequence counter
        uint8_t seq_label[20];      // Sequence label (unused, for compatibility)
        uint32_t ota_state;         // OTA state
        uint32_t crc;               // CRC32 of the structure
    } __attribute__((packed));
    
    // OTA partition info
    struct OTAPartitionInfo {
        std::string label;          // Partition label
        PartitionSubtype subtype;   // Partition subtype
        Address start_address;      // Start address in flash
        size_t size;               // Partition size
        bool encrypted;            // Is partition encrypted
        OTAState state;            // Current OTA state
        uint32_t seq;              // Sequence number
        std::string version;       // App version (if available)
        std::vector<uint8_t> sha256; // SHA256 hash
    };
    
    // OTA update status
    enum class UpdateStatus {
        IDLE,               // No update in progress
        DOWNLOADING,        // Downloading update
        VERIFYING,          // Verifying downloaded data
        INSTALLING,         // Installing to flash
        FINALIZING,         // Finalizing update
        COMPLETE,           // Update complete (pending reboot)
        ERROR,              // Update failed
        ROLLED_BACK         // Update rolled back
    };
    
    // OTA update progress callback
    using ProgressCallback = std::function<void(size_t bytes_written, size_t total_size, int percentage)>;
    
    // OTA update configuration
    struct Config {
        size_t max_update_size = 8 * 1024 * 1024;  // 8MB max update size
        uint32_t boot_timeout_ms = 30000;          // 30 second boot timeout
        uint32_t max_boot_attempts = 3;            // Maximum boot attempts
        bool enable_rollback = true;               // Enable automatic rollback
        bool verify_sha256 = true;                 // Verify SHA256 checksums
        bool enable_secure_boot = false;           // Secure boot validation
        std::string factory_version = "1.0.0";    // Factory firmware version
    };
    
    // OTA update statistics
    struct Statistics {
        uint32_t successful_updates = 0;
        uint32_t failed_updates = 0;
        uint32_t rollbacks = 0;
        uint64_t bytes_downloaded = 0;
        uint64_t total_download_time_ms = 0;
        std::chrono::steady_clock::time_point last_update_time;
        std::string last_update_version;
        UpdateStatus last_update_status = UpdateStatus::IDLE;
    };
    
    explicit OTAManager(FlashController* flash_controller);
    explicit OTAManager(FlashController* flash_controller, const Config& config);
    ~OTAManager();
    
    // Lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    bool is_initialized() const { return initialized_; }
    
    // Boot partition management
    Result<BootPartition> get_boot_partition();
    Result<void> set_boot_partition(BootPartition partition);
    Result<BootPartition> get_next_update_partition();
    Result<void> mark_app_valid(BootPartition partition);
    Result<void> mark_app_invalid(BootPartition partition);
    
    // OTA data management
    Result<void> load_ota_data();
    Result<void> save_ota_data();
    Result<void> erase_ota_data();
    Result<OTAData> get_ota_data(int index = 0); // 0 = primary, 1 = backup
    Result<void> set_ota_data(const OTAData& data, int index = 0);
    
    // Partition discovery and management
    Result<std::vector<OTAPartitionInfo>> scan_ota_partitions();
    Result<OTAPartitionInfo> get_partition_info(BootPartition partition);
    Result<Address> get_partition_address(BootPartition partition);
    Result<size_t> get_partition_size(BootPartition partition);
    
    // OTA update operations
    Result<void> begin_update(size_t update_size, BootPartition target_partition = BootPartition::UNKNOWN);
    Result<size_t> write_update_data(const void* data, size_t size);
    Result<void> end_update(bool commit = true);
    Result<void> abort_update();
    
    // Update validation and verification
    Result<void> set_expected_sha256(const std::vector<uint8_t>& sha256);
    Result<bool> verify_update_integrity();
    Result<std::vector<uint8_t>> calculate_partition_sha256(BootPartition partition);
    
    // Boot management and rollback
    Result<void> commit_update();                           // Commit current update
    Result<void> rollback_to_previous();                   // Rollback to previous valid partition
    Result<void> set_boot_attempt_counter(uint32_t count); // Set boot attempt counter
    Result<uint32_t> get_boot_attempt_counter();           // Get current boot attempts
    Result<void> reset_boot_attempt_counter();             // Reset boot counter (successful boot)
    
    // Status and information
    UpdateStatus get_update_status() const { return update_status_; }
    size_t get_update_progress() const { return bytes_written_; }
    size_t get_update_total_size() const { return update_total_size_; }
    int get_update_percentage() const;
    const Statistics& get_statistics() const { return stats_; }
    
    // Version management
    Result<void> set_app_version(const std::string& version, BootPartition partition);
    Result<std::string> get_app_version(BootPartition partition);
    Result<std::string> get_running_app_version();
    
    // Factory reset and recovery
    Result<void> factory_reset();                          // Reset to factory partition
    Result<void> erase_ota_partition(BootPartition partition); // Erase specific OTA partition
    Result<void> erase_all_ota_partitions();              // Erase all OTA partitions
    
    // Debugging and diagnostics
    Result<void> dump_ota_info() const;
    Result<void> dump_partition_info(BootPartition partition) const;
    Result<bool> validate_ota_data_integrity();
    Result<void> repair_ota_data();
    
    // Progress callback management
    void set_progress_callback(ProgressCallback callback) { progress_callback_ = callback; }
    void clear_progress_callback() { progress_callback_ = nullptr; }
    
private:
    // Internal OTA data structures
    struct OTAUpdateState {
        BootPartition target_partition = BootPartition::UNKNOWN;
        Address write_address = 0;
        size_t bytes_written = 0;
        size_t total_size = 0;
        std::vector<uint8_t> expected_sha256;
        std::chrono::steady_clock::time_point start_time;
        bool in_progress = false;
    };
    
    // Internal operations
    Result<void> scan_and_cache_partitions();
    Result<void> validate_ota_partitions();
    Result<void> initialize_ota_data();
    Result<void> repair_corrupted_ota_data();
    
    // Partition management helpers
    Result<const PartitionTable::PartitionEntry*> find_partition(PartitionSubtype subtype);
    Result<BootPartition> subtype_to_boot_partition(PartitionSubtype subtype);
    Result<PartitionSubtype> boot_partition_to_subtype(BootPartition partition);
    Result<std::string> boot_partition_to_string(BootPartition partition);
    
    // OTA data validation and CRC
    uint32_t calculate_ota_data_crc(const OTAData& data);
    Result<bool> validate_ota_data(const OTAData& data);
    Result<void> update_ota_sequence_number();
    Result<uint32_t> get_next_sequence_number();
    
    // Flash operations
    Result<void> read_partition_data(BootPartition partition, Address offset, 
                                   void* buffer, size_t size);
    Result<void> write_partition_data(BootPartition partition, Address offset, 
                                    const void* data, size_t size);
    Result<void> erase_partition_range(BootPartition partition, Address offset, size_t size);
    
    // Update state management
    Result<void> set_update_status(UpdateStatus status);
    Result<void> notify_progress();
    Result<void> cleanup_update_state();
    
    // Boot validation
    Result<void> increment_boot_attempts();
    Result<void> check_boot_timeout();
    Result<void> handle_boot_failure();
    
    // Statistics tracking
    void update_success_stats();
    void update_failure_stats();
    void update_rollback_stats();
    
    // Configuration and state
    Config config_;
    FlashController* flash_controller_;
    std::unique_ptr<PartitionTable> partition_table_;
    
    // Cached partition information
    std::vector<OTAPartitionInfo> ota_partitions_;
    Address ota_data_address_ = 0;
    size_t ota_data_size_ = 0;
    
    // Current OTA state
    std::unique_ptr<OTAUpdateState> update_state_;
    std::atomic<UpdateStatus> update_status_{UpdateStatus::IDLE};
    std::atomic<size_t> bytes_written_{0};
    std::atomic<size_t> update_total_size_{0};
    
    // Boot management
    std::atomic<BootPartition> current_boot_partition_{BootPartition::FACTORY};
    std::atomic<uint32_t> boot_attempt_counter_{0};
    
    // Callbacks and statistics
    ProgressCallback progress_callback_;
    mutable Statistics stats_;
    
    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex partition_mutex_;
    mutable std::mutex stats_mutex_;
    
    // Initialization state
    std::atomic<bool> initialized_{false};
};

} // namespace m5tab5::emulator::storage