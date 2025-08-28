#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <cstring>

namespace m5tab5::emulator::storage {

/**
 * @brief ESP32-P4 Partition Table Management
 * 
 * Implements ESP-IDF compatible partition table parsing and management.
 * Supports standard ESP32-P4 partition layout with partition types:
 * - app (application partitions)
 * - data (data partitions including NVS, PHY, OTA)
 * - Custom user partitions
 */
class PartitionTable {
public:
    // ESP32 partition types (matching esp_partition.h)
    enum class PartitionType : uint8_t {
        APP = 0x00,          // Application partition
        DATA = 0x01,         // Data partition
        CUSTOM_BASE = 0x40   // Base for custom partition types
    };
    
    // ESP32 partition subtypes
    enum class PartitionSubtype : uint8_t {
        // APP subtypes
        APP_FACTORY = 0x00,       // Factory application
        APP_OTA_MIN = 0x10,       // OTA partition minimum
        APP_OTA_0 = 0x10,         // OTA_0 partition
        APP_OTA_1 = 0x11,         // OTA_1 partition
        APP_OTA_2 = 0x12,         // OTA_2 partition
        APP_OTA_3 = 0x13,         // OTA_3 partition
        APP_OTA_4 = 0x14,         // OTA_4 partition
        APP_OTA_5 = 0x15,         // OTA_5 partition
        APP_OTA_6 = 0x16,         // OTA_6 partition
        APP_OTA_7 = 0x17,         // OTA_7 partition
        APP_OTA_8 = 0x18,         // OTA_8 partition
        APP_OTA_9 = 0x19,         // OTA_9 partition
        APP_OTA_10 = 0x1A,        // OTA_10 partition
        APP_OTA_11 = 0x1B,        // OTA_11 partition
        APP_OTA_12 = 0x1C,        // OTA_12 partition
        APP_OTA_13 = 0x1D,        // OTA_13 partition
        APP_OTA_14 = 0x1E,        // OTA_14 partition
        APP_OTA_15 = 0x1F,        // OTA_15 partition
        APP_OTA_MAX = 0x1F,       // OTA partition maximum
        APP_TEST = 0x20,          // Test application
        
        // DATA subtypes
        DATA_OTA = 0x00,          // OTA selection partition
        DATA_PHY = 0x01,          // PHY init data partition
        DATA_NVS = 0x02,          // NVS partition
        DATA_COREDUMP = 0x03,     // Core dump partition
        DATA_NVS_KEYS = 0x04,     // NVS keys partition
        DATA_EFUSE_EM = 0x05,     // Emulated eFuse partition
        DATA_UNDEFINED = 0x06,    // Undefined data partition
        DATA_ESPHTTPD = 0x80,     // ESPHTTPD partition
        DATA_FAT = 0x81,          // FAT partition
        DATA_SPIFFS = 0x82,       // SPIFFS partition
        DATA_LITTLEFS = 0x83,     // LittleFS partition
        
        // Custom subtypes start at 0x00
        CUSTOM_BASE = 0x00
    };
    
    // Partition flags
    enum class PartitionFlag : uint32_t {
        NONE = 0x00000000,
        ENCRYPTED = 0x00000001,    // Partition is encrypted
        READONLY = 0x00000002      // Partition is read-only
    };
    
    // Standard partition table locations
    static constexpr Address PARTITION_TABLE_OFFSET = 0x8000;   // 32KB from start
    static constexpr size_t PARTITION_TABLE_SIZE = 0xC00;       // 3KB
    static constexpr size_t MAX_PARTITION_ENTRIES = 95;         // Max entries
    static constexpr size_t PARTITION_ENTRY_SIZE = 32;          // Size per entry
    static constexpr uint16_t PARTITION_TABLE_MAGIC = 0x50AA;   // Magic number
    
    // Partition entry structure (matches ESP-IDF format)
    struct PartitionEntry {
        PartitionType type;
        PartitionSubtype subtype;
        Address offset;             // Partition offset in flash
        size_t size;               // Partition size in bytes
        std::string label;         // Partition label (max 16 chars)
        PartitionFlag flags;
        
        // Computed fields
        Address end_offset() const { return offset + size; }
        bool is_app_partition() const { return type == PartitionType::APP; }
        bool is_data_partition() const { return type == PartitionType::DATA; }
        bool is_ota_partition() const { 
            return type == PartitionType::APP && 
                   subtype >= PartitionSubtype::APP_OTA_MIN && 
                   subtype <= PartitionSubtype::APP_OTA_MAX; 
        }
        bool is_encrypted() const { return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(PartitionFlag::ENCRYPTED)) != 0; }
        bool is_readonly() const { return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(PartitionFlag::READONLY)) != 0; }
        
        PartitionEntry() : type(PartitionType::DATA), subtype(PartitionSubtype::DATA_UNDEFINED),
                          offset(0), size(0), flags(PartitionFlag::NONE) {}
    };
    
    // Partition table header
    struct PartitionTableHeader {
        uint16_t magic;           // Magic number (0x50AA)
        uint16_t version;         // Version (currently 1)
        uint32_t num_entries;     // Number of partition entries
        uint32_t crc32;          // CRC32 of the table
        uint8_t reserved[20];     // Reserved bytes
        
        PartitionTableHeader() : magic(PARTITION_TABLE_MAGIC), version(1), 
                               num_entries(0), crc32(0) {
            std::memset(reserved, 0, sizeof(reserved));
        }
    };
    
    PartitionTable();
    ~PartitionTable();
    
    // Partition table lifecycle
    Result<void> initialize();
    Result<void> clear();
    bool is_empty() const { return partitions_.empty(); }
    size_t get_partition_count() const { return partitions_.size(); }
    
    // Partition table I/O
    Result<void> load_from_flash(const uint8_t* flash_data, size_t flash_size);
    Result<void> save_to_flash(uint8_t* flash_data, size_t flash_size);
    Result<void> load_from_csv(const std::string& csv_content);
    Result<std::string> save_to_csv() const;
    
    // Partition management
    Result<void> add_partition(const PartitionEntry& partition);
    Result<void> remove_partition(const std::string& label);
    Result<void> remove_partition_by_index(size_t index);
    Result<PartitionEntry> get_partition(const std::string& label) const;
    Result<PartitionEntry> get_partition_by_index(size_t index) const;
    Result<size_t> find_partition_index(const std::string& label) const;
    
    // Partition queries
    Result<PartitionEntry> find_partition_by_type(PartitionType type, 
                                                 PartitionSubtype subtype = PartitionSubtype::CUSTOM_BASE) const;
    Result<std::vector<PartitionEntry>> find_partitions_by_type(PartitionType type) const;
    Result<PartitionEntry> find_partition_by_address(Address address) const;
    Result<bool> is_address_in_partition(Address address, const std::string& label) const;
    
    // Standard partition operations
    Result<PartitionEntry> get_factory_app_partition() const;
    Result<PartitionEntry> get_ota_app_partition(uint8_t ota_index) const;
    Result<PartitionEntry> get_nvs_partition() const;
    Result<PartitionEntry> get_phy_partition() const;
    Result<PartitionEntry> get_ota_data_partition() const;
    
    // Partition validation
    Result<void> validate() const;
    Result<void> validate_partition(const PartitionEntry& partition) const;
    Result<bool> check_overlap() const;
    Result<bool> is_partition_aligned(const PartitionEntry& partition) const;
    
    // Partition iteration
    std::vector<PartitionEntry> get_all_partitions() const { return partitions_; }
    std::vector<std::string> get_partition_labels() const;
    std::vector<PartitionEntry> get_app_partitions() const;
    std::vector<PartitionEntry> get_data_partitions() const;
    std::vector<PartitionEntry> get_ota_partitions() const;
    
    // Default partition table generation
    static Result<PartitionTable> create_default_table();
    static Result<PartitionTable> create_minimal_table();
    static Result<PartitionTable> create_ota_table(uint8_t num_ota_partitions = 2);
    
    // Partition table utilities
    Result<size_t> calculate_table_size() const;
    Result<uint32_t> calculate_crc32() const;
    Result<void> update_crc32();
    Result<bool> verify_crc32() const;
    
    // Address and size utilities
    static Address align_address(Address address, size_t alignment = 0x1000);
    static size_t align_size(size_t size, size_t alignment = 0x1000);
    static bool is_address_aligned(Address address, size_t alignment = 0x1000);
    static bool is_size_aligned(size_t size, size_t alignment = 0x1000);
    
    // String conversion utilities
    static std::string partition_type_to_string(PartitionType type);
    static std::string partition_subtype_to_string(PartitionType type, PartitionSubtype subtype);
    static Result<PartitionType> string_to_partition_type(const std::string& str);
    static Result<PartitionSubtype> string_to_partition_subtype(const std::string& type_str, const std::string& subtype_str);
    
    // Debug and diagnostics
    void dump_partition_table() const;
    void dump_partition(const std::string& label) const;
    Result<std::string> get_partition_info(const std::string& label) const;
    Result<std::string> get_table_summary() const;
    
private:
    // Internal operations
    Result<void> parse_partition_entry(const uint8_t* entry_data, PartitionEntry& partition);
    Result<void> serialize_partition_entry(const PartitionEntry& partition, uint8_t* entry_data) const;
    Result<void> parse_csv_line(const std::string& line, size_t line_number, PartitionEntry& partition);
    std::string partition_to_csv_line(const PartitionEntry& partition) const;
    
    // Validation helpers
    Result<void> validate_label(const std::string& label) const;
    Result<void> validate_address_range(Address offset, size_t size) const;
    Result<void> check_partition_overlap(const PartitionEntry& partition) const;
    
    // Utility functions
    uint32_t calculate_crc32(const void* data, size_t size) const;
    std::string trim_string(const std::string& str) const;
    std::vector<std::string> split_string(const std::string& str, char delimiter) const;
    
    // Partition storage
    std::vector<PartitionEntry> partitions_;
    PartitionTableHeader header_;
    
    // State tracking
    bool initialized_ = false;
};

/**
 * @brief ESP-IDF compatible partition iterator
 * 
 * Provides esp_partition_iterator interface for partition enumeration.
 */
class PartitionIterator {
public:
    explicit PartitionIterator(const PartitionTable* table);
    ~PartitionIterator() = default;
    
    // Iterator operations
    bool has_next() const;
    Result<PartitionTable::PartitionEntry> next();
    void reset();
    
    // Filtering
    void filter_by_type(PartitionTable::PartitionType type);
    void filter_by_subtype(PartitionTable::PartitionSubtype subtype);
    void filter_by_label(const std::string& label);
    void clear_filters();
    
private:
    const PartitionTable* table_;
    size_t current_index_;
    
    // Filter criteria
    std::optional<PartitionTable::PartitionType> type_filter_;
    std::optional<PartitionTable::PartitionSubtype> subtype_filter_;
    std::optional<std::string> label_filter_;
    
    bool matches_filters(const PartitionTable::PartitionEntry& partition) const;
};

} // namespace m5tab5::emulator::storage