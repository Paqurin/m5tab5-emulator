#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/storage/flash_controller.hpp"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <chrono>
#include <functional>

namespace m5tab5::emulator::storage {

/**
 * @brief SPIFFS (SPI Flash File System) Implementation for ESP32-P4
 * 
 * Provides complete SPIFFS filesystem functionality with:
 * - Authentic ESP32-P4 SPIFFS behavior and format compatibility
 * - Wear leveling and bad block handling
 * - File and directory operations (create, read, write, delete, rename)
 * - Metadata handling (timestamps, attributes, permissions)
 * - Fragmentation management and garbage collection
 * - Thread-safe operations for FreeRTOS integration
 * - Complete persistence across emulator restarts
 */
class SPIFFSFileSystem {
public:
    // SPIFFS configuration constants
    static constexpr size_t SPIFFS_PAGE_SIZE = 256;         // Standard SPIFFS page size
    static constexpr size_t SPIFFS_BLOCK_SIZE = 4096;       // Match flash sector size
    static constexpr size_t SPIFFS_MAX_NAME_LENGTH = 32;    // Maximum filename length
    static constexpr size_t SPIFFS_META_SIZE = 32;          // Metadata size per object
    static constexpr size_t SPIFFS_PAGES_PER_BLOCK = SPIFFS_BLOCK_SIZE / SPIFFS_PAGE_SIZE;
    static constexpr uint32_t SPIFFS_MAGIC = 0x20160315;    // SPIFFS format magic number
    static constexpr uint16_t SPIFFS_VERSION = 0x0001;      // Version number
    
    // SPIFFS object types
    enum class ObjectType : uint8_t {
        DELETED = 0x00,
        FILE = 0x01,
        DIRECTORY = 0x02,  
        HARD_LINK = 0x03,
        SOFT_LINK = 0x04,
        INVALID = 0xFF
    };
    
    // SPIFFS page types
    enum class PageType : uint8_t {
        DELETED = 0x00,
        OBJECT_LU = 0x01,      // Object lookup
        OBJECT_IX = 0x02,      // Object index
        OBJECT_DATA = 0x03,    // Object data
        INVALID = 0xFF
    };
    
    // File access modes
    enum class AccessMode : uint8_t {
        READ = 0x01,
        WRITE = 0x02,
        CREATE = 0x04,
        TRUNCATE = 0x08,
        APPEND = 0x10,
        EXCLUSIVE = 0x20
    };
    
    // File seek modes
    enum class SeekMode : uint8_t {
        SET = 0,    // From beginning
        CUR = 1,    // From current position
        END = 2     // From end
    };
    
    // SPIFFS superblock structure
    struct Superblock {
        uint32_t magic;                    // Magic number
        uint16_t version;                  // Version
        uint16_t page_size;                // Page size
        uint32_t block_size;               // Block size
        uint32_t block_count;              // Total blocks
        uint32_t pages_per_block;          // Pages per block
        uint32_t free_blocks;              // Free blocks
        uint32_t free_pages;               // Free pages
        uint32_t total_objects;            // Total objects
        uint32_t max_objects;              // Maximum objects
        uint32_t used_pages;               // Used pages
        uint32_t deleted_pages;            // Deleted pages
        uint64_t last_gc_time;             // Last garbage collection time
        uint32_t gc_cycles;                // Garbage collection cycles
        uint32_t wear_leveling_counter;    // Wear leveling counter
        uint8_t reserved[56];              // Reserved for future use
    } __attribute__((packed));
    
    // Object header structure
    struct ObjectHeader {
        uint32_t object_id;                // Unique object ID
        ObjectType type;                   // Object type
        uint8_t name_length;               // Name length
        uint16_t flags;                    // Object flags
        uint32_t size;                     // Object size in bytes
        uint32_t pages_used;               // Number of pages used
        uint64_t created_time;             // Creation timestamp
        uint64_t modified_time;            // Modification timestamp
        uint32_t checksum;                 // Header checksum
        char name[SPIFFS_MAX_NAME_LENGTH]; // Object name
    } __attribute__((packed));
    
    // Page header structure
    struct PageHeader {
        uint32_t object_id;                // Object ID this page belongs to
        PageType type;                     // Page type
        uint8_t flags;                     // Page flags
        uint16_t data_size;                // Data size in this page
        uint32_t page_index;               // Page index within object
        uint32_t next_page;                // Next page in chain (0 = end)
        uint32_t checksum;                 // Page header checksum
        uint8_t reserved[8];               // Reserved
    } __attribute__((packed));
    
    // File handle for open files
    struct FileHandle {
        uint32_t object_id;
        uint32_t position;
        AccessMode mode;
        bool is_open;
        std::chrono::steady_clock::time_point last_access;
        std::shared_ptr<ObjectHeader> header;
        std::vector<uint32_t> page_chain;  // Cached page chain
    };
    
    // Directory entry
    struct DirectoryEntry {
        uint32_t object_id;
        ObjectType type;
        std::string name;
        uint32_t size;
        uint64_t created_time;
        uint64_t modified_time;
        uint16_t flags;
    };
    
    // Filesystem statistics
    struct Statistics {
        uint64_t total_bytes;
        uint64_t used_bytes;
        uint64_t free_bytes;
        uint32_t total_objects;
        uint32_t free_objects;
        uint32_t total_pages;
        uint32_t used_pages;
        uint32_t free_pages;
        uint32_t deleted_pages;
        uint32_t fragmented_pages;
        uint32_t gc_cycles;
        double fragmentation_ratio;
        uint64_t total_reads;
        uint64_t total_writes;
        uint64_t bytes_read;
        uint64_t bytes_written;
        std::chrono::steady_clock::time_point last_gc_time;
    };
    
    // Configuration
    struct Config {
        Address partition_start = 0;
        size_t partition_size = 0;
        bool enable_wear_leveling = true;
        bool enable_garbage_collection = true;
        uint32_t gc_trigger_threshold = 90;  // Percentage full before GC
        uint32_t max_open_files = 16;
        bool enable_metadata_cache = true;
        size_t metadata_cache_size = 64;
        bool enable_write_cache = true;
        size_t write_cache_size = 8;  // Pages
        uint32_t max_name_length = SPIFFS_MAX_NAME_LENGTH;
        bool enable_checksums = true;
        bool enable_statistics = true;
    };
    
    explicit SPIFFSFileSystem(FlashController* flash_controller);
    explicit SPIFFSFileSystem(FlashController* flash_controller, const Config& config);
    ~SPIFFSFileSystem();
    
    // Lifecycle management
    Result<void> initialize(Address partition_start, size_t partition_size);
    Result<void> mount();
    Result<void> unmount();
    Result<void> format();
    bool is_mounted() const { return mounted_; }
    
    // File operations
    Result<int> open(const std::string& path, AccessMode mode);
    Result<void> close(int fd);
    Result<size_t> read(int fd, void* buffer, size_t size);
    Result<size_t> write(int fd, const void* data, size_t size);
    Result<long> seek(int fd, long offset, SeekMode whence);
    Result<long> tell(int fd);
    Result<void> flush(int fd);
    Result<void> sync(int fd);
    
    // File management
    Result<void> unlink(const std::string& path);
    Result<void> rename(const std::string& old_path, const std::string& new_path);
    Result<void> truncate(const std::string& path, size_t size);
    Result<void> truncate(int fd, size_t size);
    
    // File information
    Result<bool> exists(const std::string& path);
    Result<size_t> size(const std::string& path);
    Result<ObjectHeader> stat(const std::string& path);
    Result<void> utime(const std::string& path, uint64_t atime, uint64_t mtime);
    
    // Directory operations
    Result<std::vector<DirectoryEntry>> list_directory(const std::string& path = "/");
    Result<std::vector<std::string>> list_files();
    Result<void> mkdir(const std::string& path);  // Note: Limited SPIFFS directory support
    Result<void> rmdir(const std::string& path);
    
    // Filesystem information
    Statistics get_statistics() const;
    Result<void> get_filesystem_info(uint64_t& total_bytes, uint64_t& used_bytes);
    Result<uint32_t> get_free_space();
    Result<uint32_t> get_used_space();
    
    // Maintenance operations
    Result<void> garbage_collect();
    Result<void> defragment();
    Result<void> check_filesystem();
    Result<void> repair_filesystem();
    Result<double> get_fragmentation_ratio();
    
    // Advanced operations
    Result<void> set_file_flags(const std::string& path, uint16_t flags);
    Result<uint16_t> get_file_flags(const std::string& path);
    Result<void> flush_all();
    Result<void> sync_all();
    
    // Debug and diagnostics
    Result<void> dump_filesystem_info() const;
    Result<void> dump_object_info(uint32_t object_id) const;
    Result<void> dump_page_info(uint32_t page_num) const;
    Result<void> verify_filesystem_integrity();
    
    // Cache management
    Result<void> flush_metadata_cache();
    Result<void> flush_write_cache();
    Result<void> invalidate_cache();
    
private:
    // Internal data structures
    struct PageInfo {
        uint32_t object_id;
        PageType type;
        uint32_t page_index;
        uint32_t next_page;
        uint16_t data_size;
        bool is_dirty;
        std::chrono::steady_clock::time_point last_access;
    };
    
    struct ObjectInfo {
        std::shared_ptr<ObjectHeader> header;
        std::vector<uint32_t> page_chain;
        bool is_dirty;
        std::chrono::steady_clock::time_point last_access;
    };
    
    // Core filesystem operations
    Result<void> read_superblock();
    Result<void> write_superblock();
    Result<void> scan_filesystem();
    Result<void> build_object_table();
    Result<void> build_free_page_list();
    
    // Object management
    Result<uint32_t> allocate_object_id();
    Result<void> free_object(uint32_t object_id);
    Result<std::shared_ptr<ObjectHeader>> find_object(const std::string& name);
    Result<std::shared_ptr<ObjectHeader>> get_object_header(uint32_t object_id);
    Result<void> update_object_header(uint32_t object_id, const ObjectHeader& header);
    
    // Page management
    Result<uint32_t> allocate_page();
    Result<void> free_page(uint32_t page_num);
    Result<std::vector<uint32_t>> allocate_pages(size_t count);
    Result<void> free_page_chain(uint32_t start_page);
    Result<std::vector<uint32_t>> get_page_chain(uint32_t object_id);
    
    // Data I/O operations
    Result<void> read_page(uint32_t page_num, void* buffer, size_t size, size_t offset = 0);
    Result<void> write_page(uint32_t page_num, const void* data, size_t size, size_t offset = 0);
    Result<void> erase_page(uint32_t page_num);
    Result<PageHeader> read_page_header(uint32_t page_num);
    Result<void> write_page_header(uint32_t page_num, const PageHeader& header);
    
    // Cache operations
    Result<void> cache_object(uint32_t object_id, std::shared_ptr<ObjectHeader> header);
    Result<void> cache_page(uint32_t page_num, const PageInfo& info);
    Result<void> flush_dirty_pages();
    Result<void> evict_cache_entry(uint32_t object_id);
    
    // File handle management
    Result<int> allocate_file_handle(uint32_t object_id, AccessMode mode);
    Result<void> free_file_handle(int fd);
    Result<FileHandle*> get_file_handle(int fd);
    bool is_valid_file_handle(int fd) const;
    
    // Garbage collection
    Result<void> trigger_garbage_collection_if_needed();
    Result<void> perform_garbage_collection();
    Result<void> compact_object(uint32_t object_id);
    Result<std::vector<uint32_t>> find_fragmented_objects();
    
    // Wear leveling
    Result<void> balance_wear_leveling();
    Result<uint32_t> select_least_worn_block();
    Result<void> migrate_object_to_block(uint32_t object_id, uint32_t target_block);
    
    // Address translation and validation
    Result<Address> page_to_flash_address(uint32_t page_num);
    Result<uint32_t> flash_address_to_page(Address address);
    Result<bool> is_valid_page_number(uint32_t page_num);
    Result<bool> is_valid_object_id(uint32_t object_id);
    
    // Checksum operations
    uint32_t calculate_header_checksum(const ObjectHeader& header);
    uint32_t calculate_page_checksum(const PageHeader& header, const void* data, size_t size);
    Result<bool> verify_object_checksum(const ObjectHeader& header);
    Result<bool> verify_page_checksum(uint32_t page_num);
    
    // Statistics and monitoring
    void update_read_stats(size_t bytes);
    void update_write_stats(size_t bytes);
    void update_gc_stats();
    Result<void> recalculate_statistics();
    
    // Error recovery
    Result<void> recover_from_corruption();
    Result<void> rebuild_object_table();
    Result<void> mark_bad_pages();
    
    // Configuration and state
    Config config_;
    FlashController* flash_controller_;
    std::unique_ptr<Superblock> superblock_;
    
    // Object and page management
    std::unordered_map<uint32_t, std::shared_ptr<ObjectInfo>> object_cache_;
    std::unordered_map<uint32_t, PageInfo> page_cache_;
    std::unordered_map<std::string, uint32_t> name_to_object_map_;
    std::vector<uint32_t> free_pages_;
    std::vector<uint32_t> free_object_ids_;
    
    // File handles
    std::unordered_map<int, std::unique_ptr<FileHandle>> file_handles_;
    std::atomic<int> next_fd_{1};  // File descriptor counter
    
    // Write cache
    struct WriteCache {
        std::unordered_map<uint32_t, std::vector<uint8_t>> cached_pages;
        std::mutex mutex;
        size_t max_size;
        size_t current_size = 0;
    };
    std::unique_ptr<WriteCache> write_cache_;
    
    // Thread safety
    mutable std::shared_mutex filesystem_mutex_;
    mutable std::mutex cache_mutex_;
    mutable std::mutex handle_mutex_;
    mutable std::mutex stats_mutex_;
    
    // State management
    std::atomic<bool> mounted_{false};
    std::atomic<bool> initialized_{false};
    Address partition_start_ = 0;
    size_t partition_size_ = 0;
    uint32_t total_pages_ = 0;
    uint32_t pages_per_block_ = 0;
    uint32_t total_blocks_ = 0;
    
    // Statistics
    mutable Statistics stats_;
    std::chrono::steady_clock::time_point mount_time_;
};

// Bitwise operators for AccessMode enum (outside class)
inline SPIFFSFileSystem::AccessMode operator|(SPIFFSFileSystem::AccessMode lhs, SPIFFSFileSystem::AccessMode rhs) {
    return static_cast<SPIFFSFileSystem::AccessMode>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

inline SPIFFSFileSystem::AccessMode operator&(SPIFFSFileSystem::AccessMode lhs, SPIFFSFileSystem::AccessMode rhs) {
    return static_cast<SPIFFSFileSystem::AccessMode>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

inline SPIFFSFileSystem::AccessMode operator^(SPIFFSFileSystem::AccessMode lhs, SPIFFSFileSystem::AccessMode rhs) {
    return static_cast<SPIFFSFileSystem::AccessMode>(static_cast<uint8_t>(lhs) ^ static_cast<uint8_t>(rhs));
}

inline SPIFFSFileSystem::AccessMode operator~(SPIFFSFileSystem::AccessMode mode) {
    return static_cast<SPIFFSFileSystem::AccessMode>(~static_cast<uint8_t>(mode));
}

inline SPIFFSFileSystem::AccessMode& operator|=(SPIFFSFileSystem::AccessMode& lhs, SPIFFSFileSystem::AccessMode rhs) {
    lhs = lhs | rhs;
    return lhs;
}

inline SPIFFSFileSystem::AccessMode& operator&=(SPIFFSFileSystem::AccessMode& lhs, SPIFFSFileSystem::AccessMode rhs) {
    lhs = lhs & rhs;
    return lhs;
}

inline SPIFFSFileSystem::AccessMode& operator^=(SPIFFSFileSystem::AccessMode& lhs, SPIFFSFileSystem::AccessMode rhs) {
    lhs = lhs ^ rhs;
    return lhs;
}

} // namespace m5tab5::emulator::storage