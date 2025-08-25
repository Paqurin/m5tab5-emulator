#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <functional>
#include <cstdint>

namespace m5tab5::emulator::storage {

// Forward declarations
class SQLiteBackend;

/**
 * @brief Non-Volatile Storage Controller for ESP32-P4 NVS emulation
 * 
 * This class provides a complete NVS implementation that mirrors the ESP-IDF
 * NVS API behavior while using SQLite as the persistence layer.
 * 
 * Features:
 * - Key-value storage with multiple data types
 * - Namespace support for data organization
 * - Persistent storage across emulator restarts
 * - Wear leveling simulation
 * - Transaction support with commit/rollback
 * - Thread-safe operations for dual-core access
 * - Encryption support (optional)
 * - Statistics and monitoring
 */
class NVSController {
public:
    // NVS data types matching ESP-IDF
    enum class NVSType : uint8_t {
        U8 = 0x01,
        I8 = 0x11,
        U16 = 0x02,
        I16 = 0x12,
        U32 = 0x04,
        I32 = 0x14,
        U64 = 0x08,
        I64 = 0x18,
        STR = 0x21,
        BLOB = 0x42
    };

    // NVS access modes
    enum class AccessMode {
        READ_ONLY,
        READ_WRITE
    };

    // NVS error codes matching ESP-IDF
    enum class NVSError {
        OK = 0x0,
        NOT_FOUND = 0x1105,
        TYPE_MISMATCH = 0x1106,
        READ_ONLY = 0x1107,
        NOT_ENOUGH_SPACE = 0x1108,
        INVALID_NAME = 0x1109,
        INVALID_HANDLE = 0x110A,
        REMOVE_FAILED = 0x110B,
        KEY_TOO_LONG = 0x110C,
        PAGE_FULL = 0x110D,
        INVALID_STATE = 0x110E,
        INVALID_LENGTH = 0x110F
    };

    // NVS handle for namespace operations
    struct NVSHandle {
        uint32_t handle_id;
        std::string namespace_name;
        std::string partition_name;
        AccessMode mode;
        bool valid;
        
        NVSHandle() : handle_id(0), mode(AccessMode::READ_ONLY), valid(false) {}
    };

    // NVS entry for storage
    struct NVSEntry {
        std::string key;
        std::string namespace_name;
        NVSType type;
        std::vector<uint8_t> value;
        size_t size;
        uint64_t crc32;
        uint64_t timestamp;
        bool encrypted;
        
        NVSEntry() : type(NVSType::U8), size(0), crc32(0), timestamp(0), encrypted(false) {}
    };

    // NVS statistics
    struct NVSStats {
        size_t total_entries;
        size_t used_space;
        size_t free_space;
        size_t total_space;
        size_t namespaces_count;
        size_t write_operations;
        size_t read_operations;
        size_t erase_operations;
        double wear_level_avg;
        uint64_t last_commit_time;
    };

    // Configuration
    struct Config {
        std::string storage_path = "~/.m5tab5_emulator/nvs.db";
        bool enable_encryption = false;
        std::string encryption_key = "";
        size_t max_entries = 10000;
        size_t max_entry_size = 4096;
        bool enable_wear_leveling = true;
        bool auto_commit = false;
        uint32_t commit_timeout_ms = 5000;
        bool enable_statistics = true;
    };

    explicit NVSController(const Config& config = Config{});
    ~NVSController();

    // Lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    bool is_initialized() const { return initialized_; }

    // Partition management
    Result<void> init_partition(const std::string& partition_name);
    Result<void> erase_partition(const std::string& partition_name);
    std::vector<std::string> list_partitions() const;

    // Namespace operations (ESP-IDF compatible)
    Result<uint32_t> open_namespace(const std::string& namespace_name, 
                                   AccessMode mode,
                                   const std::string& partition_name = "nvs");
    Result<void> close_namespace(uint32_t handle);
    bool is_handle_valid(uint32_t handle) const;

    // Data operations - Generic
    Result<void> set_blob(uint32_t handle, const std::string& key, 
                         const void* value, size_t size);
    Result<size_t> get_blob(uint32_t handle, const std::string& key, 
                           void* value, size_t size);
    Result<size_t> get_blob_size(uint32_t handle, const std::string& key);

    // Data operations - Typed setters
    Result<void> set_u8(uint32_t handle, const std::string& key, uint8_t value);
    Result<void> set_i8(uint32_t handle, const std::string& key, int8_t value);
    Result<void> set_u16(uint32_t handle, const std::string& key, uint16_t value);
    Result<void> set_i16(uint32_t handle, const std::string& key, int16_t value);
    Result<void> set_u32(uint32_t handle, const std::string& key, uint32_t value);
    Result<void> set_i32(uint32_t handle, const std::string& key, int32_t value);
    Result<void> set_u64(uint32_t handle, const std::string& key, uint64_t value);
    Result<void> set_i64(uint32_t handle, const std::string& key, int64_t value);
    Result<void> set_str(uint32_t handle, const std::string& key, const std::string& value);

    // Data operations - Typed getters  
    Result<uint8_t> get_u8(uint32_t handle, const std::string& key);
    Result<int8_t> get_i8(uint32_t handle, const std::string& key);
    Result<uint16_t> get_u16(uint32_t handle, const std::string& key);
    Result<int16_t> get_i16(uint32_t handle, const std::string& key);
    Result<uint32_t> get_u32(uint32_t handle, const std::string& key);
    Result<int32_t> get_i32(uint32_t handle, const std::string& key);
    Result<uint64_t> get_u64(uint32_t handle, const std::string& key);
    Result<int64_t> get_i64(uint32_t handle, const std::string& key);
    Result<std::string> get_str(uint32_t handle, const std::string& key);
    Result<size_t> get_str_size(uint32_t handle, const std::string& key);

    // Entry management
    Result<void> erase_key(uint32_t handle, const std::string& key);
    Result<void> erase_all(uint32_t handle);
    bool exists(uint32_t handle, const std::string& key);
    Result<NVSType> get_key_type(uint32_t handle, const std::string& key);

    // Transaction management
    Result<void> commit(uint32_t handle);
    Result<void> commit_all();
    Result<void> rollback(uint32_t handle);

    // Enumeration and discovery
    std::vector<std::string> list_keys(uint32_t handle, NVSType type = NVSType::U8);
    std::vector<std::string> list_namespaces(const std::string& partition_name = "nvs");
    Result<size_t> get_namespace_entry_count(uint32_t handle);

    // Statistics and monitoring
    NVSStats get_stats(const std::string& partition_name = "nvs") const;
    Result<void> compact_partition(const std::string& partition_name);
    double get_wear_level(const std::string& partition_name) const;

    // Encryption support
    Result<void> set_encryption_key(const std::string& key);
    bool is_encryption_enabled() const { return config_.enable_encryption; }

    // Debug and diagnostics
    void dump_namespace(uint32_t handle) const;
    void dump_partition(const std::string& partition_name) const;
    std::string get_storage_path() const { return config_.storage_path; }

private:
    // Internal data management
    Result<void> set_entry(uint32_t handle, const std::string& key, 
                          NVSType type, const void* value, size_t size);
    Result<NVSEntry> get_entry(uint32_t handle, const std::string& key);
    
    // Handle management
    uint32_t generate_handle();
    NVSHandle* get_handle(uint32_t handle_id);
    const NVSHandle* get_handle(uint32_t handle_id) const;

    // Validation
    Result<void> validate_key(const std::string& key) const;
    Result<void> validate_namespace(const std::string& namespace_name) const;
    Result<void> validate_handle(uint32_t handle) const;

    // Utility
    std::string expand_path(const std::string& path) const;
    uint32_t calculate_crc32(const void* data, size_t size) const;
    uint64_t get_timestamp() const;

    // Storage backend interface
    Result<void> backend_set_entry(const NVSEntry& entry);
    Result<NVSEntry> backend_get_entry(const std::string& namespace_name, 
                                      const std::string& key);
    Result<void> backend_delete_entry(const std::string& namespace_name, 
                                     const std::string& key);
    Result<std::vector<std::string>> backend_list_keys(const std::string& namespace_name);

    // Configuration and state
    Config config_;
    bool initialized_ = false;
    
    // Handle management
    std::unordered_map<uint32_t, std::unique_ptr<NVSHandle>> handles_;
    uint32_t next_handle_id_ = 1;
    mutable std::mutex handles_mutex_;
    
    // Backend storage
    std::unique_ptr<SQLiteBackend> backend_;
    mutable std::mutex backend_mutex_;
    
    // Statistics
    mutable NVSStats stats_;
    mutable std::mutex stats_mutex_;
    
    // Thread safety
    mutable std::shared_mutex global_mutex_;
};

/**
 * @brief SQLite backend implementation for NVS storage
 * 
 * Provides persistent storage using SQLite database with proper indexing
 * and transaction support.
 */
class SQLiteBackend {
public:
    struct Config {
        std::string database_path;
        bool enable_wal_mode = true;
        bool enable_foreign_keys = true;
        int cache_size_kb = 2048;
        int page_size = 4096;
    };

    explicit SQLiteBackend(const Config& config);
    ~SQLiteBackend();

    // Lifecycle
    Result<void> initialize();
    Result<void> shutdown();
    bool is_initialized() const { return initialized_; }

    // Entry operations
    Result<void> insert_entry(const NVSController::NVSEntry& entry);
    Result<void> update_entry(const NVSController::NVSEntry& entry);
    Result<void> delete_entry(const std::string& namespace_name, const std::string& key);
    Result<NVSController::NVSEntry> get_entry(const std::string& namespace_name, 
                                             const std::string& key);
    Result<bool> entry_exists(const std::string& namespace_name, const std::string& key);

    // Enumeration
    Result<std::vector<std::string>> list_keys(const std::string& namespace_name);
    Result<std::vector<std::string>> list_namespaces();
    Result<size_t> count_entries(const std::string& namespace_name);

    // Maintenance
    Result<void> vacuum();
    Result<void> analyze();
    Result<NVSController::NVSStats> get_stats();

    // Transaction support
    Result<void> begin_transaction();
    Result<void> commit_transaction();
    Result<void> rollback_transaction();

private:
    struct sqlite3; // Forward declaration
    struct sqlite3_stmt; // Forward declaration

    Config config_;
    bool initialized_ = false;
    sqlite3* db_ = nullptr;
    
    // Prepared statements for performance
    sqlite3_stmt* stmt_insert_ = nullptr;
    sqlite3_stmt* stmt_update_ = nullptr;
    sqlite3_stmt* stmt_delete_ = nullptr;
    sqlite3_stmt* stmt_select_ = nullptr;
    sqlite3_stmt* stmt_exists_ = nullptr;
    sqlite3_stmt* stmt_list_keys_ = nullptr;
    sqlite3_stmt* stmt_list_namespaces_ = nullptr;
    sqlite3_stmt* stmt_count_entries_ = nullptr;

    Result<void> create_tables();
    Result<void> prepare_statements();
    void cleanup_statements();
    std::string get_sqlite_error() const;
};

} // namespace m5tab5::emulator::storage