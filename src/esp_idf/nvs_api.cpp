#include "emulator/esp_idf/nvs.h"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/shutdown_manager.hpp"

#include <unordered_map>
#include <mutex>
#include <string>
#include <filesystem>
#include <fstream>
#include <memory>

#ifdef HAVE_SQLITE3
#include <sqlite3.h>
#endif

// Enhanced NVS implementation with SQLite persistence backend
// Provides full ESP-IDF compatibility with persistent storage

namespace {
    // Forward declarations
    class NVSStorage;
    
    // Global storage instance
    std::unique_ptr<NVSStorage> g_nvs_storage;
    std::mutex g_nvs_mutex;
    nvs_handle_t g_next_handle = 1;
    std::unordered_map<nvs_handle_t, std::pair<std::string, bool>> g_handle_map; // handle -> (namespace, writable)
    bool g_initialized = false;
    
    // Storage backend class with SQLite persistence
    class NVSStorage {
    private:
        // In-memory cache for performance
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<uint8_t>>> memory_cache_;
        mutable std::mutex cache_mutex_;
        
#ifdef HAVE_SQLITE3
        sqlite3* db_ = nullptr;
        std::string db_path_;
        mutable std::mutex db_mutex_;
        bool sqlite_available_ = false;
#endif
        
        m5tab5::emulator::utils::ShutdownGuard shutdown_guard_;
        
    public:
        NVSStorage() : shutdown_guard_(m5tab5::emulator::utils::ShutdownManager::Priority::Normal, "NVS-Storage", 
                                      [this]() { this->shutdown(); }) {
            initialize();
        }
        
        ~NVSStorage() {
            shutdown();
        }
        
        void initialize() {
#ifdef HAVE_SQLITE3
            try {
                // Create emulator data directory
                auto home_dir = std::getenv("HOME");
                if (!home_dir) {
                    LOG_WARN("HOME environment variable not set, using current directory");
                    home_dir = ".";
                }
                
                std::filesystem::path data_dir = std::filesystem::path(home_dir) / ".m5tab5_emulator";
                std::filesystem::create_directories(data_dir);
                
                db_path_ = (data_dir / "nvs.db").string();
                
                // Open SQLite database
                int rc = sqlite3_open(db_path_.c_str(), &db_);
                if (rc != SQLITE_OK) {
                    LOG_ERROR("Failed to open SQLite database {}: {}", db_path_, sqlite3_errmsg(db_));
                    sqlite3_close(db_);
                    db_ = nullptr;
                } else {
                    // Create table if not exists
                    const char* create_sql = 
                        "CREATE TABLE IF NOT EXISTS nvs_data ("
                        "    namespace TEXT NOT NULL,"
                        "    key TEXT NOT NULL,"
                        "    value BLOB NOT NULL,"
                        "    PRIMARY KEY (namespace, key)"
                        ")";
                    
                    char* err_msg = nullptr;
                    rc = sqlite3_exec(db_, create_sql, nullptr, nullptr, &err_msg);
                    if (rc != SQLITE_OK) {
                        LOG_ERROR("Failed to create NVS table: {}", err_msg);
                        sqlite3_free(err_msg);
                        sqlite3_close(db_);
                        db_ = nullptr;
                    } else {
                        sqlite_available_ = true;
                        LOG_INFO("SQLite NVS storage initialized: {}", db_path_);
                        load_from_database();
                    }
                }
            } catch (const std::exception& e) {
                LOG_ERROR("Exception initializing SQLite NVS storage: {}", e.what());
            }
            
            if (!sqlite_available_) {
                LOG_WARN("SQLite not available, using in-memory NVS storage only");
            }
#else
            LOG_INFO("SQLite not compiled in, using in-memory NVS storage only");
#endif
        }
        
        void shutdown() {
#ifdef HAVE_SQLITE3
            if (db_) {
                flush_to_database();
                std::lock_guard<std::mutex> lock(db_mutex_);
                sqlite3_close(db_);
                db_ = nullptr;
                LOG_DEBUG("SQLite NVS database closed");
            }
#endif
        }
        
        bool set_blob(const std::string& namespace_name, const std::string& key, 
                     const std::vector<uint8_t>& value) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            memory_cache_[namespace_name][key] = value;
            
#ifdef HAVE_SQLITE3
            if (sqlite_available_) {
                return write_to_database(namespace_name, key, value);
            }
#endif
            return true;
        }
        
        bool get_blob(const std::string& namespace_name, const std::string& key, 
                     std::vector<uint8_t>& value) const {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            
            auto ns_it = memory_cache_.find(namespace_name);
            if (ns_it == memory_cache_.end()) {
                return false;
            }
            
            auto key_it = ns_it->second.find(key);
            if (key_it == ns_it->second.end()) {
                return false;
            }
            
            value = key_it->second;
            return true;
        }
        
        bool erase_key(const std::string& namespace_name, const std::string& key) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            
            auto ns_it = memory_cache_.find(namespace_name);
            if (ns_it != memory_cache_.end()) {
                ns_it->second.erase(key);
            }
            
#ifdef HAVE_SQLITE3
            if (sqlite_available_) {
                return delete_from_database(namespace_name, key);
            }
#endif
            return true;
        }
        
        bool erase_namespace(const std::string& namespace_name) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            memory_cache_[namespace_name].clear();
            
#ifdef HAVE_SQLITE3
            if (sqlite_available_) {
                return delete_namespace_from_database(namespace_name);
            }
#endif
            return true;
        }
        
        void clear_all() {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            memory_cache_.clear();
            
#ifdef HAVE_SQLITE3
            if (sqlite_available_) {
                clear_database();
            }
#endif
        }
        
        size_t get_used_entries(const std::string& namespace_name) const {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            auto it = memory_cache_.find(namespace_name);
            return (it != memory_cache_.end()) ? it->second.size() : 0;
        }
        
        size_t get_total_entries() const {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            size_t total = 0;
            for (const auto& ns_pair : memory_cache_) {
                total += ns_pair.second.size();
            }
            return total;
        }
        
        void commit() {
#ifdef HAVE_SQLITE3
            if (sqlite_available_) {
                flush_to_database();
            }
#endif
        }
        
    private:
#ifdef HAVE_SQLITE3
        void load_from_database() {
            if (!db_) return;
            
            std::lock_guard<std::mutex> db_lock(db_mutex_);
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            
            const char* select_sql = "SELECT namespace, key, value FROM nvs_data";
            sqlite3_stmt* stmt;
            
            int rc = sqlite3_prepare_v2(db_, select_sql, -1, &stmt, nullptr);
            if (rc != SQLITE_OK) {
                LOG_ERROR("Failed to prepare SELECT statement: {}", sqlite3_errmsg(db_));
                return;
            }
            
            int loaded_count = 0;
            while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
                const char* namespace_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
                const char* key = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
                const void* value_data = sqlite3_column_blob(stmt, 2);
                int value_size = sqlite3_column_bytes(stmt, 2);
                
                if (namespace_name && key && value_data) {
                    std::vector<uint8_t> value(static_cast<const uint8_t*>(value_data),
                                              static_cast<const uint8_t*>(value_data) + value_size);
                    memory_cache_[namespace_name][key] = std::move(value);
                    loaded_count++;
                }
            }
            
            sqlite3_finalize(stmt);
            
            if (rc != SQLITE_DONE) {
                LOG_ERROR("Error reading from database: {}", sqlite3_errmsg(db_));
            } else if (loaded_count > 0) {
                LOG_INFO("Loaded {} NVS entries from persistent storage", loaded_count);
            }
        }
        
        bool write_to_database(const std::string& namespace_name, const std::string& key, 
                              const std::vector<uint8_t>& value) {
            if (!db_) return false;
            
            std::lock_guard<std::mutex> lock(db_mutex_);
            
            const char* insert_sql = 
                "INSERT OR REPLACE INTO nvs_data (namespace, key, value) VALUES (?, ?, ?)";
            
            sqlite3_stmt* stmt;
            int rc = sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, nullptr);
            if (rc != SQLITE_OK) {
                LOG_ERROR("Failed to prepare INSERT statement: {}", sqlite3_errmsg(db_));
                return false;
            }
            
            sqlite3_bind_text(stmt, 1, namespace_name.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 2, key.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_blob(stmt, 3, value.data(), static_cast<int>(value.size()), SQLITE_STATIC);
            
            rc = sqlite3_step(stmt);
            sqlite3_finalize(stmt);
            
            if (rc != SQLITE_DONE) {
                LOG_ERROR("Failed to insert into database: {}", sqlite3_errmsg(db_));
                return false;
            }
            
            return true;
        }
        
        bool delete_from_database(const std::string& namespace_name, const std::string& key) {
            if (!db_) return false;
            
            std::lock_guard<std::mutex> lock(db_mutex_);
            
            const char* delete_sql = "DELETE FROM nvs_data WHERE namespace = ? AND key = ?";
            
            sqlite3_stmt* stmt;
            int rc = sqlite3_prepare_v2(db_, delete_sql, -1, &stmt, nullptr);
            if (rc != SQLITE_OK) {
                LOG_ERROR("Failed to prepare DELETE statement: {}", sqlite3_errmsg(db_));
                return false;
            }
            
            sqlite3_bind_text(stmt, 1, namespace_name.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 2, key.c_str(), -1, SQLITE_STATIC);
            
            rc = sqlite3_step(stmt);
            sqlite3_finalize(stmt);
            
            return rc == SQLITE_DONE;
        }
        
        bool delete_namespace_from_database(const std::string& namespace_name) {
            if (!db_) return false;
            
            std::lock_guard<std::mutex> lock(db_mutex_);
            
            const char* delete_sql = "DELETE FROM nvs_data WHERE namespace = ?";
            
            sqlite3_stmt* stmt;
            int rc = sqlite3_prepare_v2(db_, delete_sql, -1, &stmt, nullptr);
            if (rc != SQLITE_OK) {
                LOG_ERROR("Failed to prepare namespace DELETE statement: {}", sqlite3_errmsg(db_));
                return false;
            }
            
            sqlite3_bind_text(stmt, 1, namespace_name.c_str(), -1, SQLITE_STATIC);
            
            rc = sqlite3_step(stmt);
            sqlite3_finalize(stmt);
            
            return rc == SQLITE_DONE;
        }
        
        void clear_database() {
            if (!db_) return;
            
            std::lock_guard<std::mutex> lock(db_mutex_);
            
            const char* clear_sql = "DELETE FROM nvs_data";
            char* err_msg = nullptr;
            
            int rc = sqlite3_exec(db_, clear_sql, nullptr, nullptr, &err_msg);
            if (rc != SQLITE_OK) {
                LOG_ERROR("Failed to clear database: {}", err_msg);
                sqlite3_free(err_msg);
            }
        }
        
        void flush_to_database() {
            if (!db_) return;
            
            std::lock_guard<std::mutex> db_lock(db_mutex_);
            std::lock_guard<std::mutex> cache_lock(cache_mutex_);
            
            // Begin transaction for batch operations
            sqlite3_exec(db_, "BEGIN TRANSACTION", nullptr, nullptr, nullptr);
            
            for (const auto& ns_pair : memory_cache_) {
                for (const auto& key_pair : ns_pair.second) {
                    write_to_database_unlocked(ns_pair.first, key_pair.first, key_pair.second);
                }
            }
            
            sqlite3_exec(db_, "COMMIT", nullptr, nullptr, nullptr);
        }
        
        bool write_to_database_unlocked(const std::string& namespace_name, const std::string& key, 
                                       const std::vector<uint8_t>& value) {
            const char* insert_sql = 
                "INSERT OR REPLACE INTO nvs_data (namespace, key, value) VALUES (?, ?, ?)";
            
            sqlite3_stmt* stmt;
            int rc = sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, nullptr);
            if (rc != SQLITE_OK) {
                return false;
            }
            
            sqlite3_bind_text(stmt, 1, namespace_name.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 2, key.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_blob(stmt, 3, value.data(), static_cast<int>(value.size()), SQLITE_STATIC);
            
            rc = sqlite3_step(stmt);
            sqlite3_finalize(stmt);
            
            return rc == SQLITE_DONE;
        }
#endif
    };
}

extern "C" {

// Initialize NVS partition
esp_err_t nvs_flash_init() {
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    
    if (!g_initialized) {
        try {
            g_nvs_storage = std::make_unique<NVSStorage>();
            g_initialized = true;
            LOG_INFO("NVS flash initialized with persistent storage backend");
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to initialize NVS storage: {}", e.what());
            return ESP_ERR_NVS_PART_NOT_FOUND;
        }
    }
    
    return ESP_OK;
}

esp_err_t nvs_flash_init_partition(const char* partition_label) {
    return nvs_flash_init();
}

// Erase NVS partition
esp_err_t nvs_flash_erase() {
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    
    if (g_nvs_storage) {
        g_nvs_storage->clear_all();
        LOG_INFO("NVS flash erased (all data cleared)");
    }
    
    return ESP_OK;
}

esp_err_t nvs_flash_erase_partition(const char* partition_label) {
    return nvs_flash_erase();
}

// Open NVS namespace
esp_err_t nvs_open(const char* namespace_name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
    if (!namespace_name || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_initialized || !g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    nvs_handle_t handle = g_next_handle++;
    g_handle_map[handle] = {std::string(namespace_name), open_mode == NVS_READWRITE};
    *out_handle = handle;
    
    LOG_DEBUG("Opened NVS namespace '{}' with handle {} (mode: {})", 
              namespace_name, handle, (open_mode == NVS_READWRITE) ? "RW" : "RO");
    return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* partition_label, const char* namespace_name, 
                                 nvs_open_mode_t open_mode, nvs_handle_t* out_handle) {
    return nvs_open(namespace_name, open_mode, out_handle);
}

// Close NVS handle
void nvs_close(nvs_handle_t handle) {
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    g_handle_map.erase(handle);
    LOG_DEBUG("Closed NVS handle {}", handle);
}

// Generic set function
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length) {
    if (!key || !value || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    std::vector<uint8_t> data(static_cast<const uint8_t*>(value), 
                             static_cast<const uint8_t*>(value) + length);
    
    if (!g_nvs_storage->set_blob(ns, key, data)) {
        LOG_ERROR("Failed to store NVS blob for {}:{}", ns, key);
        return ESP_ERR_NVS_INVALID_STATE;
    }
    
    return ESP_OK;
}

// Typed set functions
esp_err_t nvs_set_u8(nvs_handle_t handle, const char* key, uint8_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i8(nvs_handle_t handle, const char* key, int8_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u16(nvs_handle_t handle, const char* key, uint16_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i16(nvs_handle_t handle, const char* key, int16_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i32(nvs_handle_t handle, const char* key, int32_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_u64(nvs_handle_t handle, const char* key, uint64_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_i64(nvs_handle_t handle, const char* key, int64_t value) {
    return nvs_set_blob(handle, key, &value, sizeof(value));
}

esp_err_t nvs_set_str(nvs_handle_t handle, const char* key, const char* value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    return nvs_set_blob(handle, key, value, strlen(value) + 1);
}

// Generic get function
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length) {
    if (!key || !length) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    const std::string& ns = it->second.first;
    std::vector<uint8_t> data;
    
    if (!g_nvs_storage->get_blob(ns, key, data)) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    if (out_value == nullptr) {
        // Just return size
        *length = data.size();
        return ESP_OK;
    }
    
    if (*length < data.size()) {
        *length = data.size();
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    
    memcpy(out_value, data.data(), data.size());
    *length = data.size();
    return ESP_OK;
}

// Typed get functions
esp_err_t nvs_get_u8(nvs_handle_t handle, const char* key, uint8_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint8_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i8(nvs_handle_t handle, const char* key, int8_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int8_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u16(nvs_handle_t handle, const char* key, uint16_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint16_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i16(nvs_handle_t handle, const char* key, int16_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int16_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint32_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int32_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_u64(nvs_handle_t handle, const char* key, uint64_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(uint64_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_i64(nvs_handle_t handle, const char* key, int64_t* out_value) {
    if (!out_value) return ESP_ERR_INVALID_ARG;
    size_t length = sizeof(int64_t);
    return nvs_get_blob(handle, key, out_value, &length);
}

esp_err_t nvs_get_str(nvs_handle_t handle, const char* key, char* out_value, size_t* length) {
    return nvs_get_blob(handle, key, out_value, length);
}

// Erase operations
esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key) {
    if (!key) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    g_nvs_storage->erase_key(ns, key);
    
    return ESP_OK;
}

esp_err_t nvs_erase_all(nvs_handle_t handle) {
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    if (!it->second.second) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    
    const std::string& ns = it->second.first;
    g_nvs_storage->erase_namespace(ns);
    
    return ESP_OK;
}

// Commit operations
esp_err_t nvs_commit(nvs_handle_t handle) {
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    // Force write-through to persistent storage
    g_nvs_storage->commit();
    
    return ESP_OK;
}

// Statistics and info functions
esp_err_t nvs_get_stats(const char* part_name, nvs_stats_t* nvs_stats) {
    if (!nvs_stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    
    size_t total_entries = g_nvs_storage->get_total_entries();
    
    nvs_stats->used_entries = total_entries;
    nvs_stats->free_entries = 10000 - total_entries;  // Simulated capacity
    nvs_stats->total_entries = 10000;  // Simulated total capacity
    
    return ESP_OK;
}

esp_err_t nvs_get_used_entry_count(nvs_handle_t handle, size_t* used_entries) {
    if (!used_entries) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_nvs_storage) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(g_nvs_mutex);
    auto it = g_handle_map.find(handle);
    if (it == g_handle_map.end()) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    
    const std::string& ns = it->second.first;
    *used_entries = g_nvs_storage->get_used_entries(ns);
    
    return ESP_OK;
}

// Iterator functions (basic implementation)
esp_err_t nvs_entry_find(const char* part_name, const char* namespace_name, nvs_type_t type, nvs_iterator_t* output_iterator) {
    return ESP_ERR_NVS_NOT_FOUND;  // Not implemented
}

esp_err_t nvs_entry_next(nvs_iterator_t* iterator) {
    return ESP_ERR_NVS_NOT_FOUND;  // Not implemented
}

void nvs_entry_info(nvs_iterator_t iterator, nvs_entry_info_t* out_info) {
    // Not implemented
}

void nvs_release_iterator(nvs_iterator_t iterator) {
    // Not implemented
}

} // extern "C"