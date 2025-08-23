#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/memory/cache_coherency.hpp"

#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

namespace m5tab5::emulator {

// Forward declarations
class MemoryController;

/**
 * @brief Cache controller for ESP32-P4 CPU cores
 * 
 * Implements L1 instruction and data caches with configurable
 * associativity, size, and replacement policies. Integrates with
 * the cache coherency controller for multi-core consistency.
 */

// Cache types
enum class CacheType {
    INSTRUCTION_CACHE,  // L1 instruction cache (read-only)
    DATA_CACHE,         // L1 data cache (read-write)
    UNIFIED_CACHE       // Unified instruction/data cache
};

// Cache replacement policies
enum class ReplacementPolicy {
    LRU,                // Least Recently Used
    FIFO,               // First In, First Out
    RANDOM,             // Random replacement
    LFU                 // Least Frequently Used
};

// Cache write policies
enum class WritePolicy {
    WRITE_THROUGH,      // Write to cache and memory immediately
    WRITE_BACK,         // Write to cache, defer memory write
    WRITE_AROUND        // Write directly to memory, bypass cache
};

// Cache configuration parameters
struct CacheConfig {
    CacheType type;
    u32 size_bytes;             // Total cache size
    u32 line_size;              // Cache line size (typically 64 bytes)
    u32 associativity;          // N-way associativity (1 = direct mapped)
    ReplacementPolicy replacement_policy;
    WritePolicy write_policy;
    bool enable_prefetch;       // Hardware prefetching
    u32 prefetch_distance;      // Lines to prefetch ahead
    
    CacheConfig() 
        : type(CacheType::DATA_CACHE), size_bytes(32768), line_size(64),
          associativity(4), replacement_policy(ReplacementPolicy::LRU),
          write_policy(WritePolicy::WRITE_BACK), enable_prefetch(true),
          prefetch_distance(2) {}
};

// Cache line entry
struct CacheLine {
    Address tag;                // Address tag
    std::vector<u8> data;       // Cache line data
    bool valid;                 // Valid bit
    bool dirty;                 // Dirty bit (for write-back)
    u64 last_access_cycle;      // For LRU
    u32 access_count;           // For LFU
    
    CacheLine(u32 line_size) : tag(0), data(line_size), valid(false), dirty(false),
                              last_access_cycle(0), access_count(0) {}
};

// Cache set (collection of cache lines)
struct CacheSet {
    std::vector<std::unique_ptr<CacheLine>> lines;
    u32 next_replacement_index; // For FIFO
    
    explicit CacheSet(u32 associativity, u32 line_size) 
        : next_replacement_index(0) {
        lines.reserve(associativity);
        for (u32 i = 0; i < associativity; ++i) {
            lines.emplace_back(std::make_unique<CacheLine>(line_size));
        }
    }
};

// Cache performance statistics
struct CacheStats {
    u64 hits;
    u64 misses;
    u64 evictions;
    u64 write_backs;
    u64 prefetch_hits;
    u64 prefetch_misses;
    
    CacheStats() : hits(0), misses(0), evictions(0), write_backs(0),
                  prefetch_hits(0), prefetch_misses(0) {}
                  
    double get_hit_rate() const { 
        return (hits + misses > 0) ? static_cast<double>(hits) / (hits + misses) : 0.0; 
    }
};

/**
 * @brief Main cache controller implementation
 */
class CacheController {
public:
    CacheController(u32 core_id, CacheType type);
    ~CacheController();
    
    // Lifecycle management
    Result<void> initialize(const CacheConfig& config, 
                           MemoryController& memory_controller,
                           CacheCoherencyController* coherency_controller = nullptr);
    Result<void> shutdown();
    Result<void> reset();
    
    // Cache operations
    Result<void> read(Address address, u8* buffer, size_t size);
    Result<void> write(Address address, const u8* buffer, size_t size);
    Result<void> flush_line(Address address);
    Result<void> invalidate_line(Address address);
    Result<void> flush_all();
    Result<void> invalidate_all();
    
    // Cache line operations (for coherency)
    Result<CacheLineState> get_line_state(Address address);
    Result<void> set_line_state(Address address, CacheLineState state);
    bool is_line_present(Address address) const;
    bool is_line_dirty(Address address) const;
    
    // Configuration access
    const CacheConfig& get_config() const { return config_; }
    CacheType get_type() const { return config_.type; }
    u32 get_core_id() const { return core_id_; }
    
    // Statistics
    const CacheStats& get_statistics() const { return stats_; }
    void reset_statistics();
    double get_hit_rate() const { return stats_.get_hit_rate(); }
    
    // Debug and monitoring
    void dump_cache_contents() const;
    std::vector<Address> get_cached_addresses() const;
    size_t get_dirty_line_count() const;

private:
    // Core configuration
    u32 core_id_;
    CacheConfig config_;
    bool initialized_;
    
    // Cache structure
    std::vector<std::unique_ptr<CacheSet>> cache_sets_;
    u32 num_sets_;
    u32 index_mask_;
    u32 tag_shift_;
    u32 offset_mask_;
    
    // External interfaces
    MemoryController* memory_controller_;
    CacheCoherencyController* coherency_controller_;
    
    // Statistics and synchronization
    mutable std::mutex cache_mutex_;
    CacheStats stats_;
    ClockCycle current_cycle_;
    
    // Cache operation helpers
    Result<CacheLine*> find_cache_line(Address address);
    Result<CacheLine*> allocate_cache_line(Address address);
    Result<void> evict_cache_line(CacheSet& set, u32 line_index);
    Result<void> write_back_line(CacheLine& line, Address address);
    
    // Address manipulation
    u32 get_cache_set_index(Address address) const;
    Address get_cache_tag(Address address) const;
    u32 get_cache_offset(Address address) const;
    Address get_line_address(Address address) const;
    
    // Replacement policy implementation
    u32 select_replacement_victim(const CacheSet& set);
    void update_replacement_info(CacheSet& set, u32 line_index);
    
    // Prefetching
    Result<void> handle_prefetch(Address address);
    bool should_prefetch(Address address) const;
    
    // Coherency integration
    Result<void> handle_coherency_request(Address address, CoherencyOperation op);
    void notify_coherency_controller(Address address, CoherencyOperation op);
    
    // Validation
    bool is_cache_line_aligned(Address address) const;
    bool is_valid_access_size(size_t size) const;
};

/**
 * @brief Cache controller factory
 */
class CacheControllerFactory {
public:
    static std::unique_ptr<CacheController> create_l1_icache(u32 core_id);
    static std::unique_ptr<CacheController> create_l1_dcache(u32 core_id);
    static std::unique_ptr<CacheController> create_unified_cache(u32 core_id);
    
    static CacheConfig get_default_icache_config();
    static CacheConfig get_default_dcache_config();
    static CacheConfig get_esp32p4_cache_config(CacheType type);
};

} // namespace m5tab5::emulator