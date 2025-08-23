#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/cpu/cpu_core.hpp"

#include <memory>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>
#include <array>

namespace m5tab5::emulator {

// Forward declarations
class MemoryController;

/**
 * @brief Cache coherency controller for ESP32-P4 multi-core system
 * 
 * Manages cache coherency between the dual RISC-V cores, ensuring
 * data consistency across shared memory regions while maintaining
 * performance through optimized coherency protocols.
 */

// Cache coherency protocols
enum class CoherencyProtocol {
    NONE,           // No coherency (cache disabled)
    MESI,           // Modified, Exclusive, Shared, Invalid
    MSI,            // Modified, Shared, Invalid (simplified MESI)
    MOESI,          // Modified, Owner, Exclusive, Shared, Invalid  
    WRITE_THROUGH,  // Simple write-through with invalidation
    WRITE_BACK      // Write-back with ownership tracking
};

// Cache line states for MESI protocol
enum class CacheLineState {
    INVALID = 0,    // Cache line is invalid
    SHARED = 1,     // Cache line is shared (read-only)
    EXCLUSIVE = 2,  // Cache line is exclusive (read-write, not shared)
    MODIFIED = 3    // Cache line is modified (dirty, needs write-back)
};

// Cache coherency operations
enum class CoherencyOperation {
    READ_SHARED,    // Read with intent to share
    READ_EXCLUSIVE, // Read with intent to modify
    WRITE,          // Write operation
    INVALIDATE,     // Invalidate cache line
    FLUSH,          // Flush cache line to memory
    FLUSH_ALL       // Flush entire cache
};

// Cache coherency action types
enum class CoherencyActionType {
    CACHE_HIT,                  // Cache hit - no action needed
    CACHE_MISS_READ_SHARED,     // Read miss, load as shared
    CACHE_MISS_READ_EXCLUSIVE,  // Read miss, load as exclusive
    CACHE_MISS_WRITE,          // Write miss, load and modify
    INVALIDATE_OTHERS,         // Invalidate in other cores
    WRITE_BACK,                // Write back to memory
    SHARED_TO_MODIFIED,        // Upgrade from shared to modified
    EXCLUSIVE_TO_MODIFIED      // Upgrade from exclusive to modified
};

// Cache coherency message types for inter-core communication
enum class CoherencyMessage {
    BUS_READ,       // Read request on bus
    BUS_READ_X,     // Exclusive read request
    BUS_WRITE,      // Write notification
    BUS_INVALIDATE, // Invalidate request
    BUS_FLUSH,      // Flush request
    BUS_ACK         // Acknowledgment
};

// Cache line metadata
struct CacheLineInfo {
    Address tag;                // Cache line tag (address bits)
    CacheLineState state;       // Current coherency state
    u32 owner_core;            // Core that owns this line (for MOESI)
    bool dirty;                // True if line needs write-back
    u64 last_access_cycle;     // For LRU replacement
    
    CacheLineInfo() 
        : tag(0), state(CacheLineState::INVALID), owner_core(0xFFFFFFFF),
          dirty(false), last_access_cycle(0) {}
};

// Coherency statistics
struct CoherencyStats {
    u64 cache_hits;
    u64 cache_misses;
    u64 invalidations;
    u64 invalidations_sent;
    u64 write_backs;
    u64 writebacks;
    u64 read_requests;
    u64 write_requests;
    u64 coherency_messages;
    u64 bus_transactions;
    
    CoherencyStats() : cache_hits(0), cache_misses(0), invalidations(0), invalidations_sent(0),
                      write_backs(0), writebacks(0), read_requests(0), write_requests(0),
                      coherency_messages(0), bus_transactions(0) {}
};

// Legacy alias for compatibility
using CoherencyStatistics = CoherencyStats;

// Cache coherency action structure
struct CacheCoherencyAction {
    Address address;
    CoreId requesting_core;
    CoherencyActionType action_type;
    std::vector<CoherencyActionType> additional_actions;
    
    CacheCoherencyAction() : address(0), requesting_core(CoreId::CORE_0), 
                           action_type(CoherencyActionType::CACHE_HIT) {}
};

// Core directory for tracking cache line states per core
class CoreDirectory {
public:
    explicit CoreDirectory(CoreId core_id);
    
    void add_cache_line(Address address, CacheLineState state);
    void remove_cache_line(Address address);
    bool has_cache_line(Address address) const;
    CacheLineState get_cache_line_state(Address address) const;
    void set_cache_line_state(Address address, CacheLineState state);
    void clear();
    size_t get_cache_line_count() const;
    
private:
    CoreId core_id_;
    std::unordered_map<Address, CacheLineState> cache_lines_;
};

// Per-core cache state
struct CoreCacheState {
    u32 core_id;
    std::unordered_map<Address, CacheLineInfo> cache_lines;
    CoherencyStats stats;
    std::mutex cache_mutex;
    
    explicit CoreCacheState(u32 id) : core_id(id) {}
};

/**
 * @brief Main cache coherency controller
 */
class CacheCoherencyController {
public:
    static constexpr u32 CACHE_LINE_SIZE = 64;      // 64-byte cache lines
    static constexpr u32 MAX_CORES = 4;             // Support up to 4 cores
    static constexpr u32 DEFAULT_CACHE_WAYS = 4;    // 4-way associative
    static constexpr u32 DEFAULT_CACHE_SETS = 128;  // 128 sets per way

    CacheCoherencyController();
    ~CacheCoherencyController();
    
    // Lifecycle management
    Result<void> initialize(const Configuration& config,
                           MemoryController& memory_controller);
    Result<void> shutdown();
    Result<void> reset();
    
    // Configuration
    void set_protocol(CoherencyProtocol protocol) { protocol_ = protocol; }
    CoherencyProtocol get_protocol() const { return protocol_; }
    
    void set_cache_parameters(u32 line_size, u32 ways, u32 sets);
    void enable_coherency(bool enable) { coherency_enabled_ = enable; }
    bool is_coherency_enabled() const { return coherency_enabled_; }
    
    // Core registration
    Result<void> register_core(u32 core_id);
    Result<void> unregister_core(u32 core_id);
    
    // Cache operations (called by CPU cores)
    Result<void> handle_read(u32 core_id, Address address, bool exclusive = false);
    Result<void> handle_write(u32 core_id, Address address);
    Result<void> handle_flush(u32 core_id, Address address);
    Result<void> handle_flush_all(u32 core_id);
    Result<void> handle_invalidate(u32 core_id, Address address);
    
    // Legacy interface methods (for compatibility with existing code)
    Result<void> register_cache_line(CoreId core_id, Address address, CacheLineState initial_state);
    Result<void> invalidate_cache_line(CoreId core_id, Address address);
    Result<CacheCoherencyAction> handle_read_request(CoreId core_id, Address address);
    Result<CacheCoherencyAction> handle_write_request(CoreId core_id, Address address);
    Result<void> execute_coherency_action(const CacheCoherencyAction& action);
    const CoherencyStatistics& get_statistics() const;
    void clear_statistics();
    Result<std::vector<CoreId>> get_sharing_cores(Address address) const;
    
    // Cache line state queries
    CacheLineState get_line_state(u32 core_id, Address address);
    bool is_line_dirty(u32 core_id, Address address);
    bool is_line_shared(Address address);
    
    // Statistics and monitoring
    const CoherencyStats& get_core_stats(u32 core_id) const;
    CoherencyStats get_global_stats() const;
    void reset_statistics();
    void dump_cache_states() const;
    
    // Debug and testing
    void force_invalidate_all();
    void force_write_back_all();
    std::vector<Address> get_dirty_lines(u32 core_id) const;

private:
    // Internal state
    bool initialized_;
    bool coherency_enabled_;
    CoherencyProtocol protocol_;
    MemoryController* memory_controller_;
    
    // Cache configuration
    u32 cache_line_size_;
    u32 cache_ways_;
    u32 cache_sets_;
    u32 address_mask_;
    u32 tag_shift_;
    
    // Per-core cache states
    std::vector<std::unique_ptr<CoreCacheState>> core_caches_;
    std::mutex global_coherency_mutex_;
    
    // Legacy interface support
    std::array<std::unique_ptr<CoreDirectory>, MAX_CORES> core_directories_;
    CoherencyStatistics statistics_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    CoherencyStats global_stats_;
    
    // Internal coherency operations
    Result<void> broadcast_invalidate(u32 initiating_core, Address address);
    Result<void> handle_coherency_message(u32 source_core, u32 target_core,
                                         CoherencyMessage message, Address address);
    Result<void> write_back_line(u32 core_id, Address address);
    Result<void> update_line_state(u32 core_id, Address address, CacheLineState new_state);
    
    // Address manipulation
    Address get_cache_line_address(Address address) const;
    Address get_cache_tag(Address address) const;
    u32 get_cache_set(Address address) const;
    
    // Protocol-specific handlers
    Result<void> handle_mesi_read(u32 core_id, Address address, bool exclusive);
    Result<void> handle_mesi_write(u32 core_id, Address address);
    Result<void> handle_write_through_access(u32 core_id, Address address, bool is_write);
    
    // Validation and utilities
    bool is_valid_core_id(u32 core_id) const;
    bool is_cache_line_aligned(Address address) const;
    void update_global_stats(const CoherencyStats& core_stats);
    
    // Legacy support methods
    Address align_to_cache_line(Address address) const;
    const char* get_protocol_name(CoherencyProtocol protocol);
    CacheCoherencyAction handle_mesi_read(CoreId core_id, Address address, CacheLineState current_state);
    CacheCoherencyAction handle_mesi_write(CoreId core_id, Address address, CacheLineState current_state);
    CacheCoherencyAction handle_msi_read(CoreId core_id, Address address, CacheLineState current_state);
    CacheCoherencyAction handle_msi_write(CoreId core_id, Address address, CacheLineState current_state);
};

} // namespace m5tab5::emulator