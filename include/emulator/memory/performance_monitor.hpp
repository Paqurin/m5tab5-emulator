#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/config/configuration.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <deque>
#include <memory>

namespace m5tab5::emulator {

// Forward declaration
class MemoryController;

// MemoryAccessType is defined in utils/types.hpp

/**
 * @brief Memory access event for performance tracking
 */
struct MemoryAccessEvent {
    Address address;
    size_t size;
    MemoryAccessType access_type;
    u64 cycle_count;
    bool cache_hit;
    u64 timestamp;
    
    MemoryAccessEvent() : address(0), size(0), access_type(MemoryAccessType::READ),
                         cycle_count(0), cache_hit(false), timestamp(0) {}
};

/**
 * @brief Global performance counters
 */
struct PerformanceCounters {
    u64 total_reads;
    u64 total_writes;
    u64 total_executions;
    u64 total_accesses;
    u64 total_read_bytes;
    u64 total_write_bytes;
    u64 total_cycles;
    u64 cache_hits;
    u64 cache_misses;
    
    PerformanceCounters() : total_reads(0), total_writes(0), total_executions(0),
                           total_accesses(0), total_read_bytes(0), total_write_bytes(0),
                           total_cycles(0), cache_hits(0), cache_misses(0) {}
};

/**
 * @brief Statistics for a specific memory region
 */
struct RegionStatistics {
    u64 reads;
    u64 writes;
    u64 executions;
    u64 read_bytes;
    u64 write_bytes;
    u64 total_cycles;
    u64 cache_hits;
    u64 cache_misses;
    u64 access_count;  // Total access count field used in implementation
    
    // Additional fields used in implementation
    u64 read_count;        // Matches stats.read_count usage
    u64 write_count;       // Matches stats.write_count usage  
    u64 execution_count;   // Matches stats.execution_count usage
    
    RegionStatistics() : reads(0), writes(0), executions(0), read_bytes(0),
                        write_bytes(0), total_cycles(0), cache_hits(0), cache_misses(0),
                        access_count(0), read_count(0), write_count(0), execution_count(0) {}
};

/**
 * @brief Memory region for monitoring
 */
struct MonitoringRegion {
    u8 region_id;
    std::string name;
    Address base_address;
    size_t size;
    bool enabled;
    RegionStatistics statistics;
    
    MonitoringRegion() : region_id(0), base_address(0), size(0), enabled(true) {}
    
    MonitoringRegion(u8 id, const std::string& n, Address base, size_t sz)
        : region_id(id), name(n), base_address(base), size(sz), enabled(true) {}
};

/**
 * @brief Hot spot for memory access analysis
 */
struct HotSpot {
    Address address;
    size_t access_count;
    
    HotSpot() : address(0), access_count(0) {}
};

/**
 * @brief Access pattern analysis results
 */
struct AccessPatternAnalysis {
    double sequential_access_ratio;
    double random_access_ratio;
    std::vector<HotSpot> hot_spots;
    double average_reuse_distance;
    
    AccessPatternAnalysis() : sequential_access_ratio(0.0), random_access_ratio(0.0),
                             average_reuse_distance(0.0) {}
};

/**
 * @brief Comprehensive performance report
 */
struct PerformanceReport {
    PerformanceCounters global_counters;
    std::vector<RegionStatistics> region_statistics;  // Changed from unordered_map to vector to match implementation
    double cache_hit_rate;
    double average_access_cycles;
    double bandwidth_utilization;
    u64 report_timestamp;
    AccessPatternAnalysis access_patterns;
    bool monitoring_enabled;  // Added field used in implementation
    
    PerformanceReport() : cache_hit_rate(0.0), average_access_cycles(0.0),
                         bandwidth_utilization(0.0), report_timestamp(0),
                         monitoring_enabled(false) {}
};

/**
 * @brief Performance monitoring system for memory subsystem
 * 
 * Tracks memory access patterns, cache performance, and provides
 * detailed statistics for optimization and debugging.
 */
class PerformanceMonitor {
public:
    PerformanceMonitor();
    ~PerformanceMonitor();

    // Lifecycle management
    Result<void> initialize(const Configuration& config, MemoryController& memory_controller);
    Result<void> shutdown();

    // Monitoring control
    Result<void> enable_monitoring();
    Result<void> disable_monitoring();
    bool is_monitoring_enabled() const { return monitoring_enabled_; }

    // Data recording
    Result<void> record_memory_access(const MemoryAccessEvent& event);

    // Region management
    Result<u8> add_monitoring_region(const MonitoringRegion& region);
    Result<void> remove_monitoring_region(u8 region_id);
    std::vector<MonitoringRegion> get_all_regions() const;

    // Statistics access
    const PerformanceCounters& get_global_counters() const;
    Result<RegionStatistics> get_region_statistics(u8 region_id) const;
    std::vector<MemoryAccessEvent> get_access_history() const;
    Result<PerformanceReport> generate_report() const;

    // Control operations
    void reset_counters();
    void dump_statistics() const;
    
    // Analysis operations
    AccessPatternAnalysis analyze_access_patterns() const;

private:
    // Internal state
    bool initialized_;
    bool monitoring_enabled_;
    MemoryController* memory_controller_;

    // Performance data
    PerformanceCounters global_counters_;
    std::unordered_map<u8, RegionStatistics> region_statistics_;
    std::vector<MonitoringRegion> monitoring_regions_;
    std::deque<MemoryAccessEvent> access_history_;

    // Configuration
    static constexpr size_t MAX_ACCESS_HISTORY = 10000;
    static constexpr u8 MAX_MONITORING_REGIONS = 32;
    
    // ESP32-P4 Memory Layout Constants (simplified for monitoring)
    struct MemoryRegionInfo {
        Address start_address;
        size_t size;
    };
    
    static constexpr MemoryRegionInfo FLASH_REGION = {0x40000000, 16 * 1024 * 1024};  // 16MB Flash
    static constexpr MemoryRegionInfo PSRAM_REGION = {0x48000000, 32 * 1024 * 1024}; // 32MB PSRAM
    static constexpr MemoryRegionInfo SRAM_REGION = {0x4FF00000, 768 * 1024};        // 768KB SRAM
    static constexpr MemoryRegionInfo MMIO_REGION = {0x60000000, 256 * 1024 * 1024}; // 256MB MMIO

    // Internal helpers
    Result<void> setup_default_monitoring_regions();
    Result<void> update_region_statistics(const MemoryAccessEvent& event);
    u8 find_next_region_id() const;
    bool address_in_region(Address address, const MonitoringRegion& region) const;
};

} // namespace m5tab5::emulator