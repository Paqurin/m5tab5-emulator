#include "emulator/memory/performance_monitor.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <numeric>

namespace m5tab5::emulator {

DECLARE_LOGGER("PerformanceMonitor");

PerformanceMonitor::PerformanceMonitor()
    : initialized_(false),
      monitoring_enabled_(false),
      memory_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("PerformanceMonitor created");
}

PerformanceMonitor::~PerformanceMonitor() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("PerformanceMonitor destroyed");
}

Result<void> PerformanceMonitor::initialize(const Configuration& config,
                                            MemoryController& memory_controller) {
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Performance monitor already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing performance monitor");
    
    memory_controller_ = &memory_controller;
    
    // Initialize performance counters
    reset_counters();
    
    // Set up monitoring regions
    RETURN_IF_ERROR(setup_default_monitoring_regions());
    
    monitoring_enabled_ = true;
    initialized_ = true;
    
    COMPONENT_LOG_INFO("Performance monitor initialized");
    return {};
}

Result<void> PerformanceMonitor::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down performance monitor");
    
    monitoring_regions_.clear();
    access_history_.clear();
    memory_controller_ = nullptr;
    monitoring_enabled_ = false;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Performance monitor shutdown completed");
    return {};
}

Result<void> PerformanceMonitor::enable_monitoring() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    monitoring_enabled_ = true;
    COMPONENT_LOG_INFO("Performance monitoring enabled");
    return {};
}

Result<void> PerformanceMonitor::disable_monitoring() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    monitoring_enabled_ = false;
    COMPONENT_LOG_INFO("Performance monitoring disabled");
    return {};
}

Result<void> PerformanceMonitor::record_memory_access(const MemoryAccessEvent& event) {
    if (!initialized_ || !monitoring_enabled_) {
        return {};
    }
    
    // Update global counters
    switch (event.access_type) {
        case MemoryAccessType::READ:
            global_counters_.total_reads++;
            global_counters_.total_read_bytes += event.size;
            break;
        case MemoryAccessType::WRITE:
            global_counters_.total_writes++;
            global_counters_.total_write_bytes += event.size;
            break;
        case MemoryAccessType::EXECUTE:
            global_counters_.total_executions++;
            break;
    }
    
    global_counters_.total_accesses++;
    global_counters_.total_cycles += event.cycle_count;
    
    // Update cache statistics
    if (event.cache_hit) {
        global_counters_.cache_hits++;
    } else {
        global_counters_.cache_misses++;
    }
    
    // Update region-specific statistics
    RETURN_IF_ERROR(update_region_statistics(event));
    
    // Record access in history (if enabled)
    if (access_history_.size() < MAX_ACCESS_HISTORY) {
        access_history_.push_back(event);
    } else {
        // Replace oldest entry (circular buffer)
        static size_t history_index = 0;
        access_history_[history_index] = event;
        history_index = (history_index + 1) % MAX_ACCESS_HISTORY;
    }
    
    return {};
}

Result<u8> PerformanceMonitor::add_monitoring_region(const MonitoringRegion& region) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    if (monitoring_regions_.size() >= MAX_MONITORING_REGIONS) {
        return unexpected(MAKE_ERROR(SYSTEM_RESOURCE_EXHAUSTED,
            "Maximum number of monitoring regions reached"));
    }
    
    // Validate region
    if (region.size == 0) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Monitoring region size cannot be zero"));
    }
    
    monitoring_regions_.push_back(region);
    u8 region_id = static_cast<u8>(monitoring_regions_.size() - 1);
    
    COMPONENT_LOG_INFO("Added monitoring region {}: {} (0x{:08X}-0x{:08X})",
                      region_id, region.name,
                      region.base_address,
                      region.base_address + region.size - 1);
    
    return region_id;
}

Result<void> PerformanceMonitor::remove_monitoring_region(u8 region_id) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    if (region_id >= monitoring_regions_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid monitoring region ID: " + std::to_string(region_id)));
    }
    
    std::string region_name = monitoring_regions_[region_id].name;
    monitoring_regions_.erase(monitoring_regions_.begin() + region_id);
    
    COMPONENT_LOG_INFO("Removed monitoring region {}: {}", region_id, region_name);
    return {};
}

const PerformanceCounters& PerformanceMonitor::get_global_counters() const {
    return global_counters_;
}

Result<RegionStatistics> PerformanceMonitor::get_region_statistics(u8 region_id) const {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    if (region_id >= monitoring_regions_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid monitoring region ID: " + std::to_string(region_id)));
    }
    
    return monitoring_regions_[region_id].statistics;
}

std::vector<MonitoringRegion> PerformanceMonitor::get_all_regions() const {
    return monitoring_regions_;
}

std::vector<MemoryAccessEvent> PerformanceMonitor::get_access_history() const {
    return std::vector<MemoryAccessEvent>(access_history_.begin(), access_history_.end());
}

Result<PerformanceReport> PerformanceMonitor::generate_report() const {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Performance monitor not initialized"));
    }
    
    PerformanceReport report;
    report.monitoring_enabled = monitoring_enabled_;
    report.global_counters = global_counters_;
    
    // Calculate derived metrics
    if (global_counters_.total_accesses > 0) {
        report.cache_hit_rate = static_cast<double>(global_counters_.cache_hits) / 
                               global_counters_.total_accesses;
        report.average_access_cycles = static_cast<double>(global_counters_.total_cycles) / 
                                      global_counters_.total_accesses;
    }
    
    // Calculate bandwidth utilization
    if (global_counters_.total_cycles > 0) {
        u64 total_bytes = global_counters_.total_read_bytes + global_counters_.total_write_bytes;
        report.bandwidth_utilization = static_cast<double>(total_bytes) / global_counters_.total_cycles;
    }
    
    // Add region statistics
    for (const auto& region : monitoring_regions_) {
        report.region_statistics.push_back(region.statistics);
    }
    
    // Analyze access patterns
    report.access_patterns = analyze_access_patterns();
    
    return report;
}

void PerformanceMonitor::reset_counters() {
    global_counters_ = {};
    
    // Reset region statistics
    for (auto& region : monitoring_regions_) {
        region.statistics = {};
    }
    
    access_history_.clear();
    
    COMPONENT_LOG_DEBUG("Performance counters reset");
}

void PerformanceMonitor::dump_statistics() const {
    COMPONENT_LOG_INFO("=== Performance Monitor Statistics ===");
    COMPONENT_LOG_INFO("Monitoring enabled: {}", monitoring_enabled_);
    
    // Global statistics
    COMPONENT_LOG_INFO("Global Counters:");
    COMPONENT_LOG_INFO("  Total accesses: {}", global_counters_.total_accesses);
    COMPONENT_LOG_INFO("  Total reads: {} ({} bytes)", 
                      global_counters_.total_reads, global_counters_.total_read_bytes);
    COMPONENT_LOG_INFO("  Total writes: {} ({} bytes)", 
                      global_counters_.total_writes, global_counters_.total_write_bytes);
    COMPONENT_LOG_INFO("  Total executions: {}", global_counters_.total_executions);
    COMPONENT_LOG_INFO("  Total cycles: {}", global_counters_.total_cycles);
    
    // Cache statistics
    if (global_counters_.total_accesses > 0) {
        double hit_rate = static_cast<double>(global_counters_.cache_hits) / 
                         global_counters_.total_accesses * 100.0;
        COMPONENT_LOG_INFO("  Cache hits: {} ({:.2f}%)", global_counters_.cache_hits, hit_rate);
        COMPONENT_LOG_INFO("  Cache misses: {}", global_counters_.cache_misses);
    }
    
    // Average metrics
    if (global_counters_.total_accesses > 0) {
        double avg_cycles = static_cast<double>(global_counters_.total_cycles) / 
                           global_counters_.total_accesses;
        COMPONENT_LOG_INFO("  Average cycles per access: {:.2f}", avg_cycles);
    }
    
    // Region statistics
    COMPONENT_LOG_INFO("Region Statistics:");
    for (size_t i = 0; i < monitoring_regions_.size(); ++i) {
        const auto& region = monitoring_regions_[i];
        const auto& stats = region.statistics;
        
        COMPONENT_LOG_INFO("  Region {}: {} (0x{:08X}-0x{:08X})",
                          i, region.name, region.base_address, 
                          region.base_address + region.size - 1);
        COMPONENT_LOG_INFO("    Accesses: {}", stats.access_count);
        COMPONENT_LOG_INFO("    Reads: {} ({} bytes)", stats.read_count, stats.read_bytes);
        COMPONENT_LOG_INFO("    Writes: {} ({} bytes)", stats.write_count, stats.write_bytes);
        
        if (stats.access_count > 0) {
            double avg_cycles = static_cast<double>(stats.total_cycles) / stats.access_count;
            COMPONENT_LOG_INFO("    Average cycles: {:.2f}", avg_cycles);
        }
    }
}

Result<void> PerformanceMonitor::setup_default_monitoring_regions() {
    COMPONENT_LOG_DEBUG("Setting up default monitoring regions");
    
    // Flash region
    MonitoringRegion flash_region;
    flash_region.name = "Flash";
    flash_region.base_address = FLASH_REGION.start_address;
    flash_region.size = FLASH_REGION.size;
    flash_region.statistics = {};
    
    auto flash_result = add_monitoring_region(flash_region);
    if (!flash_result) {
        return unexpected(flash_result.error());
    }
    
    // PSRAM region
    MonitoringRegion psram_region;
    psram_region.name = "PSRAM";
    psram_region.base_address = PSRAM_REGION.start_address;
    psram_region.size = PSRAM_REGION.size;
    psram_region.statistics = {};
    
    auto psram_result = add_monitoring_region(psram_region);
    if (!psram_result) {
        return unexpected(psram_result.error());
    }
    
    // SRAM region
    MonitoringRegion sram_region;
    sram_region.name = "SRAM";
    sram_region.base_address = SRAM_REGION.start_address;
    sram_region.size = SRAM_REGION.size;
    sram_region.statistics = {};
    
    auto sram_result = add_monitoring_region(sram_region);
    if (!sram_result) {
        return unexpected(sram_result.error());
    }
    
    // MMIO region
    MonitoringRegion mmio_region;
    mmio_region.name = "MMIO";
    mmio_region.base_address = MMIO_REGION.start_address;
    mmio_region.size = MMIO_REGION.size;
    mmio_region.statistics = {};
    
    auto mmio_result = add_monitoring_region(mmio_region);
    if (!mmio_result) {
        return unexpected(mmio_result.error());
    }
    
    COMPONENT_LOG_DEBUG("Default monitoring regions set up successfully");
    return {};
}

Result<void> PerformanceMonitor::update_region_statistics(const MemoryAccessEvent& event) {
    // Find which region this access belongs to
    for (auto& region : monitoring_regions_) {
        if (event.address >= region.base_address && 
            event.address < region.base_address + region.size) {
            
            auto& stats = region.statistics;
            stats.access_count++;
            stats.total_cycles += event.cycle_count;
            
            switch (event.access_type) {
                case MemoryAccessType::READ:
                    stats.read_count++;
                    stats.read_bytes += event.size;
                    break;
                case MemoryAccessType::WRITE:
                    stats.write_count++;
                    stats.write_bytes += event.size;
                    break;
                case MemoryAccessType::EXECUTE:
                    stats.execution_count++;
                    break;
            }
            
            if (event.cache_hit) {
                stats.cache_hits++;
            } else {
                stats.cache_misses++;
            }
            
            break; // Found the region, no need to continue
        }
    }
    
    return {};
}

AccessPatternAnalysis PerformanceMonitor::analyze_access_patterns() const {
    AccessPatternAnalysis analysis;
    
    if (access_history_.empty()) {
        return analysis;
    }
    
    // Analyze sequential access patterns
    size_t sequential_count = 0;
    size_t random_count = 0;
    
    for (size_t i = 1; i < access_history_.size(); ++i) {
        const auto& prev = access_history_[i - 1];
        const auto& curr = access_history_[i];
        
        // Check if this is a sequential access (within 64 bytes)
        if (curr.address >= prev.address && 
            curr.address - prev.address <= 64) {
            sequential_count++;
        } else {
            random_count++;
        }
    }
    
    size_t total_patterns = sequential_count + random_count;
    if (total_patterns > 0) {
        analysis.sequential_access_ratio = static_cast<double>(sequential_count) / total_patterns;
        analysis.random_access_ratio = static_cast<double>(random_count) / total_patterns;
    }
    
    // Analyze hot spots (most frequently accessed addresses)
    std::unordered_map<Address, size_t> access_counts;
    for (const auto& event : access_history_) {
        Address cache_line = event.address & ~0x3F; // 64-byte cache line alignment
        access_counts[cache_line]++;
    }
    
    // Find top 10 hot spots
    std::vector<std::pair<Address, size_t>> sorted_accesses(access_counts.begin(), access_counts.end());
    std::sort(sorted_accesses.begin(), sorted_accesses.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    analysis.hot_spots.clear();
    for (size_t i = 0; i < std::min(sorted_accesses.size(), size_t(10)); ++i) {
        HotSpot hot_spot;
        hot_spot.address = sorted_accesses[i].first;
        hot_spot.access_count = sorted_accesses[i].second;
        analysis.hot_spots.push_back(hot_spot);
    }
    
    // Calculate temporal locality (reuse distance)
    std::unordered_map<Address, size_t> last_access;
    std::vector<size_t> reuse_distances;
    
    for (size_t i = 0; i < access_history_.size(); ++i) {
        Address cache_line = access_history_[i].address & ~0x3F;
        
        auto it = last_access.find(cache_line);
        if (it != last_access.end()) {
            size_t reuse_distance = i - it->second;
            reuse_distances.push_back(reuse_distance);
        }
        
        last_access[cache_line] = i;
    }
    
    if (!reuse_distances.empty()) {
        analysis.average_reuse_distance = std::accumulate(reuse_distances.begin(), reuse_distances.end(), 0.0) / 
                                         reuse_distances.size();
    }
    
    return analysis;
}

}  // namespace m5tab5::emulator