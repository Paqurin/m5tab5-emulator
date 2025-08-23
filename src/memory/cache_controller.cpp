#include "emulator/memory/cache_controller.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/memory/cache_coherency.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"

#include <algorithm>
#include <random>
#include <cstring>

namespace m5tab5::emulator {

CacheController::CacheController(u32 core_id, CacheType type) 
    : core_id_(core_id), initialized_(false), memory_controller_(nullptr),
      coherency_controller_(nullptr), current_cycle_(0) {
    
    // Set default configuration based on cache type
    config_.type = type;
    
    LOG_DEBUG("CacheController created for core {} with type {}", 
              core_id_, static_cast<int>(type));
}

CacheController::~CacheController() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> CacheController::initialize(const CacheConfig& config, 
                                       MemoryController& memory_controller,
                                       CacheCoherencyController* coherency_controller) {
    if (initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController already initialized"));
    }
    
    config_ = config;
    memory_controller_ = &memory_controller;
    coherency_controller_ = coherency_controller;
    
    // Calculate cache organization parameters
    if (config_.size_bytes == 0 || config_.line_size == 0 || config_.associativity == 0) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid cache configuration parameters"));
    }
    
    num_sets_ = config_.size_bytes / (config_.line_size * config_.associativity);
    if (num_sets_ == 0) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Cache configuration results in zero sets"));
    }
    
    // Calculate bit field masks
    u32 offset_bits = __builtin_ctz(config_.line_size); // Count trailing zeros
    u32 index_bits = __builtin_ctz(num_sets_);
    
    offset_mask_ = config_.line_size - 1;
    index_mask_ = (1u << index_bits) - 1;
    tag_shift_ = offset_bits + index_bits;
    
    // Allocate cache sets
    cache_sets_.clear();
    cache_sets_.reserve(num_sets_);
    
    for (u32 i = 0; i < num_sets_; ++i) {
        cache_sets_.emplace_back(
            std::make_unique<CacheSet>(config_.associativity, config_.line_size)
        );
    }
    
    // Initialize statistics
    reset_statistics();
    
    initialized_ = true;
    
    LOG_INFO("CacheController initialized: {} KB, {}-way associative, {} byte lines, {} sets",
             config_.size_bytes / 1024, config_.associativity, config_.line_size, num_sets_);
    
    return {};
}

Result<void> CacheController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    // Flush all dirty cache lines before shutdown
    auto flush_result = flush_all();
    if (!flush_result) {
        LOG_WARN("Failed to flush all cache lines during shutdown: {}", 
                 static_cast<int>(flush_result.error().code()));
    }
    
    cache_sets_.clear();
    memory_controller_ = nullptr;
    coherency_controller_ = nullptr;
    initialized_ = false;
    
    LOG_DEBUG("CacheController shutdown complete for core {}", core_id_);
    
    return {};
}

Result<void> CacheController::reset() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController not initialized"));
    }
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    // Invalidate all cache lines
    for (auto& cache_set : cache_sets_) {
        for (auto& line : cache_set->lines) {
            line->valid = false;
            line->dirty = false;
            line->tag = 0;
            line->last_access_cycle = 0;
            line->access_count = 0;
            std::fill(line->data.begin(), line->data.end(), 0);
        }
        cache_set->next_replacement_index = 0;
    }
    
    reset_statistics();
    current_cycle_ = 0;
    
    LOG_DEBUG("CacheController reset complete for core {}", core_id_);
    
    return {};
}

Result<void> CacheController::read(Address address, u8* buffer, size_t size) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController not initialized"));
    }
    
    if (!buffer) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Null buffer provided"));
    }
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    size_t bytes_read = 0;
    Address current_addr = address;
    
    while (bytes_read < size) {
        // Try to find cache line
        auto line_result = find_cache_line(current_addr);
        CacheLine* line = nullptr;
        
        if (line_result) {
            // Cache hit
            line = line_result.value();
            stats_.hits++;
            
            // Update replacement info
            u32 set_index = get_cache_set_index(current_addr);
            auto& cache_set = *cache_sets_[set_index];
            for (u32 i = 0; i < cache_set.lines.size(); ++i) {
                if (cache_set.lines[i].get() == line) {
                    update_replacement_info(cache_set, i);
                    break;
                }
            }
        } else {
            // Cache miss - allocate new line
            stats_.misses++;
            
            auto alloc_result = allocate_cache_line(current_addr);
            if (!alloc_result) {
                return unexpected(alloc_result.error());
            }
            
            line = alloc_result.value();
            
            // Load line from memory
            Address line_addr = get_line_address(current_addr);
            auto read_result = memory_controller_->readBlock(line_addr, 
                                                           line->data.data(), 
                                                           config_.line_size);
            if (read_result != EmulatorError::Success) {
                return unexpected(MAKE_ERROR(MEMORY_ACCESS_ERROR, "Failed to read from memory"));
            }
            
            line->valid = true;
            line->dirty = false;
            line->tag = get_cache_tag(current_addr);
        }
        
        // Copy data from cache line
        u32 offset = get_cache_offset(current_addr);
        size_t bytes_to_copy = std::min(size - bytes_read, 
                                       static_cast<size_t>(config_.line_size - offset));
        
        std::memcpy(buffer + bytes_read, line->data.data() + offset, bytes_to_copy);
        
        bytes_read += bytes_to_copy;
        current_addr += bytes_to_copy;
        
        // Update access tracking
        line->last_access_cycle = ++current_cycle_;
        line->access_count++;
        
        // Handle prefetch if enabled
        if (config_.enable_prefetch) {
            handle_prefetch(current_addr);
        }
    }
    
    return {};
}

Result<void> CacheController::write(Address address, const u8* buffer, size_t size) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController not initialized"));
    }
    
    if (!buffer) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Null buffer provided"));
    }
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    size_t bytes_written = 0;
    Address current_addr = address;
    
    while (bytes_written < size) {
        CacheLine* line = nullptr;
        
        // Handle different write policies
        if (config_.write_policy == WritePolicy::WRITE_AROUND) {
            // Write directly to memory, bypass cache
            size_t bytes_to_write = std::min(size - bytes_written, 
                                           static_cast<size_t>(config_.line_size));
            
            auto write_result = memory_controller_->writeBlock(current_addr,
                                                             buffer + bytes_written,
                                                             bytes_to_write);
            if (write_result != EmulatorError::Success) {
                return unexpected(MAKE_ERROR(MEMORY_ACCESS_ERROR, "Failed to write to memory"));
            }
            
            bytes_written += bytes_to_write;
            current_addr += bytes_to_write;
            continue;
        }
        
        // For WRITE_THROUGH and WRITE_BACK policies, update cache
        auto line_result = find_cache_line(current_addr);
        
        if (line_result) {
            // Cache hit
            line = line_result.value();
            stats_.hits++;
        } else {
            // Cache miss - allocate new line
            stats_.misses++;
            
            auto alloc_result = allocate_cache_line(current_addr);
            if (!alloc_result) {
                return unexpected(alloc_result.error());
            }
            
            line = alloc_result.value();
            
            // For partial writes, load existing data first
            u32 offset = get_cache_offset(current_addr);
            size_t write_size = std::min(size - bytes_written, 
                                       static_cast<size_t>(config_.line_size - offset));
            
            if (offset != 0 || write_size < config_.line_size) {
                Address line_addr = get_line_address(current_addr);
                auto read_result = memory_controller_->readBlock(line_addr,
                                                               line->data.data(),
                                                               config_.line_size);
                if (read_result != EmulatorError::Success) {
                    return unexpected(MAKE_ERROR(MEMORY_ACCESS_ERROR, "Failed to read from memory"));
                }
            }
            
            line->valid = true;
            line->tag = get_cache_tag(current_addr);
        }
        
        // Update cache line with new data
        u32 offset = get_cache_offset(current_addr);
        size_t bytes_to_write = std::min(size - bytes_written,
                                       static_cast<size_t>(config_.line_size - offset));
        
        std::memcpy(line->data.data() + offset, buffer + bytes_written, bytes_to_write);
        
        // Handle write policy
        if (config_.write_policy == WritePolicy::WRITE_THROUGH) {
            // Write to memory immediately
            auto write_result = memory_controller_->writeBlock(current_addr,
                                                             buffer + bytes_written,
                                                             bytes_to_write);
            if (write_result != EmulatorError::Success) {
                return unexpected(MAKE_ERROR(MEMORY_ACCESS_ERROR, "Failed to write through to memory"));
            }
        } else if (config_.write_policy == WritePolicy::WRITE_BACK) {
            // Mark line as dirty
            line->dirty = true;
        }
        
        bytes_written += bytes_to_write;
        current_addr += bytes_to_write;
        
        // Update access tracking
        line->last_access_cycle = ++current_cycle_;
        line->access_count++;
        
        // Update replacement info
        u32 set_index = get_cache_set_index(current_addr - bytes_to_write);
        auto& cache_set = *cache_sets_[set_index];
        for (u32 i = 0; i < cache_set.lines.size(); ++i) {
            if (cache_set.lines[i].get() == line) {
                update_replacement_info(cache_set, i);
                break;
            }
        }
        
        // Notify coherency controller if present
        if (coherency_controller_) {
            notify_coherency_controller(current_addr - bytes_to_write, CoherencyOperation::WRITE);
        }
    }
    
    return {};
}

Result<void> CacheController::flush_all() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController not initialized"));
    }
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    for (auto& cache_set : cache_sets_) {
        for (auto& line : cache_set->lines) {
            if (line->valid && line->dirty) {
                Address line_addr = (line->tag << tag_shift_) | 
                                   ((&cache_set - &cache_sets_[0]) << __builtin_ctz(config_.line_size));
                
                auto write_result = write_back_line(*line, line_addr);
                if (!write_result) {
                    return unexpected(write_result.error());
                }
            }
        }
    }
    
    LOG_DEBUG("CacheController flushed all dirty lines for core {}", core_id_);
    
    return {};
}

Result<void> CacheController::invalidate_all() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(INVALID_STATE, "CacheController not initialized"));
    }
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    for (auto& cache_set : cache_sets_) {
        for (auto& line : cache_set->lines) {
            if (line->valid && line->dirty) {
                stats_.evictions++;
            }
            line->valid = false;
            line->dirty = false;
        }
    }
    
    LOG_DEBUG("CacheController invalidated all lines for core {}", core_id_);
    
    return {};
}

void CacheController::reset_statistics() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    stats_ = CacheStats{};
}

Result<CacheLine*> CacheController::find_cache_line(Address address) {
    u32 set_index = get_cache_set_index(address);
    Address tag = get_cache_tag(address);
    
    if (set_index >= cache_sets_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid cache set index"));
    }
    
    auto& cache_set = *cache_sets_[set_index];
    
    for (auto& line : cache_set.lines) {
        if (line->valid && line->tag == tag) {
            return line.get();
        }
    }
    
    return unexpected(MAKE_ERROR(CACHE_MISS, "Cache line not found"));
}

Result<CacheLine*> CacheController::allocate_cache_line(Address address) {
    u32 set_index = get_cache_set_index(address);
    Address tag = get_cache_tag(address);
    
    if (set_index >= cache_sets_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid cache set index"));
    }
    
    auto& cache_set = *cache_sets_[set_index];
    
    // Find an invalid line first
    for (u32 i = 0; i < cache_set.lines.size(); ++i) {
        if (!cache_set.lines[i]->valid) {
            cache_set.lines[i]->tag = tag;
            return cache_set.lines[i].get();
        }
    }
    
    // All lines are valid, need to evict
    u32 victim_index = select_replacement_victim(cache_set);
    auto& victim_line = *cache_set.lines[victim_index];
    
    // Write back if dirty
    if (victim_line.dirty) {
        Address victim_addr = (victim_line.tag << tag_shift_) | 
                             (set_index << __builtin_ctz(config_.line_size));
        auto write_back_result = write_back_line(victim_line, victim_addr);
        if (!write_back_result) {
            return unexpected(write_back_result.error());
        }
        stats_.evictions++;
    }
    
    // Initialize new line
    victim_line.tag = tag;
    victim_line.valid = false; // Will be set by caller
    victim_line.dirty = false;
    
    return &victim_line;
}

Result<void> CacheController::write_back_line(CacheLine& line, Address address) {
    if (!line.dirty) {
        return {}; // Nothing to write back
    }
    
    auto write_result = memory_controller_->writeBlock(address, 
                                                     line.data.data(), 
                                                     config_.line_size);
    if (write_result != EmulatorError::Success) {
        return unexpected(MAKE_ERROR(MEMORY_ACCESS_ERROR, "Failed to write back cache line"));
    }
    
    line.dirty = false;
    stats_.write_backs++;
    
    return {};
}

u32 CacheController::get_cache_set_index(Address address) const {
    return (address >> __builtin_ctz(config_.line_size)) & index_mask_;
}

Address CacheController::get_cache_tag(Address address) const {
    return address >> tag_shift_;
}

u32 CacheController::get_cache_offset(Address address) const {
    return address & offset_mask_;
}

Address CacheController::get_line_address(Address address) const {
    return address & ~static_cast<Address>(offset_mask_);
}

u32 CacheController::select_replacement_victim(const CacheSet& set) {
    switch (config_.replacement_policy) {
        case ReplacementPolicy::LRU: {
            u32 lru_index = 0;
            ClockCycle oldest_access = set.lines[0]->last_access_cycle;
            for (u32 i = 1; i < set.lines.size(); ++i) {
                if (set.lines[i]->last_access_cycle < oldest_access) {
                    oldest_access = set.lines[i]->last_access_cycle;
                    lru_index = i;
                }
            }
            return lru_index;
        }
        
        case ReplacementPolicy::FIFO:
            return const_cast<CacheSet&>(set).next_replacement_index;
        
        case ReplacementPolicy::RANDOM: {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<u32> dist(0, set.lines.size() - 1);
            return dist(gen);
        }
        
        case ReplacementPolicy::LFU: {
            u32 lfu_index = 0;
            u32 lowest_count = set.lines[0]->access_count;
            for (u32 i = 1; i < set.lines.size(); ++i) {
                if (set.lines[i]->access_count < lowest_count) {
                    lowest_count = set.lines[i]->access_count;
                    lfu_index = i;
                }
            }
            return lfu_index;
        }
    }
    
    return 0; // Default fallback
}

void CacheController::update_replacement_info(CacheSet& set, u32 line_index) {
    // Update FIFO index
    if (config_.replacement_policy == ReplacementPolicy::FIFO) {
        set.next_replacement_index = (set.next_replacement_index + 1) % set.lines.size();
    }
    
    // LRU and LFU are updated during access
}

Result<void> CacheController::handle_prefetch(Address address) {
    if (!config_.enable_prefetch) {
        return {};
    }
    
    // Simple next-line prefetching
    for (u32 i = 1; i <= config_.prefetch_distance; ++i) {
        Address prefetch_addr = get_line_address(address) + (i * config_.line_size);
        
        // Check if already in cache
        auto existing_line = find_cache_line(prefetch_addr);
        if (existing_line) {
            stats_.prefetch_hits++;
            continue; // Already cached
        }
        
        // Try to allocate and prefetch
        auto alloc_result = allocate_cache_line(prefetch_addr);
        if (alloc_result) {
            auto line = alloc_result.value();
            auto read_result = memory_controller_->readBlock(prefetch_addr,
                                                           line->data.data(),
                                                           config_.line_size);
            if (read_result == EmulatorError::Success) {
                line->valid = true;
                line->tag = get_cache_tag(prefetch_addr);
            } else {
                stats_.prefetch_misses++;
            }
        }
    }
    
    return {};
}

void CacheController::notify_coherency_controller(Address address, CoherencyOperation op) {
    if (coherency_controller_) {
        // Notify coherency controller of cache operation
        // This would trigger coherency protocol actions
    }
}

void CacheController::dump_cache_contents() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    LOG_INFO("Cache contents for core {} (type {}):", 
             core_id_, static_cast<int>(config_.type));
    LOG_INFO("  Configuration: {} sets, {} ways, {} byte lines",
             num_sets_, config_.associativity, config_.line_size);
    LOG_INFO("  Statistics: {} hits, {} misses, {:.2f}% hit rate",
             stats_.hits, stats_.misses, get_hit_rate() * 100.0);
    
    u32 valid_lines = 0, dirty_lines = 0;
    for (const auto& cache_set : cache_sets_) {
        for (const auto& line : cache_set->lines) {
            if (line->valid) {
                valid_lines++;
                if (line->dirty) dirty_lines++;
            }
        }
    }
    
    LOG_INFO("  Occupancy: {} valid lines, {} dirty lines", valid_lines, dirty_lines);
}

std::vector<Address> CacheController::get_cached_addresses() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    std::vector<Address> addresses;
    
    for (u32 set_index = 0; set_index < cache_sets_.size(); ++set_index) {
        const auto& cache_set = *cache_sets_[set_index];
        for (const auto& line : cache_set.lines) {
            if (line->valid) {
                Address address = (line->tag << tag_shift_) | 
                                 (set_index << __builtin_ctz(config_.line_size));
                addresses.push_back(address);
            }
        }
    }
    
    return addresses;
}

size_t CacheController::get_dirty_line_count() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    size_t dirty_count = 0;
    
    for (const auto& cache_set : cache_sets_) {
        for (const auto& line : cache_set->lines) {
            if (line->valid && line->dirty) {
                dirty_count++;
            }
        }
    }
    
    return dirty_count;
}

// Factory methods
std::unique_ptr<CacheController> CacheControllerFactory::create_l1_icache(u32 core_id) {
    return std::make_unique<CacheController>(core_id, CacheType::INSTRUCTION_CACHE);
}

std::unique_ptr<CacheController> CacheControllerFactory::create_l1_dcache(u32 core_id) {
    return std::make_unique<CacheController>(core_id, CacheType::DATA_CACHE);
}

std::unique_ptr<CacheController> CacheControllerFactory::create_unified_cache(u32 core_id) {
    return std::make_unique<CacheController>(core_id, CacheType::UNIFIED_CACHE);
}

CacheConfig CacheControllerFactory::get_default_icache_config() {
    CacheConfig config;
    config.type = CacheType::INSTRUCTION_CACHE;
    config.size_bytes = 16384; // 16KB
    config.line_size = 32;     // 32-byte lines
    config.associativity = 2;  // 2-way associative
    config.replacement_policy = ReplacementPolicy::LRU;
    config.write_policy = WritePolicy::WRITE_THROUGH; // I-cache is typically write-through
    config.enable_prefetch = true;
    config.prefetch_distance = 1;
    return config;
}

CacheConfig CacheControllerFactory::get_default_dcache_config() {
    CacheConfig config;
    config.type = CacheType::DATA_CACHE;
    config.size_bytes = 32768; // 32KB
    config.line_size = 64;     // 64-byte lines
    config.associativity = 4;  // 4-way associative
    config.replacement_policy = ReplacementPolicy::LRU;
    config.write_policy = WritePolicy::WRITE_BACK;
    config.enable_prefetch = true;
    config.prefetch_distance = 2;
    return config;
}

CacheConfig CacheControllerFactory::get_esp32p4_cache_config(CacheType type) {
    CacheConfig config;
    config.type = type;
    
    if (type == CacheType::INSTRUCTION_CACHE) {
        config.size_bytes = 16384; // ESP32-P4 L1 I-cache
        config.line_size = 32;
        config.associativity = 2;
        config.write_policy = WritePolicy::WRITE_THROUGH;
    } else {
        config.size_bytes = 32768; // ESP32-P4 L1 D-cache  
        config.line_size = 64;
        config.associativity = 4;
        config.write_policy = WritePolicy::WRITE_BACK;
    }
    
    config.replacement_policy = ReplacementPolicy::LRU;
    config.enable_prefetch = true;
    config.prefetch_distance = 2;
    
    return config;
}

} // namespace m5tab5::emulator