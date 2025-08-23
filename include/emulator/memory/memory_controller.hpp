#pragma once

#include "emulator/core/types.hpp"
#include "emulator/memory/memory_interface.hpp"
#include "emulator/memory/memory_region.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <thread>
#include <queue>
#include <map>

namespace m5tab5::emulator {

// Forward declaration
class MemoryMappingUnit;

// Cache statistics structure
struct CacheStatistics {
    u64 hits = 0;
    u64 misses = 0;
    u64 evictions = 0;
    
    double hit_rate() const {
        auto total = hits + misses;
        return total > 0 ? static_cast<double>(hits) / total : 0.0;
    }
};

/**
 * @brief Memory controller managing all memory regions and caching
 * 
 * Provides unified memory access for:
 * - Flash memory (16MB)
 * - PSRAM (32MB) 
 * - SRAM (768KB)
 * - Cache management (8KB)
 * - Memory-mapped peripherals
 * - DMA operations
 */
class MemoryController : public MemoryInterface {
public:
    // Construction and lifecycle
    MemoryController();
    ~MemoryController();

    // Initialize and manage memory controller
    Result<void> initialize(const Configuration& config);
    Result<void> shutdown();
    Result<void> reset();

    // Primary memory access interface
    Result<u8> read_u8(Address address);
    Result<u16> read_u16(Address address);
    Result<u32> read_u32(Address address);
    Result<std::vector<u8>> read_bytes(Address address, size_t count);
    
    Result<void> write_u8(Address address, u8 value);
    Result<void> write_u16(Address address, u16 value);
    Result<void> write_u32(Address address, u32 value);
    Result<void> write_bytes(Address address, const u8* data, size_t count);

    // Address validation and information
    Result<bool> is_valid_address(Address address) const;
    Result<MemoryType> get_memory_type(Address address) const;

    // Cache statistics and management
    const CacheStatistics& get_cache_statistics() const;
    void clear_cache_statistics();

    // MemoryInterface implementation (adapter methods)
    EmulatorError read8(Address address, uint8_t& value) override;
    EmulatorError read16(Address address, uint16_t& value) override;
    EmulatorError read32(Address address, uint32_t& value) override;
    EmulatorError write8(Address address, uint8_t value) override;
    EmulatorError write16(Address address, uint16_t value) override;
    EmulatorError write32(Address address, uint32_t value) override;
    
    bool isValidAddress(Address address) const override;
    bool isWritableAddress(Address address) const override;
    bool isExecutableAddress(Address address) const override;
    
    // Bulk operations inherited from MemoryInterface

private:
    // Internal helper methods
    Result<SharedPtr<MemoryRegion>> find_memory_region(Address address) const;
    Result<void> initialize_flash_region(size_t size);
    Result<void> initialize_psram_region(size_t size);
    Result<void> initialize_sram_region(size_t size);
    Result<void> initialize_mmio_region();
    Result<void> initialize_cache();
    Result<void> setup_memory_mappings();
    
    // Cache management
    Result<std::vector<u8>> try_cache_read(Address address, size_t count);
    void update_cache(Address address, const std::vector<u8>& data);
    void invalidate_cache_range(Address address, size_t count);
    
    // Cache data structures
    struct CacheLine {
        std::vector<u8> data;
        bool valid = false;
    };
    
    // State tracking
    bool initialized_;
    
    // Memory regions storage
    std::map<std::string, SharedPtr<MemoryRegion>> memory_regions_;
    
    // Cache management
    std::unordered_map<Address, CacheLine> cache_lines_;
    size_t cache_line_size_ = 32;
    CacheStatistics cache_stats_;
    
    // Memory Management Unit (TODO: Implement MemoryMappingUnit)
    // std::unique_ptr<MemoryMappingUnit> mmu_;
};

} // namespace m5tab5::emulator