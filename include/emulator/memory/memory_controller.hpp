#pragma once

#include "emulator/core/types.hpp"
#include "emulator/memory/memory_interface.hpp"
#include "emulator/memory/memory_region.hpp"
#include "emulator/memory/cache_controller.hpp"

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <thread>
#include <queue>

namespace m5tab5::emulator {

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
    struct MemoryConfig {
        bool enable_cache = true;
        bool enable_mmu = true;
        bool strict_alignment = false;
        uint32_t cache_line_size = 32;
        uint32_t cache_associativity = 4;
    };

    explicit MemoryController(const MemoryConfig& config);
    ~MemoryController();

    // MemoryInterface implementation
    EmulatorError read8(Address address, uint8_t& value) override;
    EmulatorError read16(Address address, uint16_t& value) override;
    EmulatorError read32(Address address, uint32_t& value) override;
    EmulatorError write8(Address address, uint8_t value) override;
    EmulatorError write16(Address address, uint16_t value) override;
    EmulatorError write32(Address address, uint32_t value) override;

    // Bulk operations
    EmulatorError readBlock(Address address, void* buffer, size_t size) override;
    EmulatorError writeBlock(Address address, const void* buffer, size_t size) override;

    // Memory region management
    EmulatorError addRegion(std::unique_ptr<MemoryRegion> region);
    EmulatorError removeRegion(Address base_address);
    MemoryRegion* findRegion(Address address);

    // Cache operations
    EmulatorError flushCache();
    EmulatorError invalidateCache(Address address, size_t size);
    CacheController& getCacheController() { return *cache_; }

    // Memory mapping and protection
    EmulatorError mapMemory(Address virtual_addr, Address physical_addr, 
                           size_t size, uint32_t permissions);
    EmulatorError unmapMemory(Address virtual_addr, size_t size);
    EmulatorError setMemoryProtection(Address address, size_t size, uint32_t permissions);

    // DMA operations
    EmulatorError dmaTransfer(Address src, Address dst, size_t size, 
                             uint32_t channel_id = 0);
    bool isDMABusy(uint32_t channel_id = 0) const;

    // Memory statistics
    struct MemoryStatistics {
        uint64_t read_operations = 0;
        uint64_t write_operations = 0;
        uint64_t cache_hits = 0;
        uint64_t cache_misses = 0;
        uint64_t dma_transfers = 0;
        uint64_t page_faults = 0;
        size_t allocated_memory = 0;
    };

    const MemoryStatistics& getStatistics() const { return stats_; }
    void resetStatistics();

    // Debug and inspection
    std::vector<MemoryRegion*> getAllRegions() const;
    EmulatorError dumpMemory(Address address, size_t size, std::vector<uint8_t>& output);
    EmulatorError loadMemoryFromFile(Address address, const std::string& filename);
    EmulatorError saveMemoryToFile(Address address, size_t size, const std::string& filename);

private:
    // Memory region lookup optimization
    struct RegionEntry {
        Address base;
        Address end;
        MemoryRegion* region;
    };

    // Internal operations
    EmulatorError validateAccess(Address address, size_t size, bool is_write);
    EmulatorError translateAddress(Address virtual_addr, Address& physical_addr);
    MemoryRegion* findRegionFast(Address address);
    
    // DMA implementation
    struct DMAChannel {
        bool busy = false;
        Address src = 0;
        Address dst = 0;
        size_t remaining = 0;
        std::thread transfer_thread;
    };

    void dmaTransferWorker(uint32_t channel_id);

    // Configuration
    MemoryConfig config_;

    // Memory regions
    std::vector<std::unique_ptr<MemoryRegion>> regions_;
    std::vector<RegionEntry> region_lookup_; // Sorted for binary search
    mutable std::shared_mutex regions_mutex_;

    // Forward declarations to avoid circular dependencies
    class MemoryRegion;
    class CacheController;
    
    // Cache controller
    std::unique_ptr<CacheController> cache_;

    // Memory Management Unit (MMU)
    struct MMUEntry {
        Address virtual_base;
        Address physical_base;
        size_t size;
        uint32_t permissions;
    };
    std::vector<MMUEntry> mmu_table_;
    mutable std::shared_mutex mmu_mutex_;

    // DMA channels
    static constexpr uint32_t MAX_DMA_CHANNELS = 8;
    std::array<DMAChannel, MAX_DMA_CHANNELS> dma_channels_;

    // Statistics and monitoring
    mutable std::mutex stats_mutex_;
    MemoryStatistics stats_;

    // Performance optimization
    mutable Address last_access_address_ = 0;
    mutable MemoryRegion* last_access_region_ = nullptr;
};

} // namespace m5tab5::emulator