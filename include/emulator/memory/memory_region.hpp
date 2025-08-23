#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>
#include <fstream>

namespace m5tab5::emulator {

// Forward declarations for MMIO callbacks
class MemoryRegion;
using MMIOReadCallback = std::function<u8(Address)>;
using MMIOWriteCallback = std::function<void(Address, u8)>;

/**
 * @brief Memory region types for ESP32-P4 emulation
 */
enum class MemoryType {
    Flash,          // NOR Flash memory (read-only, executable)
    PSRAM,          // External PSRAM (read/write, cacheable)
    IRAM,           // Internal RAM for instructions (read/write/execute, fast)
    DRAM,           // Internal RAM for data (read/write, fast)
    SRAM,           // Static RAM (read/write, fast)
    ROM,            // Boot ROM (read-only, executable)
    MMIO,           // Memory-mapped I/O peripheral registers
    PERIPHERAL,     // Memory-mapped peripheral registers (alias for MMIO)
    RESERVED,       // Reserved/unmapped regions
    CACHE           // Cache memory regions
};

/**
 * @brief Represents a contiguous region of emulated memory
 * 
 * Each memory region has specific attributes like access permissions,
 * caching behavior, and performance characteristics that match the
 * ESP32-P4 memory architecture.
 */
class MemoryRegion {
public:
    struct AccessStats {
        u64 read_count;
        u64 write_count;
        u64 execute_count;
        u64 cache_hits;
        u64 cache_misses;
        
        AccessStats() : read_count(0), write_count(0), execute_count(0),
                       cache_hits(0), cache_misses(0) {}
    };

    MemoryRegion(const std::string& name,
                 Address start_address,
                 size_t size,
                 MemoryType type,
                 bool writable = true,
                 bool executable = false,
                 bool cacheable = true);
    
    virtual ~MemoryRegion();

    // Memory access interface
    virtual Result<void> read(Address offset, u8* buffer, size_t size);
    virtual Result<void> write(Address offset, const u8* buffer, size_t size);
    virtual Result<void> fill(Address offset, u8 value, size_t size);
    
    // Additional memory access methods
    Result<void> read_bytes(Address address, u8* buffer, size_t count) const;
    Result<void> write_bytes(Address address, const u8* buffer, size_t count);

    // Memory management
    Result<void> initialize();
    void shutdown();
    Result<void> reset();
    void clear();

    // Address translation and validation
    bool contains_address(Address address) const;
    bool is_valid_range(Address address, size_t size) const;
    Address translate_address(Address absolute_address) const;

    // Property access
    const std::string& get_name() const { return name_; }
    Address get_start_address() const { return start_address_; }
    Address get_end_address() const { return start_address_ + size_ - 1; }
    size_t get_size() const { return size_; }
    MemoryType get_type() const { return type_; }
    
    bool is_writable() const { return writable_; }
    bool is_executable() const { return executable_; }
    bool is_cacheable() const { return cacheable_; }
    bool is_initialized() const { return initialized_; }

    // Access control
    void set_writable(bool writable) { writable_ = writable; }
    void set_executable(bool executable) { executable_ = executable; }
    void set_read_only() { writable_ = false; }

    // Statistics and debugging
    const AccessStats& get_access_stats() const { return stats_; }
    void reset_statistics();
    void dump_info() const;

    // File I/O operations
    Result<void> load_from_file(const std::string& file_path, Address offset = 0);
    Result<void> save_to_file(const std::string& file_path) const;
    
    // MMIO callback management
    void register_mmio_read_callback(Address address, MMIOReadCallback callback);
    void register_mmio_write_callback(Address address, MMIOWriteCallback callback);
    
    // Memory content access (for debugging/inspection)
    const u8* get_data_ptr() const { return data_.get(); }
    u8* get_data() const;
    Result<std::vector<u8>> read_range(Address offset, size_t size) const;

protected:
    // Internal helpers
    virtual Result<void> allocate_storage();
    virtual void deallocate_storage();
    void update_access_stats(bool is_read, bool is_write, bool is_execute);
    
    // MMIO handling
    Result<void> handle_mmio_read(Address address, u8* buffer, size_t count) const;
    Result<void> handle_mmio_write(Address address, const u8* buffer, size_t count);

private:
    // Region properties
    std::string name_;
    Address start_address_;
    size_t size_;
    MemoryType type_;
    bool writable_;
    bool executable_;
    bool cacheable_;
    bool initialized_;

    // Storage
    std::unique_ptr<u8[]> data_;
    
    // MMIO callbacks
    std::unordered_map<Address, MMIOReadCallback> mmio_read_callbacks_;
    std::unordered_map<Address, MMIOWriteCallback> mmio_write_callbacks_;
    
    // Statistics
    mutable AccessStats stats_;
    
    // Validation helpers
    bool is_valid_offset(Address offset) const;
    bool is_valid_access(Address offset, size_t access_size) const;
};

/**
 * @brief Specialized memory region for Flash storage
 */
class FlashRegion : public MemoryRegion {
public:
    FlashRegion(const std::string& name, Address start_address, size_t size);
    
    // Flash-specific operations
    Result<void> erase_sector(Address sector_address);
    Result<void> erase_range(Address start_address, size_t size);
    Result<void> program_page(Address page_address, const u8* data, size_t size);
    
    // Flash characteristics
    static constexpr size_t SECTOR_SIZE = 4096;    // 4KB sectors
    static constexpr size_t PAGE_SIZE = 256;       // 256-byte pages
    static constexpr size_t BLOCK_SIZE = 65536;    // 64KB blocks

private:
    std::vector<bool> programmed_pages_;
};

/**
 * @brief Specialized memory region for PSRAM
 */
class PsramRegion : public MemoryRegion {
public:
    PsramRegion(const std::string& name, Address start_address, size_t size);
    
    // PSRAM-specific features
    void enable_burst_mode(bool enable) { burst_mode_enabled_ = enable; }
    bool is_burst_mode_enabled() const { return burst_mode_enabled_; }

private:
    bool burst_mode_enabled_;
};

/**
 * @brief Memory region factory
 */
class MemoryRegionFactory {
public:
    static std::unique_ptr<MemoryRegion> create_flash_region(
        const std::string& name, Address start, size_t size);
    
    static std::unique_ptr<MemoryRegion> create_psram_region(
        const std::string& name, Address start, size_t size);
        
    static std::unique_ptr<MemoryRegion> create_iram_region(
        const std::string& name, Address start, size_t size);
        
    static std::unique_ptr<MemoryRegion> create_dram_region(
        const std::string& name, Address start, size_t size);
        
    static std::unique_ptr<MemoryRegion> create_peripheral_region(
        const std::string& name, Address start, size_t size);
};

} // namespace m5tab5::emulator