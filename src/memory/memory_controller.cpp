#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("MemoryController");

MemoryController::MemoryController() 
    : initialized_(false) {
    COMPONENT_LOG_DEBUG("MemoryController created");
}

MemoryController::~MemoryController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("MemoryController destroyed");
}

Result<void> MemoryController::initialize(const Configuration& config) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Memory controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing memory controller");
    
    try {
        // Initialize memory regions based on configuration
        RETURN_IF_ERROR(initialize_flash_region(config.get_flash_size()));
        RETURN_IF_ERROR(initialize_psram_region(config.get_psram_size()));
        RETURN_IF_ERROR(initialize_sram_region(config.get_sram_size()));
        RETURN_IF_ERROR(initialize_mmio_region());
        
        // Initialize cache
        cache_line_size_ = config.get_cache_line_size();
        RETURN_IF_ERROR(initialize_cache());
        
        // Initialize MMU
        mmu_ = std::make_unique<MemoryMappingUnit>();
        RETURN_IF_ERROR(mmu_->initialize(config));
        
        // Set up memory region mappings
        RETURN_IF_ERROR(setup_memory_mappings());
        
        initialized_ = true;
        COMPONENT_LOG_INFO("Memory controller initialized successfully");
        
        return {};
        
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during memory controller initialization: " + std::string(e.what())));
    }
}

Result<void> MemoryController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down memory controller");
    
    // Shutdown MMU
    if (mmu_) {
        mmu_->shutdown();
        mmu_.reset();
    }
    
    // Clear memory regions
    memory_regions_.clear();
    
    // Clear cache
    cache_lines_.clear();
    
    initialized_ = false;
    COMPONENT_LOG_INFO("Memory controller shutdown completed");
    
    return {};
}

Result<void> MemoryController::reset() {
    COMPONENT_LOG_INFO("Resetting memory controller");
    
    // Clear cache
    cache_lines_.clear();
    cache_stats_ = {};
    
    // Reset MMU
    if (mmu_) {
        RETURN_IF_ERROR(mmu_->reset());
    }
    
    // Reset memory regions (keep data but reset state)
    for (auto& [name, region] : memory_regions_) {
        if (region->is_writable()) {
            // Clear writable memory
            std::memset(region->get_data(), 0, region->get_size());
        }
    }
    
    COMPONENT_LOG_INFO("Memory controller reset completed");
    return {};
}

Result<u8> MemoryController::read_u8(Address address) {
    auto data = read_bytes(address, 1);
    if (!data) {
        return std::unexpected(data.error());
    }
    return data.value()[0];
}

Result<u16> MemoryController::read_u16(Address address) {
    auto data = read_bytes(address, 2);
    if (!data) {
        return std::unexpected(data.error());
    }
    
    // Little-endian conversion
    u16 value = static_cast<u16>(data.value()[0]) |
                (static_cast<u16>(data.value()[1]) << 8);
    return value;
}

Result<u32> MemoryController::read_u32(Address address) {
    auto data = read_bytes(address, 4);
    if (!data) {
        return std::unexpected(data.error());
    }
    
    // Little-endian conversion
    u32 value = static_cast<u32>(data.value()[0]) |
                (static_cast<u32>(data.value()[1]) << 8) |
                (static_cast<u32>(data.value()[2]) << 16) |
                (static_cast<u32>(data.value()[3]) << 24);
    return value;
}

Result<std::vector<u8>> MemoryController::read_bytes(Address address, size_t count) {
    if (count == 0) {
        return std::vector<u8>{};
    }
    
    // Check for address alignment if required
    if (address % 4 != 0 && count >= 4) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Unaligned memory access at address 0x" + std::to_string(address)));
    }
    
    // Try cache first for cacheable regions
    auto region = find_memory_region(address);
    if (!region) {
        return std::unexpected(region.error());
    }
    
    if (region.value()->is_cacheable()) {
        auto cached_data = try_cache_read(address, count);
        if (cached_data) {
            cache_stats_.hits++;
            return cached_data.value();
        }
        cache_stats_.misses++;
    }
    
    // Read from memory region
    std::vector<u8> data(count);
    RETURN_IF_ERROR(region.value()->read_bytes(address, data.data(), count));
    
    // Update cache if region is cacheable
    if (region.value()->is_cacheable()) {
        update_cache(address, data);
    }
    
    return data;
}

Result<void> MemoryController::write_u8(Address address, u8 value) {
    return write_bytes(address, &value, 1);
}

Result<void> MemoryController::write_u16(Address address, u16 value) {
    // Little-endian conversion
    u8 data[2] = {
        static_cast<u8>(value & 0xFF),
        static_cast<u8>((value >> 8) & 0xFF)
    };
    return write_bytes(address, data, 2);
}

Result<void> MemoryController::write_u32(Address address, u32 value) {
    // Little-endian conversion
    u8 data[4] = {
        static_cast<u8>(value & 0xFF),
        static_cast<u8>((value >> 8) & 0xFF),
        static_cast<u8>((value >> 16) & 0xFF),
        static_cast<u8>((value >> 24) & 0xFF)
    };
    return write_bytes(address, data, 4);
}

Result<void> MemoryController::write_bytes(Address address, const u8* data, size_t count) {
    if (count == 0 || !data) {
        return {};
    }
    
    // Check for address alignment if required
    if (address % 4 != 0 && count >= 4) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Unaligned memory access at address 0x" + std::to_string(address)));
    }
    
    auto region = find_memory_region(address);
    if (!region) {
        return std::unexpected(region.error());
    }
    
    if (!region.value()->is_writable()) {
        return std::unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
            "Attempt to write to read-only memory at address 0x" + std::to_string(address)));
    }
    
    // Write to memory region
    RETURN_IF_ERROR(region.value()->write_bytes(address, data, count));
    
    // Invalidate cache entries for this address range
    if (region.value()->is_cacheable()) {
        invalidate_cache_range(address, count);
    }
    
    return {};
}

Result<bool> MemoryController::is_valid_address(Address address) const {
    auto region = find_memory_region(address);
    return region.has_value();
}

Result<MemoryType> MemoryController::get_memory_type(Address address) const {
    auto region = find_memory_region(address);
    if (!region) {
        return std::unexpected(region.error());
    }
    return region.value()->get_type();
}

const CacheStatistics& MemoryController::get_cache_statistics() const {
    return cache_stats_;
}

void MemoryController::clear_cache_statistics() {
    cache_stats_ = {};
}

Result<SharedPtr<MemoryRegion>> MemoryController::find_memory_region(Address address) const {
    for (const auto& [name, region] : memory_regions_) {
        if (region->contains_address(address)) {
            return region;
        }
    }
    
    return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
        "Invalid memory address: 0x" + std::to_string(address)));
}

Result<void> MemoryController::initialize_flash_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing Flash region: {} bytes", size);
    
    auto flash_region = std::make_shared<MemoryRegion>(
        "Flash", FLASH_REGION.start_address, size, FLASH_REGION.type,
        FLASH_REGION.writable, FLASH_REGION.executable, FLASH_REGION.cacheable
    );
    
    RETURN_IF_ERROR(flash_region->initialize());
    memory_regions_["Flash"] = flash_region;
    
    return {};
}

Result<void> MemoryController::initialize_psram_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing PSRAM region: {} bytes", size);
    
    auto psram_region = std::make_shared<MemoryRegion>(
        "PSRAM", PSRAM_REGION.start_address, size, PSRAM_REGION.type,
        PSRAM_REGION.writable, PSRAM_REGION.executable, PSRAM_REGION.cacheable
    );
    
    RETURN_IF_ERROR(psram_region->initialize());
    memory_regions_["PSRAM"] = psram_region;
    
    return {};
}

Result<void> MemoryController::initialize_sram_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing SRAM region: {} bytes", size);
    
    auto sram_region = std::make_shared<MemoryRegion>(
        "SRAM", SRAM_REGION.start_address, size, SRAM_REGION.type,
        SRAM_REGION.writable, SRAM_REGION.executable, SRAM_REGION.cacheable
    );
    
    RETURN_IF_ERROR(sram_region->initialize());
    memory_regions_["SRAM"] = sram_region;
    
    return {};
}

Result<void> MemoryController::initialize_mmio_region() {
    COMPONENT_LOG_DEBUG("Initializing MMIO region");
    
    auto mmio_region = std::make_shared<MemoryRegion>(
        "MMIO", MMIO_REGION.start_address, MMIO_REGION.size, MMIO_REGION.type,
        MMIO_REGION.writable, MMIO_REGION.executable, MMIO_REGION.cacheable
    );
    
    RETURN_IF_ERROR(mmio_region->initialize());
    memory_regions_["MMIO"] = mmio_region;
    
    return {};
}

Result<void> MemoryController::initialize_cache() {
    COMPONENT_LOG_DEBUG("Initializing cache with line size: {} bytes", cache_line_size_);
    
    // Initialize cache with a reasonable number of lines
    const size_t cache_size = 8192;  // 8KB cache
    cache_lines_.reserve(cache_size / cache_line_size_);
    
    cache_stats_ = {};
    
    return {};
}

Result<void> MemoryController::setup_memory_mappings() {
    COMPONENT_LOG_DEBUG("Setting up memory mappings");
    
    // Set up standard ESP32-P4 memory mappings
    // This is where we would configure virtual-to-physical address translations
    // For now, we use direct mapping (virtual == physical)
    
    return {};
}

Result<std::vector<u8>> MemoryController::try_cache_read(Address address, size_t count) {
    Address cache_line_addr = address & ~(cache_line_size_ - 1);
    
    auto it = cache_lines_.find(cache_line_addr);
    if (it == cache_lines_.end()) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED, "Cache miss"));
    }
    
    const auto& cache_line = it->second;
    size_t offset = address - cache_line_addr;
    
    if (offset + count > cache_line.data.size()) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED, "Cache read overflow"));
    }
    
    std::vector<u8> data(count);
    std::memcpy(data.data(), cache_line.data.data() + offset, count);
    
    return data;
}

void MemoryController::update_cache(Address address, const std::vector<u8>& data) {
    Address cache_line_addr = address & ~(cache_line_size_ - 1);
    
    auto& cache_line = cache_lines_[cache_line_addr];
    if (cache_line.data.empty()) {
        cache_line.data.resize(cache_line_size_);
        cache_line.valid = true;
    }
    
    size_t offset = address - cache_line_addr;
    size_t copy_size = std::min(data.size(), cache_line_size_ - offset);
    
    std::memcpy(cache_line.data.data() + offset, data.data(), copy_size);
}

void MemoryController::invalidate_cache_range(Address address, size_t count) {
    Address start_line = address & ~(cache_line_size_ - 1);
    Address end_line = (address + count - 1) & ~(cache_line_size_ - 1);
    
    for (Address line_addr = start_line; line_addr <= end_line; line_addr += cache_line_size_) {
        auto it = cache_lines_.find(line_addr);
        if (it != cache_lines_.end()) {
            cache_lines_.erase(it);
        }
    }
}

}  // namespace m5tab5::emulator