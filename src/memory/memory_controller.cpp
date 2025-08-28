#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/storage/flash_controller.hpp"
#include "emulator/storage/partition_table.hpp"
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
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Memory controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing memory controller");
    
    try {
        // Initialize memory regions based on configuration
        auto memory_config = config.getMemoryConfig();
        
        // Initialize ESP32-P4 memory regions in proper order
        // Boot ROM must be initialized first as CPU starts from reset vector
        RETURN_IF_ERROR(initialize_boot_rom_region());
        
        // Flash XIP region (execute-in-place via MMU)
        RETURN_IF_ERROR(initialize_flash_region(memory_config.flash_size));
        
        // External PSRAM (cached via L2 cache)
        RETURN_IF_ERROR(initialize_psram_region(memory_config.psram_size));
        
        // Internal L2 memory (high-speed SRAM)
        RETURN_IF_ERROR(initialize_sram_region(memory_config.sram_size));
        
        // Memory-mapped I/O peripherals
        RETURN_IF_ERROR(initialize_mmio_region());
        
        // Initialize cache
        cache_line_size_ = memory_config.cache_line_size;
        RETURN_IF_ERROR(initialize_cache());
        
        // Initialize MMU (TODO: Implement MemoryMappingUnit)
        // mmu_ = std::make_unique<MemoryMappingUnit>();
        // RETURN_IF_ERROR(mmu_->initialize(config));
        
        // Set up memory region mappings
        RETURN_IF_ERROR(setup_memory_mappings());
        
        // Validate memory layout
        RETURN_IF_ERROR(validate_memory_layout());
        
        initialized_ = true;
        COMPONENT_LOG_INFO("Memory controller initialized successfully with {} regions", memory_regions_.size());
        
        return {};
        
    } catch (const std::exception& e) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during memory controller initialization: " + std::string(e.what())));
    }
}

Result<void> MemoryController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down memory controller");
    
    // Shutdown Flash Controller
    if (flash_controller_) {
        auto shutdown_result = flash_controller_->shutdown();
        if (!shutdown_result.has_value()) {
            COMPONENT_LOG_WARN("Flash controller shutdown failed: {}", 
                              static_cast<int>(shutdown_result.error()));
        }
        flash_controller_.reset();
    }
    
    // Clear flash memory region
    flash_memory_region_.reset();
    
    // Shutdown Boot ROM
    if (boot_rom_) {
        boot_rom_->shutdown();
        boot_rom_.reset();
    }
    
    // Shutdown MMU (TODO: Implement MemoryMappingUnit)
    // if (mmu_) {
    //     mmu_->shutdown();
    //     mmu_.reset();
    // }
    
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
    
    // Reset MMU (TODO: Implement MemoryMappingUnit)
    // if (mmu_) {
    //     RETURN_IF_ERROR(mmu_->reset());
    // }
    
    // Reset memory regions (keep data but reset state)
    for (auto& [name, region] : memory_regions_) {
        if (region->is_writable()) {
            // Clear writable memory
            // Clear writable memory by getting direct access to data
            const auto* data_ptr = region->get_data_ptr();
            if (data_ptr) {
                std::memset(const_cast<u8*>(data_ptr), 0, region->get_size());
            }
        }
    }
    
    COMPONENT_LOG_INFO("Memory controller reset completed");
    return {};
}

Result<u8> MemoryController::read_u8(Address address) {
    auto data = read_bytes(address, 1);
    if (!data) {
        return unexpected(data.error());
    }
    return data.value()[0];
}

Result<u16> MemoryController::read_u16(Address address) {
    auto data = read_bytes(address, 2);
    if (!data) {
        return unexpected(data.error());
    }
    
    // Little-endian conversion
    u16 value = static_cast<u16>(data.value()[0]) |
                (static_cast<u16>(data.value()[1]) << 8);
    return value;
}

Result<u32> MemoryController::read_u32(Address address) {
    auto data = read_bytes(address, 4);
    if (!data) {
        return unexpected(data.error());
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
        return unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Unaligned memory access at address 0x" + std::to_string(address)));
    }
    
    // Try cache first for cacheable regions
    auto region = find_memory_region(address);
    if (!region) {
        return unexpected(region.error());
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
    Address region_offset = region.value()->translate_address(address);
    RETURN_IF_ERROR(region.value()->read(region_offset, data.data(), count));
    
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
        return unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Unaligned memory access at address 0x" + std::to_string(address)));
    }
    
    auto region = find_memory_region(address);
    if (!region) {
        return unexpected(region.error());
    }
    
    if (!region.value()->is_writable()) {
        return unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
            "Attempt to write to read-only memory at address 0x" + std::to_string(address)));
    }
    
    // Write to memory region
    Address region_offset = region.value()->translate_address(address);
    RETURN_IF_ERROR(region.value()->write(region_offset, data, count));
    
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
        return unexpected(region.error());
    }
    return region.value()->get_type();
}

const CacheStatistics& MemoryController::get_cache_statistics() const {
    return cache_stats_;
}

void MemoryController::clear_cache_statistics() {
    cache_stats_ = {};
}

Result<void> MemoryController::initialize_boot_rom_region() {
    COMPONENT_LOG_DEBUG("Initializing ESP32-P4 Boot ROM region");
    
    boot_rom_ = std::make_unique<BootROM>();
    RETURN_IF_ERROR(boot_rom_->initialize());
    
    // Add Boot ROM memory region to our regions
    auto boot_rom_memory = boot_rom_->getMemoryRegion();
    memory_regions_["BootROM"] = boot_rom_memory;
    
    // Log the reset vector for CPU initialization
    auto reset_vector_result = boot_rom_->getResetVector();
    if (reset_vector_result.has_value()) {
        COMPONENT_LOG_INFO("Boot ROM initialized: 32KB at 0x{:08X}, reset vector at 0x{:08X}",
                          BOOT_ROM_BASE, reset_vector_result.value());
    } else {
        COMPONENT_LOG_INFO("Boot ROM initialized: 32KB at 0x{:08X}", BOOT_ROM_BASE);
    }
    
    return {};
}

Result<SharedPtr<MemoryRegion>> MemoryController::find_memory_region(Address address) const {
    for (const auto& [name, region] : memory_regions_) {
        if (region->contains_address(address)) {
            return region;
        }
    }
    
    return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
        "Invalid memory address: 0x" + std::to_string(address)));
}

Result<void> MemoryController::initialize_flash_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing Flash Controller and XIP region: {} MB at 0x{:08X}", 
                       size / (1024 * 1024), FLASH_LAYOUT.start_address);
    
    // Use the actual requested size, but at least 16MB for ESP32-P4
    size_t flash_size = std::max(size, static_cast<size_t>(16 * 1024 * 1024));
    
    // Create flash controller with configuration
    storage::FlashController::Config flash_config;
    flash_config.enable_persistence = true;
    flash_config.enable_statistics = true;
    flash_config.simulate_timing = true;
    
    flash_controller_ = std::make_unique<storage::FlashController>(flash_config);
    
    // Initialize the flash controller
    auto init_result = flash_controller_->initialize();
    if (!init_result.has_value()) {
        COMPONENT_LOG_ERROR("Failed to initialize flash controller: {}", 
                           static_cast<int>(init_result.error()));
        return unexpected(init_result.error());
    }
    
    // Create flash memory region that wraps the flash controller
    flash_memory_region_ = std::make_shared<storage::FlashMemoryRegion>(flash_controller_.get());
    
    // Register the flash region in the memory map (cast to MemoryInterface)
    auto flash_memory_interface = std::static_pointer_cast<MemoryInterface>(flash_memory_region_);
    
    // For now, create a wrapper MemoryRegion for compatibility
    auto flash_region_wrapper = std::make_shared<MemoryRegion>(
        "Flash_XIP", FLASH_LAYOUT.start_address, flash_size, MemoryType::Flash,
        FLASH_LAYOUT.writable, FLASH_LAYOUT.executable, FLASH_LAYOUT.cacheable
    );
    
    RETURN_IF_ERROR(flash_region_wrapper->initialize());
    memory_regions_["Flash"] = flash_region_wrapper;
    memory_regions_["flash_xip"] = flash_region_wrapper;
    memory_regions_["flash"] = flash_region_wrapper;
    
    COMPONENT_LOG_INFO("Flash Controller and XIP region initialized: {} MB at 0x{:08X}-0x{:08X}",
                      flash_size / (1024 * 1024),
                      FLASH_LAYOUT.start_address,
                      FLASH_LAYOUT.start_address + flash_size - 1);
    
    // Log partition table info if available
    auto* partition_table = flash_controller_->get_partition_table();
    if (partition_table && partition_table->get_partition_count() > 0) {
        COMPONENT_LOG_INFO("Flash partition table loaded with {} partitions", 
                          partition_table->get_partition_count());
    }
    
    return {};
}

Result<void> MemoryController::initialize_psram_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing ESP32-P4 PSRAM region: {} MB at 0x{:08X}", 
                       size / (1024 * 1024), PSRAM_LAYOUT.start_address);
    
    // Use the actual requested size, but at least 8MB for basic operation
    size_t psram_size = std::max(size, static_cast<size_t>(8 * 1024 * 1024));
    
    auto psram_region = std::make_shared<MemoryRegion>(
        "PSRAM", PSRAM_LAYOUT.start_address, psram_size, MemoryType::PSRAM,
        PSRAM_LAYOUT.writable, PSRAM_LAYOUT.executable, PSRAM_LAYOUT.cacheable
    );
    
    RETURN_IF_ERROR(psram_region->initialize());
    memory_regions_["PSRAM"] = psram_region;
    
    COMPONENT_LOG_INFO("PSRAM region initialized: {} MB at 0x{:08X}-0x{:08X}",
                      psram_size / (1024 * 1024),
                      PSRAM_LAYOUT.start_address,
                      PSRAM_LAYOUT.start_address + psram_size - 1);
    
    return {};
}

Result<void> MemoryController::initialize_sram_region(size_t size) {
    COMPONENT_LOG_DEBUG("Initializing ESP32-P4 L2 SRAM region: {} KB at 0x{:08X}", 
                       size / 1024, SRAM_LAYOUT.start_address);
    
    // Use actual ESP32-P4 SRAM layout (768KB L2 memory)
    size_t sram_size = std::max(size, static_cast<size_t>(768 * 1024));
    
    auto sram_region = std::make_shared<MemoryRegion>(
        "L2_SRAM", SRAM_LAYOUT.start_address, sram_size, MemoryType::SRAM,
        SRAM_LAYOUT.writable, SRAM_LAYOUT.executable, SRAM_LAYOUT.cacheable
    );
    
    RETURN_IF_ERROR(sram_region->initialize());
    memory_regions_["SRAM"] = sram_region;
    
    COMPONENT_LOG_INFO("L2 SRAM region initialized: {} KB at 0x{:08X}-0x{:08X}",
                      sram_size / 1024,
                      SRAM_LAYOUT.start_address,
                      SRAM_LAYOUT.start_address + sram_size - 1);
    
    return {};
}

Result<void> MemoryController::initialize_mmio_region() {
    COMPONENT_LOG_DEBUG("Initializing MMIO region");
    
    auto mmio_region = std::make_shared<MemoryRegion>(
        "MMIO", MMIO_LAYOUT.start_address, MMIO_LAYOUT.size, MemoryType::MMIO,
        MMIO_LAYOUT.writable, MMIO_LAYOUT.executable, MMIO_LAYOUT.cacheable
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
    COMPONENT_LOG_DEBUG("Setting up ESP32-P4 memory mappings");
    
    // Set up standard ESP32-P4 memory mappings
    // This is where we would configure virtual-to-physical address translations
    // For now, we use direct mapping (virtual == physical)
    
    // Log all mapped regions for debugging
    for (const auto& [name, region] : memory_regions_) {
        COMPONENT_LOG_DEBUG("Mapped region '{}': 0x{:08X}-0x{:08X} ({}KB) - R{} W{} X{}",
                           name,
                           region->get_start_address(),
                           region->get_end_address(),
                           region->get_size() / 1024,
                           "R",  // Always readable
                           region->is_writable() ? "W" : "-",
                           region->is_executable() ? "X" : "-");
    }
    
    COMPONENT_LOG_INFO("Memory layout setup completed with {} regions", memory_regions_.size());
    
    return {};
}

// Validate that all essential ESP32-P4 memory regions are properly mapped
Result<void> MemoryController::validate_memory_layout() {
    COMPONENT_LOG_DEBUG("Validating ESP32-P4 memory layout");
    
    // Check Boot ROM is mapped at correct address
    auto boot_rom_valid = is_valid_address(BOOT_ROM_BASE);
    if (!boot_rom_valid.has_value() || !boot_rom_valid.value()) {
        return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Boot ROM not mapped at 0x40000000"));
    }
    
    // Check Boot ROM reset vector is accessible
    auto reset_vector_valid = is_valid_address(BOOT_ROM_RESET_VECTOR);
    if (!reset_vector_valid.has_value() || !reset_vector_valid.value()) {
        return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Boot ROM reset vector not accessible at 0x40000080"));
    }
    
    // Check Flash XIP region is mapped
    auto flash_valid = is_valid_address(FLASH_LAYOUT.start_address);
    if (!flash_valid.has_value() || !flash_valid.value()) {
        return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Flash XIP region not mapped at 0x42000000"));
    }
    
    // Check L2 SRAM is mapped
    auto sram_valid = is_valid_address(SRAM_LAYOUT.start_address);
    if (!sram_valid.has_value() || !sram_valid.value()) {
        return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "L2 SRAM not mapped at 0x4FF00000"));
    }
    
    // Check PSRAM is mapped
    auto psram_valid = is_valid_address(PSRAM_LAYOUT.start_address);
    if (!psram_valid.has_value() || !psram_valid.value()) {
        return unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "PSRAM not mapped at 0x48000000"));
    }
    
    COMPONENT_LOG_INFO("ESP32-P4 memory layout validation successful");
    return {};
}

Result<std::vector<u8>> MemoryController::try_cache_read(Address address, size_t count) {
    Address cache_line_addr = address & ~(cache_line_size_ - 1);
    
    auto it = cache_lines_.find(cache_line_addr);
    if (it == cache_lines_.end()) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Cache miss"));
    }
    
    const auto& cache_line = it->second;
    size_t offset = address - cache_line_addr;
    
    if (offset + count > cache_line.data.size()) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Cache read overflow"));
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

// MemoryInterface implementation (adapter methods)
EmulatorError MemoryController::read8(Address address, uint8_t& value) {
    auto result = read_u8(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::InvalidAddress;
}

EmulatorError MemoryController::read16(Address address, uint16_t& value) {
    auto result = read_u16(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::InvalidAddress;
}

EmulatorError MemoryController::read32(Address address, uint32_t& value) {
    auto result = read_u32(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::InvalidAddress;
}

EmulatorError MemoryController::write8(Address address, uint8_t value) {
    auto result = write_u8(address, value);
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

EmulatorError MemoryController::write16(Address address, uint16_t value) {
    auto result = write_u16(address, value);
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

EmulatorError MemoryController::write32(Address address, uint32_t value) {
    auto result = write_u32(address, value);
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

bool MemoryController::isValidAddress(Address address) const {
    auto result = is_valid_address(address);
    return result.has_value() && result.value();
}

bool MemoryController::isWritableAddress(Address address) const {
    // Check if address is in a writable region
    auto region = find_memory_region(address);
    return region.has_value() && region.value()->is_writable();
}

bool MemoryController::isExecutableAddress(Address address) const {
    // Check if address is in an executable region
    auto region = find_memory_region(address);
    return region.has_value() && region.value()->is_executable();
}

}  // namespace m5tab5::emulator