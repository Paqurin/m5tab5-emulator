#include "emulator/memory/mpu.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("MemoryProtectionUnit");

MemoryProtectionUnit::MemoryProtectionUnit() 
    : initialized_(false),
      enabled_(false),
      memory_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("MemoryProtectionUnit created");
}

MemoryProtectionUnit::~MemoryProtectionUnit() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("MemoryProtectionUnit destroyed");
}

Result<void> MemoryProtectionUnit::initialize(const Configuration& config, 
                                              MemoryController& memory_controller) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Memory Protection Unit already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing Memory Protection Unit");
    
    memory_controller_ = &memory_controller;
    
    // Initialize default protection regions
    RETURN_IF_ERROR(setup_default_regions());
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Memory Protection Unit initialized with {} regions", regions_.size());
    
    return {};
}

Result<void> MemoryProtectionUnit::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down Memory Protection Unit");
    
    regions_.clear();
    memory_controller_ = nullptr;
    enabled_ = false;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Memory Protection Unit shutdown completed");
    return {};
}

Result<void> MemoryProtectionUnit::enable() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    enabled_ = true;
    COMPONENT_LOG_INFO("Memory Protection Unit enabled");
    return {};
}

Result<void> MemoryProtectionUnit::disable() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    enabled_ = false;
    COMPONENT_LOG_INFO("Memory Protection Unit disabled");
    return {};
}

Result<u8> MemoryProtectionUnit::add_region(const MpuRegion& region) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    if (regions_.size() >= MAX_MPU_REGIONS) {
        return std::unexpected(MAKE_ERROR(SYSTEM_RESOURCE_EXHAUSTED,
            "Maximum number of MPU regions reached"));
    }
    
    // Validate region
    RETURN_IF_ERROR(validate_region(region));
    
    // Check for overlaps with existing regions
    for (const auto& existing_region : regions_) {
        if (regions_overlap(region, existing_region)) {
            return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
                "Region overlaps with existing region"));
        }
    }
    
    // Add region
    regions_.push_back(region);
    u8 region_id = static_cast<u8>(regions_.size() - 1);
    
    COMPONENT_LOG_INFO("Added MPU region {}: 0x{:08X}-0x{:08X} ({})",
                      region_id, region.base_address, 
                      region.base_address + region.size - 1,
                      get_permissions_string(region.permissions));
    
    return region_id;
}

Result<void> MemoryProtectionUnit::remove_region(u8 region_id) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    if (region_id >= regions_.size()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid region ID: " + std::to_string(region_id)));
    }
    
    regions_.erase(regions_.begin() + region_id);
    
    COMPONENT_LOG_INFO("Removed MPU region {}", region_id);
    return {};
}

Result<void> MemoryProtectionUnit::update_region(u8 region_id, const MpuRegion& region) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    if (region_id >= regions_.size()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid region ID: " + std::to_string(region_id)));
    }
    
    // Validate new region
    RETURN_IF_ERROR(validate_region(region));
    
    // Check for overlaps with other regions (excluding this one)
    for (size_t i = 0; i < regions_.size(); ++i) {
        if (i != region_id && regions_overlap(region, regions_[i])) {
            return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
                "Updated region overlaps with existing region"));
        }
    }
    
    regions_[region_id] = region;
    
    COMPONENT_LOG_INFO("Updated MPU region {}: 0x{:08X}-0x{:08X} ({})",
                      region_id, region.base_address, 
                      region.base_address + region.size - 1,
                      get_permissions_string(region.permissions));
    
    return {};
}

Result<void> MemoryProtectionUnit::check_access(Address address, size_t size, 
                                                 MpuPermissions required_permissions,
                                                 CoreId core_id) {
    if (!initialized_ || !enabled_) {
        return {}; // MPU disabled - allow all access
    }
    
    // Check if the entire access range is within allowed regions
    Address end_address = address + size - 1;
    
    for (Address current_addr = address; current_addr <= end_address; ++current_addr) {
        auto region_result = find_region_for_address(current_addr);
        if (!region_result) {
            statistics_.access_violations++;
            return std::unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
                "Address 0x" + std::to_string(current_addr) + " not covered by any MPU region"));
        }
        
        const MpuRegion& region = region_result.value();
        
        // Check permissions
        if (!has_permission(region.permissions, required_permissions)) {
            statistics_.access_violations++;
            return std::unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
                "Insufficient permissions for address 0x" + std::to_string(current_addr) + 
                " (required: " + get_permissions_string(required_permissions) +
                ", available: " + get_permissions_string(region.permissions) + ")"));
        }
        
        // Check core-specific permissions if applicable
        if (region.core_specific && region.allowed_core != core_id) {
            statistics_.access_violations++;
            return std::unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
                "Core " + std::to_string(static_cast<int>(core_id)) + 
                " not allowed to access address 0x" + std::to_string(current_addr)));
        }
    }
    
    statistics_.access_checks++;
    return {};
}

Result<const MpuRegion&> MemoryProtectionUnit::get_region(u8 region_id) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory Protection Unit not initialized"));
    }
    
    if (region_id >= regions_.size()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid region ID: " + std::to_string(region_id)));
    }
    
    return regions_[region_id];
}

std::vector<MpuRegion> MemoryProtectionUnit::get_all_regions() const {
    return regions_;
}

bool MemoryProtectionUnit::is_enabled() const {
    return enabled_;
}

const MpuStatistics& MemoryProtectionUnit::get_statistics() const {
    return statistics_;
}

void MemoryProtectionUnit::clear_statistics() {
    statistics_ = {};
}

Result<void> MemoryProtectionUnit::setup_default_regions() {
    COMPONENT_LOG_DEBUG("Setting up default MPU regions");
    
    // Flash region - read and execute only
    MpuRegion flash_region{};
    flash_region.base_address = FLASH_REGION.start_address;
    flash_region.size = FLASH_REGION.size;
    flash_region.permissions = MpuPermissions::READ | MpuPermissions::EXECUTE;
    flash_region.cacheable = true;
    flash_region.bufferable = false;
    flash_region.core_specific = false;
    flash_region.allowed_core = CoreId::INVALID;
    
    auto flash_result = add_region(flash_region);
    if (!flash_result) {
        return std::unexpected(flash_result.error());
    }
    
    // PSRAM region - read and write
    MpuRegion psram_region{};
    psram_region.base_address = PSRAM_REGION.start_address;
    psram_region.size = PSRAM_REGION.size;
    psram_region.permissions = MpuPermissions::READ | MpuPermissions::WRITE;
    psram_region.cacheable = true;
    psram_region.bufferable = true;
    psram_region.core_specific = false;
    psram_region.allowed_core = CoreId::INVALID;
    
    auto psram_result = add_region(psram_region);
    if (!psram_result) {
        return std::unexpected(psram_result.error());
    }
    
    // SRAM region - read, write, and execute
    MpuRegion sram_region{};
    sram_region.base_address = SRAM_REGION.start_address;
    sram_region.size = SRAM_REGION.size;
    sram_region.permissions = MpuPermissions::READ | MpuPermissions::WRITE | MpuPermissions::EXECUTE;
    sram_region.cacheable = true;
    sram_region.bufferable = true;
    sram_region.core_specific = false;
    sram_region.allowed_core = CoreId::INVALID;
    
    auto sram_result = add_region(sram_region);
    if (!sram_result) {
        return std::unexpected(sram_result.error());
    }
    
    // MMIO region - read and write, not cacheable
    MpuRegion mmio_region{};
    mmio_region.base_address = MMIO_REGION.start_address;
    mmio_region.size = MMIO_REGION.size;
    mmio_region.permissions = MpuPermissions::READ | MpuPermissions::WRITE;
    mmio_region.cacheable = false;
    mmio_region.bufferable = false;
    mmio_region.core_specific = false;
    mmio_region.allowed_core = CoreId::INVALID;
    
    auto mmio_result = add_region(mmio_region);
    if (!mmio_result) {
        return std::unexpected(mmio_result.error());
    }
    
    COMPONENT_LOG_DEBUG("Default MPU regions set up successfully");
    return {};
}

Result<void> MemoryProtectionUnit::validate_region(const MpuRegion& region) const {
    // Check address alignment
    if (region.base_address % MPU_REGION_MIN_SIZE != 0) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Region base address not aligned to minimum size"));
    }
    
    // Check size
    if (region.size == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Region size cannot be zero"));
    }
    
    if (region.size % MPU_REGION_MIN_SIZE != 0) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Region size not aligned to minimum size"));
    }
    
    if (region.size > MPU_REGION_MAX_SIZE) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Region size exceeds maximum"));
    }
    
    // Check for address overflow
    if (region.base_address + region.size < region.base_address) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Region address range overflow"));
    }
    
    // Validate permissions
    if (region.permissions == MpuPermissions::NONE) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Region must have at least one permission"));
    }
    
    return {};
}

bool MemoryProtectionUnit::regions_overlap(const MpuRegion& region1, const MpuRegion& region2) const {
    Address end1 = region1.base_address + region1.size - 1;
    Address end2 = region2.base_address + region2.size - 1;
    
    return !(region1.base_address > end2 || region2.base_address > end1);
}

Result<const MpuRegion&> MemoryProtectionUnit::find_region_for_address(Address address) const {
    for (const auto& region : regions_) {
        if (address >= region.base_address && 
            address < region.base_address + region.size) {
            return region;
        }
    }
    
    return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
        "No region found for address: 0x" + std::to_string(address)));
}

bool MemoryProtectionUnit::has_permission(MpuPermissions available, MpuPermissions required) const {
    return (static_cast<u32>(available) & static_cast<u32>(required)) == static_cast<u32>(required);
}

std::string MemoryProtectionUnit::get_permissions_string(MpuPermissions permissions) const {
    std::string result;
    
    if (static_cast<u32>(permissions) & static_cast<u32>(MpuPermissions::READ)) {
        result += "R";
    }
    if (static_cast<u32>(permissions) & static_cast<u32>(MpuPermissions::WRITE)) {
        result += "W";
    }
    if (static_cast<u32>(permissions) & static_cast<u32>(MpuPermissions::EXECUTE)) {
        result += "X";
    }
    
    if (result.empty()) {
        result = "NONE";
    }
    
    return result;
}

void MemoryProtectionUnit::dump_regions() const {
    COMPONENT_LOG_INFO("=== MPU Regions ===");
    COMPONENT_LOG_INFO("MPU enabled: {}", enabled_);
    COMPONENT_LOG_INFO("Total regions: {}", regions_.size());
    
    for (size_t i = 0; i < regions_.size(); ++i) {
        const auto& region = regions_[i];
        COMPONENT_LOG_INFO("Region {}: 0x{:08X}-0x{:08X} size={} perm={} cache={} buffer={}",
                          i, region.base_address, 
                          region.base_address + region.size - 1,
                          region.size,
                          get_permissions_string(region.permissions),
                          region.cacheable,
                          region.bufferable);
        
        if (region.core_specific) {
            COMPONENT_LOG_INFO("  Core-specific: Core {}", static_cast<int>(region.allowed_core));
        }
    }
    
    COMPONENT_LOG_INFO("Statistics:");
    COMPONENT_LOG_INFO("  Access checks: {}", statistics_.access_checks);
    COMPONENT_LOG_INFO("  Access violations: {}", statistics_.access_violations);
}

}  // namespace m5tab5::emulator