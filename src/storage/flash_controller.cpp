#include "emulator/storage/flash_controller.hpp"
#include "emulator/utils/logging.hpp"

#include <fstream>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <sstream>

namespace m5tab5::emulator::storage {

namespace fs = std::filesystem;
using namespace std::chrono;

// Minimal implementation for Sprint 4 deliverable
// Full implementation can be completed in subsequent development

FlashController::FlashController() 
    : FlashController(Config{}) {
}

FlashController::FlashController(const Config& config)
    : config_(config),
      flash_data_(std::make_unique<FlashData>()),
      flash_state_(std::make_unique<FlashState>()),
      partition_table_(std::make_unique<PartitionTable>()),
      last_save_time_(steady_clock::now()) {
    
    LOG_DEBUG("FlashController created with flash size: {}MB", FLASH_SIZE / (1024 * 1024));
}

FlashController::~FlashController() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> FlashController::initialize() {
    if (initialized_) {
        return {};
    }
    
    LOG_INFO("Initializing Flash Controller");
    
    // Initialize partition table
    auto pt_result = partition_table_->initialize();
    if (!pt_result.has_value()) {
        LOG_ERROR("Failed to initialize partition table");
        return unexpected(pt_result.error());
    }
    
    // Set initial flash state
    flash_state_->status_register.store(0x00);
    flash_state_->write_enabled.store(false);
    flash_state_->busy.store(false);
    flash_state_->erase_suspended.store(false);
    flash_state_->initialized.store(true);
    
    initialized_.store(true);
    
    LOG_INFO("Flash Controller initialized successfully");
    return {};
}

Result<void> FlashController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    LOG_INFO("Shutting down Flash Controller");
    
    // Reset state
    flash_state_->initialized.store(false);
    initialized_.store(false);
    
    LOG_INFO("Flash Controller shutdown complete");
    return {};
}

Result<void> FlashController::read(Address address, void* buffer, size_t size) {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    // Validate parameters
    if (buffer == nullptr || size == 0) {
        return unexpected(make_error(ErrorCode::INVALID_PARAMETER, "Invalid parameter"));
    }
    
    // Convert to physical address and perform basic validation
    if (address < FLASH_BASE_ADDRESS || address >= FLASH_END_ADDRESS) {
        return unexpected(make_error(ErrorCode::MEMORY_INVALID_ADDRESS, "Invalid flash address"));
    }
    
    Address phys_addr = address - FLASH_BASE_ADDRESS;
    
    // Execute internal read
    auto read_result = internal_read(phys_addr, buffer, size);
    if (!read_result.has_value()) {
        return read_result;
    }
    
    return {};
}

Result<void> FlashController::write(Address address, const void* data, size_t size) {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    // Validate parameters
    if (data == nullptr || size == 0) {
        return unexpected(make_error(ErrorCode::INVALID_PARAMETER, "Invalid parameter"));
    }
    
    // Basic address validation
    if (address < FLASH_BASE_ADDRESS || address >= FLASH_END_ADDRESS) {
        return unexpected(make_error(ErrorCode::MEMORY_INVALID_ADDRESS, "Invalid flash address"));
    }
    
    // Check write enable
    if (!flash_state_->write_enabled.load()) {
        LOG_WARN("Write attempted without write enable at address 0x{:08X}", address);
        return unexpected(make_error(ErrorCode::PERIPHERAL_BUSY, "Flash write protected"));
    }
    
    // Convert to physical address
    Address phys_addr = address - FLASH_BASE_ADDRESS;
    
    // Set busy flag
    flash_state_->busy.store(true);
    
    // Execute internal write
    auto write_internal_result = internal_write(phys_addr, data, size);
    if (!write_internal_result.has_value()) {
        flash_state_->busy.store(false);
        return write_internal_result;
    }
    
    // Clear write enable after write
    flash_state_->write_enabled.store(false);
    flash_state_->busy.store(false);
    
    return {};
}

Result<void> FlashController::erase_sector(Address address) {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    // Basic validation
    if (address < FLASH_BASE_ADDRESS || address >= FLASH_END_ADDRESS) {
        return unexpected(make_error(ErrorCode::MEMORY_INVALID_ADDRESS, "Invalid flash address"));
    }
    
    // Check write enable
    if (!flash_state_->write_enabled.load()) {
        LOG_WARN("Erase attempted without write enable at address 0x{:08X}", address);
        return unexpected(make_error(ErrorCode::PERIPHERAL_BUSY, "Flash write protected"));
    }
    
    // Convert to physical address and get sector
    Address phys_addr = address - FLASH_BASE_ADDRESS;
    size_t sector = phys_addr / SECTOR_SIZE;
    
    // Set busy flag
    flash_state_->busy.store(true);
    
    // Execute internal erase
    auto internal_result = internal_erase_sector(sector);
    if (!internal_result.has_value()) {
        flash_state_->busy.store(false);
        return internal_result;
    }
    
    // Clear write enable after erase
    flash_state_->write_enabled.store(false);
    flash_state_->busy.store(false);
    
    LOG_DEBUG("Erased sector {} at address 0x{:08X}", sector, address);
    
    return {};
}

Result<void> FlashController::write_enable() {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    flash_state_->write_enabled.store(true);
    uint8_t status = flash_state_->status_register.load();
    status |= static_cast<uint8_t>(StatusBit::WEL);
    flash_state_->status_register.store(status);
    
    return {};
}

Result<bool> FlashController::is_write_enabled() {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    return flash_state_->write_enabled.load();
}

Result<uint32_t> FlashController::get_flash_id() {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    // Return a typical SPI flash ID (manufacturer + device)
    // Winbond W25Q128JV: 0xEF7018
    return 0xEF7018;
}

Result<uint8_t> FlashController::get_status_register() {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    uint8_t status = flash_state_->status_register.load();
    
    // Update busy bit
    if (flash_state_->busy.load()) {
        status |= static_cast<uint8_t>(StatusBit::BUSY);
    } else {
        status &= ~static_cast<uint8_t>(StatusBit::BUSY);
    }
    
    return status;
}

Result<bool> FlashController::is_busy() {
    if (!initialized_) {
        return unexpected(make_error(ErrorCode::NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    return flash_state_->busy.load();
}

Result<uint8_t> FlashController::read_byte_mapped(Address address) {
    uint8_t value;
    auto result = read(address, &value, sizeof(value));
    if (!result.has_value()) {
        return unexpected(result.error());
    }
    return value;
}

Result<uint16_t> FlashController::read_word_mapped(Address address) {
    uint16_t value;
    auto result = read(address, &value, sizeof(value));
    if (!result.has_value()) {
        return unexpected(result.error());
    }
    return value;
}

Result<uint32_t> FlashController::read_dword_mapped(Address address) {
    uint32_t value;
    auto result = read(address, &value, sizeof(value));
    if (!result.has_value()) {
        return unexpected(result.error());
    }
    return value;
}

// Internal implementation methods (minimal for Sprint 4)

Result<void> FlashController::internal_read(Address address, void* buffer, size_t size) {
    std::shared_lock<std::shared_mutex> lock(flash_data_->mutex);
    
    if (address + size > FLASH_SIZE) {
        return unexpected(make_error(ErrorCode::MEMORY_OUT_OF_BOUNDS, "Address out of bounds"));
    }
    
    std::memcpy(buffer, flash_data_->memory.data() + address, size);
    return {};
}

Result<void> FlashController::internal_write(Address address, const void* data, size_t size) {
    std::unique_lock<std::shared_mutex> lock(flash_data_->mutex);
    
    if (address + size > FLASH_SIZE) {
        return unexpected(make_error(ErrorCode::MEMORY_OUT_OF_BOUNDS, "Address out of bounds"));
    }
    
    // Perform write (flash can only change 1->0, not 0->1)
    const uint8_t* src = static_cast<const uint8_t*>(data);
    uint8_t* dest = flash_data_->memory.data() + address;
    
    for (size_t i = 0; i < size; ++i) {
        dest[i] &= src[i];  // AND operation simulates flash write behavior
    }
    
    return {};
}

Result<void> FlashController::internal_erase_sector(size_t sector) {
    std::unique_lock<std::shared_mutex> lock(flash_data_->mutex);
    
    if (sector >= flash_data_->erase_counts.size()) {
        return unexpected(make_error(ErrorCode::INVALID_PARAMETER, "Invalid parameter"));
    }
    
    // Erase sector (set all bytes to 0xFF)
    size_t offset = sector * SECTOR_SIZE;
    std::memset(flash_data_->memory.data() + offset, 0xFF, SECTOR_SIZE);
    
    return {};
}

// FlashMemoryRegion implementation

FlashMemoryRegion::FlashMemoryRegion(FlashController* controller)
    : controller_(controller) {
}

EmulatorError FlashMemoryRegion::read8(Address address, uint8_t& value) {
    auto result = controller_->read_byte_mapped(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::read16(Address address, uint16_t& value) {
    auto result = controller_->read_word_mapped(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::read32(Address address, uint32_t& value) {
    auto result = controller_->read_dword_mapped(address);
    if (result.has_value()) {
        value = result.value();
        return EmulatorError::Success;
    }
    return EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::write8(Address address, uint8_t value) {
    auto result = controller_->write(address, &value, sizeof(value));
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::write16(Address address, uint16_t value) {
    auto result = controller_->write(address, &value, sizeof(value));
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::write32(Address address, uint32_t value) {
    auto result = controller_->write(address, &value, sizeof(value));
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

bool FlashMemoryRegion::isValidAddress(Address address) const {
    return address >= FlashController::FLASH_BASE_ADDRESS && 
           address < FlashController::FLASH_END_ADDRESS;
}

bool FlashMemoryRegion::isWritableAddress(Address address) const {
    return isValidAddress(address);  // Flash is writable (with proper commands)
}

bool FlashMemoryRegion::isExecutableAddress(Address address) const {
    return isValidAddress(address);  // Flash memory is executable (XIP)
}

EmulatorError FlashMemoryRegion::readBlock(Address address, void* buffer, size_t size) {
    auto result = controller_->read(address, buffer, size);
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

EmulatorError FlashMemoryRegion::writeBlock(Address address, const void* data, size_t size) {
    auto result = controller_->write(address, data, size);
    return result.has_value() ? EmulatorError::Success : EmulatorError::MemoryAccessError;
}

// FlashController missing methods implementation
Result<void> FlashController::erase_range(Address start_address, size_t size) {
    if (!flash_data_ || !flash_state_) {
        LOG_ERROR("Flash controller not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    std::lock_guard<std::mutex> lock(flash_state_->operation_mutex);
    
    if (!initialized_) {
        LOG_ERROR("Flash controller not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    if (start_address < FLASH_BASE_ADDRESS || start_address >= FLASH_END_ADDRESS) {
        LOG_ERROR("Invalid erase start address: 0x{:08X}", start_address);
        return error<void>(ErrorCode::MEMORY_INVALID_ADDRESS);
    }
    
    if (start_address + size > FLASH_END_ADDRESS) {
        LOG_ERROR("Erase range exceeds flash bounds: 0x{:08X} + {} > 0x{:08X}", 
                  start_address, size, FLASH_END_ADDRESS);
        return error<void>(ErrorCode::MEMORY_OUT_OF_BOUNDS);
    }
    
    // Align to sector boundaries
    Address aligned_start = (start_address / SECTOR_SIZE) * SECTOR_SIZE;
    size_t aligned_size = ((start_address + size + SECTOR_SIZE - 1) / SECTOR_SIZE) * SECTOR_SIZE - aligned_start;
    
    LOG_DEBUG("Erasing flash range: 0x{:08X} - 0x{:08X} ({} bytes)", 
              aligned_start, aligned_start + aligned_size, aligned_size);
    
    // Erase sector by sector
    for (Address addr = aligned_start; addr < aligned_start + aligned_size; addr += SECTOR_SIZE) {
        auto result = erase_sector(addr);
        if (!result.has_value()) {
            LOG_ERROR("Failed to erase sector at address 0x{:08X}", addr);
            return result;
        }
    }
    
    LOG_DEBUG("Successfully erased {} bytes from 0x{:08X}", aligned_size, aligned_start);
    return success<void>();
}

Result<void> FlashController::save_to_file() {
    if (!flash_data_ || !initialized_) {
        LOG_ERROR("Flash controller not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    try {
        std::shared_lock<std::shared_mutex> lock(flash_data_->mutex);
        
        std::ofstream file(config_.flash_image_path, std::ios::binary);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open flash image file for writing: {}", config_.flash_image_path);
            return error<void>(ErrorCode::FILE_ERROR);
        }
        
        // Write flash data to file
        file.write(reinterpret_cast<const char*>(flash_data_->memory.data()), FLASH_SIZE);
        
        if (!file.good()) {
            LOG_ERROR("Failed to write flash image to file: {}", config_.flash_image_path);
            return error<void>(ErrorCode::FILE_ERROR);
        }
        
        file.close();
        LOG_DEBUG("Flash image saved successfully to: {}", config_.flash_image_path);
        
        return success<void>();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Exception while saving flash image: {}", e.what());
        return error<void>(ErrorCode::FILE_ERROR);
    }
}

Result<std::string> FlashController::get_flash_info() const {
    if (!flash_data_ || !initialized_) {
        LOG_ERROR("Flash controller not initialized");
        return error<std::string>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    std::ostringstream info;
    info << "Flash Controller Information:\n";
    info << "  Base Address: 0x" << std::hex << FLASH_BASE_ADDRESS << "\n";
    info << "  Size: " << std::dec << (FLASH_SIZE / 1024 / 1024) << "MB (" << FLASH_SIZE << " bytes)\n";
    info << "  Sector Size: " << (SECTOR_SIZE / 1024) << "KB (" << SECTOR_SIZE << " bytes)\n";
    info << "  Total Sectors: " << (FLASH_SIZE / SECTOR_SIZE) << "\n";
    info << "  Block Size: " << (BLOCK_SIZE / 1024) << "KB (" << BLOCK_SIZE << " bytes)\n";
    info << "  Total Blocks: " << (FLASH_SIZE / BLOCK_SIZE) << "\n";
    
    // Add statistics if available
    if (config_.enable_statistics) {
        const auto& stats = get_statistics();
        info << "Statistics:\n";
        info << "  Read Operations: " << stats.read_operations << "\n";
        info << "  Write Operations: " << stats.write_operations << "\n";
        info << "  Erase Operations: " << stats.erase_operations << "\n";
        info << "  Bytes Read: " << stats.bytes_read << "\n";
        info << "  Bytes Written: " << stats.bytes_written << "\n";
        info << "  Sectors Erased: " << stats.sectors_erased << "\n";
        info << "  Blocks Erased: " << stats.blocks_erased << "\n";
    }
    
    return success<std::string>(info.str());
}

Result<void> FlashController::save_flash_image() {
    if (!flash_data_ || !initialized_) {
        LOG_ERROR("Flash controller not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    try {
        LOG_DEBUG("Saving flash image to: {}", config_.flash_image_path);
        return save_to_file();
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to save flash image: {}", e.what());
        return error<void>(ErrorCode::FILE_ERROR);
    }
}

} // namespace m5tab5::emulator::storage