#include "emulator/memory/memory_region.hpp"
#include "emulator/utils/logging.hpp"
#include <cstring>
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("MemoryRegion");

MemoryRegion::MemoryRegion(const std::string& name,
                           Address start_address, 
                           size_t size,
                           MemoryType type,
                           bool writable,
                           bool executable,
                           bool cacheable)
    : name_(name),
      start_address_(start_address),
      size_(size),
      type_(type),
      writable_(writable),
      executable_(executable),
      cacheable_(cacheable),
      initialized_(false) {
    
    COMPONENT_LOG_DEBUG("MemoryRegion '{}' created: addr=0x{:08X}, size={} bytes, type={}", 
                       name_, start_address_, size_, static_cast<int>(type_));
}

MemoryRegion::~MemoryRegion() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("MemoryRegion '{}' destroyed", name_);
}

Result<void> MemoryRegion::initialize() {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Memory region '" + name_ + "' already initialized"));
    }
    
    if (size_ == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Memory region size cannot be zero"));
    }
    
    try {
        // Allocate memory for the region
        data_ = std::make_unique<u8[]>(size_);
        
        // Initialize memory based on type
        switch (type_) {
            case MemoryType::FLASH:
                // Flash typically starts with 0xFF
                std::memset(data_.get(), 0xFF, size_);
                break;
                
            case MemoryType::PSRAM:
            case MemoryType::SRAM:
                // RAM starts uninitialized (zeros for simulation)
                std::memset(data_.get(), 0x00, size_);
                break;
                
            case MemoryType::MMIO:
                // MMIO region doesn't need backing memory for all addresses
                // We'll handle MMIO through callbacks
                std::memset(data_.get(), 0x00, size_);
                break;
                
            default:
                std::memset(data_.get(), 0x00, size_);
                break;
        }
        
        initialized_ = true;
        
        COMPONENT_LOG_INFO("MemoryRegion '{}' initialized successfully", name_);
        return {};
        
    } catch (const std::bad_alloc& e) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALLOCATION_FAILED,
            "Failed to allocate memory for region '" + name_ + "': " + e.what()));
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during memory region initialization: " + std::string(e.what())));
    }
}

void MemoryRegion::shutdown() {
    if (!initialized_) {
        return;
    }
    
    // Clear MMIO callbacks
    mmio_read_callbacks_.clear();
    mmio_write_callbacks_.clear();
    
    // Release memory
    data_.reset();
    
    initialized_ = false;
    COMPONENT_LOG_DEBUG("MemoryRegion '{}' shutdown completed", name_);
}

bool MemoryRegion::contains_address(Address address) const {
    return address >= start_address_ && address < (start_address_ + size_);
}

Result<void> MemoryRegion::read_bytes(Address address, u8* buffer, size_t count) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory region '" + name_ + "' not initialized"));
    }
    
    if (!buffer || count == 0) {
        return {};
    }
    
    if (!contains_address(address)) {
        return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Address 0x" + std::to_string(address) + " is outside region '" + name_ + "'"));
    }
    
    if (address + count > start_address_ + size_) {
        return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Read operation would exceed region '" + name_ + "' bounds"));
    }
    
    size_t offset = address - start_address_;
    
    // Handle MMIO reads through callbacks
    if (type_ == MemoryType::MMIO) {
        return handle_mmio_read(address, buffer, count);
    }
    
    // Standard memory read
    std::memcpy(buffer, data_.get() + offset, count);
    
    COMPONENT_LOG_TRACE("Read {} bytes from region '{}' at offset 0x{:08X}", 
                       count, name_, offset);
    
    return {};
}

Result<void> MemoryRegion::write_bytes(Address address, const u8* buffer, size_t count) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory region '" + name_ + "' not initialized"));
    }
    
    if (!writable_) {
        return std::unexpected(MAKE_ERROR(MEMORY_ACCESS_VIOLATION,
            "Attempt to write to read-only region '" + name_ + "'"));
    }
    
    if (!buffer || count == 0) {
        return {};
    }
    
    if (!contains_address(address)) {
        return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Address 0x" + std::to_string(address) + " is outside region '" + name_ + "'"));
    }
    
    if (address + count > start_address_ + size_) {
        return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Write operation would exceed region '" + name_ + "' bounds"));
    }
    
    size_t offset = address - start_address_;
    
    // Handle MMIO writes through callbacks
    if (type_ == MemoryType::MMIO) {
        return handle_mmio_write(address, buffer, count);
    }
    
    // Standard memory write
    std::memcpy(data_.get() + offset, buffer, count);
    
    COMPONENT_LOG_TRACE("Wrote {} bytes to region '{}' at offset 0x{:08X}", 
                       count, name_, offset);
    
    return {};
}

Result<void> MemoryRegion::load_from_file(const std::string& file_path, Address offset) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory region '" + name_ + "' not initialized"));
    }
    
    if (offset >= size_) {
        return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Load offset exceeds region size"));
    }
    
    try {
        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
            return std::unexpected(MAKE_ERROR(IO_READ_FAILED,
                "Failed to open file: " + file_path));
        }
        
        // Get file size
        file.seekg(0, std::ios::end);
        size_t file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        
        if (offset + file_size > size_) {
            return std::unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
                "File size exceeds available region space"));
        }
        
        // Read file into memory region
        file.read(reinterpret_cast<char*>(data_.get() + offset), file_size);
        
        if (file.fail()) {
            return std::unexpected(MAKE_ERROR(IO_READ_FAILED,
                "Failed to read from file: " + file_path));
        }
        
        COMPONENT_LOG_INFO("Loaded {} bytes from '{}' into region '{}' at offset 0x{:08X}",
                          file_size, file_path, name_, offset);
        
        return {};
        
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during file load: " + std::string(e.what())));
    }
}

Result<void> MemoryRegion::save_to_file(const std::string& file_path) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory region '" + name_ + "' not initialized"));
    }
    
    try {
        std::ofstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
            return std::unexpected(MAKE_ERROR(IO_WRITE_FAILED,
                "Failed to create file: " + file_path));
        }
        
        // Write region data to file
        file.write(reinterpret_cast<const char*>(data_.get()), size_);
        
        if (file.fail()) {
            return std::unexpected(MAKE_ERROR(IO_WRITE_FAILED,
                "Failed to write to file: " + file_path));
        }
        
        COMPONENT_LOG_INFO("Saved {} bytes from region '{}' to '{}'",
                          size_, name_, file_path);
        
        return {};
        
    } catch (const std::exception& e) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during file save: " + std::string(e.what())));
    }
}

void MemoryRegion::register_mmio_read_callback(Address address, MMIOReadCallback callback) {
    if (type_ == MemoryType::MMIO && contains_address(address)) {
        mmio_read_callbacks_[address] = callback;
        COMPONENT_LOG_DEBUG("Registered MMIO read callback for address 0x{:08X}", address);
    }
}

void MemoryRegion::register_mmio_write_callback(Address address, MMIOWriteCallback callback) {
    if (type_ == MemoryType::MMIO && contains_address(address)) {
        mmio_write_callbacks_[address] = callback;
        COMPONENT_LOG_DEBUG("Registered MMIO write callback for address 0x{:08X}", address);
    }
}

u8* MemoryRegion::get_data() const {
    return data_.get();
}

Result<void> MemoryRegion::handle_mmio_read(Address address, u8* buffer, size_t count) const {
    // For MMIO, we handle byte-by-byte access through callbacks
    for (size_t i = 0; i < count; ++i) {
        Address current_addr = address + i;
        
        auto it = mmio_read_callbacks_.find(current_addr);
        if (it != mmio_read_callbacks_.end()) {
            // Use registered callback
            buffer[i] = it->second(current_addr);
        } else {
            // Default behavior: read from backing memory
            size_t offset = current_addr - start_address_;
            if (offset < size_) {
                buffer[i] = data_[offset];
            } else {
                buffer[i] = 0xFF;  // Default for unmapped MMIO
            }
        }
    }
    
    return {};
}

Result<void> MemoryRegion::handle_mmio_write(Address address, const u8* buffer, size_t count) {
    // For MMIO, we handle byte-by-byte access through callbacks
    for (size_t i = 0; i < count; ++i) {
        Address current_addr = address + i;
        
        auto it = mmio_write_callbacks_.find(current_addr);
        if (it != mmio_write_callbacks_.end()) {
            // Use registered callback
            it->second(current_addr, buffer[i]);
        } else {
            // Default behavior: write to backing memory
            size_t offset = current_addr - start_address_;
            if (offset < size_) {
                data_[offset] = buffer[i];
            }
            // Ignore writes to unmapped MMIO addresses
        }
    }
    
    return {};
}

}  // namespace m5tab5::emulator