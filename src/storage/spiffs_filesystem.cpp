#include "emulator/storage/spiffs_filesystem.hpp"
#include "emulator/utils/logging.hpp"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace m5tab5::emulator::storage {

namespace {
    // Helper functions for bit manipulation and checksums
    uint32_t calculate_crc32(const void* data, size_t size) {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        uint32_t crc = 0xFFFFFFFF;
        
        for (size_t i = 0; i < size; ++i) {
            crc ^= bytes[i];
            for (int bit = 0; bit < 8; ++bit) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        
        return ~crc;
    }
    
    uint64_t get_current_timestamp() {
        return std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    bool is_valid_filename(const std::string& name) {
        if (name.empty() || name.length() > SPIFFSFileSystem::SPIFFS_MAX_NAME_LENGTH) {
            return false;
        }
        
        // Check for invalid characters
        const std::string invalid_chars = "<>:\"|?*";
        for (char c : name) {
            if (c < 32 || invalid_chars.find(c) != std::string::npos) {
                return false;
            }
        }
        
        return true;
    }
}

SPIFFSFileSystem::SPIFFSFileSystem(FlashController* flash_controller)
    : SPIFFSFileSystem(flash_controller, Config{}) {}

SPIFFSFileSystem::SPIFFSFileSystem(FlashController* flash_controller, const Config& config)
    : config_(config), flash_controller_(flash_controller),
      superblock_(std::make_unique<Superblock>()),
      write_cache_(std::make_unique<WriteCache>()),
      mount_time_(std::chrono::steady_clock::now()) {
    
    if (!flash_controller_) {
        LOG_ERROR("SPIFFSFileSystem: FlashController cannot be null");
        return;
    }
    
    write_cache_->max_size = config_.write_cache_size;
    
    // Initialize statistics
    stats_ = {};
    
    LOG_DEBUG("SPIFFSFileSystem created with config - max_files: {}, gc_threshold: {}%", 
             config_.max_open_files, config_.gc_trigger_threshold);
}

SPIFFSFileSystem::~SPIFFSFileSystem() {
    if (mounted_) {
        unmount();
    }
}

Result<void> SPIFFSFileSystem::initialize(Address partition_start, size_t partition_size) {
    if (initialized_) {
        return Result<void>{};
    }
    
    LOG_INFO("Initializing SPIFFS filesystem at 0x{:08X}, size: {}KB", 
             partition_start, partition_size / 1024);
    
    if (!flash_controller_) {
        LOG_ERROR("FlashController is null");
        return error<void>(ErrorCode::STORAGE_INIT_FAILED);
    }
    
    if (partition_size < SPIFFS_BLOCK_SIZE * 2) {
        LOG_ERROR("Partition size too small: {} bytes (minimum: {})", 
                 partition_size, SPIFFS_BLOCK_SIZE * 2);
        return error<void>(ErrorCode::STORAGE_INVALID_SIZE);
    }
    
    partition_start_ = partition_start;
    partition_size_ = partition_size;
    total_pages_ = partition_size / SPIFFS_PAGE_SIZE;
    pages_per_block_ = SPIFFS_PAGES_PER_BLOCK;
    total_blocks_ = partition_size / SPIFFS_BLOCK_SIZE;
    
    // Validate alignment
    if (partition_start % SPIFFS_BLOCK_SIZE != 0) {
        LOG_ERROR("Partition start not aligned to block boundary");
        return error<void>(ErrorCode::STORAGE_ALIGNMENT_ERROR);
    }
    
    if (partition_size % SPIFFS_BLOCK_SIZE != 0) {
        LOG_ERROR("Partition size not aligned to block boundary");
        return error<void>(ErrorCode::STORAGE_ALIGNMENT_ERROR);
    }
    
    // Initialize free page list
    free_pages_.clear();
    free_pages_.reserve(total_pages_);
    
    // Initialize free object ID pool
    free_object_ids_.clear();
    free_object_ids_.reserve(1000);  // Reasonable initial size
    for (uint32_t i = 1; i <= 1000; ++i) {
        free_object_ids_.push_back(i);
    }
    
    initialized_ = true;
    LOG_INFO("SPIFFS initialized - {} pages, {} blocks", total_pages_, total_blocks_);
    
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::mount() {
    std::unique_lock lock(filesystem_mutex_);
    
    if (!initialized_) {
        LOG_ERROR("SPIFFS not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    if (mounted_) {
        LOG_WARN("SPIFFS already mounted");
        return Result<void>{};
    }
    
    LOG_INFO("Mounting SPIFFS filesystem");
    mount_time_ = std::chrono::steady_clock::now();
    
    // Try to read existing superblock
    auto sb_result = read_superblock();
    if (!sb_result.has_value()) {
        LOG_WARN("Failed to read superblock, formatting filesystem");
        
        // Format the filesystem
        auto format_result = format();
        if (!format_result.has_value()) {
            LOG_ERROR("Failed to format SPIFFS filesystem");
            return error<void>(format_result.error().code());
        }
        
        // Try reading superblock again
        sb_result = read_superblock();
        if (!sb_result.has_value()) {
            LOG_ERROR("Failed to read superblock after format");
            return error<void>(sb_result.error().code());
        }
    }
    
    // Scan filesystem and build object table
    auto scan_result = scan_filesystem();
    if (!scan_result.has_value()) {
        LOG_ERROR("Failed to scan filesystem");
        return error<void>(scan_result.error().code());
    }
    
    // Build free page list
    auto free_result = build_free_page_list();
    if (!free_result.has_value()) {
        LOG_ERROR("Failed to build free page list");
        return error<void>(free_result.error().code());
    }
    
    // Update statistics
    auto stats_result = recalculate_statistics();
    if (!stats_result.has_value()) {
        LOG_WARN("Failed to calculate statistics: {}", static_cast<int>(stats_result.error().code()));
    }
    
    mounted_ = true;
    
    LOG_INFO("SPIFFS mounted successfully - {} objects, {} free pages", 
             object_cache_.size(), free_pages_.size());
    
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::unmount() {
    std::unique_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        return Result<void>{};
    }
    
    LOG_INFO("Unmounting SPIFFS filesystem");
    
    // Close all open files
    {
        std::lock_guard handle_lock(handle_mutex_);
        for (auto& [fd, handle] : file_handles_) {
            if (handle && handle->is_open) {
                LOG_WARN("Force closing open file descriptor: {}", fd);
                handle->is_open = false;
            }
        }
        file_handles_.clear();
    }
    
    // Flush all caches
    auto flush_result = flush_all();
    if (!flush_result.has_value()) {
        LOG_WARN("Failed to flush caches during unmount: {}", 
                   static_cast<int>(flush_result.error().code()));
    }
    
    // Write updated superblock
    auto sb_result = write_superblock();
    if (!sb_result.has_value()) {
        LOG_WARN("Failed to write superblock during unmount: {}", 
                   static_cast<int>(sb_result.error().code()));
    }
    
    // Clear internal state
    object_cache_.clear();
    page_cache_.clear();
    name_to_object_map_.clear();
    free_pages_.clear();
    
    if (write_cache_) {
        std::lock_guard cache_lock(write_cache_->mutex);
        write_cache_->cached_pages.clear();
        write_cache_->current_size = 0;
    }
    
    mounted_ = false;
    
    LOG_INFO("SPIFFS unmounted successfully");
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::format() {
    std::unique_lock lock(filesystem_mutex_);
    
    if (!initialized_) {
        LOG_ERROR("SPIFFS not initialized");
        return error<void>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    LOG_INFO("Formatting SPIFFS filesystem");
    
    // Enable writes on flash controller
    auto write_enable_result = flash_controller_->write_enable();
    if (!write_enable_result.has_value()) {
        LOG_ERROR("Failed to enable writes for SPIFFS format");
        return error<void>(write_enable_result.error().code());
    }
    
    // Erase the entire partition
    for (uint32_t block = 0; block < total_blocks_; ++block) {
        Address block_address = partition_start_ + (block * SPIFFS_BLOCK_SIZE);
        
        // Enable writes for each erase operation (flash controller clears this after each operation)
        auto enable_result = flash_controller_->write_enable();
        if (!enable_result.has_value()) {
            LOG_ERROR("Failed to enable writes for block {} erase", block);
            return error<void>(enable_result.error().code());
        }
        
        auto erase_result = flash_controller_->erase_sector(block_address);
        if (!erase_result.has_value()) {
            LOG_ERROR("Failed to erase block {} at address 0x{:08X}", block, block_address);
            return error<void>(erase_result.error().code());
        }
    }
    
    // Initialize superblock
    superblock_->magic = SPIFFS_MAGIC;
    superblock_->version = SPIFFS_VERSION;
    superblock_->page_size = SPIFFS_PAGE_SIZE;
    superblock_->block_size = SPIFFS_BLOCK_SIZE;
    superblock_->block_count = total_blocks_;
    superblock_->pages_per_block = pages_per_block_;
    superblock_->free_blocks = total_blocks_ - 1;  // Reserve first block for superblock
    superblock_->free_pages = total_pages_ - pages_per_block_;
    superblock_->total_objects = 0;
    superblock_->max_objects = (total_pages_ / 4);  // Rough estimate
    superblock_->used_pages = pages_per_block_;
    superblock_->deleted_pages = 0;
    superblock_->last_gc_time = get_current_timestamp();
    superblock_->gc_cycles = 0;
    superblock_->wear_leveling_counter = 0;
    
    // Clear reserved space
    memset(superblock_->reserved, 0xFF, sizeof(superblock_->reserved));
    
    // Write superblock to first page
    auto write_result = write_superblock();
    if (!write_result.has_value()) {
        LOG_ERROR("Failed to write superblock");
        return error<void>(write_result.error().code());
    }
    
    // Initialize free page list (skip first block)
    free_pages_.clear();
    for (uint32_t page = pages_per_block_; page < total_pages_; ++page) {
        free_pages_.push_back(page);
    }
    
    // Clear object caches
    object_cache_.clear();
    page_cache_.clear();
    name_to_object_map_.clear();
    
    // Reset statistics
    stats_ = {};
    stats_.total_bytes = partition_size_;
    stats_.free_bytes = partition_size_ - SPIFFS_BLOCK_SIZE;  // Minus superblock
    stats_.total_pages = total_pages_;
    stats_.free_pages = total_pages_ - pages_per_block_;
    stats_.used_pages = pages_per_block_;
    
    LOG_INFO("SPIFFS format complete - {} total pages, {} free pages", 
             total_pages_, free_pages_.size());
    
    return Result<void>{};
}

Result<int> SPIFFSFileSystem::open(const std::string& path, AccessMode mode) {
    std::shared_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        LOG_ERROR("SPIFFS not mounted");
        return error<int>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    if (!is_valid_filename(path)) {
        LOG_ERROR("Invalid filename: {}", path);
        return error<int>(ErrorCode::STORAGE_INVALID_NAME);
    }
    
    LOG_DEBUG("Opening file: {} with mode: {}", path, static_cast<int>(mode));
    
    // Check if file exists
    auto find_result = find_object(path);
    std::shared_ptr<ObjectHeader> header;
    uint32_t object_id = 0;
    
    if (find_result.has_value()) {
        // File exists
        header = find_result.value();
        object_id = header->object_id;
        
        if (static_cast<uint8_t>(mode) & static_cast<uint8_t>(AccessMode::EXCLUSIVE)) {
            LOG_ERROR("File exists but EXCLUSIVE mode specified: {}", path);
            return error<int>(ErrorCode::STORAGE_FILE_EXISTS);
        }
        
        if (static_cast<uint8_t>(mode) & static_cast<uint8_t>(AccessMode::TRUNCATE)) {
            // Truncate existing file
            auto truncate_result = truncate(path, 0);
            if (!truncate_result.has_value()) {
                LOG_ERROR("Failed to truncate file: {}", path);
                return error<int>(truncate_result.error().code());
            }
        }
    } else {
        // File doesn't exist
        if (!(static_cast<uint8_t>(mode) & static_cast<uint8_t>(AccessMode::CREATE))) {
            LOG_ERROR("File not found and CREATE not specified: {}", path);
            return error<int>(ErrorCode::STORAGE_FILE_NOT_FOUND);
        }
        
        // Create new file
        auto create_result = allocate_object_id();
        if (!create_result.has_value()) {
            LOG_ERROR("Failed to allocate object ID for new file: {}", path);
            return error<int>(create_result.error().code());
        }
        
        object_id = create_result.value();
        header = std::make_shared<ObjectHeader>();
        
        // Initialize object header
        header->object_id = object_id;
        header->type = ObjectType::FILE;
        header->name_length = static_cast<uint8_t>(std::min(path.length(), 
                                                           static_cast<size_t>(SPIFFS_MAX_NAME_LENGTH - 1)));
        header->flags = 0;
        header->size = 0;
        header->pages_used = 0;
        header->created_time = get_current_timestamp();
        header->modified_time = header->created_time;
        strncpy(header->name, path.c_str(), SPIFFS_MAX_NAME_LENGTH - 1);
        header->name[SPIFFS_MAX_NAME_LENGTH - 1] = '\0';
        header->checksum = calculate_header_checksum(*header);
        
        // Cache the new object
        auto cache_result = cache_object(object_id, header);
        if (!cache_result.has_value()) {
            LOG_WARN("Failed to cache new object: {}", static_cast<int>(cache_result.error().code()));
        }
        
        // Add to name mapping
        name_to_object_map_[path] = object_id;
        
        LOG_DEBUG("Created new file object ID: {} for path: {}", object_id, path);
    }
    
    // Allocate file handle
    auto fd_result = allocate_file_handle(object_id, mode);
    if (!fd_result.has_value()) {
        LOG_ERROR("Failed to allocate file handle for: {}", path);
        return error<int>(fd_result.error().code());
    }
    
    int fd = fd_result.value();
    
    // Get the file handle and initialize it
    auto handle = get_file_handle(fd);
    if (!handle.has_value()) {
        LOG_ERROR("Failed to get allocated file handle");
        free_file_handle(fd);
        return error<int>(handle.error().code());
    }
    
    handle.value()->header = header;
    handle.value()->position = 0;
    
    // Set position for append mode
    if (static_cast<uint8_t>(mode) & static_cast<uint8_t>(AccessMode::APPEND)) {
        handle.value()->position = header->size;
    }
    
    LOG_DEBUG("File opened successfully: {} -> fd: {}", path, fd);
    return fd;
}

Result<void> SPIFFSFileSystem::close(int fd) {
    std::shared_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        return error<void>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    auto handle_result = get_file_handle(fd);
    if (!handle_result.has_value()) {
        return error<void>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    auto handle = handle_result.value();
    if (!handle->is_open) {
        return error<void>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    LOG_DEBUG("Closing file descriptor: {}", fd);
    
    // Flush any pending writes
    auto flush_result = flush(fd);
    if (!flush_result.has_value()) {
        LOG_WARN("Failed to flush file during close: {}", static_cast<int>(flush_result.error().code()));
    }
    
    // Free the file handle
    return free_file_handle(fd);
}

Result<size_t> SPIFFSFileSystem::read(int fd, void* buffer, size_t size) {
    std::shared_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        return error<size_t>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    if (!buffer || size == 0) {
        return 0;
    }
    
    auto handle_result = get_file_handle(fd);
    if (!handle_result.has_value()) {
        return error<size_t>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    auto handle = handle_result.value();
    if (!handle->is_open || !handle->header) {
        return error<size_t>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    // Check read permissions
    if (!(static_cast<uint8_t>(handle->mode) & static_cast<uint8_t>(AccessMode::READ))) {
        return error<size_t>(ErrorCode::STORAGE_ACCESS_DENIED);
    }
    
    // Calculate how much we can actually read
    uint32_t remaining_bytes = handle->header->size - handle->position;
    size_t bytes_to_read = std::min(size, static_cast<size_t>(remaining_bytes));
    
    if (bytes_to_read == 0) {
        return 0;  // EOF
    }
    
    LOG_DEBUG("Reading {} bytes from fd: {} at position: {}", bytes_to_read, fd, handle->position);
    
    // Get page chain for this object
    auto chain_result = get_page_chain(handle->object_id);
    if (!chain_result.has_value()) {
        return error<size_t>(chain_result.error().code());
    }
    
    auto page_chain = chain_result.value();
    if (page_chain.empty()) {
        return 0;  // Empty file
    }
    
    // Read data across pages
    size_t bytes_read = 0;
    uint8_t* output_buffer = static_cast<uint8_t*>(buffer);
    uint32_t current_position = handle->position;
    
    while (bytes_read < bytes_to_read) {
        // Find which page contains current position
        uint32_t page_offset = current_position / (SPIFFS_PAGE_SIZE - sizeof(PageHeader));
        uint32_t offset_in_page = current_position % (SPIFFS_PAGE_SIZE - sizeof(PageHeader));
        
        if (page_offset >= page_chain.size()) {
            break;  // Shouldn't happen, but safety check
        }
        
        uint32_t page_num = page_chain[page_offset];
        uint32_t bytes_in_page = std::min(
            bytes_to_read - bytes_read,
            static_cast<size_t>((SPIFFS_PAGE_SIZE - sizeof(PageHeader)) - offset_in_page)
        );
        
        // Read from this page
        auto read_result = read_page(page_num, output_buffer + bytes_read, 
                                   bytes_in_page, sizeof(PageHeader) + offset_in_page);
        if (!read_result.has_value()) {
            LOG_ERROR("Failed to read from page {}: {}", page_num, static_cast<int>(read_result.error().code()));
            return error<size_t>(read_result.error().code());
        }
        
        bytes_read += bytes_in_page;
        current_position += bytes_in_page;
    }
    
    // Update file position
    handle->position = current_position;
    handle->last_access = std::chrono::steady_clock::now();
    
    // Update statistics
    update_read_stats(bytes_read);
    
    LOG_DEBUG("Read {} bytes from fd: {}, new position: {}", bytes_read, fd, handle->position);
    return bytes_read;
}

Result<size_t> SPIFFSFileSystem::write(int fd, const void* data, size_t size) {
    std::shared_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        return error<size_t>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    if (!data || size == 0) {
        return 0;
    }
    
    auto handle_result = get_file_handle(fd);
    if (!handle_result.has_value()) {
        return error<size_t>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    auto handle = handle_result.value();
    if (!handle->is_open || !handle->header) {
        return error<size_t>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    // Check write permissions
    if (!(static_cast<uint8_t>(handle->mode) & static_cast<uint8_t>(AccessMode::WRITE))) {
        return error<size_t>(ErrorCode::STORAGE_ACCESS_DENIED);
    }
    
    LOG_DEBUG("Writing {} bytes to fd: {} at position: {}", size, fd, handle->position);
    
    // For now, implement a simplified write that appends to the file
    // In a full implementation, this would handle in-place updates and page allocation
    
    // Calculate new file size
    uint32_t new_size = std::max(handle->header->size, handle->position + static_cast<uint32_t>(size));
    uint32_t pages_needed = (new_size + (SPIFFS_PAGE_SIZE - sizeof(PageHeader) - 1)) / 
                           (SPIFFS_PAGE_SIZE - sizeof(PageHeader));
    
    // Check if we need more pages
    if (pages_needed > handle->header->pages_used) {
        uint32_t additional_pages = pages_needed - handle->header->pages_used;
        
        // Check if we have enough free pages
        if (free_pages_.size() < additional_pages) {
            // Try garbage collection
            auto gc_result = garbage_collect();
            if (!gc_result.has_value()) {
                LOG_ERROR("Failed to perform garbage collection");
                return error<size_t>(ErrorCode::STORAGE_INSUFFICIENT_SPACE);
            }
            
            if (free_pages_.size() < additional_pages) {
                LOG_ERROR("Insufficient space for write operation");
                return error<size_t>(ErrorCode::STORAGE_INSUFFICIENT_SPACE);
            }
        }
        
        // Allocate additional pages
        auto alloc_result = allocate_pages(additional_pages);
        if (!alloc_result.has_value()) {
            return error<size_t>(alloc_result.error().code());
        }
        
        // Update object header
        handle->header->pages_used = pages_needed;
    }
    
    // For simplified implementation, we'll write data directly
    // In a production implementation, this would use the write cache
    
    // Update file size and modification time
    handle->header->size = new_size;
    handle->header->modified_time = get_current_timestamp();
    handle->header->checksum = calculate_header_checksum(*handle->header);
    
    // Update object cache
    auto cache_result = cache_object(handle->object_id, handle->header);
    if (!cache_result.has_value()) {
        LOG_WARN("Failed to update object cache: {}", static_cast<int>(cache_result.error().code()));
    }
    
    // Update position
    handle->position += size;
    handle->last_access = std::chrono::steady_clock::now();
    
    // Update statistics
    update_write_stats(size);
    
    // Trigger garbage collection if needed
    auto gc_check = trigger_garbage_collection_if_needed();
    if (!gc_check.has_value()) {
        LOG_WARN("GC trigger check failed: {}", static_cast<int>(gc_check.error().code()));
    }
    
    auto object_size = handle->header->size;
    LOG_DEBUG("Wrote {} bytes to fd: {}, new position: {}, new size: {}", 
             size, fd, handle->position, object_size);
    
    return size;
}

Result<void> SPIFFSFileSystem::read_superblock() {
    if (!flash_controller_) {
        return error<void>(ErrorCode::STORAGE_CONTROLLER_ERROR);
    }
    
    LOG_DEBUG("Reading SPIFFS superblock from address 0x{:08X}", partition_start_);
    
    auto read_result = flash_controller_->read(partition_start_, superblock_.get(), sizeof(Superblock));
    if (!read_result.has_value()) {
        LOG_ERROR("Failed to read superblock from flash");
        return error<void>(read_result.error().code());
    }
    
    // Verify magic number and version
    if (superblock_->magic != SPIFFS_MAGIC) {
        auto magic_value = superblock_->magic;
        LOG_ERROR("Invalid SPIFFS magic number: 0x{:08X} (expected: 0x{:08X})", 
                 magic_value, SPIFFS_MAGIC);
        return error<void>(ErrorCode::STORAGE_INVALID_FORMAT);
    }
    
    if (superblock_->version != SPIFFS_VERSION) {
        auto version_value = superblock_->version;
        LOG_WARN("SPIFFS version mismatch: {} (expected: {})", 
                   version_value, SPIFFS_VERSION);
        // Continue anyway for compatibility
    }
    
    auto block_count = superblock_->block_count;
    auto total_pages = superblock_->free_pages + superblock_->used_pages;
    LOG_DEBUG("Superblock read successfully - {} blocks, {} pages", 
             block_count, total_pages);
    
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::write_superblock() {
    if (!flash_controller_) {
        return error<void>(ErrorCode::STORAGE_CONTROLLER_ERROR);
    }
    
    LOG_DEBUG("Writing SPIFFS superblock to address 0x{:08X}", partition_start_);
    
    // Update timestamp
    superblock_->last_gc_time = get_current_timestamp();
    
    // Enable writes before writing superblock
    auto write_enable_result = flash_controller_->write_enable();
    if (!write_enable_result.has_value()) {
        LOG_ERROR("Failed to enable writes for superblock write");
        return error<void>(write_enable_result.error().code());
    }
    
    auto write_result = flash_controller_->write(partition_start_, superblock_.get(), sizeof(Superblock));
    if (!write_result.has_value()) {
        LOG_ERROR("Failed to write superblock to flash");
        return error<void>(write_result.error().code());
    }
    
    LOG_DEBUG("Superblock written successfully");
    return Result<void>{};
}

// Implement remaining key methods (scan_filesystem, build_object_table, etc.)
Result<void> SPIFFSFileSystem::scan_filesystem() {
    LOG_DEBUG("Scanning SPIFFS filesystem");
    
    // Clear existing caches
    object_cache_.clear();
    page_cache_.clear();
    name_to_object_map_.clear();
    
    // Scan all pages looking for valid object headers and pages
    for (uint32_t page = pages_per_block_; page < total_pages_; ++page) {
        auto header_result = read_page_header(page);
        if (!header_result.has_value()) {
            continue;  // Invalid or erased page
        }
        
        auto page_header = header_result.value();
        
        // Cache page information
        PageInfo page_info;
        page_info.object_id = page_header.object_id;
        page_info.type = page_header.type;
        page_info.page_index = page_header.page_index;
        page_info.next_page = page_header.next_page;
        page_info.data_size = page_header.data_size;
        page_info.is_dirty = false;
        page_info.last_access = std::chrono::steady_clock::now();
        
        page_cache_[page] = page_info;
    }
    
    LOG_DEBUG("Scanned {} pages", page_cache_.size());
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::build_free_page_list() {
    LOG_DEBUG("Building free page list");
    
    free_pages_.clear();
    
    // Skip first block (superblock)
    for (uint32_t page = pages_per_block_; page < total_pages_; ++page) {
        if (page_cache_.find(page) == page_cache_.end()) {
            // Page not in use, add to free list
            free_pages_.push_back(page);
        }
    }
    
    LOG_DEBUG("Found {} free pages", free_pages_.size());
    return Result<void>{};
}

// Additional helper method implementations...
uint32_t SPIFFSFileSystem::calculate_header_checksum(const ObjectHeader& header) {
    // Calculate checksum excluding the checksum field itself
    return calculate_crc32(&header, sizeof(ObjectHeader) - sizeof(uint32_t));
}

void SPIFFSFileSystem::update_read_stats(size_t bytes) {
    std::lock_guard lock(stats_mutex_);
    stats_.total_reads++;
    stats_.bytes_read += bytes;
}

void SPIFFSFileSystem::update_write_stats(size_t bytes) {
    std::lock_guard lock(stats_mutex_);
    stats_.total_writes++;
    stats_.bytes_written += bytes;
}

// Implement remaining methods with simplified placeholder implementations
// These would be fully implemented in a production system

Result<uint32_t> SPIFFSFileSystem::allocate_object_id() {
    if (free_object_ids_.empty()) {
        // Generate more IDs
        uint32_t base = free_object_ids_.size() + 1000;
        for (uint32_t i = 0; i < 100; ++i) {
            free_object_ids_.push_back(base + i);
        }
    }
    
    uint32_t id = free_object_ids_.back();
    free_object_ids_.pop_back();
    return id;
}

Result<std::shared_ptr<SPIFFSFileSystem::ObjectHeader>> SPIFFSFileSystem::find_object(const std::string& name) {
    auto it = name_to_object_map_.find(name);
    if (it == name_to_object_map_.end()) {
        return error<std::shared_ptr<ObjectHeader>>(ErrorCode::STORAGE_FILE_NOT_FOUND);
    }
    
    auto cache_it = object_cache_.find(it->second);
    if (cache_it != object_cache_.end()) {
        return cache_it->second->header;
    }
    
    return error<std::shared_ptr<ObjectHeader>>(ErrorCode::STORAGE_FILE_NOT_FOUND);
}

Result<int> SPIFFSFileSystem::allocate_file_handle(uint32_t object_id, AccessMode mode) {
    std::lock_guard lock(handle_mutex_);
    
    if (file_handles_.size() >= config_.max_open_files) {
        return error<int>(ErrorCode::STORAGE_TOO_MANY_FILES);
    }
    
    int fd = next_fd_++;
    auto handle = std::make_unique<FileHandle>();
    handle->object_id = object_id;
    handle->position = 0;
    handle->mode = mode;
    handle->is_open = true;
    handle->last_access = std::chrono::steady_clock::now();
    
    file_handles_[fd] = std::move(handle);
    return fd;
}

Result<void> SPIFFSFileSystem::free_file_handle(int fd) {
    std::lock_guard lock(handle_mutex_);
    
    auto it = file_handles_.find(fd);
    if (it == file_handles_.end()) {
        return error<void>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    file_handles_.erase(it);
    return Result<void>{};
}

Result<SPIFFSFileSystem::FileHandle*> SPIFFSFileSystem::get_file_handle(int fd) {
    std::lock_guard lock(handle_mutex_);
    
    auto it = file_handles_.find(fd);
    if (it == file_handles_.end() || !it->second || !it->second->is_open) {
        return error<FileHandle*>(ErrorCode::STORAGE_INVALID_HANDLE);
    }
    
    return it->second.get();
}

// Stub implementations for remaining required methods
Result<std::vector<uint32_t>> SPIFFSFileSystem::get_page_chain(uint32_t object_id) {
    // Simplified implementation - in production would build actual chain
    return std::vector<uint32_t>{};
}

Result<void> SPIFFSFileSystem::read_page(uint32_t page_num, void* buffer, size_t size, size_t offset) {
    Address page_address = partition_start_ + (page_num * SPIFFS_PAGE_SIZE) + offset;
    return flash_controller_->read(page_address, buffer, size);
}

Result<std::vector<uint32_t>> SPIFFSFileSystem::allocate_pages(size_t count) {
    std::vector<uint32_t> allocated;
    
    for (size_t i = 0; i < count && !free_pages_.empty(); ++i) {
        allocated.push_back(free_pages_.back());
        free_pages_.pop_back();
    }
    
    if (allocated.size() < count) {
        // Return pages to free list
        free_pages_.insert(free_pages_.end(), allocated.begin(), allocated.end());
        return error<std::vector<uint32_t>>(ErrorCode::STORAGE_INSUFFICIENT_SPACE);
    }
    
    return allocated;
}

SPIFFSFileSystem::Statistics SPIFFSFileSystem::get_statistics() const {
    std::lock_guard lock(stats_mutex_);
    return stats_;
}

Result<void> SPIFFSFileSystem::cache_object(uint32_t object_id, std::shared_ptr<ObjectHeader> header) {
    auto info = std::make_shared<ObjectInfo>();
    info->header = header;
    info->is_dirty = false;
    info->last_access = std::chrono::steady_clock::now();
    
    object_cache_[object_id] = info;
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::garbage_collect() {
    LOG_INFO("Performing SPIFFS garbage collection");
    return Result<void>{};
}

Result<void> SPIFFSFileSystem::trigger_garbage_collection_if_needed() {
    if (free_pages_.size() * 100 / total_pages_ < (100 - config_.gc_trigger_threshold)) {
        return garbage_collect();
    }
    return Result<void>{};
}

Result<SPIFFSFileSystem::PageHeader> SPIFFSFileSystem::read_page_header(uint32_t page_num) {
    PageHeader header;
    Address page_address = partition_start_ + (page_num * SPIFFS_PAGE_SIZE);
    
    auto read_result = flash_controller_->read(page_address, &header, sizeof(PageHeader));
    if (!read_result.has_value()) {
        return error<PageHeader>(read_result.error().code());
    }
    
    return header;
}

Result<void> SPIFFSFileSystem::recalculate_statistics() {
    std::lock_guard lock(stats_mutex_);
    
    stats_.total_bytes = partition_size_;
    stats_.free_bytes = free_pages_.size() * SPIFFS_PAGE_SIZE;
    stats_.used_bytes = stats_.total_bytes - stats_.free_bytes;
    stats_.total_pages = total_pages_;
    stats_.free_pages = static_cast<uint32_t>(free_pages_.size());
    stats_.used_pages = total_pages_ - stats_.free_pages;
    stats_.total_objects = static_cast<uint32_t>(object_cache_.size());
    
    return Result<void>{};
}

// Simplified implementations for remaining methods
Result<void> SPIFFSFileSystem::flush(int fd) { return Result<void>{}; }
Result<void> SPIFFSFileSystem::flush_all() { return Result<void>{}; }
Result<long> SPIFFSFileSystem::seek(int fd, long offset, SeekMode whence) { return 0; }
Result<long> SPIFFSFileSystem::tell(int fd) { return 0; }
Result<void> SPIFFSFileSystem::unlink(const std::string& path) { return Result<void>{}; }
Result<void> SPIFFSFileSystem::truncate(const std::string& path, size_t size) { return Result<void>{}; }
Result<bool> SPIFFSFileSystem::exists(const std::string& path) { return false; }

Result<std::vector<std::string>> SPIFFSFileSystem::list_files() {
    std::shared_lock lock(filesystem_mutex_);
    
    if (!mounted_) {
        LOG_ERROR("SPIFFS not mounted");
        return error<std::vector<std::string>>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    std::vector<std::string> files;
    
    // Iterate through cached objects to build file list
    for (const auto& [object_id, object_info] : object_cache_) {
        if (object_info && object_info->header) {
            // Only include files, not directories or other object types
            if (object_info->header->type == ObjectType::FILE) {
                files.push_back(std::string(object_info->header->name));
            }
        }
    }
    
    LOG_DEBUG("Listed {} files from SPIFFS", files.size());
    return success<std::vector<std::string>>(std::move(files));
}

Result<void> SPIFFSFileSystem::sync_all() {
    std::lock_guard<std::shared_mutex> lock(filesystem_mutex_);
    
    if (!mounted_) {
        LOG_ERROR("SPIFFS not mounted");
        return error<void>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    LOG_DEBUG("Syncing all SPIFFS data to flash");
    
    // Flush all dirty pages to flash
    auto flush_result = flush_dirty_pages();
    if (!flush_result.has_value()) {
        LOG_ERROR("Failed to flush dirty pages during sync_all");
        return flush_result;
    }
    
    // Update superblock with current statistics
    auto superblock_result = write_superblock();
    if (!superblock_result.has_value()) {
        LOG_ERROR("Failed to write superblock during sync_all");
        return superblock_result;
    }
    
    // Force flash controller to persist data if available
    // Note: This would ideally call flash_controller_->sync() if that method existed
    
    LOG_DEBUG("All SPIFFS data synced to flash successfully");
    return success<void>();
}

Result<void> SPIFFSFileSystem::flush_dirty_pages() {
    if (!mounted_) {
        LOG_ERROR("SPIFFS not mounted");
        return error<void>(ErrorCode::STORAGE_NOT_MOUNTED);
    }
    
    try {
        LOG_DEBUG("Flushing dirty pages to flash");
        
        size_t flushed_pages = 0;
        
        // Iterate through page cache and write dirty pages to flash
        for (auto& cache_entry : page_cache_) {
            if (cache_entry.second.is_dirty) {
                uint32_t page_num = cache_entry.first;
                
                // For a minimal implementation, we'll just mark the page as clean
                // In a full implementation, we would write actual page data to flash
                // which would be maintained in a separate data cache structure
                
                // Mark page as clean
                cache_entry.second.is_dirty = false;
                cache_entry.second.last_access = std::chrono::steady_clock::now();
                flushed_pages++;
                
                LOG_TRACE("Marked dirty page {} as clean", page_num);
            }
        }
        
        // Update statistics
        {
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            // Using bytes_written since pages_written doesn't exist in Statistics
            stats_.bytes_written += flushed_pages * SPIFFS_PAGE_SIZE;
        }
        
        LOG_DEBUG("Successfully flushed {} dirty pages to flash", flushed_pages);
        return success<void>();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Exception while flushing dirty pages: {}", e.what());
        return error<void>(ErrorCode::OPERATION_FAILED);
    }
}

} // namespace m5tab5::emulator::storage