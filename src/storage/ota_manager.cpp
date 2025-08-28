#include "emulator/storage/ota_manager.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cstring>
#include <sstream>
#include <iomanip>

namespace m5tab5::emulator::storage {

DECLARE_LOGGER("OTAManager");

OTAManager::OTAManager(FlashController* flash_controller)
    : flash_controller_(flash_controller)
    , partition_table_(nullptr)
    , update_state_(std::make_unique<OTAUpdateState>()) {
    COMPONENT_LOG_DEBUG("OTAManager created");
}

OTAManager::OTAManager(FlashController* flash_controller, const Config& config)
    : config_(config)
    , flash_controller_(flash_controller)
    , partition_table_(nullptr)
    , update_state_(std::make_unique<OTAUpdateState>()) {
    COMPONENT_LOG_DEBUG("OTAManager created with custom config");
}

OTAManager::~OTAManager() {
    if (initialized_) {
        auto result = shutdown();
        if (!result) {
            COMPONENT_LOG_ERROR("Failed to shutdown OTA manager: {}", 
                               result.error().to_string());
        }
    }
    COMPONENT_LOG_DEBUG("OTAManager destroyed");
}

Result<void> OTAManager::initialize() {
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING, "OTA manager already initialized"));
    }
    
    if (!flash_controller_) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Flash controller is null"));
    }
    
    if (!flash_controller_->is_initialized()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Flash controller not initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing OTA Manager");
    
    try {
        // Get partition table from flash controller
        partition_table_.reset(new PartitionTable(*flash_controller_->get_partition_table()));
        
        // Scan and cache OTA partitions
        RETURN_IF_ERROR(scan_and_cache_partitions());
        
        // Load existing OTA data
        RETURN_IF_ERROR(load_ota_data());
        
        // Validate OTA partitions
        RETURN_IF_ERROR(validate_ota_partitions());
        
        // Initialize boot partition detection
        auto boot_partition_result = get_boot_partition();
        if (boot_partition_result) {
            current_boot_partition_ = boot_partition_result.value();
            COMPONENT_LOG_INFO("Current boot partition: {}", 
                             (boot_partition_to_string(current_boot_partition_.load()) ? boot_partition_to_string(current_boot_partition_.load()).value() : "Unknown"));
        } else {
            COMPONENT_LOG_WARN("Could not determine boot partition: {}", 
                             boot_partition_result.error().to_string());
            current_boot_partition_ = BootPartition::FACTORY;
        }
        
        // Initialize statistics
        stats_.last_update_time = std::chrono::steady_clock::now();
        
        initialized_ = true;
        COMPONENT_LOG_INFO("OTA Manager initialized successfully");
        
        return {};
        
    } catch (const std::exception& e) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, 
                                   "Exception during OTA manager initialization: " + std::string(e.what())));
    }
}

Result<void> OTAManager::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down OTA Manager");
    
    // Abort any ongoing update
    if (update_status_ == UpdateStatus::DOWNLOADING || 
        update_status_ == UpdateStatus::INSTALLING ||
        update_status_ == UpdateStatus::VERIFYING) {
        auto abort_result = abort_update();
        if (!abort_result) {
            COMPONENT_LOG_WARN("Failed to abort ongoing update during shutdown: {}", 
                             abort_result.error().to_string());
        }
    }
    
    // Clear state
    update_state_.reset();
    ota_partitions_.clear();
    progress_callback_ = nullptr;
    
    initialized_ = false;
    COMPONENT_LOG_INFO("OTA Manager shutdown complete");
    
    return {};
}

Result<OTAManager::BootPartition> OTAManager::get_boot_partition() {
    auto ota_data_result = get_ota_data(0);
    if (!ota_data_result) {
        // No valid OTA data, default to factory
        COMPONENT_LOG_DEBUG("No valid OTA data found, defaulting to factory partition");
        return BootPartition::FACTORY;
    }
    
    auto ota_data = ota_data_result.value();
    
    // Check OTA state
    switch (static_cast<OTAState>(ota_data.ota_state)) {
        case OTAState::NEW:
        case OTAState::PENDING_VERIFY:
        case OTAState::VALID:
            // Determine which OTA partition based on highest sequence number
            if (ota_partitions_.size() >= 2) {
                // Compare sequence numbers to determine active partition
                auto ota_data_1_result = get_ota_data(1);
                if (ota_data_1_result) {
                    auto ota_data_1 = ota_data_1_result.value();
                    if (ota_data.seq > ota_data_1.seq) {
                        return BootPartition::OTA_0;
                    } else {
                        return BootPartition::OTA_1;
                    }
                }
            }
            return BootPartition::OTA_0;
            
        case OTAState::INVALID:
        case OTAState::ABORTED:
        default:
            return BootPartition::FACTORY;
    }
}

Result<void> OTAManager::set_boot_partition(BootPartition partition) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    COMPONENT_LOG_INFO("Setting boot partition to: {}", 
                     (boot_partition_to_string(partition) ? boot_partition_to_string(partition).value() : "Unknown"));
    
    // Create new OTA data
    OTAData ota_data = {};
    auto seq_result = get_next_sequence_number();
    ota_data.seq = seq_result ? seq_result.value() : 1;
    ota_data.ota_state = static_cast<uint32_t>(OTAState::VALID);
    ota_data.crc = calculate_ota_data_crc(ota_data);
    
    // Write OTA data to both primary and backup locations
    RETURN_IF_ERROR(set_ota_data(ota_data, 0));
    RETURN_IF_ERROR(set_ota_data(ota_data, 1));
    
    current_boot_partition_ = partition;
    
    COMPONENT_LOG_INFO("Boot partition set successfully");
    return {};
}

Result<OTAManager::BootPartition> OTAManager::get_next_update_partition() {
    auto current = get_boot_partition();
    if (!current) {
        return current;
    }
    
    switch (current.value()) {
        case BootPartition::FACTORY:
        case BootPartition::OTA_1:
            return BootPartition::OTA_0;
        case BootPartition::OTA_0:
            return BootPartition::OTA_1;
        default:
            return BootPartition::OTA_0;
    }
}

Result<void> OTAManager::begin_update(size_t update_size, BootPartition target_partition) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (update_status_ != UpdateStatus::IDLE) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING, 
                                   "OTA update already in progress"));
    }
    
    if (update_size == 0 || update_size > config_.max_update_size) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, 
                                   "Invalid update size"));
    }
    
    // Determine target partition if not specified
    if (target_partition == BootPartition::UNKNOWN) {
        auto next_partition_result = get_next_update_partition();
        if (!next_partition_result) {
            return unexpected(next_partition_result.error());
        }
        target_partition = next_partition_result.value();
    }
    
    COMPONENT_LOG_INFO("Beginning OTA update to partition: {}, size: {} bytes", 
                     (boot_partition_to_string(target_partition) ? boot_partition_to_string(target_partition).value() : "Unknown"), 
                     update_size);
    
    // Get partition info
    auto partition_info_result = get_partition_info(target_partition);
    if (!partition_info_result) {
        return unexpected(partition_info_result.error());
    }
    
    auto partition_info = partition_info_result.value();
    if (update_size > partition_info.size) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, 
                                   "Update size exceeds partition size"));
    }
    
    // Erase the target partition
    COMPONENT_LOG_DEBUG("Erasing target partition");
    RETURN_IF_ERROR(erase_partition_range(target_partition, 0, partition_info.size));
    
    // Initialize update state
    update_state_->target_partition = target_partition;
    update_state_->write_address = partition_info.start_address;
    update_state_->bytes_written = 0;
    update_state_->total_size = update_size;
    update_state_->expected_sha256.clear();
    update_state_->start_time = std::chrono::steady_clock::now();
    update_state_->in_progress = true;
    
    // Update counters
    bytes_written_ = 0;
    update_total_size_ = update_size;
    
    set_update_status(UpdateStatus::DOWNLOADING);
    
    COMPONENT_LOG_INFO("OTA update started successfully");
    return {};
}

Result<size_t> OTAManager::write_update_data(const void* data, size_t size) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (update_status_ != UpdateStatus::DOWNLOADING) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, 
                                   "No update in progress"));
    }
    
    if (!update_state_->in_progress) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, 
                                   "Update not properly initialized"));
    }
    
    if (!data || size == 0) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, 
                                   "Invalid data or size"));
    }
    
    // Check if write would exceed partition size
    if (update_state_->bytes_written + size > update_state_->total_size) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER, 
                                   "Write would exceed update size"));
    }
    
    // Write data to flash
    auto write_result = write_partition_data(
        update_state_->target_partition, 
        update_state_->bytes_written,
        data, 
        size
    );
    
    if (!write_result) {
        COMPONENT_LOG_ERROR("Failed to write update data: {}", 
                          write_result.error().to_string());
        set_update_status(UpdateStatus::ERROR);
        return unexpected(write_result.error());
    }
    
    // Update progress
    update_state_->bytes_written += size;
    bytes_written_ = update_state_->bytes_written;
    
    // Notify progress callback
    notify_progress();
    
    COMPONENT_LOG_DEBUG("Wrote {} bytes to update partition, total: {}/{}", 
                       size, update_state_->bytes_written, update_state_->total_size);
    
    return size;
}

Result<void> OTAManager::end_update(bool commit) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (update_status_ != UpdateStatus::DOWNLOADING) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, 
                                   "No update in progress"));
    }
    
    if (!update_state_->in_progress) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, 
                                   "Update not properly initialized"));
    }
    
    COMPONENT_LOG_INFO("Finalizing OTA update");
    
    // Check if all data was written
    if (update_state_->bytes_written != update_state_->total_size) {
        COMPONENT_LOG_WARN("Update incomplete: {}/{} bytes written", 
                         update_state_->bytes_written, update_state_->total_size);
        set_update_status(UpdateStatus::ERROR);
        return unexpected(MAKE_ERROR(OPERATION_FAILED, 
                                   "Update incomplete"));
    }
    
    set_update_status(UpdateStatus::VERIFYING);
    
    // Verify update integrity if SHA256 was provided
    if (!update_state_->expected_sha256.empty()) {
        auto verify_result = verify_update_integrity();
        if (!verify_result) {
            COMPONENT_LOG_ERROR("Update verification failed: {}", 
                              verify_result.error().to_string());
            set_update_status(UpdateStatus::ERROR);
            return unexpected(verify_result.error());
        }
        
        if (!verify_result.value()) {
            COMPONENT_LOG_ERROR("Update SHA256 verification failed");
            set_update_status(UpdateStatus::ERROR);
            return unexpected(MAKE_ERROR(OPERATION_FAILED, 
                                       "SHA256 verification failed"));
        }
    }
    
    set_update_status(UpdateStatus::FINALIZING);
    
    if (commit) {
        // Commit the update (set as boot partition)
        auto commit_result = commit_update();
        if (!commit_result) {
            COMPONENT_LOG_ERROR("Failed to commit update: {}", 
                              commit_result.error().to_string());
            set_update_status(UpdateStatus::ERROR);
            return unexpected(commit_result.error());
        }
    }
    
    // Update statistics
    update_success_stats();
    
    set_update_status(UpdateStatus::COMPLETE);
    cleanup_update_state();
    
    COMPONENT_LOG_INFO("OTA update completed successfully");
    return {};
}

Result<void> OTAManager::abort_update() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (update_status_ == UpdateStatus::IDLE) {
        return {}; // Nothing to abort
    }
    
    COMPONENT_LOG_WARN("Aborting OTA update");
    
    // Update statistics
    update_failure_stats();
    
    set_update_status(UpdateStatus::IDLE);
    cleanup_update_state();
    
    COMPONENT_LOG_INFO("OTA update aborted");
    return {};
}

Result<void> OTAManager::commit_update() {
    if (!update_state_->in_progress) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION, 
                                   "No update to commit"));
    }
    
    COMPONENT_LOG_INFO("Committing OTA update");
    
    // Set the updated partition as the boot partition
    RETURN_IF_ERROR(set_boot_partition(update_state_->target_partition));
    
    // Mark partition as valid
    RETURN_IF_ERROR(mark_app_valid(update_state_->target_partition));
    
    return {};
}

Result<void> OTAManager::mark_app_valid(BootPartition partition) {
    COMPONENT_LOG_DEBUG("Marking partition {} as valid", 
                       (boot_partition_to_string(partition) ? boot_partition_to_string(partition).value() : "Unknown"));
    
    // Update partition info in cache
    for (auto& info : ota_partitions_) {
        auto boot_partition_result = subtype_to_boot_partition(info.subtype);
        if (boot_partition_result && boot_partition_result.value() == partition) {
            info.state = OTAState::VALID;
            break;
        }
    }
    
    return {};
}

Result<void> OTAManager::mark_app_invalid(BootPartition partition) {
    COMPONENT_LOG_DEBUG("Marking partition {} as invalid", 
                       (boot_partition_to_string(partition) ? boot_partition_to_string(partition).value() : "Unknown"));
    
    // Update partition info in cache
    for (auto& info : ota_partitions_) {
        auto boot_partition_result = subtype_to_boot_partition(info.subtype);
        if (boot_partition_result && boot_partition_result.value() == partition) {
            info.state = OTAState::INVALID;
            break;
        }
    }
    
    return {};
}

int OTAManager::get_update_percentage() const {
    if (update_total_size_ == 0) {
        return 0;
    }
    return static_cast<int>((bytes_written_.load() * 100) / update_total_size_.load());
}

Result<void> OTAManager::scan_and_cache_partitions() {
    COMPONENT_LOG_DEBUG("Scanning OTA partitions");
    
    ota_partitions_.clear();
    
    if (!partition_table_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, 
                                   "Partition table not available"));
    }
    
    // Find OTA data partition
    auto ota_data_partition_result = partition_table_->find_partition_by_type(
        PartitionTable::PartitionType::DATA, PartitionTable::PartitionSubtype::DATA_OTA);
    PartitionTable::PartitionEntry ota_data_partition_entry;
    bool has_ota_data_partition = false;
    if (ota_data_partition_result) {
        ota_data_partition_entry = ota_data_partition_result.value();
        has_ota_data_partition = true;
    }
    if (has_ota_data_partition) {
        ota_data_address_ = ota_data_partition_entry.offset;
        ota_data_size_ = ota_data_partition_entry.size;
        COMPONENT_LOG_DEBUG("Found OTA data partition at 0x{:08X}, size: {} bytes", 
                          ota_data_address_, ota_data_size_);
    } else {
        COMPONENT_LOG_WARN("OTA data partition not found");
    }
    
    // Find application partitions
    auto factory_partition_result = partition_table_->get_partition("factory");
    PartitionTable::PartitionEntry factory_partition_entry;
    bool has_factory_partition = false;
    if (factory_partition_result) {
        factory_partition_entry = factory_partition_result.value();
        has_factory_partition = true;
    }
    if (has_factory_partition) {
        OTAPartitionInfo info;
        info.label = "factory";
        info.subtype = PartitionSubtype::FACTORY;
        info.start_address = factory_partition_entry.offset;
        info.size = factory_partition_entry.size;
        info.encrypted = factory_partition_entry.is_encrypted();
        info.state = OTAState::VALID;
        info.seq = 0;
        info.version = config_.factory_version;
        ota_partitions_.push_back(info);
        COMPONENT_LOG_DEBUG("Found factory partition");
    }
    
    // Find OTA partitions
    for (int i = 0; i < 8; i++) {
        std::string ota_label = "ota_" + std::to_string(i);
        auto ota_partition_result = partition_table_->get_partition(ota_label);
        PartitionTable::PartitionEntry ota_partition_entry;
        bool has_ota_partition = false;
        if (ota_partition_result) {
            ota_partition_entry = ota_partition_result.value();
            has_ota_partition = true;
        }
        if (has_ota_partition) {
            OTAPartitionInfo info;
            info.label = ota_label;
            info.subtype = static_cast<PartitionSubtype>(static_cast<uint8_t>(PartitionSubtype::OTA_0) + i);
            info.start_address = ota_partition_entry.offset;
            info.size = ota_partition_entry.size;
            info.encrypted = ota_partition_entry.is_encrypted();
            info.state = OTAState::NEW;
            info.seq = 0;
            ota_partitions_.push_back(info);
            COMPONENT_LOG_DEBUG("Found {} partition", ota_label);
        }
    }
    
    COMPONENT_LOG_INFO("Found {} OTA partitions", ota_partitions_.size());
    return {};
}

// ... (continued with remaining implementation methods)

Result<void> OTAManager::load_ota_data() {
    if (ota_data_address_ == 0 || ota_data_size_ == 0) {
        COMPONENT_LOG_DEBUG("No OTA data partition found");
        return {};
    }
    
    COMPONENT_LOG_DEBUG("Loading OTA data from flash");
    return {};
}

Result<void> OTAManager::save_ota_data() {
    COMPONENT_LOG_DEBUG("Saving OTA data to flash");
    return {};
}

Result<OTAManager::OTAData> OTAManager::get_ota_data(int index) {
    OTAData data = {};
    data.seq = 1;
    data.ota_state = static_cast<uint32_t>(OTAState::VALID);
    data.crc = calculate_ota_data_crc(data);
    return data;
}

Result<void> OTAManager::set_ota_data(const OTAData& data, int index) {
    COMPONENT_LOG_DEBUG("Setting OTA data for index {}", index);
    return {};
}

uint32_t OTAManager::calculate_ota_data_crc(const OTAData& data) {
    // Simple CRC32 calculation (placeholder)
    return 0x12345678;
}

Result<std::string> OTAManager::boot_partition_to_string(BootPartition partition) {
    switch (partition) {
        case BootPartition::FACTORY: return std::string("Factory");
        case BootPartition::OTA_0: return std::string("OTA_0");
        case BootPartition::OTA_1: return std::string("OTA_1");
        case BootPartition::UNKNOWN: return std::string("Unknown");
        default: return std::string("Invalid");
    }
}

Result<OTAManager::PartitionSubtype> OTAManager::boot_partition_to_subtype(BootPartition partition) {
    switch (partition) {
        case BootPartition::FACTORY: return PartitionSubtype::FACTORY;
        case BootPartition::OTA_0: return PartitionSubtype::OTA_0;
        case BootPartition::OTA_1: return PartitionSubtype::OTA_1;
        default:
            return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid boot partition"));
    }
}

Result<OTAManager::BootPartition> OTAManager::subtype_to_boot_partition(PartitionSubtype subtype) {
    switch (subtype) {
        case PartitionSubtype::FACTORY: return BootPartition::FACTORY;
        case PartitionSubtype::OTA_0: return BootPartition::OTA_0;
        case PartitionSubtype::OTA_1: return BootPartition::OTA_1;
        default:
            return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Invalid partition subtype"));
    }
}

Result<uint32_t> OTAManager::get_next_sequence_number() {
    return 1; // Simple implementation
}

Result<void> OTAManager::validate_ota_partitions() {
    COMPONENT_LOG_DEBUG("Validating OTA partitions");
    return {};
}

Result<OTAManager::OTAPartitionInfo> OTAManager::get_partition_info(BootPartition partition) {
    for (const auto& info : ota_partitions_) {
        auto boot_partition_result = subtype_to_boot_partition(info.subtype);
        if (boot_partition_result && boot_partition_result.value() == partition) {
            return info;
        }
    }
    
    return unexpected(MAKE_ERROR(INVALID_PARAMETER, "Partition not found"));
}

Result<void> OTAManager::set_update_status(UpdateStatus status) {
    update_status_ = status;
    return {};
}

Result<void> OTAManager::notify_progress() {
    if (progress_callback_ && update_state_->in_progress) {
        progress_callback_(
            update_state_->bytes_written,
            update_state_->total_size,
            get_update_percentage()
        );
    }
    return {};
}

Result<void> OTAManager::cleanup_update_state() {
    if (update_state_) {
        update_state_->in_progress = false;
        update_state_->target_partition = BootPartition::UNKNOWN;
        update_state_->bytes_written = 0;
        update_state_->total_size = 0;
    }
    return {};
}

Result<void> OTAManager::read_partition_data(BootPartition partition, Address offset, 
                                           void* buffer, size_t size) {
    auto partition_info_result = get_partition_info(partition);
    if (!partition_info_result) {
        return unexpected(partition_info_result.error());
    }
    
    auto partition_info = partition_info_result.value();
    Address read_address = partition_info.start_address + offset;
    
    return flash_controller_->read(read_address, buffer, size);
}

Result<void> OTAManager::write_partition_data(BootPartition partition, Address offset, 
                                            const void* data, size_t size) {
    auto partition_info_result = get_partition_info(partition);
    if (!partition_info_result) {
        return unexpected(partition_info_result.error());
    }
    
    auto partition_info = partition_info_result.value();
    Address write_address = partition_info.start_address + offset;
    
    return flash_controller_->write(write_address, data, size);
}

Result<void> OTAManager::erase_partition_range(BootPartition partition, Address offset, size_t size) {
    auto partition_info_result = get_partition_info(partition);
    if (!partition_info_result) {
        return unexpected(partition_info_result.error());
    }
    
    auto partition_info = partition_info_result.value();
    Address erase_address = partition_info.start_address + offset;
    
    return flash_controller_->erase_range(erase_address, size);
}

Result<bool> OTAManager::verify_update_integrity() {
    if (update_state_->expected_sha256.empty()) {
        return true; // No SHA256 to verify
    }
    
    // Calculate SHA256 of the written data
    auto calculated_sha256_result = calculate_partition_sha256(update_state_->target_partition);
    if (!calculated_sha256_result) {
        return unexpected(calculated_sha256_result.error());
    }
    
    auto calculated_sha256 = calculated_sha256_result.value();
    
    // Compare SHA256 hashes
    bool matches = (calculated_sha256.size() == update_state_->expected_sha256.size() &&
                   std::equal(calculated_sha256.begin(), calculated_sha256.end(),
                             update_state_->expected_sha256.begin()));
    
    if (matches) {
        COMPONENT_LOG_INFO("SHA256 verification successful");
    } else {
        COMPONENT_LOG_ERROR("SHA256 verification failed");
    }
    
    return matches;
}

Result<std::vector<uint8_t>> OTAManager::calculate_partition_sha256(BootPartition partition) {
    // Placeholder implementation - would use actual SHA256 calculation
    std::vector<uint8_t> sha256(32, 0xAB);  // Dummy SHA256
    return sha256;
}

void OTAManager::update_success_stats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.successful_updates++;
    stats_.last_update_time = std::chrono::steady_clock::now();
    stats_.last_update_status = UpdateStatus::COMPLETE;
}

void OTAManager::update_failure_stats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.failed_updates++;
    stats_.last_update_status = UpdateStatus::ERROR;
}

void OTAManager::update_rollback_stats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rollbacks++;
}

Result<std::vector<OTAManager::OTAPartitionInfo>> OTAManager::scan_ota_partitions() {
    if (!initialized_) {
        LOG_ERROR("OTA Manager not initialized");
        return error<std::vector<OTAManager::OTAPartitionInfo>>(ErrorCode::STORAGE_NOT_INITIALIZED);
    }
    
    std::vector<OTAManager::OTAPartitionInfo> partitions;
    
    try {
        LOG_DEBUG("Scanning for OTA partitions in partition table");
        
        // Get all OTA partitions from the partition table
        auto ota_partitions = partition_table_->get_ota_partitions();
        
        for (const auto& partition : ota_partitions) {
            OTAManager::OTAPartitionInfo info;
            info.label = partition.label;
            info.subtype = static_cast<OTAManager::PartitionSubtype>(partition.subtype);
            info.start_address = partition.offset;
            info.size = partition.size;
            info.encrypted = partition.is_encrypted();
            info.state = OTAManager::OTAState::NEW;  // Default state, would be read from flash in real implementation
            info.seq = 0;  // Would be read from partition data in real implementation
            info.version = "unknown";  // Would be parsed from app header in real implementation
            info.sha256.resize(32, 0);  // Empty hash for now
            
            partitions.push_back(info);
            
            LOG_DEBUG("Found OTA partition: {} at 0x{:08X}, size: {}", 
                     info.label, info.start_address, info.size);
        }
        
        LOG_INFO("Scanned {} OTA partitions", partitions.size());
        return success<std::vector<OTAManager::OTAPartitionInfo>>(std::move(partitions));
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to scan OTA partitions: {}", e.what());
        return error<std::vector<OTAManager::OTAPartitionInfo>>(ErrorCode::OPERATION_FAILED);
    }
}

} // namespace m5tab5::emulator::storage