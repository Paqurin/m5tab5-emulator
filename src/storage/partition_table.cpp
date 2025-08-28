#include "emulator/storage/partition_table.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"

#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cstdint>

namespace m5tab5::emulator::storage {

// Import error handling utilities
using m5tab5::emulator::success;
using m5tab5::emulator::error;
using m5tab5::emulator::ErrorCode;

// CRC32 lookup table for partition table validation
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

PartitionTable::PartitionTable() {
    initialize();
}

PartitionTable::~PartitionTable() = default;

Result<void> PartitionTable::initialize() {
    partitions_.clear();
    header_ = PartitionTableHeader{};
    initialized_ = true;
    return success();
}

Result<void> PartitionTable::clear() {
    partitions_.clear();
    header_.num_entries = 0;
    header_.crc32 = 0;
    return success();
}

Result<void> PartitionTable::load_from_flash(const uint8_t* flash_data, size_t flash_size) {
    if (flash_data == nullptr || flash_size < sizeof(PartitionTableHeader)) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // Parse header
    std::memcpy(&header_, flash_data, sizeof(PartitionTableHeader));
    
    // Validate magic number
    if (header_.magic != PARTITION_TABLE_MAGIC) {
        LOG_WARN("Invalid partition table magic: 0x{:04X} (expected 0x{:04X})", 
                 header_.magic, PARTITION_TABLE_MAGIC);
        return error(ErrorCode::INVALID_FILE_FORMAT);
    }
    
    // Clear existing partitions
    partitions_.clear();
    
    // Calculate expected table size
    size_t expected_size = sizeof(PartitionTableHeader) + (header_.num_entries * PARTITION_ENTRY_SIZE);
    if (flash_size < expected_size) {
        LOG_ERROR("Partition table size insufficient: {} < {}", flash_size, expected_size);
        return error(ErrorCode::BUFFER_TOO_SMALL);
    }
    
    // Parse partition entries
    const uint8_t* entry_data = flash_data + sizeof(PartitionTableHeader);
    
    for (uint32_t i = 0; i < header_.num_entries; ++i) {
        PartitionEntry entry;
        auto parse_result = parse_partition_entry(entry_data + (i * PARTITION_ENTRY_SIZE), entry);
        if (!parse_result.has_value()) {
            LOG_ERROR("Failed to parse partition entry {}: {}", i, static_cast<int>(parse_result.error().code()));
            return error<void>(parse_result.error().code());
        }
        
        partitions_.push_back(entry);
    }
    
    // Verify CRC32
    auto verify_result = verify_crc32();
    if (!verify_result.has_value() || !verify_result.value()) {
        LOG_WARN("Partition table CRC32 verification failed");
        return error(ErrorCode::CHECKSUM_MISMATCH);
    }
    
    // Validate partition table
    auto validate_result = validate();
    if (!validate_result.has_value()) {
        LOG_WARN("Partition table validation failed: {}", static_cast<int>(validate_result.error().code()));
        return error<void>(validate_result.error().code());
    }
    
    LOG_INFO("Loaded partition table with {} partitions", partitions_.size());
    return success();
}

Result<void> PartitionTable::save_to_flash(uint8_t* flash_data, size_t flash_size) {
    if (flash_data == nullptr) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // Update header
    header_.num_entries = static_cast<uint32_t>(partitions_.size());
    
    // Calculate required size
    size_t required_size = sizeof(PartitionTableHeader) + (partitions_.size() * PARTITION_ENTRY_SIZE);
    if (flash_size < required_size) {
        return error(ErrorCode::BUFFER_TOO_SMALL);
    }
    
    // Update CRC32
    auto crc_result = update_crc32();
    if (!crc_result.has_value()) {
        return error<void>(crc_result.error().code());
    }
    
    // Clear buffer
    std::memset(flash_data, 0xFF, flash_size);
    
    // Write header
    std::memcpy(flash_data, &header_, sizeof(PartitionTableHeader));
    
    // Write partition entries
    uint8_t* entry_data = flash_data + sizeof(PartitionTableHeader);
    
    for (size_t i = 0; i < partitions_.size(); ++i) {
        auto serialize_result = serialize_partition_entry(partitions_[i], entry_data + (i * PARTITION_ENTRY_SIZE));
        if (!serialize_result.has_value()) {
            return error<void>(serialize_result.error().code());
        }
    }
    
    LOG_DEBUG("Saved partition table with {} partitions", partitions_.size());
    return success();
}

Result<void> PartitionTable::add_partition(const PartitionEntry& partition) {
    // Validate partition
    auto validate_result = validate_partition(partition);
    if (!validate_result.has_value()) {
        return error<void>(validate_result.error().code());
    }
    
    // Check for duplicate labels
    for (const auto& existing : partitions_) {
        if (existing.label == partition.label) {
            LOG_ERROR("Partition label '{}' already exists", partition.label);
            return error(ErrorCode::DUPLICATE_ENTRY);
        }
    }
    
    // Check for overlaps
    auto overlap_result = check_partition_overlap(partition);
    if (!overlap_result.has_value()) {
        return error<void>(overlap_result.error().code());
    }
    
    // Add partition
    partitions_.push_back(partition);
    
    // Sort partitions by offset
    std::sort(partitions_.begin(), partitions_.end(), 
              [](const PartitionEntry& a, const PartitionEntry& b) {
                  return a.offset < b.offset;
              });
    
    LOG_DEBUG("Added partition '{}' at 0x{:08X}, size: {} KB", 
              partition.label, partition.offset, partition.size / 1024);
    
    return success();
}

Result<PartitionTable::PartitionEntry> PartitionTable::get_partition(const std::string& label) const {
    auto it = std::find_if(partitions_.begin(), partitions_.end(),
                          [&label](const PartitionEntry& p) {
                              return p.label == label;
                          });
    
    if (it == partitions_.end()) {
        return error<PartitionTable::PartitionEntry>(ErrorCode::NOT_FOUND);
    }
    
    return *it;
}

Result<PartitionTable::PartitionEntry> PartitionTable::find_partition_by_type(
    PartitionType type, PartitionSubtype subtype) const {
    
    auto it = std::find_if(partitions_.begin(), partitions_.end(),
                          [type, subtype](const PartitionEntry& p) {
                              return p.type == type && 
                                    (subtype == PartitionSubtype::CUSTOM_BASE || p.subtype == subtype);
                          });
    
    if (it == partitions_.end()) {
        return error<PartitionTable::PartitionEntry>(ErrorCode::NOT_FOUND);
    }
    
    return *it;
}

Result<PartitionTable::PartitionEntry> PartitionTable::find_partition_by_address(Address address) const {
    auto it = std::find_if(partitions_.begin(), partitions_.end(),
                          [address](const PartitionEntry& p) {
                              return address >= p.offset && address < p.end_offset();
                          });
    
    if (it == partitions_.end()) {
        return error<PartitionTable::PartitionEntry>(ErrorCode::NOT_FOUND);
    }
    
    return *it;
}

Result<void> PartitionTable::validate() const {
    if (partitions_.empty()) {
        return success();  // Empty table is valid
    }
    
    // Check each partition
    for (const auto& partition : partitions_) {
        auto validate_result = validate_partition(partition);
        if (!validate_result.has_value()) {
            return error<void>(validate_result.error().code());
        }
    }
    
    // Check for overlaps
    auto overlap_result = check_overlap();
    if (!overlap_result.has_value()) {
        return error(ErrorCode::INVALID_CONFIGURATION);
    }
    
    if (overlap_result.value()) {
        LOG_ERROR("Partition table contains overlapping partitions");
        return error(ErrorCode::INVALID_CONFIGURATION);
    }
    
    return success();
}

Result<void> PartitionTable::validate_partition(const PartitionEntry& partition) const {
    // Validate label
    auto label_result = validate_label(partition.label);
    if (!label_result.has_value()) {
        return error<void>(label_result.error().code());
    }
    
    // Validate address range
    auto addr_result = validate_address_range(partition.offset, partition.size);
    if (!addr_result.has_value()) {
        return error<void>(addr_result.error().code());
    }
    
    // Check alignment
    auto align_result = is_partition_aligned(partition);
    if (!align_result.has_value()) {
        return error(ErrorCode::ALIGNMENT_ERROR);
    }
    
    if (!align_result.value()) {
        LOG_WARN("Partition '{}' is not properly aligned (offset: 0x{:08X}, size: {})", 
                 partition.label, partition.offset, partition.size);
        // Warning only - not a critical error for emulation
    }
    
    return success();
}

Result<bool> PartitionTable::check_overlap() const {
    for (size_t i = 0; i < partitions_.size(); ++i) {
        for (size_t j = i + 1; j < partitions_.size(); ++j) {
            const auto& p1 = partitions_[i];
            const auto& p2 = partitions_[j];
            
            // Check if partitions overlap
            if (!(p1.end_offset() <= p2.offset || p2.end_offset() <= p1.offset)) {
                LOG_ERROR("Partition overlap detected: '{}' (0x{:08X}-0x{:08X}) overlaps with '{}' (0x{:08X}-0x{:08X})",
                          p1.label, p1.offset, p1.end_offset(),
                          p2.label, p2.offset, p2.end_offset());
                return true;
            }
        }
    }
    
    return false;
}

Result<PartitionTable> PartitionTable::create_default_table() {
    PartitionTable table;
    
    // NVS partition (20KB)
    PartitionEntry nvs;
    nvs.type = PartitionType::DATA;
    nvs.subtype = PartitionSubtype::DATA_NVS;
    nvs.offset = 0x9000;  // After partition table
    nvs.size = 0x5000;    // 20KB
    nvs.label = "nvs";
    nvs.flags = PartitionFlag::NONE;
    
    auto nvs_result = table.add_partition(nvs);
    if (!nvs_result.has_value()) {
        return error<PartitionTable>(nvs_result.error().code());
    }
    
    // PHY init data partition (4KB)
    PartitionEntry phy;
    phy.type = PartitionType::DATA;
    phy.subtype = PartitionSubtype::DATA_PHY;
    phy.offset = 0xf000;
    phy.size = 0x1000;    // 4KB
    phy.label = "phy_init";
    phy.flags = PartitionFlag::NONE;
    
    auto phy_result = table.add_partition(phy);
    if (!phy_result.has_value()) {
        return error<PartitionTable>(phy_result.error().code());
    }
    
    // Factory application partition (remaining space)
    PartitionEntry factory;
    factory.type = PartitionType::APP;
    factory.subtype = PartitionSubtype::APP_FACTORY;
    factory.offset = 0x10000;   // 64KB
    factory.size = 0xFF0000;    // ~16MB - 64KB
    factory.label = "factory";
    factory.flags = PartitionFlag::NONE;
    
    auto factory_result = table.add_partition(factory);
    if (!factory_result.has_value()) {
        return error<PartitionTable>(factory_result.error().code());
    }
    
    LOG_INFO("Created default partition table with {} partitions", table.get_partition_count());
    return table;
}

// Internal implementation methods

Result<void> PartitionTable::parse_partition_entry(const uint8_t* entry_data, PartitionEntry& partition) {
    if (entry_data == nullptr) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // ESP32 partition entry format (32 bytes):
    // Bytes 0-1: Magic (0xAA50)
    // Byte 2: Type
    // Byte 3: Subtype  
    // Bytes 4-7: Offset
    // Bytes 8-11: Size
    // Bytes 12-27: Label (16 bytes, null-terminated)
    // Bytes 28-31: Flags
    
    uint16_t entry_magic = *reinterpret_cast<const uint16_t*>(entry_data);
    if (entry_magic != 0x50AA) {  // Little-endian magic
        // Check if this is an empty entry
        bool is_empty = true;
        for (int i = 0; i < PARTITION_ENTRY_SIZE; ++i) {
            if (entry_data[i] != 0xFF) {
                is_empty = false;
                break;
            }
        }
        
        if (is_empty) {
            return error<void>(ErrorCode::END_OF_DATA);  // End of partition table
        }
        
        LOG_ERROR("Invalid partition entry magic: 0x{:04X}", entry_magic);
        return error(ErrorCode::INVALID_FILE_FORMAT);
    }
    
    partition.type = static_cast<PartitionType>(entry_data[2]);
    partition.subtype = static_cast<PartitionSubtype>(entry_data[3]);
    partition.offset = *reinterpret_cast<const uint32_t*>(entry_data + 4);
    partition.size = *reinterpret_cast<const uint32_t*>(entry_data + 8);
    
    // Extract label (ensure null termination)
    char label_buffer[17] = {0};
    std::memcpy(label_buffer, entry_data + 12, 16);
    label_buffer[16] = '\0';
    partition.label = std::string(label_buffer);
    
    partition.flags = static_cast<PartitionFlag>(*reinterpret_cast<const uint32_t*>(entry_data + 28));
    
    return success();
}

Result<void> PartitionTable::serialize_partition_entry(const PartitionEntry& partition, uint8_t* entry_data) const {
    if (entry_data == nullptr) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // Clear entry
    std::memset(entry_data, 0x00, PARTITION_ENTRY_SIZE);
    
    // Set magic
    *reinterpret_cast<uint16_t*>(entry_data) = 0x50AA;
    
    // Set partition data
    entry_data[2] = static_cast<uint8_t>(partition.type);
    entry_data[3] = static_cast<uint8_t>(partition.subtype);
    *reinterpret_cast<uint32_t*>(entry_data + 4) = partition.offset;
    *reinterpret_cast<uint32_t*>(entry_data + 8) = partition.size;
    
    // Set label (truncate if necessary)
    std::string label = partition.label.substr(0, 15);  // Max 15 chars + null
    std::memcpy(entry_data + 12, label.c_str(), std::min(label.length(), size_t(15)));
    
    // Set flags
    *reinterpret_cast<uint32_t*>(entry_data + 28) = static_cast<uint32_t>(partition.flags);
    
    return success();
}

uint32_t PartitionTable::calculate_crc32(const void* data, size_t size) const {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    
    for (size_t i = 0; i < size; ++i) {
        crc = crc32_table[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
    }
    
    return crc ^ 0xFFFFFFFF;
}

Result<uint32_t> PartitionTable::calculate_crc32() const {
    // Calculate CRC32 over partition entries only (not header)
    size_t data_size = partitions_.size() * PARTITION_ENTRY_SIZE;
    std::vector<uint8_t> data(data_size);
    
    for (size_t i = 0; i < partitions_.size(); ++i) {
        auto serialize_result = serialize_partition_entry(partitions_[i], data.data() + (i * PARTITION_ENTRY_SIZE));
        if (!serialize_result.has_value()) {
            return error<uint32_t>(serialize_result.error().code());
        }
    }
    
    return calculate_crc32(data.data(), data_size);
}

Result<void> PartitionTable::update_crc32() {
    auto crc_result = calculate_crc32();
    if (!crc_result.has_value()) {
        return error<void>(crc_result.error().code());
    }
    
    header_.crc32 = crc_result.value();
    return success();
}

Result<bool> PartitionTable::verify_crc32() const {
    auto crc_result = calculate_crc32();
    if (!crc_result.has_value()) {
        return error<bool>(crc_result.error().code());
    }
    
    return crc_result.value() == header_.crc32;
}

Result<void> PartitionTable::validate_label(const std::string& label) const {
    if (label.empty() || label.length() > 15) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // Check for invalid characters
    for (char c : label) {
        if (!std::isalnum(c) && c != '_' && c != '-') {
            return error(ErrorCode::INVALID_PARAMETER);
        }
    }
    
    return success();
}

Result<void> PartitionTable::validate_address_range(Address offset, size_t size) const {
    if (size == 0) {
        return error(ErrorCode::INVALID_PARAMETER);
    }
    
    // Check against flash size (16MB)
    if (offset + size > 0x1000000) {
        return error(ErrorCode::ADDRESS_OUT_OF_BOUNDS);
    }
    
    return success();
}

Result<void> PartitionTable::check_partition_overlap(const PartitionEntry& partition) const {
    for (const auto& existing : partitions_) {
        if (existing.label == partition.label) {
            continue;  // Skip self
        }
        
        // Check if partitions overlap
        if (!(existing.end_offset() <= partition.offset || partition.end_offset() <= existing.offset)) {
            LOG_ERROR("Partition '{}' would overlap with existing partition '{}'", 
                      partition.label, existing.label);
            return error(ErrorCode::ADDRESS_OVERLAP);
        }
    }
    
    return success();
}

Address PartitionTable::align_address(Address address, size_t alignment) {
    return (address + alignment - 1) & ~(alignment - 1);
}

size_t PartitionTable::align_size(size_t size, size_t alignment) {
    return (size + alignment - 1) & ~(alignment - 1);
}

bool PartitionTable::is_address_aligned(Address address, size_t alignment) {
    return (address & (alignment - 1)) == 0;
}

Result<bool> PartitionTable::is_partition_aligned(const PartitionEntry& partition) const {
    return is_address_aligned(partition.offset, 0x1000) && 
           is_address_aligned(partition.size, 0x1000);
}

void PartitionTable::dump_partition_table() const {
    LOG_INFO("=== Partition Table ===");
    LOG_INFO("Magic: 0x{:04X}, Version: {}, Entries: {}, CRC32: 0x{:08X}", 
             header_.magic, header_.version, header_.num_entries, header_.crc32);
    LOG_INFO("");
    
    for (const auto& partition : partitions_) {
        LOG_INFO("Partition: {}", partition.label);
        LOG_INFO("  Type: {} (0x{:02X}), Subtype: {} (0x{:02X})", 
                 partition_type_to_string(partition.type), static_cast<uint8_t>(partition.type),
                 partition_subtype_to_string(partition.type, partition.subtype), static_cast<uint8_t>(partition.subtype));
        LOG_INFO("  Offset: 0x{:08X}, Size: {} KB (0x{:08X})", 
                 partition.offset, partition.size / 1024, partition.size);
        LOG_INFO("  Flags: 0x{:08X}{}{}", 
                 static_cast<uint32_t>(partition.flags),
                 partition.is_encrypted() ? " [ENCRYPTED]" : "",
                 partition.is_readonly() ? " [READONLY]" : "");
        LOG_INFO("");
    }
}

std::string PartitionTable::partition_type_to_string(PartitionType type) {
    switch (type) {
        case PartitionType::APP: return "app";
        case PartitionType::DATA: return "data";
        default: return "custom";
    }
}

std::string PartitionTable::partition_subtype_to_string(PartitionType type, PartitionSubtype subtype) {
    if (type == PartitionType::APP) {
        switch (subtype) {
            case PartitionSubtype::APP_FACTORY: return "factory";
            case PartitionSubtype::APP_TEST: return "test";
            default:
                if (subtype >= PartitionSubtype::APP_OTA_MIN && subtype <= PartitionSubtype::APP_OTA_MAX) {
                    return "ota_" + std::to_string(static_cast<uint8_t>(subtype) - static_cast<uint8_t>(PartitionSubtype::APP_OTA_MIN));
                }
                return "unknown";
        }
    } else if (type == PartitionType::DATA) {
        switch (subtype) {
            case PartitionSubtype::DATA_OTA: return "ota";
            case PartitionSubtype::DATA_PHY: return "phy";
            case PartitionSubtype::DATA_NVS: return "nvs";
            case PartitionSubtype::DATA_COREDUMP: return "coredump";
            case PartitionSubtype::DATA_NVS_KEYS: return "nvs_keys";
            case PartitionSubtype::DATA_EFUSE_EM: return "efuse";
            case PartitionSubtype::DATA_SPIFFS: return "spiffs";
            case PartitionSubtype::DATA_LITTLEFS: return "littlefs";
            case PartitionSubtype::DATA_FAT: return "fat";
            default: return "undefined";
        }
    }
    
    return "custom";
}

// PartitionIterator implementation

PartitionIterator::PartitionIterator(const PartitionTable* table)
    : table_(table), current_index_(0) {
}

bool PartitionIterator::has_next() const {
    if (!table_) return false;
    
    auto partitions = table_->get_all_partitions();
    
    for (size_t i = current_index_; i < partitions.size(); ++i) {
        if (matches_filters(partitions[i])) {
            return true;
        }
    }
    
    return false;
}

Result<PartitionTable::PartitionEntry> PartitionIterator::next() {
    if (!table_) {
        return error<PartitionTable::PartitionEntry>(ErrorCode::INVALID_STATE);
    }
    
    auto partitions = table_->get_all_partitions();
    
    for (size_t i = current_index_; i < partitions.size(); ++i) {
        if (matches_filters(partitions[i])) {
            current_index_ = i + 1;
            return partitions[i];
        }
    }
    
    return error<PartitionTable::PartitionEntry>(ErrorCode::NOT_FOUND);
}

void PartitionIterator::reset() {
    current_index_ = 0;
}

bool PartitionIterator::matches_filters(const PartitionTable::PartitionEntry& partition) const {
    if (type_filter_ && partition.type != type_filter_.value()) {
        return false;
    }
    
    if (subtype_filter_ && partition.subtype != subtype_filter_.value()) {
        return false;
    }
    
    if (label_filter_ && partition.label != label_filter_.value()) {
        return false;
    }
    
    return true;
}

// Partition filtering methods
std::vector<PartitionTable::PartitionEntry> PartitionTable::get_app_partitions() const {
    std::vector<PartitionEntry> app_partitions;
    for (const auto& partition : partitions_) {
        if (partition.type == PartitionType::APP) {
            app_partitions.push_back(partition);
        }
    }
    return app_partitions;
}

std::vector<PartitionTable::PartitionEntry> PartitionTable::get_data_partitions() const {
    std::vector<PartitionEntry> data_partitions;
    for (const auto& partition : partitions_) {
        if (partition.type == PartitionType::DATA) {
            data_partitions.push_back(partition);
        }
    }
    return data_partitions;
}

std::vector<PartitionTable::PartitionEntry> PartitionTable::get_ota_partitions() const {
    std::vector<PartitionEntry> ota_partitions;
    for (const auto& partition : partitions_) {
        if (partition.type == PartitionType::APP) {
            // Check for OTA partition subtypes
            if (static_cast<uint8_t>(partition.subtype) >= static_cast<uint8_t>(PartitionSubtype::APP_OTA_0) && 
                static_cast<uint8_t>(partition.subtype) <= static_cast<uint8_t>(PartitionSubtype::APP_OTA_15)) {
                ota_partitions.push_back(partition);
            }
        }
    }
    return ota_partitions;
}

std::vector<std::string> PartitionTable::get_partition_labels() const {
    std::vector<std::string> labels;
    for (const auto& partition : partitions_) {
        labels.push_back(partition.label);
    }
    return labels;
}

} // namespace m5tab5::emulator::storage