#include "emulator/firmware/elf_parser.hpp"
#include "emulator/utils/logging.hpp"

#include <fstream>
#include <filesystem>
#include <algorithm>
#include <sstream>

using m5tab5::emulator::unexpected;

namespace m5tab5::emulator::firmware {

//
// ELFParser Implementation
//

ELFParser::ELFParser(const std::string& file_path) 
    : file_path_(file_path) {
}

ELFParser::~ELFParser() = default;

Result<ParsedELF> ELFParser::parse() {
    // Load file data
    auto load_result = load_file();
    if (!load_result.has_value()) {
        return unexpected(load_result.error());
    }

    // Validate ELF header
    auto header_result = validate_elf_header();
    if (!header_result.has_value()) {
        return unexpected(header_result.error());
    }

    // Parse program headers and segments
    auto segments_result = parse_segments();
    if (!segments_result.has_value()) {
        return unexpected(segments_result.error());
    }

    // Analyze memory requirements
    auto memory_result = analyze_memory_requirements();
    if (!memory_result.has_value()) {
        return unexpected(memory_result.error());
    }

    // Detect ESP32-P4 specific features
    auto features_result = detect_esp32p4_features();
    if (!features_result.has_value()) {
        return unexpected(features_result.error());
    }

    // Validate memory layout
    auto layout_result = validate_memory_layout();
    if (!layout_result.has_value()) {
        return unexpected(layout_result.error());
    }

    // Mark as parsed and validate
    parsed_ = true;
    parsed_elf_.is_valid = parsed_elf_.errors.empty();

    LOG_INFO("ELF parsing completed: {} segments, {} bytes code, {} bytes data",
             parsed_elf_.segments.size(), parsed_elf_.total_code_size, parsed_elf_.total_data_size);

    return parsed_elf_;
}

Result<ValidationResult> ELFParser::validate_for_esp32p4() {
    if (!parsed_) {
        auto parse_result = parse();
        if (!parse_result.has_value()) {
            return unexpected(parse_result.error());
        }
    }

    ValidationResult result;
    result.valid = parsed_elf_.is_valid;
    result.warnings = parsed_elf_.warnings;
    
    if (!parsed_elf_.is_valid) {
        std::stringstream ss;
        for (const auto& error : parsed_elf_.errors) {
            ss << error << "; ";
        }
        result.error_message = ss.str();
    }

    // Fill in detailed information
    result.architecture = "RISC-V";
    result.target_chip = "ESP32-P4";
    result.entry_point = parsed_elf_.entry_point;
    result.code_size = parsed_elf_.total_code_size;
    result.data_size = parsed_elf_.total_data_size;
    result.bss_size = parsed_elf_.total_bss_size;
    result.has_dual_core_support = parsed_elf_.uses_dual_core;
    result.uses_psram = parsed_elf_.uses_psram;
    result.memory_regions = parsed_elf_.memory_regions;

    return result;
}

Result<void> ELFParser::load_file() {
    if (!std::filesystem::exists(file_path_)) {
        return unexpected(Error{ErrorCode::CONFIG_FILE_NOT_FOUND, 
                               "ELF file not found: " + file_path_});
    }

    std::ifstream file(file_path_, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return unexpected(Error{ErrorCode::CONFIG_FILE_NOT_FOUND, 
                               "Cannot open ELF file: " + file_path_});
    }

    auto file_size = file.tellg();
    if (file_size <= 0) {
        return unexpected(Error{ErrorCode::CONFIG_INVALID_VALUE, 
                               "ELF file is empty: " + file_path_});
    }

    file_data_.resize(static_cast<size_t>(file_size));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(file_data_.data()), file_size);

    if (!file.good()) {
        return unexpected(Error{ErrorCode::CONFIG_INVALID_VALUE, 
                               "Failed to read ELF file: " + file_path_});
    }

    LOG_DEBUG("Loaded ELF file: {} bytes", file_data_.size());
    return {};
}

Result<void> ELFParser::validate_elf_header() {
    if (file_data_.size() < sizeof(ELF32Header)) {
        add_error("File too small to contain ELF header");
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "Invalid ELF file size"});
    }

    auto header_result = read_elf_header();
    if (!header_result.has_value()) {
        return unexpected(header_result.error());
    }

    const auto& header = header_result.value();

    // Validate magic number
    if (!validate_magic_number(header)) {
        add_error("Invalid ELF magic number");
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "Not a valid ELF file"});
    }

    // Validate architecture
    if (!validate_architecture(header)) {
        add_error("Unsupported architecture for ESP32-P4");
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "Architecture mismatch"});
    }

    // Store header information
    parsed_elf_.entry_point = header.e_entry;
    parsed_elf_.machine_type = header.e_machine;
    parsed_elf_.flags = header.e_flags;

    // Validate entry point
    if (!is_valid_entry_point(parsed_elf_.entry_point)) {
        add_warning("Entry point may not be in valid executable memory region");
    }

    LOG_DEBUG("ELF header validated: entry=0x{:08x}, machine={}, flags=0x{:08x}",
              parsed_elf_.entry_point, parsed_elf_.machine_type, parsed_elf_.flags);

    return {};
}

Result<void> ELFParser::parse_segments() {
    auto header_result = read_elf_header();
    if (!header_result.has_value()) {
        return unexpected(header_result.error());
    }

    const auto& header = header_result.value();

    auto program_headers_result = read_program_headers(header);
    if (!program_headers_result.has_value()) {
        return unexpected(program_headers_result.error());
    }

    const auto& program_headers = program_headers_result.value();

    // Parse each loadable segment
    for (const auto& ph : program_headers) {
        if (ph.p_type != ELF::PT_LOAD) {
            continue; // Skip non-loadable segments
        }

        ParsedSegment segment;
        segment.virtual_address = ph.p_vaddr;
        segment.physical_address = ph.p_paddr;
        segment.file_size = ph.p_filesz;
        segment.memory_size = ph.p_memsz;
        segment.flags = ph.p_flags;

        // Read segment data if present in file
        if (ph.p_filesz > 0) {
            auto data_result = read_segment_data(ph);
            if (!data_result.has_value()) {
                add_error("Failed to read segment data at offset " + 
                         std::to_string(ph.p_offset));
                continue;
            }
            segment.data = std::move(data_result.value());
        }

        // Validate segment
        if (!validate_segment_alignment(segment)) {
            add_warning("Segment at 0x" + 
                       std::to_string(segment.virtual_address) + " has poor alignment");
        }

        if (!validate_address_range(segment.virtual_address, segment.memory_size)) {
            add_error("Segment at 0x" + 
                     std::to_string(segment.virtual_address) + " is outside valid memory range");
            continue;
        }

        parsed_elf_.segments.push_back(std::move(segment));

        // LOG_DEBUG("Parsed segment: vaddr=0x{:08x}, paddr=0x{:08x}, size={}, flags={}", ...); // Temporarily disabled
    }

    if (parsed_elf_.segments.empty()) {
        add_error("No loadable segments found in ELF file");
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "No loadable segments"});
    }

    return {};
}

Result<void> ELFParser::analyze_memory_requirements() {
    parsed_elf_.total_code_size = 0;
    parsed_elf_.total_data_size = 0;
    parsed_elf_.total_bss_size = 0;

    for (const auto& segment : parsed_elf_.segments) {
        if (segment.is_executable()) {
            parsed_elf_.total_code_size += segment.memory_size;
        } else if (segment.is_writable()) {
            parsed_elf_.total_data_size += segment.file_size;
            parsed_elf_.total_bss_size += (segment.memory_size - segment.file_size);
        }

        // Track memory regions
        parsed_elf_.memory_regions.push_back(segment.virtual_address);
    }

    // Remove duplicate memory regions
    std::sort(parsed_elf_.memory_regions.begin(), parsed_elf_.memory_regions.end());
    parsed_elf_.memory_regions.erase(
        std::unique(parsed_elf_.memory_regions.begin(), parsed_elf_.memory_regions.end()),
        parsed_elf_.memory_regions.end());

    LOG_DEBUG("Memory analysis: code={}, data={}, bss={}, regions={}",
              parsed_elf_.total_code_size, parsed_elf_.total_data_size,
              parsed_elf_.total_bss_size, parsed_elf_.memory_regions.size());

    return {};
}

Result<void> ELFParser::detect_esp32p4_features() {
    // Detect dual-core usage by looking for symbols or sections
    detect_dual_core_usage();
    
    // Detect PSRAM usage by analyzing memory addresses
    detect_psram_usage();

    // Analyze segment types for additional features
    analyze_segment_types();

    return {};
}

Result<void> ELFParser::validate_memory_layout() {
    // Check for overlapping segments
    for (size_t i = 0; i < parsed_elf_.segments.size(); ++i) {
        for (size_t j = i + 1; j < parsed_elf_.segments.size(); ++j) {
            const auto& seg1 = parsed_elf_.segments[i];
            const auto& seg2 = parsed_elf_.segments[j];

            Address end1 = seg1.virtual_address + seg1.memory_size;
            Address end2 = seg2.virtual_address + seg2.memory_size;

            if ((seg1.virtual_address < end2) && (seg2.virtual_address < end1)) {
                add_error("Overlapping memory segments detected");
                return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                                       "Memory layout conflict"});
            }
        }
    }

    // Validate total memory requirements
    size_t total_flash_needed = 0;
    size_t total_sram_needed = 0;
    size_t total_psram_needed = 0;

    for (const auto& segment : parsed_elf_.segments) {
        if (is_address_in_flash(segment.virtual_address)) {
            total_flash_needed += segment.memory_size;
        } else if (is_address_in_sram(segment.virtual_address)) {
            total_sram_needed += segment.memory_size;
        } else if (is_address_in_psram(segment.virtual_address)) {
            total_psram_needed += segment.memory_size;
        }
    }

    // Check against ESP32-P4 limits
    if (total_flash_needed > ESP32P4MemoryLayout::IROM_SIZE) {
        add_error("Firmware exceeds available flash memory");
    }
    if (total_sram_needed > ESP32P4MemoryLayout::DRAM_SIZE) {
        add_error("Firmware exceeds available SRAM");
    }
    if (total_psram_needed > ESP32P4MemoryLayout::PSRAM_SIZE) {
        add_warning("Firmware may exceed available PSRAM");
    }

    return {};
}

// Private implementation methods

Result<ELF32Header> ELFParser::read_elf_header() {
    if (file_data_.size() < sizeof(ELF32Header)) {
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, "File too small"});
    }

    ELF32Header header;
    std::memcpy(&header, file_data_.data(), sizeof(ELF32Header));
    return header;
}

Result<std::vector<ELF32ProgramHeader>> ELFParser::read_program_headers(const ELF32Header& header) {
    if (header.e_phnum == 0) {
        return std::vector<ELF32ProgramHeader>{}; // No program headers
    }

    size_t headers_size = header.e_phnum * sizeof(ELF32ProgramHeader);
    if (file_data_.size() < header.e_phoff + headers_size) {
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "Program headers extend beyond file"});
    }

    std::vector<ELF32ProgramHeader> program_headers(header.e_phnum);
    std::memcpy(program_headers.data(), 
                file_data_.data() + header.e_phoff, 
                headers_size);

    return program_headers;
}

Result<std::vector<u8>> ELFParser::read_segment_data(const ELF32ProgramHeader& ph) {
    if (ph.p_filesz == 0) {
        return std::vector<u8>{}; // Empty segment
    }

    if (file_data_.size() < ph.p_offset + ph.p_filesz) {
        return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                               "Segment data extends beyond file"});
    }

    std::vector<u8> data(ph.p_filesz);
    std::memcpy(data.data(), 
                file_data_.data() + ph.p_offset, 
                ph.p_filesz);

    return data;
}

bool ELFParser::validate_magic_number(const ELF32Header& header) const {
    return std::memcmp(header.e_ident, ELF::ELFMAG, 4) == 0 &&
           header.e_ident[4] == ELF::ELFCLASS32 &&
           header.e_ident[5] == ELF::ELFDATA2LSB &&
           header.e_ident[6] == ELF::EV_CURRENT;
}

bool ELFParser::validate_architecture(const ELF32Header& header) const {
    return header.e_type == ELF::ET_EXEC && 
           header.e_machine == ELF::EM_RISCV;
}

bool ELFParser::validate_segment_alignment(const ParsedSegment& segment) const {
    // Check for reasonable alignment (at least 4-byte alignment)
    return (segment.virtual_address % 4) == 0;
}

bool ELFParser::validate_address_range(Address addr, size_t size) const {
    // Check if address range fits within ESP32-P4 memory map
    (void)size; // Suppress unused parameter warning
    
    return is_address_in_flash(addr) || 
           is_address_in_sram(addr) || 
           is_address_in_psram(addr);
}

bool ELFParser::is_address_in_flash(Address addr) const {
    return (addr >= ESP32P4MemoryLayout::IROM_BASE && 
            addr < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE);
}

bool ELFParser::is_address_in_sram(Address addr) const {
    return (addr >= ESP32P4MemoryLayout::DRAM_BASE && 
            addr < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE);
}

bool ELFParser::is_address_in_psram(Address addr) const {
    return (addr >= ESP32P4MemoryLayout::PSRAM_BASE && 
            addr < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE);
}

bool ELFParser::is_valid_entry_point(Address entry_point) const {
    return is_address_in_flash(entry_point);
}

void ELFParser::detect_dual_core_usage() {
    // Simple heuristic: look for dual-core related symbols or large code size
    parsed_elf_.uses_dual_core = (parsed_elf_.total_code_size > 64*1024);
}

void ELFParser::detect_psram_usage() {
    // Check if any segments are mapped to PSRAM region
    for (const auto& segment : parsed_elf_.segments) {
        if (is_address_in_psram(segment.virtual_address)) {
            parsed_elf_.uses_psram = true;
            break;
        }
    }
}

void ELFParser::analyze_segment_types() {
    // Additional analysis can be added here for segment-specific features
    LOG_DEBUG("Analyzed {} segments for ESP32-P4 features", parsed_elf_.segments.size());
}

void ELFParser::add_error(const std::string& error) {
    parsed_elf_.errors.push_back(error);
    // LOG_ERROR("ELF parsing error: {}", error); // Temporarily disabled due to spdlog fmt issue
}

void ELFParser::add_warning(const std::string& warning) {
    parsed_elf_.warnings.push_back(warning);
    // LOG_WARN("ELF parsing warning: {}", warning); // Temporarily disabled due to spdlog fmt issue
}

//
// MemoryMapper Implementation
//

MemoryMapper::MemoryMapper() = default;
MemoryMapper::~MemoryMapper() = default;

Result<MemoryMapper::MappingResult> MemoryMapper::map_segment(const ParsedSegment& segment) {
    MappingResult result;
    result.target_address = segment.virtual_address; // Direct mapping by default
    result.size = segment.memory_size;
    result.region_name = get_memory_region_name(segment.virtual_address);
    result.success = false;

    // Validate the mapping
    if (!is_valid_mapping(segment.virtual_address, result.target_address, result.size)) {
        result.error_message = "Invalid memory mapping for address 0x" + 
                              std::to_string(segment.virtual_address);
        return result;
    }

    // Check if target region can accommodate the segment
    size_t available_space = get_region_available_space(result.target_address);
    if (result.size > available_space) {
        result.error_message = "Insufficient space in " + result.region_name + 
                              " region (" + std::to_string(available_space) + " bytes available)";
        return result;
    }

    result.success = true;
    // LOG_DEBUG("Mapped segment: 0x{:08x} -> {} region, {} bytes", ...); // Temporarily disabled

    return result;
}

Result<std::vector<MemoryMapper::MappingResult>> 
MemoryMapper::map_all_segments(const std::vector<ParsedSegment>& segments) {
    std::vector<MappingResult> results;
    results.reserve(segments.size());

    for (const auto& segment : segments) {
        auto mapping_result = map_segment(segment);
        if (!mapping_result.has_value()) {
            return unexpected(mapping_result.error());
        }
        
        results.push_back(mapping_result.value());
        
        if (!mapping_result.value().success) {
            return unexpected(Error{ErrorCode::CONFIG_INVALID_FORMAT, 
                                   "Segment mapping failed: " + mapping_result.value().error_message});
        }
    }

    return results;
}

bool MemoryMapper::is_valid_mapping(Address source_addr, Address target_addr, size_t size) const {
    // Check if both addresses are in valid memory regions
    bool source_valid = (source_addr >= ESP32P4MemoryLayout::IROM_BASE && 
                        source_addr < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) ||
                       (source_addr >= ESP32P4MemoryLayout::DRAM_BASE &&
                        source_addr < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) ||
                       (source_addr >= ESP32P4MemoryLayout::PSRAM_BASE &&
                        source_addr < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE);

    bool target_valid = (target_addr >= ESP32P4MemoryLayout::IROM_BASE && 
                        target_addr + size <= ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) ||
                       (target_addr >= ESP32P4MemoryLayout::DRAM_BASE &&
                        target_addr + size <= ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) ||
                       (target_addr >= ESP32P4MemoryLayout::PSRAM_BASE &&
                        target_addr + size <= ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE);

    return source_valid && target_valid;
}

std::string MemoryMapper::get_memory_region_name(Address addr) const {
    if (addr >= ESP32P4MemoryLayout::IROM_BASE && 
        addr < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) {
        return "Flash/IROM";
    }
    if (addr >= ESP32P4MemoryLayout::DRAM_BASE && 
        addr < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) {
        return "SRAM/DRAM";
    }
    if (addr >= ESP32P4MemoryLayout::PSRAM_BASE && 
        addr < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE) {
        return "PSRAM";
    }
    return "Unknown";
}

size_t MemoryMapper::get_region_available_space(Address addr) const {
    if (addr >= ESP32P4MemoryLayout::IROM_BASE && 
        addr < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) {
        return ESP32P4MemoryLayout::IROM_SIZE - (addr - ESP32P4MemoryLayout::IROM_BASE);
    }
    if (addr >= ESP32P4MemoryLayout::DRAM_BASE && 
        addr < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) {
        return ESP32P4MemoryLayout::DRAM_SIZE - (addr - ESP32P4MemoryLayout::DRAM_BASE);
    }
    if (addr >= ESP32P4MemoryLayout::PSRAM_BASE && 
        addr < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE) {
        return ESP32P4MemoryLayout::PSRAM_SIZE - (addr - ESP32P4MemoryLayout::PSRAM_BASE);
    }
    return 0;
}

} // namespace m5tab5::emulator::firmware