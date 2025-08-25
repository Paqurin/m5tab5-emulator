#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/firmware/firmware_loader.hpp"

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <cstdint>

namespace m5tab5::emulator::firmware {

// Forward declaration from firmware_loader.hpp
struct MappingResult;

/**
 * @brief ELF file format constants for ESP32-P4
 */
namespace ELF {
    // ELF header constants
    static constexpr u8 ELFMAG[4] = {0x7f, 'E', 'L', 'F'};
    static constexpr u8 ELFCLASS32 = 1;
    static constexpr u8 ELFDATA2LSB = 1;
    static constexpr u8 EV_CURRENT = 1;
    static constexpr u16 ET_EXEC = 2;
    static constexpr u16 EM_RISCV = 243;  // RISC-V architecture

    // Program header types
    static constexpr u32 PT_NULL = 0;
    static constexpr u32 PT_LOAD = 1;
    static constexpr u32 PT_DYNAMIC = 2;
    static constexpr u32 PT_INTERP = 3;
    static constexpr u32 PT_NOTE = 4;

    // Program header flags
    static constexpr u32 PF_X = 1;  // Execute
    static constexpr u32 PF_W = 2;  // Write
    static constexpr u32 PF_R = 4;  // Read
}

/**
 * @brief ELF32 file header structure
 */
#pragma pack(push, 1)
struct ELF32Header {
    u8 e_ident[16];     // ELF identification
    u16 e_type;         // Object file type
    u16 e_machine;      // Machine type
    u32 e_version;      // Object file version
    u32 e_entry;        // Entry point address
    u32 e_phoff;        // Program header offset
    u32 e_shoff;        // Section header offset
    u32 e_flags;        // Processor-specific flags
    u16 e_ehsize;       // ELF header size
    u16 e_phentsize;    // Program header entry size
    u16 e_phnum;        // Number of program header entries
    u16 e_shentsize;    // Section header entry size
    u16 e_shnum;        // Number of section header entries
    u16 e_shstrndx;     // Section header string table index
};

/**
 * @brief ELF32 program header structure
 */
struct ELF32ProgramHeader {
    u32 p_type;         // Segment type
    u32 p_offset;       // Segment file offset
    u32 p_vaddr;        // Segment virtual address
    u32 p_paddr;        // Segment physical address
    u32 p_filesz;       // Segment size in file
    u32 p_memsz;        // Segment size in memory
    u32 p_flags;        // Segment flags
    u32 p_align;        // Segment alignment
};

/**
 * @brief ELF32 section header structure
 */
struct ELF32SectionHeader {
    u32 sh_name;        // Section name (string table index)
    u32 sh_type;        // Section type
    u32 sh_flags;       // Section flags
    u32 sh_addr;        // Section virtual address
    u32 sh_offset;      // Section file offset
    u32 sh_size;        // Section size
    u32 sh_link;        // Link to another section
    u32 sh_info;        // Additional section information
    u32 sh_addralign;   // Section alignment
    u32 sh_entsize;     // Entry size if section holds table
};
#pragma pack(pop)

/**
 * @brief Parsed ELF segment information
 */
struct ParsedSegment {
    Address virtual_address;
    Address physical_address;
    size_t file_size;
    size_t memory_size;
    u32 flags;
    std::vector<u8> data;
    
    bool is_executable() const { return (flags & ELF::PF_X) != 0; }
    bool is_writable() const { return (flags & ELF::PF_W) != 0; }
    bool is_readable() const { return (flags & ELF::PF_R) != 0; }
    
    std::string get_permissions() const {
        std::string perms;
        perms += is_readable() ? 'R' : '-';
        perms += is_writable() ? 'W' : '-';
        perms += is_executable() ? 'X' : '-';
        return perms;
    }
};

/**
 * @brief Parsed ELF file information
 */
struct ParsedELF {
    // Header information
    Address entry_point;
    u16 machine_type;
    u32 flags;
    
    // Segments
    std::vector<ParsedSegment> segments;
    
    // Statistics
    size_t total_code_size = 0;
    size_t total_data_size = 0;
    size_t total_bss_size = 0;
    
    // ESP32-P4 specific analysis
    bool uses_dual_core = false;
    bool uses_psram = false;
    std::vector<Address> memory_regions;
    
    // Validation results
    bool is_valid = false;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
};

/**
 * @brief Professional ELF parser for ESP32-P4 firmware
 * 
 * Features:
 * - Validates ELF format and ESP32-P4 compatibility
 * - Parses program headers and segments
 * - Extracts memory layout and requirements
 * - Performs security and integrity checks
 * - Provides detailed error reporting
 */
class ELFParser {
public:
    explicit ELFParser(const std::string& file_path);
    ~ELFParser();

    // Core parsing operations
    Result<ParsedELF> parse();
    Result<ValidationResult> validate_for_esp32p4();
    
    // File operations
    Result<void> load_file();
    Result<void> validate_elf_header();
    Result<void> parse_program_headers();
    Result<void> parse_segments();
    
    // Analysis operations
    Result<void> analyze_memory_requirements();
    Result<void> detect_esp32p4_features();
    Result<void> validate_memory_layout();
    
    // Accessors
    const ParsedELF& get_parsed_elf() const { return parsed_elf_; }
    size_t get_file_size() const { return file_data_.size(); }
    bool is_parsed() const { return parsed_; }

private:
    std::string file_path_;
    std::vector<u8> file_data_;
    ParsedELF parsed_elf_;
    bool parsed_ = false;

    // Internal parsing methods
    Result<ELF32Header> read_elf_header();
    Result<std::vector<ELF32ProgramHeader>> read_program_headers(const ELF32Header& header);
    Result<std::vector<u8>> read_segment_data(const ELF32ProgramHeader& ph);
    
    // Validation methods
    bool validate_magic_number(const ELF32Header& header) const;
    bool validate_architecture(const ELF32Header& header) const;
    bool validate_segment_alignment(const ParsedSegment& segment) const;
    bool validate_address_range(Address addr, size_t size) const;
    
    // ESP32-P4 specific validation
    bool is_address_in_flash(Address addr) const;
    bool is_address_in_sram(Address addr) const;
    bool is_address_in_psram(Address addr) const;
    bool is_valid_entry_point(Address entry_point) const;
    
    // Analysis helpers
    void analyze_segment_types();
    void detect_dual_core_usage();
    void detect_psram_usage();
    void calculate_memory_statistics();
    
    // Error reporting
    void add_error(const std::string& error);
    void add_warning(const std::string& warning);
};

/**
 * @brief Memory segment mapper for ESP32-P4 memory layout
 * 
 * Maps ELF segments to appropriate ESP32-P4 memory regions:
 * - Flash (0x40000000): Executable code
 * - SRAM (0x4FF00000): Data and BSS
 * - PSRAM (0x48000000): Large data buffers
 */
class MemoryMapper {
public:
    using MappingResult = ::m5tab5::emulator::firmware::MappingResult;

    MemoryMapper();
    ~MemoryMapper();

    // Mapping operations
    Result<MappingResult> map_segment(const ParsedSegment& segment);
    Result<std::vector<MappingResult>> map_all_segments(const std::vector<ParsedSegment>& segments);
    
    // Memory region validation
    bool is_valid_mapping(Address source_addr, Address target_addr, size_t size) const;
    std::string get_memory_region_name(Address addr) const;
    
    // Configuration
    void set_strict_mapping(bool strict) { strict_mapping_ = strict; }
    void set_allow_psram(bool allow) { allow_psram_ = allow; }

private:
    bool strict_mapping_ = true;
    bool allow_psram_ = true;

    // Mapping logic
    Address find_best_mapping_address(const ParsedSegment& segment) const;
    bool check_address_conflicts(Address addr, size_t size, 
                                const std::vector<MappingResult>& existing_mappings) const;
    
    // ESP32-P4 memory layout helpers
    bool is_executable_region(Address addr) const;
    bool is_writable_region(Address addr) const;
    size_t get_region_available_space(Address addr) const;
};

} // namespace m5tab5::emulator::firmware