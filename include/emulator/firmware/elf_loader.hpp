#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/firmware/firmware_loader.hpp"

// Need to include elf_parser.hpp for complete types
#include "emulator/firmware/elf_parser.hpp"

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

namespace m5tab5::emulator {
    // Forward declarations
    class MemoryController;
    class DualCoreManager;
    class BootROM;
}

namespace m5tab5::emulator::firmware {

/**
 * @brief RISC-V relocation types for ESP32-P4
 */
enum class RiscVRelocationType : u32 {
    R_RISCV_NONE = 0,
    R_RISCV_32 = 1,
    R_RISCV_64 = 2,
    R_RISCV_RELATIVE = 3,
    R_RISCV_COPY = 4,
    R_RISCV_JUMP_SLOT = 5,
    R_RISCV_TLS_DTPMOD32 = 6,
    R_RISCV_TLS_DTPMOD64 = 7,
    R_RISCV_TLS_DTPREL32 = 8,
    R_RISCV_TLS_DTPREL64 = 9,
    R_RISCV_TLS_TPREL32 = 10,
    R_RISCV_TLS_TPREL64 = 11,
    R_RISCV_BRANCH = 16,
    R_RISCV_JAL = 17,
    R_RISCV_CALL = 18,
    R_RISCV_CALL_PLT = 19,
    R_RISCV_GOT_HI20 = 20,
    R_RISCV_TLS_GOT_HI20 = 21,
    R_RISCV_TLS_GD_HI20 = 22,
    R_RISCV_PCREL_HI20 = 23,
    R_RISCV_PCREL_LO12_I = 24,
    R_RISCV_PCREL_LO12_S = 25,
    R_RISCV_HI20 = 26,
    R_RISCV_LO12_I = 27,
    R_RISCV_LO12_S = 28,
    R_RISCV_TPREL_HI20 = 29,
    R_RISCV_TPREL_LO12_I = 30,
    R_RISCV_TPREL_LO12_S = 31,
    R_RISCV_TPREL_ADD = 32,
    R_RISCV_ADD8 = 33,
    R_RISCV_ADD16 = 34,
    R_RISCV_ADD32 = 35,
    R_RISCV_ADD64 = 36,
    R_RISCV_SUB8 = 37,
    R_RISCV_SUB16 = 38,
    R_RISCV_SUB32 = 39,
    R_RISCV_SUB64 = 40,
    R_RISCV_GNU_VTINHERIT = 41,
    R_RISCV_GNU_VTENTRY = 42,
    R_RISCV_ALIGN = 43,
    R_RISCV_RVC_BRANCH = 44,
    R_RISCV_RVC_JUMP = 45,
    R_RISCV_RVC_LUI = 46,
    R_RISCV_GPREL_I = 47,
    R_RISCV_GPREL_S = 48,
    R_RISCV_TPREL_I = 49,
    R_RISCV_TPREL_S = 50,
    R_RISCV_RELAX = 51,
    R_RISCV_SUB6 = 52,
    R_RISCV_SET6 = 53,
    R_RISCV_SET8 = 54,
    R_RISCV_SET16 = 55,
    R_RISCV_SET32 = 56,
    R_RISCV_32_PCREL = 57
};

// ELF32SectionHeader now available from elf_parser.hpp

/**
 * @brief ELF32 relocation entry
 */
#pragma pack(push, 1)
struct ELF32Rel {
    u32 r_offset;       // Offset of the relocation
    u32 r_info;         // Relocation type and symbol index
};

/**
 * @brief ELF32 relocation entry with addend
 */
struct ELF32Rela {
    u32 r_offset;       // Offset of the relocation
    u32 r_info;         // Relocation type and symbol index
    i32 r_addend;       // Addend
};

/**
 * @brief ELF32 symbol table entry
 */
struct ELF32Sym {
    u32 st_name;        // Symbol name (string table index)
    u32 st_value;       // Symbol value
    u32 st_size;        // Symbol size
    u8 st_info;         // Symbol type and binding
    u8 st_other;        // Symbol visibility
    u16 st_shndx;       // Section index
};

/**
 * @brief ESP-IDF application image header
 */
struct esp_image_header_t {
    u8 magic;           // Magic number
    u8 segment_count;   // Number of segments
    u8 spi_mode;        // SPI flash mode
    u8 spi_speed_size;  // SPI speed and flash size
    u32 entry_addr;     // Entry point address
    u8 wp_pin;          // WP pin when SPI pins set via efuse (read by ROM bootloader)
    u8 spi_pin_drv[3];  // Drive settings for SPI pins (read by ROM bootloader)
    u16 chip_id;        // Chip identification number
    u8 min_chip_rev;    // Minimum chip revision supported
    u8 reserved[8];     // Reserved bytes
    u8 hash_appended;   // If 1, a SHA256 digest is appended after the image
};

/**
 * @brief ESP-IDF application segment header
 */
struct esp_image_segment_header_t {
    u32 load_addr;      // Address where segment should be loaded
    u32 data_len;       // Length of data
};
#pragma pack(pop)

/**
 * @brief Parsed relocation entry
 */
struct ParsedRelocation {
    Address offset;
    RiscVRelocationType type;
    u32 symbol_index;
    i32 addend;
    
    // Helper methods
    bool needs_symbol_resolution() const;
    bool is_pc_relative() const;
    std::string get_type_name() const;
};

/**
 * @brief Parsed symbol entry
 */
struct ParsedSymbol {
    std::string name;
    Address value;
    u32 size;
    u8 info;
    u8 visibility;
    u16 section_index;
    
    // Helper methods
    bool is_function() const { return (info & 0xF) == 2; }
    bool is_object() const { return (info & 0xF) == 1; }
    bool is_global() const { return (info >> 4) == 1; }
    bool is_weak() const { return (info >> 4) == 2; }
    bool is_undefined() const { return section_index == 0; }
};

/**
 * @brief CPU execution context for application startup
 */
struct CpuExecutionContext {
    Address entry_point = 0;
    Address stack_pointer = 0;
    Address global_pointer = 0;
    Address thread_pointer = 0;
    
    // RISC-V register initial values
    std::array<u32, 32> registers{}; // x0-x31
    std::array<u32, 32> float_registers{}; // f0-f31
    
    // Control and status registers
    u32 mstatus = 0;
    u32 mie = 0;
    u32 mtvec = 0;
    u32 mscratch = 0;
    u32 mepc = 0;
    u32 mcause = 0;
    u32 mtval = 0;
    u32 mip = 0;
    
    // Validity
    bool valid = false;
};

/**
 * @brief Comprehensive ELF loader for ESP32-P4 applications
 * 
 * Provides complete ELF loading pipeline:
 * - RISC-V relocation processing
 * - Symbol resolution and patching
 * - Memory segment loading with protection
 * - CPU execution context setup
 * - Integration with Boot ROM
 * - ESP-IDF application image support
 */
class ELFLoader {
public:
    /**
     * @brief Loading progress callback
     * @param stage Current stage name
     * @param progress Progress (0.0-1.0)
     * @param message Progress message
     */
    using ProgressCallback = std::function<void(const std::string& stage, float progress, const std::string& message)>;

    /**
     * @brief ELF loading result
     */
    struct LoadingResult {
        bool success = false;
        std::string error_message;
        Address entry_point = 0;
        Address stack_top = 0;
        size_t total_size = 0;
        std::vector<std::string> warnings;
        CpuExecutionContext execution_context;
    };

    ELFLoader();
    ~ELFLoader();

    // Core lifecycle
    Result<void> initialize();
    void shutdown();

    // Integration points
    void set_memory_controller(std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller);
    void set_cpu_manager(std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager);
    void set_boot_rom(std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom);

    // Main ELF loading pipeline
    Result<LoadingResult> load_elf_application(const std::string& file_path, 
                                               ProgressCallback progress_callback = nullptr);

    // ESP-IDF application image support
    Result<LoadingResult> load_esp_application_image(const std::string& file_path,
                                                     ProgressCallback progress_callback = nullptr);

    // Boot ROM integration
    Result<void> integrate_with_boot_rom(Address boot_entry_point);

    // CPU execution context management
    Result<CpuExecutionContext> create_execution_context(const ParsedELF& parsed_elf);
    Result<void> setup_cpu_context(const CpuExecutionContext& context);

    // Memory segment operations
    Result<void> load_segments_to_memory(const std::vector<ParsedSegment>& segments,
                                         ProgressCallback progress_callback = nullptr);

    // RISC-V relocation processing
    Result<void> process_relocations(const ParsedELF& parsed_elf);

    // Symbol resolution
    Result<void> resolve_symbols();

    // Configuration
    void set_enable_relocations(bool enable) { enable_relocations_ = enable; }
    void set_enable_symbol_resolution(bool enable) { enable_symbol_resolution_ = enable; }
    void set_stack_size(size_t size) { stack_size_ = size; }
    void set_heap_size(size_t size) { heap_size_ = size; }

private:
    // Core components (injected dependencies)
    std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller_;
    std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager_;
    std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom_;

    // Configuration
    bool enable_relocations_ = true;
    bool enable_symbol_resolution_ = true;
    size_t stack_size_ = 64 * 1024; // 64KB default stack
    size_t heap_size_ = 256 * 1024; // 256KB default heap

    // Symbol table and relocation data
    std::vector<ParsedSymbol> symbol_table_;
    std::vector<ParsedRelocation> relocations_;
    std::unordered_map<std::string, Address> global_symbol_table_;

    // ESP32-P4 memory layout constants
    static constexpr Address DEFAULT_STACK_TOP = 0x4FFBFF00;    // Near top of SRAM
    static constexpr Address DEFAULT_HEAP_BASE = 0x48000000;    // PSRAM base
    static constexpr Address INTERRUPT_VECTOR_TABLE = 0x40000400; // Standard RISC-V location

    // ELF parsing and processing
    Result<ParsedELF> parse_elf_with_sections(const std::string& file_path);
    Result<std::vector<ELF32SectionHeader>> read_section_headers(const std::vector<u8>& file_data,
                                                                const ELF32Header& header);
    Result<std::vector<u8>> read_section_data(const std::vector<u8>& file_data,
                                              const ELF32SectionHeader& section);
    Result<std::string> read_string_table(const std::vector<u8>& file_data,
                                          const ELF32SectionHeader& string_table_section);

    // Symbol table processing
    Result<void> parse_symbol_table(const std::vector<u8>& file_data,
                                    const std::vector<ELF32SectionHeader>& sections);
    Result<void> parse_relocations(const std::vector<u8>& file_data,
                                   const std::vector<ELF32SectionHeader>& sections);

    // RISC-V relocation handlers
    Result<void> apply_relocation(const ParsedRelocation& relocation, Address base_address);
    Result<void> handle_r_riscv_32(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_pcrel_hi20(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_pcrel_lo12_i(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_pcrel_lo12_s(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_hi20(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_lo12_i(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_lo12_s(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_jal(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_branch(Address target_address, Address symbol_value, i32 addend);
    Result<void> handle_r_riscv_call(Address target_address, Address symbol_value, i32 addend);

    // Instruction patching utilities
    Result<u32> read_instruction(Address address);
    Result<void> write_instruction(Address address, u32 instruction);
    u32 encode_immediate_i(i32 immediate);
    u32 encode_immediate_s(i32 immediate);
    u32 encode_immediate_b(i32 immediate);
    u32 encode_immediate_u(i32 immediate);
    u32 encode_immediate_j(i32 immediate);

    // Memory layout and protection
    Result<void> setup_memory_regions(const std::vector<ParsedSegment>& segments);
    Result<void> setup_stack_region(Address stack_top, size_t stack_size);
    Result<void> setup_heap_region(Address heap_base, size_t heap_size);
    Result<void> configure_memory_protection(const ParsedSegment& segment);

    // CPU context initialization
    Result<void> initialize_registers(const CpuExecutionContext& context);
    Result<void> setup_interrupt_vector_table(Address ivt_address);
    Result<void> configure_control_status_registers(const CpuExecutionContext& context);

    // ESP-IDF application image processing
    Result<esp_image_header_t> parse_esp_image_header(const std::vector<u8>& file_data);
    Result<std::vector<ParsedSegment>> parse_esp_image_segments(const std::vector<u8>& file_data,
                                                                const esp_image_header_t& header);
    Result<bool> validate_esp_image_checksum(const std::vector<u8>& file_data);

    // Utilities
    bool is_valid_esp32p4_address(Address address, size_t size) const;
    std::string get_memory_region_name(Address address) const;
    void update_progress(ProgressCallback callback, const std::string& stage, 
                        float progress, const std::string& message);

    // Error handling
    void add_warning(const std::string& warning);
    std::vector<std::string> warnings_;
};

} // namespace m5tab5::emulator::firmware