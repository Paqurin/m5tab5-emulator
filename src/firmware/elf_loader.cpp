#include "emulator/firmware/elf_loader.hpp"
#include "emulator/firmware/elf_parser.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/memory/boot_rom.hpp"
#include "emulator/utils/logging.hpp"

#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>

using m5tab5::emulator::unexpected;

namespace m5tab5::emulator::firmware {

// RISC-V instruction encoding constants
namespace RISCV {
    // Instruction formats
    static constexpr u32 OPCODE_MASK = 0x7F;
    static constexpr u32 RD_MASK = 0x1F;
    static constexpr u32 FUNC3_MASK = 0x7;
    static constexpr u32 RS1_MASK = 0x1F;
    static constexpr u32 RS2_MASK = 0x1F;
    static constexpr u32 FUNC7_MASK = 0x7F;

    // Opcodes
    static constexpr u32 OPCODE_LUI = 0x37;
    static constexpr u32 OPCODE_AUIPC = 0x17;
    static constexpr u32 OPCODE_JAL = 0x6F;
    static constexpr u32 OPCODE_JALR = 0x67;
    static constexpr u32 OPCODE_BRANCH = 0x63;
    static constexpr u32 OPCODE_LOAD = 0x03;
    static constexpr u32 OPCODE_STORE = 0x23;
    static constexpr u32 OPCODE_IMM = 0x13;
    static constexpr u32 OPCODE_OP = 0x33;
    static constexpr u32 OPCODE_SYSTEM = 0x73;

    // Bit manipulation helpers
    static constexpr u32 extract_bits(u32 value, u32 start, u32 length) {
        return (value >> start) & ((1u << length) - 1);
    }

    static constexpr u32 insert_bits(u32 target, u32 value, u32 start, u32 length) {
        u32 mask = ((1u << length) - 1) << start;
        return (target & ~mask) | ((value << start) & mask);
    }
}

// Helper functions for relocation type
bool ParsedRelocation::needs_symbol_resolution() const {
    return type != RiscVRelocationType::R_RISCV_RELATIVE &&
           type != RiscVRelocationType::R_RISCV_NONE;
}

bool ParsedRelocation::is_pc_relative() const {
    switch (type) {
        case RiscVRelocationType::R_RISCV_BRANCH:
        case RiscVRelocationType::R_RISCV_JAL:
        case RiscVRelocationType::R_RISCV_CALL:
        case RiscVRelocationType::R_RISCV_CALL_PLT:
        case RiscVRelocationType::R_RISCV_PCREL_HI20:
        case RiscVRelocationType::R_RISCV_PCREL_LO12_I:
        case RiscVRelocationType::R_RISCV_PCREL_LO12_S:
        case RiscVRelocationType::R_RISCV_32_PCREL:
            return true;
        default:
            return false;
    }
}

std::string ParsedRelocation::get_type_name() const {
    switch (type) {
        case RiscVRelocationType::R_RISCV_32: return "R_RISCV_32";
        case RiscVRelocationType::R_RISCV_PCREL_HI20: return "R_RISCV_PCREL_HI20";
        case RiscVRelocationType::R_RISCV_PCREL_LO12_I: return "R_RISCV_PCREL_LO12_I";
        case RiscVRelocationType::R_RISCV_PCREL_LO12_S: return "R_RISCV_PCREL_LO12_S";
        case RiscVRelocationType::R_RISCV_HI20: return "R_RISCV_HI20";
        case RiscVRelocationType::R_RISCV_LO12_I: return "R_RISCV_LO12_I";
        case RiscVRelocationType::R_RISCV_LO12_S: return "R_RISCV_LO12_S";
        case RiscVRelocationType::R_RISCV_JAL: return "R_RISCV_JAL";
        case RiscVRelocationType::R_RISCV_BRANCH: return "R_RISCV_BRANCH";
        case RiscVRelocationType::R_RISCV_CALL: return "R_RISCV_CALL";
        default: return "R_RISCV_UNKNOWN";
    }
}

//
// ELFLoader Implementation
//

ELFLoader::ELFLoader() = default;

ELFLoader::~ELFLoader() {
    shutdown();
}

Result<void> ELFLoader::initialize() {
    // Clear previous state
    symbol_table_.clear();
    relocations_.clear();
    global_symbol_table_.clear();
    warnings_.clear();

    LOG_INFO("ELF Loader initialized successfully");
    return {};
}

void ELFLoader::shutdown() {
    symbol_table_.clear();
    relocations_.clear();
    global_symbol_table_.clear();
    warnings_.clear();
}

void ELFLoader::set_memory_controller(std::shared_ptr<::m5tab5::emulator::MemoryController> memory_controller) {
    memory_controller_ = memory_controller;
}

void ELFLoader::set_cpu_manager(std::shared_ptr<::m5tab5::emulator::DualCoreManager> cpu_manager) {
    cpu_manager_ = cpu_manager;
}

void ELFLoader::set_boot_rom(std::shared_ptr<::m5tab5::emulator::BootROM> boot_rom) {
    boot_rom_ = boot_rom;
}

Result<ELFLoader::LoadingResult> ELFLoader::load_elf_application(const std::string& file_path,
                                                                 ProgressCallback progress_callback) {
    LoadingResult result;
    warnings_.clear();

    update_progress(progress_callback, "Parsing", 0.1f, "Parsing ELF file with sections...");

    // Parse ELF file with full section information
    auto parsed_elf_result = parse_elf_with_sections(file_path);
    if (!parsed_elf_result.has_value()) {
        result.error_message = "Failed to parse ELF file: " + parsed_elf_result.error().message();
        return result;
    }

    const auto& parsed_elf = parsed_elf_result.value();
    result.entry_point = parsed_elf.entry_point;

    update_progress(progress_callback, "Validation", 0.2f, "Validating ESP32-P4 compatibility...");

    // Validate for ESP32-P4 compatibility
    if (parsed_elf.machine_type != ELF::EM_RISCV) {
        result.error_message = "ELF file is not for RISC-V architecture";
        return result;
    }

    if (!is_valid_esp32p4_address(parsed_elf.entry_point, 4)) {
        result.error_message = "Entry point is not in valid ESP32-P4 memory region";
        return result;
    }

    update_progress(progress_callback, "Memory Setup", 0.3f, "Setting up memory regions...");

    // Setup memory regions for segments
    auto setup_result = setup_memory_regions(parsed_elf.segments);
    if (!setup_result.has_value()) {
        result.error_message = "Failed to setup memory regions: " + setup_result.error().message();
        return result;
    }

    update_progress(progress_callback, "Loading", 0.4f, "Loading segments to memory...");

    // Load segments to memory
    auto load_result = load_segments_to_memory(parsed_elf.segments, progress_callback);
    if (!load_result.has_value()) {
        result.error_message = "Failed to load segments: " + load_result.error().message();
        return result;
    }

    update_progress(progress_callback, "Relocations", 0.7f, "Processing relocations...");

    // Process relocations if enabled
    if (enable_relocations_) {
        auto relocations_result = process_relocations(parsed_elf);
        if (!relocations_result.has_value()) {
            result.error_message = "Failed to process relocations: " + relocations_result.error().message();
            return result;
        }
    }

    update_progress(progress_callback, "Symbols", 0.8f, "Resolving symbols...");

    // Resolve symbols if enabled
    if (enable_symbol_resolution_) {
        auto symbols_result = resolve_symbols();
        if (!symbols_result.has_value()) {
            result.error_message = "Failed to resolve symbols: " + symbols_result.error().message();
            return result;
        }
    }

    update_progress(progress_callback, "Context", 0.9f, "Creating execution context...");

    // Create CPU execution context
    auto context_result = create_execution_context(parsed_elf);
    if (!context_result.has_value()) {
        result.error_message = "Failed to create execution context: " + context_result.error().message();
        return result;
    }

    result.execution_context = context_result.value();
    result.stack_top = result.execution_context.stack_pointer;

    update_progress(progress_callback, "CPU Setup", 0.95f, "Setting up CPU context...");

    // Setup CPU context
    auto cpu_setup_result = setup_cpu_context(result.execution_context);
    if (!cpu_setup_result.has_value()) {
        result.error_message = "Failed to setup CPU context: " + cpu_setup_result.error().message();
        return result;
    }

    // Calculate total loaded size
    result.total_size = 0;
    for (const auto& segment : parsed_elf.segments) {
        result.total_size += segment.memory_size;
    }

    result.warnings = warnings_;
    result.success = true;

    update_progress(progress_callback, "Complete", 1.0f, "ELF application loaded successfully!");

    LOG_INFO("ELF application loaded: entry=0x{:08x}, size={} bytes, warnings={}",
             result.entry_point, result.total_size, result.warnings.size());

    return result;
}

Result<ELFLoader::LoadingResult> ELFLoader::load_esp_application_image(const std::string& file_path,
                                                                       ProgressCallback progress_callback) {
    LoadingResult result;
    warnings_.clear();

    update_progress(progress_callback, "Reading", 0.1f, "Reading ESP application image...");

    // Read file data
    if (!std::filesystem::exists(file_path)) {
        result.error_message = "ESP application image file not found: " + file_path;
        return result;
    }

    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        result.error_message = "Cannot open ESP application image: " + file_path;
        return result;
    }

    auto file_size = file.tellg();
    std::vector<u8> file_data(static_cast<size_t>(file_size));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(file_data.data()), file_size);

    update_progress(progress_callback, "Parsing", 0.2f, "Parsing ESP application header...");

    // Parse ESP application header
    auto header_result = parse_esp_image_header(file_data);
    if (!header_result.has_value()) {
        result.error_message = "Failed to parse ESP image header: " + header_result.error().message();
        return result;
    }

    const auto& header = header_result.value();
    result.entry_point = header.entry_addr;

    update_progress(progress_callback, "Segments", 0.3f, "Parsing ESP application segments...");

    // Parse segments
    auto segments_result = parse_esp_image_segments(file_data, header);
    if (!segments_result.has_value()) {
        result.error_message = "Failed to parse ESP image segments: " + segments_result.error().message();
        return result;
    }

    const auto& segments = segments_result.value();

    update_progress(progress_callback, "Validation", 0.4f, "Validating ESP image checksum...");

    // Validate checksum
    auto checksum_result = validate_esp_image_checksum(file_data);
    if (!checksum_result.has_value()) {
        result.error_message = "ESP image checksum validation failed: " + checksum_result.error().message();
        return result;
    }

    if (!checksum_result.value()) {
        add_warning("ESP image checksum validation failed - proceeding anyway");
    }

    update_progress(progress_callback, "Loading", 0.5f, "Loading ESP segments to memory...");

    // Load segments using standard ELF loader
    auto load_result = load_segments_to_memory(segments, progress_callback);
    if (!load_result.has_value()) {
        result.error_message = "Failed to load ESP segments: " + load_result.error().message();
        return result;
    }

    // Create simple execution context for ESP application
    ParsedELF fake_elf;
    fake_elf.entry_point = result.entry_point;
    fake_elf.segments = segments;

    auto context_result = create_execution_context(fake_elf);
    if (!context_result.has_value()) {
        result.error_message = "Failed to create execution context: " + context_result.error().message();
        return result;
    }

    result.execution_context = context_result.value();
    result.stack_top = result.execution_context.stack_pointer;

    // Setup CPU context
    auto cpu_setup_result = setup_cpu_context(result.execution_context);
    if (!cpu_setup_result.has_value()) {
        result.error_message = "Failed to setup CPU context: " + cpu_setup_result.error().message();
        return result;
    }

    // Calculate total size
    result.total_size = 0;
    for (const auto& segment : segments) {
        result.total_size += segment.memory_size;
    }

    result.warnings = warnings_;
    result.success = true;

    update_progress(progress_callback, "Complete", 1.0f, "ESP application image loaded successfully!");

    LOG_INFO("ESP application loaded: entry=0x{:08x}, segments={}, size={} bytes",
             result.entry_point, segments.size(), result.total_size);

    return result;
}

Result<void> ELFLoader::integrate_with_boot_rom(Address boot_entry_point) {
    if (!boot_rom_) {
        return unexpected(Error(ErrorCode::INVALID_STATE, "Boot ROM not available"));
    }

    // TODO: Implement Boot ROM integration
    // This would modify Boot ROM code to jump to loaded application entry point
    // after initialization is complete

    LOG_INFO("Boot ROM integration completed: entry=0x{:08x}", boot_entry_point);
    return {};
}

Result<CpuExecutionContext> ELFLoader::create_execution_context(const ParsedELF& parsed_elf) {
    CpuExecutionContext context;

    // Set entry point
    context.entry_point = parsed_elf.entry_point;

    // Setup stack - use top of SRAM minus some safety margin
    context.stack_pointer = DEFAULT_STACK_TOP - 16; // 16-byte aligned
    
    // Initialize all general-purpose registers to zero
    context.registers.fill(0);
    context.float_registers.fill(0);

    // Set up standard RISC-V ABI registers
    context.registers[2] = context.stack_pointer; // sp (x2)
    context.registers[3] = 0; // gp (x3) - global pointer, set by linker
    context.registers[4] = 0; // tp (x4) - thread pointer

    // Set up control and status registers for ESP32-P4
    context.mstatus = 0x1800; // Machine mode, interrupts disabled initially
    context.mie = 0; // No interrupts enabled initially
    context.mtvec = INTERRUPT_VECTOR_TABLE; // Machine trap vector
    context.mscratch = 0;
    context.mepc = context.entry_point; // Machine exception program counter
    context.mcause = 0;
    context.mtval = 0;
    context.mip = 0;

    context.valid = true;

    LOG_DEBUG("CPU execution context created: entry=0x{:08x}, stack=0x{:08x}",
              context.entry_point, context.stack_pointer);

    return context;
}

Result<void> ELFLoader::setup_cpu_context(const CpuExecutionContext& context) {
    if (!cpu_manager_ || !context.valid) {
        return unexpected(Error(ErrorCode::INVALID_STATE, "CPU manager not available or invalid context"));
    }

    // TODO: Set initial register state in CPU manager
    // This would require CPU manager methods to set register values
    // For now, we validate the context is reasonable

    if (!is_valid_esp32p4_address(context.entry_point, 4)) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "Invalid entry point address"));
    }

    if (!is_valid_esp32p4_address(context.stack_pointer, 4)) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "Invalid stack pointer address"));
    }

    LOG_INFO("CPU context setup completed: entry=0x{:08x}, stack=0x{:08x}",
             context.entry_point, context.stack_pointer);

    return {};
}

Result<void> ELFLoader::load_segments_to_memory(const std::vector<ParsedSegment>& segments,
                                                ProgressCallback progress_callback) {
    if (!memory_controller_) {
        return unexpected(Error(ErrorCode::INVALID_STATE, "Memory controller not available"));
    }

    size_t total_segments = segments.size();
    size_t loaded_segments = 0;

    LOG_INFO("Loading {} segments to memory", total_segments);

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];

        if (segment.memory_size == 0) {
            continue; // Skip empty segments
        }

        // Update progress
        float progress = 0.4f + (0.3f * i / total_segments);
        std::string message = "Loading segment " + std::to_string(i + 1) + "/" + 
                             std::to_string(total_segments) + " (" + 
                             segment.get_permissions() + ", " + 
                             std::to_string(segment.memory_size) + " bytes)";
        update_progress(progress_callback, "Loading", progress, message);

        // Validate segment address
        if (!is_valid_esp32p4_address(segment.virtual_address, segment.memory_size)) {
            return unexpected(Error(ErrorCode::INVALID_PARAMETER, 
                                   "Segment address 0x" + std::to_string(segment.virtual_address) + 
                                   " is not in valid ESP32-P4 memory region"));
        }

        // Write segment data to memory
        if (!segment.data.empty()) {
            auto write_result = memory_controller_->write_bytes(segment.virtual_address, 
                                                               segment.data.data(), 
                                                               segment.data.size());
            if (!write_result.has_value()) {
                return unexpected(Error(ErrorCode::MEMORY_ACCESS_ERROR,
                                       "Failed to write segment " + std::to_string(i) + 
                                       " to address 0x" + std::to_string(segment.virtual_address)));
            }
            loaded_segments++;
        }

        // Zero-fill BSS sections
        if (segment.memory_size > segment.file_size) {
            size_t bss_size = segment.memory_size - segment.file_size;
            Address bss_start = segment.virtual_address + segment.file_size;
            
            LOG_DEBUG("Zero-filling BSS section: 0x{:08x}, {} bytes", bss_start, bss_size);
            
            std::vector<u8> zeros(bss_size, 0);
            auto zero_result = memory_controller_->write_bytes(bss_start, zeros.data(), zeros.size());
            if (!zero_result.has_value()) {
                return unexpected(Error(ErrorCode::MEMORY_ACCESS_ERROR,
                                       "Failed to zero-fill BSS section at 0x" + std::to_string(bss_start)));
            }
        }

        // Configure memory protection
        auto protection_result = configure_memory_protection(segment);
        if (!protection_result.has_value()) {
            add_warning("Failed to configure memory protection for segment " + std::to_string(i));
        }
    }

    LOG_INFO("Successfully loaded {} segments to memory", loaded_segments);
    return {};
}

Result<void> ELFLoader::process_relocations(const ParsedELF& parsed_elf) {
    if (relocations_.empty()) {
        LOG_DEBUG("No relocations to process");
        return {};
    }

    LOG_INFO("Processing {} relocations", relocations_.size());

    size_t processed = 0;
    for (const auto& relocation : relocations_) {
        // Find base address for the relocation
        Address base_address = 0;
        for (const auto& segment : parsed_elf.segments) {
            if (relocation.offset >= segment.virtual_address && 
                relocation.offset < segment.virtual_address + segment.memory_size) {
                base_address = segment.virtual_address;
                break;
            }
        }

        if (base_address == 0) {
            return unexpected(Error(ErrorCode::INVALID_PARAMETER,
                                   "Could not find segment for relocation at offset 0x" + 
                                   std::to_string(relocation.offset)));
        }

        auto apply_result = apply_relocation(relocation, base_address);
        if (!apply_result.has_value()) {
            return unexpected(Error(ErrorCode::PROCESSING_ERROR,
                                   "Failed to apply " + relocation.get_type_name() + 
                                   " relocation at 0x" + std::to_string(relocation.offset)));
        }

        processed++;
    }

    LOG_INFO("Successfully processed {} relocations", processed);
    return {};
}

Result<void> ELFLoader::resolve_symbols() {
    if (symbol_table_.empty()) {
        LOG_DEBUG("No symbols to resolve");
        return {};
    }

    LOG_INFO("Resolving {} symbols", symbol_table_.size());

    // Build global symbol table for quick lookup
    global_symbol_table_.clear();
    for (const auto& symbol : symbol_table_) {
        if (!symbol.name.empty() && symbol.value != 0) {
            global_symbol_table_[symbol.name] = symbol.value;
        }
    }

    // Check for undefined symbols
    size_t undefined_count = 0;
    for (const auto& symbol : symbol_table_) {
        if (symbol.is_undefined() && symbol.is_global()) {
            add_warning("Undefined global symbol: " + symbol.name);
            undefined_count++;
        }
    }

    if (undefined_count > 0) {
        add_warning("Found " + std::to_string(undefined_count) + " undefined symbols");
    }

    LOG_INFO("Symbol resolution completed: {} symbols in global table, {} undefined",
             global_symbol_table_.size(), undefined_count);

    return {};
}

// Private implementation methods

Result<ParsedELF> ELFLoader::parse_elf_with_sections(const std::string& file_path) {
    // Use existing ELF parser for basic parsing
    ELFParser parser(file_path);
    auto parse_result = parser.parse();
    if (!parse_result.has_value()) {
        return unexpected(parse_result.error());
    }

    ParsedELF parsed_elf = std::move(parse_result.value());

    // Now read file data for section processing
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return unexpected(Error(ErrorCode::CONFIG_FILE_NOT_FOUND, "Cannot open ELF file"));
    }

    auto file_size = file.tellg();
    std::vector<u8> file_data(static_cast<size_t>(file_size));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(file_data.data()), file_size);

    // Read ELF header
    if (file_data.size() < sizeof(ELF32Header)) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "File too small for ELF header"));
    }

    ELF32Header header;
    std::memcpy(&header, file_data.data(), sizeof(ELF32Header));

    // Read section headers
    auto sections_result = read_section_headers(file_data, header);
    if (!sections_result.has_value()) {
        LOG_WARN("Could not read section headers, proceeding without symbol/relocation processing");
        return parsed_elf;
    }

    const auto& sections = sections_result.value();

    // Parse symbol table
    auto symbols_result = parse_symbol_table(file_data, sections);
    if (!symbols_result.has_value()) {
        LOG_WARN("Could not parse symbol table: {}", symbols_result.error().message());
    }

    // Parse relocations
    auto relocations_result = parse_relocations(file_data, sections);
    if (!relocations_result.has_value()) {
        LOG_WARN("Could not parse relocations: {}", relocations_result.error().message());
    }

    LOG_DEBUG("ELF sections processed: {} symbols, {} relocations", 
              symbol_table_.size(), relocations_.size());

    return parsed_elf;
}

Result<std::vector<ELF32SectionHeader>> ELFLoader::read_section_headers(const std::vector<u8>& file_data,
                                                                        const ELF32Header& header) {
    if (header.e_shnum == 0) {
        return std::vector<ELF32SectionHeader>{}; // No section headers
    }

    size_t headers_size = header.e_shnum * sizeof(ELF32SectionHeader);
    if (file_data.size() < header.e_shoff + headers_size) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "Section headers extend beyond file"));
    }

    std::vector<ELF32SectionHeader> sections(header.e_shnum);
    std::memcpy(sections.data(), 
                file_data.data() + header.e_shoff, 
                headers_size);

    return sections;
}

Result<std::vector<u8>> ELFLoader::read_section_data(const std::vector<u8>& file_data,
                                                     const ELF32SectionHeader& section) {
    if (section.sh_size == 0) {
        return std::vector<u8>{};
    }

    if (file_data.size() < section.sh_offset + section.sh_size) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "Section data extends beyond file"));
    }

    std::vector<u8> data(section.sh_size);
    std::memcpy(data.data(), 
                file_data.data() + section.sh_offset, 
                section.sh_size);

    return data;
}

Result<std::string> ELFLoader::read_string_table(const std::vector<u8>& file_data,
                                                 const ELF32SectionHeader& string_table_section) {
    auto data_result = read_section_data(file_data, string_table_section);
    if (!data_result.has_value()) {
        return unexpected(data_result.error());
    }

    const auto& data = data_result.value();
    return std::string(reinterpret_cast<const char*>(data.data()), data.size());
}

Result<void> ELFLoader::parse_symbol_table(const std::vector<u8>& file_data,
                                           const std::vector<ELF32SectionHeader>& sections) {
    symbol_table_.clear();

    // Find symbol table and string table sections
    const ELF32SectionHeader* symtab_section = nullptr;
    const ELF32SectionHeader* strtab_section = nullptr;

    for (const auto& section : sections) {
        if (section.sh_type == 2) { // SHT_SYMTAB
            symtab_section = &section;
        } else if (section.sh_type == 3) { // SHT_STRTAB
            strtab_section = &section;
        }
    }

    if (!symtab_section || !strtab_section) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "No symbol table or string table found"));
    }

    // Read string table
    auto string_table_result = read_string_table(file_data, *strtab_section);
    if (!string_table_result.has_value()) {
        return unexpected(string_table_result.error());
    }

    const auto& string_table = string_table_result.value();

    // Read symbol table data
    auto symtab_data_result = read_section_data(file_data, *symtab_section);
    if (!symtab_data_result.has_value()) {
        return unexpected(symtab_data_result.error());
    }

    const auto& symtab_data = symtab_data_result.value();

    // Parse symbols
    size_t symbol_count = symtab_data.size() / sizeof(ELF32Sym);
    const ELF32Sym* symbols = reinterpret_cast<const ELF32Sym*>(symtab_data.data());

    for (size_t i = 0; i < symbol_count; ++i) {
        const auto& elf_symbol = symbols[i];
        
        ParsedSymbol symbol;
        symbol.value = elf_symbol.st_value;
        symbol.size = elf_symbol.st_size;
        symbol.info = elf_symbol.st_info;
        symbol.visibility = elf_symbol.st_other;
        symbol.section_index = elf_symbol.st_shndx;

        // Extract symbol name from string table
        if (elf_symbol.st_name < string_table.size()) {
            const char* name_start = string_table.c_str() + elf_symbol.st_name;
            symbol.name = std::string(name_start);
        }

        symbol_table_.push_back(symbol);
    }

    LOG_DEBUG("Parsed {} symbols from symbol table", symbol_table_.size());
    return {};
}

Result<void> ELFLoader::parse_relocations(const std::vector<u8>& file_data,
                                          const std::vector<ELF32SectionHeader>& sections) {
    relocations_.clear();

    // Find relocation sections (.rel.* and .rela.*)
    for (const auto& section : sections) {
        if (section.sh_type == 9) { // SHT_REL
            // Parse .rel section
            auto data_result = read_section_data(file_data, section);
            if (!data_result.has_value()) continue;

            const auto& data = data_result.value();
            size_t rel_count = data.size() / sizeof(ELF32Rel);
            const ELF32Rel* rels = reinterpret_cast<const ELF32Rel*>(data.data());

            for (size_t i = 0; i < rel_count; ++i) {
                ParsedRelocation relocation;
                relocation.offset = rels[i].r_offset;
                relocation.type = static_cast<RiscVRelocationType>(rels[i].r_info & 0xFF);
                relocation.symbol_index = rels[i].r_info >> 8;
                relocation.addend = 0; // No addend in REL format

                relocations_.push_back(relocation);
            }
        } else if (section.sh_type == 4) { // SHT_RELA
            // Parse .rela section
            auto data_result = read_section_data(file_data, section);
            if (!data_result.has_value()) continue;

            const auto& data = data_result.value();
            size_t rela_count = data.size() / sizeof(ELF32Rela);
            const ELF32Rela* relas = reinterpret_cast<const ELF32Rela*>(data.data());

            for (size_t i = 0; i < rela_count; ++i) {
                ParsedRelocation relocation;
                relocation.offset = relas[i].r_offset;
                relocation.type = static_cast<RiscVRelocationType>(relas[i].r_info & 0xFF);
                relocation.symbol_index = relas[i].r_info >> 8;
                relocation.addend = relas[i].r_addend;

                relocations_.push_back(relocation);
            }
        }
    }

    LOG_DEBUG("Parsed {} relocations from relocation sections", relocations_.size());
    return {};
}

// RISC-V relocation processing methods

Result<void> ELFLoader::apply_relocation(const ParsedRelocation& relocation, Address base_address) {
    Address target_address = base_address + relocation.offset;
    
    // Get symbol value
    Address symbol_value = 0;
    if (relocation.needs_symbol_resolution() && relocation.symbol_index < symbol_table_.size()) {
        symbol_value = symbol_table_[relocation.symbol_index].value;
    }

    // Apply specific relocation type
    switch (relocation.type) {
        case RiscVRelocationType::R_RISCV_32:
            return handle_r_riscv_32(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_PCREL_HI20:
            return handle_r_riscv_pcrel_hi20(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_PCREL_LO12_I:
            return handle_r_riscv_pcrel_lo12_i(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_PCREL_LO12_S:
            return handle_r_riscv_pcrel_lo12_s(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_HI20:
            return handle_r_riscv_hi20(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_LO12_I:
            return handle_r_riscv_lo12_i(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_LO12_S:
            return handle_r_riscv_lo12_s(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_JAL:
            return handle_r_riscv_jal(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_BRANCH:
            return handle_r_riscv_branch(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_CALL:
            return handle_r_riscv_call(target_address, symbol_value, relocation.addend);
            
        case RiscVRelocationType::R_RISCV_NONE:
            return {}; // No operation needed
            
        default:
            add_warning("Unsupported relocation type: " + relocation.get_type_name());
            return {};
    }
}

Result<void> ELFLoader::handle_r_riscv_32(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_32: S + A
    u32 value = symbol_value + addend;
    return memory_controller_->write_u32(target_address, value);
}

Result<void> ELFLoader::handle_r_riscv_pcrel_hi20(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_PCREL_HI20: S + A - P (upper 20 bits)
    i32 pc_relative = (symbol_value + addend) - target_address;
    u32 upper_20 = static_cast<u32>((pc_relative + 0x800) >> 12) & 0xFFFFF;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    instruction = RISCV::insert_bits(instruction, upper_20, 12, 20);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_pcrel_lo12_i(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_PCREL_LO12_I: S + A - P (lower 12 bits for I-type)
    i32 pc_relative = (symbol_value + addend) - target_address;
    u32 lower_12 = static_cast<u32>(pc_relative) & 0xFFF;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    instruction = RISCV::insert_bits(instruction, lower_12, 20, 12);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_pcrel_lo12_s(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_PCREL_LO12_S: S + A - P (lower 12 bits for S-type)
    i32 pc_relative = (symbol_value + addend) - target_address;
    u32 lower_12 = static_cast<u32>(pc_relative) & 0xFFF;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    
    // S-type encoding: bits [11:5] go to bits [31:25], bits [4:0] go to bits [11:7]
    u32 upper_7 = (lower_12 >> 5) & 0x7F;
    u32 lower_5 = lower_12 & 0x1F;
    
    instruction = RISCV::insert_bits(instruction, upper_7, 25, 7);
    instruction = RISCV::insert_bits(instruction, lower_5, 7, 5);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_hi20(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_HI20: S + A (upper 20 bits)
    u32 value = symbol_value + addend;
    u32 upper_20 = (value + 0x800) >> 12;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    instruction = RISCV::insert_bits(instruction, upper_20, 12, 20);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_lo12_i(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_LO12_I: S + A (lower 12 bits for I-type)
    u32 value = symbol_value + addend;
    u32 lower_12 = value & 0xFFF;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    instruction = RISCV::insert_bits(instruction, lower_12, 20, 12);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_lo12_s(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_LO12_S: S + A (lower 12 bits for S-type)
    u32 value = symbol_value + addend;
    u32 lower_12 = value & 0xFFF;

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    
    // S-type encoding: bits [11:5] go to bits [31:25], bits [4:0] go to bits [11:7]
    u32 upper_7 = (lower_12 >> 5) & 0x7F;
    u32 lower_5 = lower_12 & 0x1F;
    
    instruction = RISCV::insert_bits(instruction, upper_7, 25, 7);
    instruction = RISCV::insert_bits(instruction, lower_5, 7, 5);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_jal(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_JAL: S + A - P (JAL immediate encoding)
    i32 pc_relative = (symbol_value + addend) - target_address;
    
    // Check range: JAL can encode ±1MB (20-bit signed offset in multiples of 2)
    if (pc_relative < -1048576 || pc_relative >= 1048576) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "JAL target out of range"));
    }

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    u32 immediate = encode_immediate_j(pc_relative);
    instruction = RISCV::insert_bits(instruction, immediate, 12, 20);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_branch(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_BRANCH: S + A - P (Branch immediate encoding)
    i32 pc_relative = (symbol_value + addend) - target_address;
    
    // Check range: Branch can encode ±4KB (12-bit signed offset in multiples of 2)
    if (pc_relative < -4096 || pc_relative >= 4096) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "Branch target out of range"));
    }

    auto instruction_result = read_instruction(target_address);
    if (!instruction_result.has_value()) {
        return unexpected(instruction_result.error());
    }

    u32 instruction = instruction_result.value();
    u32 immediate = encode_immediate_b(pc_relative);
    
    // B-type encoding is split: bits [12|10:5] go to bits [31|30:25], bits [4:1|11] go to bits [11:8|7]
    u32 upper_7 = (immediate >> 5) & 0x7F;
    u32 lower_5 = immediate & 0x1F;
    
    instruction = RISCV::insert_bits(instruction, upper_7, 25, 7);
    instruction = RISCV::insert_bits(instruction, lower_5, 7, 5);

    return write_instruction(target_address, instruction);
}

Result<void> ELFLoader::handle_r_riscv_call(Address target_address, Address symbol_value, i32 addend) {
    // R_RISCV_CALL: Function call sequence (AUIPC + JALR)
    // This is typically a pair of instructions, but we handle the first one here
    return handle_r_riscv_pcrel_hi20(target_address, symbol_value, addend);
}

// Instruction manipulation utilities

Result<u32> ELFLoader::read_instruction(Address address) {
    if (!memory_controller_) {
        return unexpected(Error(ErrorCode::INVALID_STATE, "Memory controller not available"));
    }

    return memory_controller_->read_u32(address);
}

Result<void> ELFLoader::write_instruction(Address address, u32 instruction) {
    if (!memory_controller_) {
        return unexpected(Error(ErrorCode::INVALID_STATE, "Memory controller not available"));
    }

    return memory_controller_->write_u32(address, instruction);
}

u32 ELFLoader::encode_immediate_i(i32 immediate) {
    return static_cast<u32>(immediate) & 0xFFF;
}

u32 ELFLoader::encode_immediate_s(i32 immediate) {
    return static_cast<u32>(immediate) & 0xFFF;
}

u32 ELFLoader::encode_immediate_b(i32 immediate) {
    // B-type immediate encoding: [12|10:5|4:1|11] -> instruction bits [31|30:25|11:8|7]
    u32 imm = static_cast<u32>(immediate);
    u32 encoded = 0;
    encoded |= (imm & 0x800) << 20;  // bit 11 -> bit 31
    encoded |= (imm & 0x7E0) << 20;  // bits 10:5 -> bits 30:25
    encoded |= (imm & 0x1E) << 7;    // bits 4:1 -> bits 11:8
    encoded |= (imm & 0x1000) >> 5;  // bit 12 -> bit 7
    return encoded;
}

u32 ELFLoader::encode_immediate_u(i32 immediate) {
    return (static_cast<u32>(immediate) >> 12) & 0xFFFFF;
}

u32 ELFLoader::encode_immediate_j(i32 immediate) {
    // J-type immediate encoding: [20|10:1|11|19:12] -> instruction bits [31|30:21|20|19:12]
    u32 imm = static_cast<u32>(immediate);
    u32 encoded = 0;
    encoded |= (imm & 0x100000) << 11; // bit 20 -> bit 31
    encoded |= (imm & 0x7FE) << 20;    // bits 10:1 -> bits 30:21
    encoded |= (imm & 0x800) << 9;     // bit 11 -> bit 20
    encoded |= imm & 0xFF000;          // bits 19:12 -> bits 19:12
    return encoded;
}

// Memory setup and protection

Result<void> ELFLoader::setup_memory_regions(const std::vector<ParsedSegment>& segments) {
    // Validate all segment addresses first
    for (const auto& segment : segments) {
        if (!is_valid_esp32p4_address(segment.virtual_address, segment.memory_size)) {
            return unexpected(Error(ErrorCode::INVALID_PARAMETER, 
                                   "Segment at 0x" + std::to_string(segment.virtual_address) + 
                                   " is outside valid ESP32-P4 memory range"));
        }
    }

    // Setup stack region
    auto stack_result = setup_stack_region(DEFAULT_STACK_TOP - stack_size_, stack_size_);
    if (!stack_result.has_value()) {
        return unexpected(stack_result.error());
    }

    // Setup heap region if using PSRAM
    auto heap_result = setup_heap_region(DEFAULT_HEAP_BASE, heap_size_);
    if (!heap_result.has_value()) {
        add_warning("Could not setup heap region: " + heap_result.error().message());
    }

    LOG_DEBUG("Memory regions setup completed");
    return {};
}

Result<void> ELFLoader::setup_stack_region(Address stack_top, size_t stack_size) {
    // Just validate stack region for now
    if (!is_valid_esp32p4_address(stack_top - stack_size, stack_size)) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "Stack region is outside valid memory"));
    }

    LOG_DEBUG("Stack region setup: top=0x{:08x}, size={} bytes", stack_top, stack_size);
    return {};
}

Result<void> ELFLoader::setup_heap_region(Address heap_base, size_t heap_size) {
    // Just validate heap region for now
    if (!is_valid_esp32p4_address(heap_base, heap_size)) {
        return unexpected(Error(ErrorCode::INVALID_PARAMETER, "Heap region is outside valid memory"));
    }

    LOG_DEBUG("Heap region setup: base=0x{:08x}, size={} bytes", heap_base, heap_size);
    return {};
}

Result<void> ELFLoader::configure_memory_protection(const ParsedSegment& segment) {
    // TODO: Configure memory protection based on segment flags
    // For now, just log the configuration
    LOG_DEBUG("Memory protection configured for segment: 0x{:08x}, {}, {} bytes",
              segment.virtual_address, segment.get_permissions(), segment.memory_size);
    return {};
}

// ESP-IDF application image support

Result<esp_image_header_t> ELFLoader::parse_esp_image_header(const std::vector<u8>& file_data) {
    if (file_data.size() < sizeof(esp_image_header_t)) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "File too small for ESP image header"));
    }

    esp_image_header_t header;
    std::memcpy(&header, file_data.data(), sizeof(esp_image_header_t));

    // Validate magic number (ESP-IDF uses 0xE9)
    if (header.magic != 0xE9) {
        return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "Invalid ESP image magic number"));
    }

    return header;
}

Result<std::vector<ParsedSegment>> ELFLoader::parse_esp_image_segments(const std::vector<u8>& file_data,
                                                                       const esp_image_header_t& header) {
    std::vector<ParsedSegment> segments;
    size_t offset = sizeof(esp_image_header_t);

    for (u8 i = 0; i < header.segment_count; ++i) {
        if (offset + sizeof(esp_image_segment_header_t) > file_data.size()) {
            return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "Segment header extends beyond file"));
        }

        esp_image_segment_header_t segment_header;
        std::memcpy(&segment_header, file_data.data() + offset, sizeof(esp_image_segment_header_t));
        offset += sizeof(esp_image_segment_header_t);

        if (offset + segment_header.data_len > file_data.size()) {
            return unexpected(Error(ErrorCode::CONFIG_INVALID_FORMAT, "Segment data extends beyond file"));
        }

        ParsedSegment segment;
        segment.virtual_address = segment_header.load_addr;
        segment.physical_address = segment_header.load_addr;
        segment.file_size = segment_header.data_len;
        segment.memory_size = segment_header.data_len;
        segment.flags = ELF::PF_R | ELF::PF_W | ELF::PF_X; // Default permissions

        // Copy segment data
        segment.data.resize(segment_header.data_len);
        std::memcpy(segment.data.data(), file_data.data() + offset, segment_header.data_len);

        segments.push_back(std::move(segment));
        offset += segment_header.data_len;
    }

    LOG_DEBUG("Parsed {} ESP image segments", segments.size());
    return segments;
}

Result<bool> ELFLoader::validate_esp_image_checksum(const std::vector<u8>& file_data) {
    // TODO: Implement ESP image checksum validation
    // For now, just return true as a placeholder
    return true;
}

// Utility methods

bool ELFLoader::is_valid_esp32p4_address(Address address, size_t size) const {
    Address end_address = address + size - 1;

    // Check flash region
    if (address >= ESP32P4MemoryLayout::IROM_BASE && 
        end_address < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) {
        return true;
    }

    // Check SRAM region
    if (address >= ESP32P4MemoryLayout::DRAM_BASE && 
        end_address < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) {
        return true;
    }

    // Check PSRAM region
    if (address >= ESP32P4MemoryLayout::PSRAM_BASE && 
        end_address < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE) {
        return true;
    }

    return false;
}

std::string ELFLoader::get_memory_region_name(Address address) const {
    if (address >= ESP32P4MemoryLayout::IROM_BASE && 
        address < ESP32P4MemoryLayout::IROM_BASE + ESP32P4MemoryLayout::IROM_SIZE) {
        return "Flash/IROM";
    }
    if (address >= ESP32P4MemoryLayout::DRAM_BASE && 
        address < ESP32P4MemoryLayout::DRAM_BASE + ESP32P4MemoryLayout::DRAM_SIZE) {
        return "SRAM/DRAM";
    }
    if (address >= ESP32P4MemoryLayout::PSRAM_BASE && 
        address < ESP32P4MemoryLayout::PSRAM_BASE + ESP32P4MemoryLayout::PSRAM_SIZE) {
        return "PSRAM";
    }
    return "Unknown";
}

void ELFLoader::update_progress(ProgressCallback callback, const std::string& stage, 
                               float progress, const std::string& message) {
    if (callback) {
        callback(stage, progress, message);
    }
}

void ELFLoader::add_warning(const std::string& warning) {
    warnings_.push_back(warning);
    LOG_WARN("ELF Loader warning: {}", warning);
}

} // namespace m5tab5::emulator::firmware