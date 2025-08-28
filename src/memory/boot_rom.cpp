#include "emulator/memory/boot_rom.hpp"
#include "emulator/esp_idf/esp32p4_bootloader.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/types.hpp"

#include <cstring>
#include <algorithm>
#include <thread>
#include <chrono>

namespace m5tab5::emulator {

DECLARE_LOGGER("BootROM");

BootROM::BootROM()
    : initialized_(false)
    , mmu_configured_(false)
    , current_entry_point_(0)
    , strapping_pins_(0)
    , flash_mode_(FlashMode::QIO)
    , flash_frequency_(80000000)  // 80MHz default
    , flash_size_(16 * 1024 * 1024)  // 16MB default
    , esp32p4_boot_sequence_(nullptr)
{
    // Initialize boot parameters with defaults
    boot_params_.magic_number = 0xE9E5E9E5;  // ESP32-P4 magic
    boot_params_.boot_mode = static_cast<u32>(BootMode::FLASH_BOOT);
    boot_params_.cpu_frequency = 400000000;  // 400MHz
    boot_params_.flash_frequency = flash_frequency_;
    boot_params_.flash_mode = static_cast<u32>(flash_mode_);
    boot_params_.flash_size = flash_size_;
    boot_params_.bootloader_address = FLASH_BOOTLOADER_BASE;
    boot_params_.application_address = FLASH_APP_BASE;
    
    // Clear reserved fields
    std::memset(boot_params_.reserved, 0, sizeof(boot_params_.reserved));
    
    COMPONENT_LOG_DEBUG("BootROM created with default parameters");
}

BootROM::~BootROM() {
    if (initialized_) {
        shutdown();
    }
    
    if (esp32p4_boot_sequence_) {
        esp32p4_boot_sequence_.reset();
    }
    
    COMPONENT_LOG_DEBUG("BootROM destroyed");
}

Result<void> BootROM::initialize() {
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING, "Boot ROM already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing ESP32-P4 Boot ROM emulation");
    
    try {
        // Create Boot ROM memory region
        memory_region_ = std::make_shared<MemoryRegion>(
            "BootROM", BOOT_ROM_BASE, 32 * 1024, MemoryType::ROM,
            false,  // Not writable
            true,   // Executable
            false   // Not cacheable (direct access)
        );
        
        RETURN_IF_ERROR(memory_region_->initialize());
        
        // Generate Boot ROM binary
        RETURN_IF_ERROR(generateBootROMBinary());
        
        // Temporarily enable writes for initialization
        memory_region_->set_writable(true);
        
        // Load binary into memory region
        RETURN_IF_ERROR(memory_region_->write(0, rom_data_.data(), rom_data_.size()));
        
        // Set read-only after loading
        memory_region_->set_read_only();
        
        initialized_ = true;
        COMPONENT_LOG_INFO("Boot ROM initialized successfully");
        
        return {};
        
    } catch (const std::exception& e) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Exception during Boot ROM initialization: " + std::string(e.what())));
    }
}

void BootROM::shutdown() {
    if (!initialized_) {
        return;
    }
    
    COMPONENT_LOG_INFO("Shutting down Boot ROM");
    
    if (memory_region_) {
        memory_region_->shutdown();
        memory_region_.reset();
    }
    
    initialized_ = false;
    mmu_configured_ = false;
    current_entry_point_ = 0;
    
    // Cleanup ESP32-P4 integration
    if (esp32p4_boot_sequence_) {
        esp32p4_boot_sequence_.reset();
    }
    memory_controller_.reset();
    cpu_manager_.reset();
    boot_sequence_callback_ = nullptr;
}

Result<void> BootROM::reset() {
    COMPONENT_LOG_INFO("Resetting Boot ROM");
    
    mmu_configured_ = false;
    current_entry_point_ = 0;
    
    // Reset boot parameters to defaults
    boot_params_.boot_mode = static_cast<u32>(detectBootMode());
    
    return {};
}

Result<void> BootROM::executeBootSequence() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Boot ROM not initialized"));
    }
    
    COMPONENT_LOG_INFO("Executing ESP32-P4 boot sequence");
    
    // Step 1: Initialize clocks
    RETURN_IF_ERROR(initializeClocks());
    
    // Step 2: Initialize power domains
    RETURN_IF_ERROR(initializePowerDomains());
    
    // Step 3: Configure MMU for flash XIP
    RETURN_IF_ERROR(configureMmu());
    
    // Step 4: Detect boot mode
    BootMode boot_mode = detectBootMode();
    COMPONENT_LOG_INFO("Boot mode detected: {}", static_cast<u32>(boot_mode));
    
    // Step 5: Load and transfer to bootloader/application
    switch (boot_mode) {
        case BootMode::FLASH_BOOT:
            RETURN_IF_ERROR(loadBootloader());
            current_entry_point_ = boot_params_.bootloader_address;
            break;
            
        case BootMode::UART_BOOT:
        case BootMode::USB_BOOT:
            // Direct application boot for download modes
            current_entry_point_ = boot_params_.application_address;
            break;
            
        default:
            return unexpected(MAKE_ERROR(OPERATION_FAILED,
                "Unsupported boot mode: " + std::to_string(static_cast<u32>(boot_mode))));
    }
    
    COMPONENT_LOG_INFO("Boot sequence completed, entry point: 0x{:08X}", current_entry_point_);
    return {};
}

Result<Address> BootROM::getResetVector() const {
    return BOOT_ROM_RESET_VECTOR;
}

Result<Address> BootROM::getBootloaderEntryPoint() const {
    return boot_params_.bootloader_address;
}

Result<Address> BootROM::getApplicationEntryPoint() const {
    return current_entry_point_;
}

Result<void> BootROM::setBootParameters(const BootParams& params) {
    boot_params_ = params;
    
    // Update internal state
    flash_mode_ = static_cast<FlashMode>(params.flash_mode);
    flash_frequency_ = params.flash_frequency;
    flash_size_ = params.flash_size;
    
    COMPONENT_LOG_DEBUG("Boot parameters updated");
    return {};
}

Result<BootROM::BootParams> BootROM::getBootParameters() const {
    return boot_params_;
}

void BootROM::setStrappingPins(u32 pin_state) {
    strapping_pins_ = pin_state;
    boot_params_.boot_mode = static_cast<u32>(detectBootMode());
    COMPONENT_LOG_DEBUG("Strapping pins set to 0x{:08X}, boot mode: {}", 
                       pin_state, boot_params_.boot_mode);
}

BootROM::BootMode BootROM::detectBootMode() const {
    return decodeStrappingPins(strapping_pins_);
}

Result<void> BootROM::executeEsp32P4BootSequence() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Boot ROM not initialized"));
    }
    
    COMPONENT_LOG_INFO("Starting ESP32-P4 enhanced boot sequence");
    
    // Phase 1: ESP32-P4 hardware initialization
    RETURN_IF_ERROR(esp32p4_hardware_initialization());
    
    // Phase 2: Flash configuration validation
    RETURN_IF_ERROR(esp32p4_validate_flash_configuration());
    
    // Phase 3: Initialize basic peripherals
    RETURN_IF_ERROR(esp32p4_initialize_basic_peripherals());
    
    // Phase 4: Transfer to ESP32-P4 bootloader
    RETURN_IF_ERROR(esp32p4_transfer_to_bootloader());
    
    // Phase 5: Execute custom boot sequence callback if provided
    if (boot_sequence_callback_) {
        COMPONENT_LOG_DEBUG("Executing custom boot sequence callback");
        RETURN_IF_ERROR(boot_sequence_callback_());
    }
    
    COMPONENT_LOG_INFO("ESP32-P4 boot sequence completed successfully");
    return {};
}

void BootROM::setBootSequenceCallback(std::function<Result<void>()> callback) {
    boot_sequence_callback_ = callback;
    COMPONENT_LOG_DEBUG("Boot sequence callback registered");
}

void BootROM::setMemoryController(std::shared_ptr<MemoryController> memory_controller) {
    memory_controller_ = memory_controller;
    COMPONENT_LOG_DEBUG("Memory controller registered with BootROM");
}

void BootROM::setDualCoreManager(std::shared_ptr<DualCoreManager> cpu_manager) {
    cpu_manager_ = cpu_manager;
    COMPONENT_LOG_DEBUG("Dual core manager registered with BootROM");
}

void BootROM::setFlashConfiguration(FlashMode mode, u32 frequency, u32 size) {
    flash_mode_ = mode;
    flash_frequency_ = frequency;
    flash_size_ = size;
    
    boot_params_.flash_mode = static_cast<u32>(mode);
    boot_params_.flash_frequency = frequency;
    boot_params_.flash_size = size;
    
    COMPONENT_LOG_DEBUG("Flash configuration: mode={}, freq={}Hz, size={}MB",
                       static_cast<u32>(mode), frequency, size / (1024 * 1024));
}

Result<void> BootROM::initializeMMU() {
    if (mmu_configured_) {
        return {};
    }
    
    COMPONENT_LOG_DEBUG("Initializing MMU for flash XIP");
    
    // Map flash region for XIP access
    RETURN_IF_ERROR(mapFlashRegion(FLASH_XIP_BASE, 0, flash_size_));
    
    mmu_configured_ = true;
    return {};
}

Result<void> BootROM::mapFlashRegion(Address virtual_base, Address physical_base, size_t size) {
    // For now, this is a placeholder for MMU mapping
    // In a full implementation, this would configure page tables
    COMPONENT_LOG_DEBUG("Mapping flash region: virt=0x{:08X}, phys=0x{:08X}, size={}MB",
                       virtual_base, physical_base, size / (1024 * 1024));
    return {};
}

Result<void> BootROM::generateBootROMBinary() {
    COMPONENT_LOG_DEBUG("Generating Boot ROM binary");
    
    // Clear ROM data
    rom_data_.fill(0);
    
    // Generate reset vector and jump to boot code
    generateResetHandler();
    
    // Generate main boot sequence
    generateBootSequence();
    
    // Generate parameter data section
    generateParameterData();
    
    return {};
}

void BootROM::generateResetHandler() {
    // Reset vector at 0x40000080 (offset 0x80 in ROM)
    constexpr size_t reset_vector_offset = 0x80;
    
    // Generate jump to boot sequence (offset 0x1000)
    constexpr Address boot_sequence_addr = BOOT_ROM_BASE + 0x1000;
    i32 jump_offset = boot_sequence_addr - BOOT_ROM_RESET_VECTOR;
    
    // JAL x0, boot_sequence (jump without saving return address)
    u32 jal_instruction = BootROMGenerator::generateJAL(0, jump_offset);
    
    // Write to ROM data
    std::memcpy(&rom_data_[reset_vector_offset], &jal_instruction, sizeof(u32));
    
    // Also place some basic instructions at the beginning of ROM for immediate execution
    // This ensures the CPU can fetch valid instructions right away
    constexpr size_t start_offset = 0x0;
    u32 startup_instructions[] = {
        BootROMGenerator::NOP,                          // 0x40000000: NOP
        BootROMGenerator::NOP,                          // 0x40000004: NOP
        BootROMGenerator::generateJAL(0, 0x80 - 0x08), // 0x40000008: JAL to reset vector
        BootROMGenerator::NOP                           // 0x4000000C: NOP
    };
    
    for (size_t i = 0; i < sizeof(startup_instructions) / sizeof(u32); ++i) {
        std::memcpy(&rom_data_[start_offset + i * 4], &startup_instructions[i], sizeof(u32));
    }
    
    COMPONENT_LOG_DEBUG("Generated reset handler: JAL to 0x{:08X}, startup code at 0x{:08X}", 
                       boot_sequence_addr, BOOT_ROM_BASE);
}

void BootROM::generateBootSequence() {
    // Boot sequence at offset 0x1000
    constexpr size_t boot_sequence_offset = 0x1000;
    size_t current_offset = boot_sequence_offset;
    
    // Generate initialization sequence
    auto init_instructions = BootROMGenerator::generateInitSequence();
    for (u32 instr : init_instructions) {
        if (current_offset + 4 <= rom_data_.size()) {
            std::memcpy(&rom_data_[current_offset], &instr, sizeof(u32));
            current_offset += 4;
        }
    }
    
    COMPONENT_LOG_DEBUG("Boot sequence at offset 0x{:X}, added {} init instructions",
                       boot_sequence_offset, init_instructions.size());
    
    // Generate strapping pin check
    auto strapping_instructions = BootROMGenerator::generateStrappingCheck();
    for (u32 instr : strapping_instructions) {
        if (current_offset + 4 <= rom_data_.size()) {
            std::memcpy(&rom_data_[current_offset], &instr, sizeof(u32));
            current_offset += 4;
        }
    }
    
    // Generate MMU setup
    auto mmu_instructions = BootROMGenerator::generateMMUSetup();
    for (u32 instr : mmu_instructions) {
        if (current_offset + 4 <= rom_data_.size()) {
            std::memcpy(&rom_data_[current_offset], &instr, sizeof(u32));
            current_offset += 4;
        }
    }
    
    // Generate jump to bootloader
    auto jump_instructions = BootROMGenerator::generateJumpToBootloader(boot_params_.bootloader_address);
    for (u32 instr : jump_instructions) {
        if (current_offset + 4 <= rom_data_.size()) {
            std::memcpy(&rom_data_[current_offset], &instr, sizeof(u32));
            current_offset += 4;
        }
    }
    
    COMPONENT_LOG_DEBUG("Generated boot sequence: {} bytes", current_offset - boot_sequence_offset);
}

void BootROM::generateParameterData() {
    // Parameter data at offset 0x5F00
    constexpr size_t param_offset = 0x5F00;
    
    if (param_offset + sizeof(BootParams) <= rom_data_.size()) {
        std::memcpy(&rom_data_[param_offset], &boot_params_, sizeof(BootParams));
        COMPONENT_LOG_DEBUG("Generated parameter data at offset 0x{:X}", param_offset);
    }
}

Result<void> BootROM::initializeClocks() {
    COMPONENT_LOG_DEBUG("Initializing system clocks");
    // Placeholder for clock initialization
    return {};
}

Result<void> BootROM::initializePowerDomains() {
    COMPONENT_LOG_DEBUG("Initializing power domains");
    // Placeholder for power domain initialization
    return {};
}

Result<void> BootROM::configureMmu() {
    COMPONENT_LOG_DEBUG("Configuring MMU");
    return initializeMMU();
}

Result<void> BootROM::loadBootloader() {
    COMPONENT_LOG_DEBUG("Loading bootloader from flash");
    // Placeholder for bootloader loading
    return {};
}

Result<void> BootROM::transferControl(Address entry_point) {
    COMPONENT_LOG_INFO("Transferring control to 0x{:08X}", entry_point);
    current_entry_point_ = entry_point;
    return {};
}

BootROM::BootMode BootROM::decodeStrappingPins(u32 pin_state) const {
    // ESP32-P4 strapping pin decoding logic
    // This is simplified - actual ESP32-P4 uses multiple GPIO pins
    
    // Bits 0-2: Boot mode selection
    u32 boot_mode_bits = pin_state & 0x7;
    
    switch (boot_mode_bits) {
        case 0: return BootMode::FLASH_BOOT;
        case 1: return BootMode::UART_BOOT;
        case 2: return BootMode::SPI_BOOT;
        case 3: return BootMode::SDIO_BOOT;
        case 4: return BootMode::JTAG_BOOT;
        case 5: return BootMode::USB_BOOT;
        default: return BootMode::FLASH_BOOT;  // Default to flash boot
    }
}

// BootROMGenerator implementation

u32 BootROMGenerator::generateLUI(u8 rd, u32 imm) {
    return encodeUType(0x37, rd, imm);  // LUI opcode
}

u32 BootROMGenerator::generateAUIPC(u8 rd, u32 imm) {
    return encodeUType(0x17, rd, imm);  // AUIPC opcode
}

u32 BootROMGenerator::generateJAL(u8 rd, i32 imm) {
    return encodeJType(0x6F, rd, imm);  // JAL opcode
}

u32 BootROMGenerator::generateJALR(u8 rd, u8 rs1, i32 imm) {
    return encodeIType(0x67, rd, 0, rs1, imm);  // JALR opcode
}

u32 BootROMGenerator::generateLW(u8 rd, u8 rs1, i32 imm) {
    return encodeIType(0x03, rd, 2, rs1, imm);  // LW opcode (funct3=2)
}

u32 BootROMGenerator::generateSW(u8 rs2, u8 rs1, i32 imm) {
    // SW is S-type instruction
    u32 imm_11_5 = (imm >> 5) & 0x7F;
    u32 imm_4_0 = imm & 0x1F;
    return (imm_11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (2 << 12) | (imm_4_0 << 7) | 0x23;
}

u32 BootROMGenerator::generateADDI(u8 rd, u8 rs1, i32 imm) {
    return encodeIType(0x13, rd, 0, rs1, imm);  // ADDI opcode
}

std::vector<u32> BootROMGenerator::generateInitSequence() {
    std::vector<u32> instructions;
    
    // Initialize stack pointer (x2) to top of L2 SRAM
    Address stack_top = SRAM_LAYOUT.start_address + SRAM_LAYOUT.size - 16;  // Leave some headroom
    u32 stack_upper = stack_top >> 12;
    u32 stack_lower = stack_top & 0xFFF;
    
    instructions.push_back(generateLUI(2, stack_upper));     // lui x2, stack_upper
    instructions.push_back(generateADDI(2, 2, stack_lower));  // addi x2, x2, stack_lower
    
    // Initialize global pointer (x3) for global data access
    u32 gp_addr = SRAM_LAYOUT.start_address + (SRAM_LAYOUT.size / 2);  // Middle of SRAM
    u32 gp_upper = gp_addr >> 12;
    u32 gp_lower = gp_addr & 0xFFF;
    instructions.push_back(generateLUI(3, gp_upper));        // lui x3, gp_upper  
    instructions.push_back(generateADDI(3, 3, gp_lower));    // addi x3, x3, gp_lower
    
    // Clear important registers
    instructions.push_back(generateADDI(1, 0, 0));           // addi x1, x0, 0 (ra)
    instructions.push_back(generateADDI(4, 0, 0));           // addi x4, x0, 0 (tp)
    instructions.push_back(generateADDI(5, 0, 0));           // addi x5, x0, 0 (t0)
    
    // Add a simple infinite loop as a placeholder boot sequence
    // This ensures the CPU has something to execute
    instructions.push_back(generateJAL(0, -4));              // jal x0, -4 (infinite loop)
    
    return instructions;
}

std::vector<u32> BootROMGenerator::generateStrappingCheck() {
    std::vector<u32> instructions;
    
    // Load strapping pin register (placeholder address)
    Address strapping_reg = 0x40040000;  // Example GPIO register
    u32 reg_upper = strapping_reg >> 12;
    u32 reg_lower = strapping_reg & 0xFFF;
    
    instructions.push_back(generateLUI(5, reg_upper));      // lui x5, reg_upper
    instructions.push_back(generateLW(6, 5, reg_lower));   // lw x6, reg_lower(x5)
    
    return instructions;
}

std::vector<u32> BootROMGenerator::generateMMUSetup() {
    std::vector<u32> instructions;
    
    // MMU setup is complex, for now just add placeholder
    instructions.push_back(NOP);  // Placeholder for MMU configuration
    instructions.push_back(NOP);
    
    return instructions;
}

std::vector<u32> BootROMGenerator::generateJumpToBootloader(Address bootloader_addr) {
    std::vector<u32> instructions;
    
    // Load bootloader address and jump
    u32 addr_upper = bootloader_addr >> 12;
    u32 addr_lower = bootloader_addr & 0xFFF;
    
    instructions.push_back(generateLUI(7, addr_upper));     // lui x7, addr_upper
    instructions.push_back(generateJALR(0, 7, addr_lower)); // jalr x0, addr_lower(x7)
    
    return instructions;
}

u32 BootROMGenerator::encodeIType(u32 opcode, u8 rd, u8 funct3, u8 rs1, i32 imm) {
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode;
}

u32 BootROMGenerator::encodeUType(u32 opcode, u8 rd, u32 imm) {
    return (imm << 12) | (rd << 7) | opcode;
}

u32 BootROMGenerator::encodeJType(u32 opcode, u8 rd, i32 imm) {
    u32 imm_20 = (imm >> 20) & 1;
    u32 imm_10_1 = (imm >> 1) & 0x3FF;
    u32 imm_11 = (imm >> 11) & 1;
    u32 imm_19_12 = (imm >> 12) & 0xFF;
    
    return (imm_20 << 31) | (imm_10_1 << 21) | (imm_11 << 20) | (imm_19_12 << 12) | (rd << 7) | opcode;
}

// ESP32-P4 specific boot ROM function implementations

Result<void> BootROM::esp32p4_hardware_initialization() {
    COMPONENT_LOG_DEBUG("ESP32-P4 hardware initialization");
    
    // Initialize system clocks to minimal operating frequency
    // Real ESP32-P4 Boot ROM sets up basic 40MHz clock
    
    // Initialize power management unit
    // Configure voltage regulators and power domains
    
    // Setup basic GPIO configuration
    // Configure strapping pins and essential I/O
    
    // Initialize UART0 for early debug output
    // This allows boot messages to be displayed
    
    // Simulate hardware initialization delay
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    COMPONENT_LOG_DEBUG("ESP32-P4 hardware initialization completed");
    return {};
}

Result<void> BootROM::esp32p4_validate_flash_configuration() {
    COMPONENT_LOG_DEBUG("Validating ESP32-P4 flash configuration");
    
    // Read and validate flash chip ID
    // Check flash size and speed capabilities
    // Verify flash is accessible and readable
    
    // Validate flash mode (QIO, QOUT, DIO, DOUT)
    if (flash_mode_ != FlashMode::QIO && flash_mode_ != FlashMode::QOUT &&
        flash_mode_ != FlashMode::DIO && flash_mode_ != FlashMode::DOUT) {
        return unexpected(MAKE_ERROR(CONFIG_INVALID_VALUE, "Invalid flash mode configured"));
    }
    
    // Validate flash frequency
    if (flash_frequency_ < 20000000 || flash_frequency_ > 120000000) {
        return unexpected(MAKE_ERROR(CONFIG_INVALID_VALUE, "Flash frequency out of range"));
    }
    
    // Validate flash size
    if (flash_size_ < 2 * 1024 * 1024 || flash_size_ > 128 * 1024 * 1024) {
        return unexpected(MAKE_ERROR(CONFIG_INVALID_VALUE, "Flash size out of range"));
    }
    
    COMPONENT_LOG_INFO("Flash configuration validated: {}MB, {}MHz, mode={}",
                      flash_size_ / (1024 * 1024), flash_frequency_ / 1000000,
                      static_cast<u32>(flash_mode_));
    
    return {};
}

Result<void> BootROM::esp32p4_initialize_basic_peripherals() {
    COMPONENT_LOG_DEBUG("Initializing ESP32-P4 basic peripherals");
    
    // Initialize UART0 for console output
    // Configure GPIO pins for UART functionality
    // Set up basic UART parameters (115200 baud, 8N1)
    
    // Initialize GPIO controller for strapping pin reading
    // Configure GPIO matrix for basic I/O functionality
    
    // Initialize system timer for delays and timing
    // Set up basic interrupt handling
    
    // Initialize flash controller for memory access
    // Configure SPI flash interface for reading
    
    COMPONENT_LOG_DEBUG("Basic peripherals initialized");
    return {};
}

Result<void> BootROM::esp32p4_transfer_to_bootloader() {
    COMPONENT_LOG_DEBUG("Transferring control to ESP32-P4 bootloader");
    
    // Create ESP32-P4 boot sequence if not already created
    if (!esp32p4_boot_sequence_) {
        if (memory_controller_ && cpu_manager_) {
            esp32p4_boot_sequence_ = std::make_unique<esp_idf::ESP32P4BootSequence>();
            esp32p4_boot_sequence_->set_memory_controller(memory_controller_);
            esp32p4_boot_sequence_->set_cpu_manager(cpu_manager_);
            
            COMPONENT_LOG_DEBUG("ESP32-P4 boot sequence created and configured");
        } else {
            COMPONENT_LOG_WARN("Memory controller or CPU manager not available, using simplified boot");
        }
    }
    
    // Execute ESP32-P4 boot sequence if available
    if (esp32p4_boot_sequence_) {
        auto boot_result = esp32p4_boot_sequence_->execute_complete_boot_sequence();
        if (!boot_result.has_value()) {
            COMPONENT_LOG_ERROR("ESP32-P4 boot sequence failed: {}", boot_result.error().message());
            return unexpected(boot_result.error());
        }
        
        // Update current entry point to application entry
        if (esp32p4_boot_sequence_->is_boot_complete()) {
            current_entry_point_ = boot_params_.application_address;
            COMPONENT_LOG_INFO("Boot ROM transfer completed, application entry: 0x{:08X}",
                              current_entry_point_);
        }
    } else {
        // Fallback to traditional boot ROM behavior
        COMPONENT_LOG_DEBUG("Using traditional boot ROM sequence");
        RETURN_IF_ERROR(loadBootloader());
        current_entry_point_ = boot_params_.bootloader_address;
    }
    
    return {};
}

} // namespace m5tab5::emulator