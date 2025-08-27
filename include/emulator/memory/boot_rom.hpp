#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/memory/memory_region.hpp"

#include <array>
#include <memory>

namespace m5tab5::emulator {

// Forward declarations

/**
 * @brief ESP32-P4 Boot ROM emulation
 * 
 * Emulates the ESP32-P4 Boot ROM with:
 * - Reset vector at 0x40000080
 * - Boot sequence implementation
 * - Strapping pin detection
 * - MMU initialization for flash XIP
 * - Power domain management
 * - Transfer to application code
 */
class BootROM {
public:
    struct BootParams {
        u32 magic_number;           // Magic number for validation
        u32 boot_mode;              // Boot mode from strapping pins
        u32 cpu_frequency;          // CPU frequency setting
        u32 flash_frequency;        // Flash SPI frequency
        u32 flash_mode;             // Flash SPI mode (QIO, QOUT, DIO, DOUT)
        u32 flash_size;             // Flash size detected
        u32 bootloader_address;     // Bootloader entry point
        u32 application_address;    // Application entry point
        u32 reserved[24];           // Reserved for future use
    };

    enum class BootMode : u32 {
        FLASH_BOOT = 0,         // Normal flash boot
        UART_BOOT = 1,          // UART download mode
        SPI_BOOT = 2,           // SPI download mode
        SDIO_BOOT = 3,          // SDIO download mode
        JTAG_BOOT = 4,          // JTAG boot mode
        USB_BOOT = 5            // USB download mode
    };

    enum class FlashMode : u32 {
        QIO = 0,                // Quad I/O
        QOUT = 1,               // Quad Output
        DIO = 2,                // Dual I/O
        DOUT = 3                // Dual Output
    };

    BootROM();
    ~BootROM();

    // Initialization
    Result<void> initialize();
    void shutdown();
    Result<void> reset();

    // Boot ROM access
    std::shared_ptr<MemoryRegion> getMemoryRegion() const { return memory_region_; }
    
    // Boot sequence
    Result<void> executeBootSequence();
    Result<Address> getResetVector() const;
    Result<Address> getBootloaderEntryPoint() const;
    Result<Address> getApplicationEntryPoint() const;

    // Boot parameters
    Result<void> setBootParameters(const BootParams& params);
    Result<BootParams> getBootParameters() const;
    
    // Strapping pin simulation
    void setStrappingPins(u32 pin_state);
    u32 getStrappingPins() const { return strapping_pins_; }
    BootMode detectBootMode() const;

    // Flash configuration
    void setFlashConfiguration(FlashMode mode, u32 frequency, u32 size);
    
    // MMU management
    Result<void> initializeMMU();
    Result<void> mapFlashRegion(Address virtual_base, Address physical_base, size_t size);

private:
    // Boot ROM binary data generation
    Result<void> generateBootROMBinary();
    void generateResetHandler();
    void generateBootSequence();
    void generateParameterData();
    
    // Boot sequence implementation
    Result<void> initializeClocks();
    Result<void> initializePowerDomains();
    Result<void> configureMmu();
    Result<void> loadBootloader();
    Result<void> transferControl(Address entry_point);
    
    // Strapping pin logic
    BootMode decodeStrappingPins(u32 pin_state) const;
    
    // Memory region management
    std::shared_ptr<MemoryRegion> memory_region_;
    std::array<u8, 32 * 1024> rom_data_;  // 32KB Boot ROM
    
    // Boot configuration
    BootParams boot_params_;
    u32 strapping_pins_;
    FlashMode flash_mode_;
    u32 flash_frequency_;
    u32 flash_size_;
    
    // State tracking
    bool initialized_;
    bool mmu_configured_;
    Address current_entry_point_;
};

/**
 * @brief Boot ROM instruction generator
 * 
 * Generates RISC-V instructions for the boot sequence
 */
class BootROMGenerator {
public:
    static constexpr u32 NOP = 0x00000013;          // addi x0, x0, 0
    static constexpr u32 EBREAK = 0x00100073;       // ebreak instruction
    
    // Generate common RISC-V instructions
    static u32 generateLUI(u8 rd, u32 imm);         // Load Upper Immediate
    static u32 generateAUIPC(u8 rd, u32 imm);       // Add Upper Immediate to PC
    static u32 generateJAL(u8 rd, i32 imm);         // Jump and Link
    static u32 generateJALR(u8 rd, u8 rs1, i32 imm); // Jump and Link Register
    static u32 generateLW(u8 rd, u8 rs1, i32 imm);  // Load Word
    static u32 generateSW(u8 rs2, u8 rs1, i32 imm); // Store Word
    static u32 generateADDI(u8 rd, u8 rs1, i32 imm); // Add Immediate
    
    // Generate boot sequence instructions
    static std::vector<u32> generateInitSequence();
    static std::vector<u32> generateStrappingCheck();
    static std::vector<u32> generateMMUSetup();
    static std::vector<u32> generateJumpToBootloader(Address bootloader_addr);

private:
    // Instruction encoding helpers
    static u32 encodeIType(u32 opcode, u8 rd, u8 funct3, u8 rs1, i32 imm);
    static u32 encodeUType(u32 opcode, u8 rd, u32 imm);
    static u32 encodeJType(u32 opcode, u8 rd, i32 imm);
};

} // namespace m5tab5::emulator