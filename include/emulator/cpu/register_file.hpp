#pragma once

#include "emulator/core/types.hpp"
#include <array>
#include <string>
#include <unordered_map>

namespace m5tab5::emulator {

/**
 * @brief RISC-V register file implementation
 * 
 * Manages:
 * - 32 general-purpose registers (x0-x31)
 * - Control and Status Registers (CSRs)
 * - Floating-point registers (f0-f31) if enabled
 * - Performance counters
 */
class RegisterFile {
public:
    // Register indices
    enum Register : uint8_t {
        // General-purpose registers (ABI names)
        ZERO = 0,  // x0 - Hard-wired zero
        RA = 1,    // x1 - Return address
        SP = 2,    // x2 - Stack pointer
        GP = 3,    // x3 - Global pointer
        TP = 4,    // x4 - Thread pointer
        T0 = 5,    // x5 - Temporary
        T1 = 6,    // x6 - Temporary
        T2 = 7,    // x7 - Temporary
        S0 = 8,    // x8 - Saved register/frame pointer
        S1 = 9,    // x9 - Saved register
        A0 = 10,   // x10 - Function argument/return value
        A1 = 11,   // x11 - Function argument/return value
        A2 = 12,   // x12 - Function argument
        A3 = 13,   // x13 - Function argument
        A4 = 14,   // x14 - Function argument
        A5 = 15,   // x15 - Function argument
        A6 = 16,   // x16 - Function argument
        A7 = 17,   // x17 - Function argument
        S2 = 18,   // x18 - Saved register
        S3 = 19,   // x19 - Saved register
        S4 = 20,   // x20 - Saved register
        S5 = 21,   // x21 - Saved register
        S6 = 22,   // x22 - Saved register
        S7 = 23,   // x23 - Saved register
        S8 = 24,   // x24 - Saved register
        S9 = 25,   // x25 - Saved register
        S10 = 26,  // x26 - Saved register
        S11 = 27,  // x27 - Saved register
        T3 = 28,   // x28 - Temporary
        T4 = 29,   // x29 - Temporary
        T5 = 30,   // x30 - Temporary
        T6 = 31    // x31 - Temporary
    };

    // Important CSR addresses
    enum CSR : uint32_t {
        // Machine-level CSRs
        MSTATUS = 0x300,    // Machine status
        MISA = 0x301,       // ISA and extensions
        MIE = 0x304,        // Machine interrupt enable
        MTVEC = 0x305,      // Machine trap vector
        MSCRATCH = 0x340,   // Machine scratch register
        MEPC = 0x341,       // Machine exception PC
        MCAUSE = 0x342,     // Machine trap cause
        MTVAL = 0x343,      // Machine trap value
        MIP = 0x344,        // Machine interrupt pending
        
        // Performance counters
        MCYCLE = 0xB00,     // Machine cycle counter
        MINSTRET = 0xB02,   // Machine instructions retired
        MCYCLEH = 0xB80,    // Upper 32 bits of mcycle
        MINSTRETH = 0xB82,  // Upper 32 bits of minstret
        
        // User-level CSRs
        CYCLE = 0xC00,      // Cycle counter
        INSTRET = 0xC02,    // Instructions retired
        CYCLEH = 0xC80,     // Upper 32 bits of cycle
        INSTRETH = 0xC82    // Upper 32 bits of instret
    };

    RegisterFile();
    ~RegisterFile();

    // General-purpose register access
    uint32_t read(uint8_t reg) const;
    void write(uint8_t reg, uint32_t value);

    // CSR access
    uint32_t readCSR(uint32_t csr) const;
    void writeCSR(uint32_t csr, uint32_t value);
    bool isValidCSR(uint32_t csr) const;

    // Floating-point register access (if enabled)
    uint32_t readFloat(uint8_t reg) const;
    void writeFloat(uint8_t reg, uint32_t value);
    bool hasFloatingPoint() const { return has_floating_point_; }
    void enableFloatingPoint(bool enable) { has_floating_point_ = enable; }

    // Reset to initial state
    void reset();

    // Stack operations (convenience functions)
    void pushStack(uint32_t value);
    uint32_t popStack();
    uint32_t getStackPointer() const { return read(SP); }
    void setStackPointer(uint32_t value) { write(SP, value); }

    // Debug and inspection
    std::string getRegisterName(uint8_t reg) const;
    std::string getCSRName(uint32_t csr) const;
    std::string dump() const;
    
    // Performance counters
    void incrementCycleCounter() { cycle_counter_++; }
    void incrementInstructionCounter() { instruction_counter_++; }
    uint64_t getCycleCounter() const { return cycle_counter_; }
    uint64_t getInstructionCounter() const { return instruction_counter_; }

    // Interrupt and exception handling
    struct TrapInfo {
        bool interrupt;
        uint32_t cause;
        uint32_t value;
        Address pc;
    };
    
    void enterTrap(const TrapInfo& trap_info);
    void exitTrap();
    bool inTrap() const { return trap_level_ > 0; }

private:
    // Register storage
    std::array<uint32_t, 32> gp_registers_{};
    std::array<uint32_t, 32> fp_registers_{};
    std::unordered_map<uint32_t, uint32_t> csr_registers_;

    // State flags
    bool has_floating_point_ = false;
    uint32_t trap_level_ = 0;

    // Performance counters (64-bit)
    uint64_t cycle_counter_ = 0;
    uint64_t instruction_counter_ = 0;

    // CSR validation and access control
    bool isCSRReadable(uint32_t csr) const;
    bool isCSRWritable(uint32_t csr) const;
    uint32_t getPrivilegeLevel() const; // M=3, S=1, U=0

    // Initialize default CSR values
    void initializeCSRs();
    
    // Handle special CSR side effects
    void handleCSRWrite(uint32_t csr, uint32_t value);
    uint32_t handleCSRRead(uint32_t csr) const;
};

} // namespace m5tab5::emulator