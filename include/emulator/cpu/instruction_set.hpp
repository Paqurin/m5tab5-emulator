#pragma once

#include "emulator/core/types.hpp"
#include <functional>
#include <unordered_map>

namespace m5tab5::emulator {

// RISC-V instruction formats
enum class InstructionFormat {
    R_TYPE,     // Register-register operations
    I_TYPE,     // Immediate operations
    S_TYPE,     // Store operations
    B_TYPE,     // Branch operations
    U_TYPE,     // Upper immediate
    J_TYPE,     // Jump operations
    COMPRESSED  // RV32C compressed instructions
};

// Exception types
enum class ExceptionType {
    InstructionAddressMisaligned = 0,
    InstructionAccessFault = 1,
    IllegalInstruction = 2,
    Breakpoint = 3,
    LoadAddressMisaligned = 4,
    LoadAccessFault = 5,
    StoreAddressMisaligned = 6,
    StoreAccessFault = 7,
    EnvironmentCallFromUMode = 8,
    EnvironmentCallFromMMode = 11,
    InstructionPageFault = 12,
    LoadPageFault = 13,
    StorePageFault = 15
};

// Trap types
enum class TrapType {
    UserSoftwareInterrupt = 0,
    SupervisorSoftwareInterrupt = 1,
    MachineSoftwareInterrupt = 3,
    UserTimerInterrupt = 4,
    SupervisorTimerInterrupt = 5,
    MachineTimerInterrupt = 7,
    UserExternalInterrupt = 8,
    SupervisorExternalInterrupt = 9,
    MachineExternalInterrupt = 11
};

// Forward declaration for InstructionType
enum class InstructionType;

// Decoded instruction representation
struct DecodedInstruction {
    uint32_t raw_instruction = 0;
    InstructionFormat format = InstructionFormat::R_TYPE;
    InstructionType type; // Instruction type for execution dispatch (no default to avoid circular include)
    
    // Common fields
    uint8_t opcode = 0;
    uint8_t rd = 0;     // Destination register
    uint8_t rs1 = 0;    // Source register 1
    uint8_t rs2 = 0;    // Source register 2
    uint8_t funct3 = 0; // Function code 3-bit
    uint8_t funct7 = 0; // Function code 7-bit
    
    // Immediate values
    int32_t immediate = 0;
    
    // Execution info
    bool valid = false;
    bool compressed = false; // RV32C compressed instruction flag
    std::string mnemonic;
    Address pc = 0;
};

// Forward declaration
class CPUCore;
class RegisterFile;

/**
 * @brief RISC-V instruction set implementation
 * 
 * Implements the RV32IMAC instruction set:
 * - RV32I: Base integer instruction set
 * - RV32M: Integer multiplication and division
 * - RV32A: Atomic instructions
 * - RV32C: Compressed instructions
 */
class InstructionSet {
public:
    using InstructionHandler = std::function<EmulatorError(CPUCore&, const DecodedInstruction&)>;

    explicit InstructionSet();
    ~InstructionSet();

    // Instruction decoding
    EmulatorError decode(uint32_t instruction, Address pc, DecodedInstruction& decoded);
    
    // Instruction execution
    EmulatorError execute(CPUCore& cpu, const DecodedInstruction& instruction);

    // Disassembly
    std::string disassemble(const DecodedInstruction& instruction);

    // Instruction set features
    bool supportsCompressed() const { return supports_compressed_; }
    bool supportsMultiplication() const { return supports_multiplication_; }
    bool supportsAtomic() const { return supports_atomic_; }
    bool supportsFloatingPoint() const { return supports_floating_point_; }

    void setFeatureSupport(bool compressed, bool multiplication, 
                          bool atomic, bool floating_point);

private:
    // Instruction format decoders
    void decodeRType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeIType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeSType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeBType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeUType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeJType(uint32_t instruction, DecodedInstruction& decoded);
    void decodeCompressed(uint32_t instruction, DecodedInstruction& decoded);

    // Immediate extraction
    int32_t extractIImmediate(uint32_t instruction);
    int32_t extractSImmediate(uint32_t instruction);
    int32_t extractBImmediate(uint32_t instruction);
    int32_t extractUImmediate(uint32_t instruction);
    int32_t extractJImmediate(uint32_t instruction);

    // Instruction handlers by category
    void registerBaseInstructions();
    void registerMultiplicationInstructions();
    void registerAtomicInstructions();
    void registerCompressedInstructions();
    void registerSystemInstructions();

    // Base instruction handlers (RV32I)
    EmulatorError executeLUI(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeAUIPC(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeJAL(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeJALR(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeBranch(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeLoad(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeStore(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeALUImmediate(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeALU(CPUCore& cpu, const DecodedInstruction& inst);
    
    // System instructions
    EmulatorError executeECALL(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeEBREAK(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeCSR(CPUCore& cpu, const DecodedInstruction& inst);

    // Multiplication/Division (RV32M)
    EmulatorError executeMUL(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeDIV(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeREM(CPUCore& cpu, const DecodedInstruction& inst);

    // Atomic operations (RV32A)
    EmulatorError executeLR(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeSC(CPUCore& cpu, const DecodedInstruction& inst);
    EmulatorError executeAMO(CPUCore& cpu, const DecodedInstruction& inst);

    // Helper functions
    EmulatorError checkAlignment(Address address, size_t size);
    EmulatorError signExtend(uint32_t value, int bits, int32_t& result);

    // Instruction table
    std::unordered_map<uint32_t, InstructionHandler> instruction_table_;
    
    // Feature flags
    bool supports_compressed_ = true;
    bool supports_multiplication_ = true;
    bool supports_atomic_ = true;
    bool supports_floating_point_ = false;

    // Mnemonic table for disassembly
    std::unordered_map<uint32_t, std::string> mnemonic_table_;
};

} // namespace m5tab5::emulator