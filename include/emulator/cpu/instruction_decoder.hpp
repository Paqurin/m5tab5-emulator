#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"

#include <unordered_map>

namespace m5tab5::emulator {

// Forward declarations for types defined elsewhere
enum class InstructionFormat;

/**
 * @brief RISC-V instruction decoding for ESP32-P4 emulation
 * 
 * Decodes both standard 32-bit and compressed 16-bit RISC-V instructions
 * supporting the RV32IMAC instruction set used by ESP32-P4.
 */

// Use InstructionFormat from instruction_set.hpp

// Instruction types for execution dispatch
enum class InstructionType {
    // Basic instruction categories
    ALU,        // Arithmetic and logic
    LOAD,       // Memory load
    STORE,      // Memory store
    BRANCH,     // Conditional branch
    JUMP,       // Unconditional jump
    SYSTEM,     // System calls and CSR
    MULTIPLY,   // Multiplication/division (M extension)
    ATOMIC,     // Atomic operations (A extension)
    COMPRESSED, // Compressed instructions (C extension)
    
    // Specific RISC-V instructions
    LUI,        // Load Upper Immediate
    AUIPC,      // Add Upper Immediate to PC
    JAL,        // Jump And Link
    JALR,       // Jump And Link Register
    
    // Branch instructions
    BEQ,        // Branch if Equal
    BNE,        // Branch if Not Equal
    BLT,        // Branch if Less Than
    BGE,        // Branch if Greater or Equal
    BLTU,       // Branch if Less Than (Unsigned)
    BGEU,       // Branch if Greater or Equal (Unsigned)
    
    // Load instructions
    LB,         // Load Byte
    LH,         // Load Halfword
    LW,         // Load Word
    LBU,        // Load Byte Unsigned
    LHU,        // Load Halfword Unsigned
    
    // Store instructions
    SB,         // Store Byte
    SH,         // Store Halfword
    SW,         // Store Word
    
    // Immediate arithmetic
    ADDI,       // Add Immediate
    SLTI,       // Set Less Than Immediate
    SLTIU,      // Set Less Than Immediate Unsigned
    XORI,       // XOR Immediate
    ORI,        // OR Immediate
    ANDI,       // AND Immediate
    SLLI,       // Shift Left Logical Immediate
    SRLI,       // Shift Right Logical Immediate
    SRAI,       // Shift Right Arithmetic Immediate
    
    // Register arithmetic
    ADD,        // Add
    SUB,        // Subtract
    SLL,        // Shift Left Logical
    SLT,        // Set Less Than
    SLTU,       // Set Less Than Unsigned
    XOR,        // XOR
    SRL,        // Shift Right Logical
    SRA,        // Shift Right Arithmetic
    OR,         // OR
    AND,        // AND
    
    // M Extension - Integer Multiply/Divide
    MUL,        // Multiply
    MULH,       // Multiply High Signed
    MULHSU,     // Multiply High Signed x Unsigned
    MULHU,      // Multiply High Unsigned
    DIV,        // Divide Signed
    DIVU,       // Divide Unsigned
    REM,        // Remainder Signed
    REMU,       // Remainder Unsigned
    
    // CSR Instructions (Zicsr Extension)
    CSRRW,      // CSR Read Write
    CSRRS,      // CSR Read Set
    CSRRC,      // CSR Read Clear
    CSRRWI,     // CSR Read Write Immediate
    CSRRSI,     // CSR Read Set Immediate
    CSRRCI,     // CSR Read Clear Immediate
    
    // System Instructions
    ECALL,      // Environment Call
    EBREAK,     // Environment Break
    MRET,       // Machine Return
    
    // A Extension - Atomic Instructions
    LR_W,       // Load Reserved Word
    SC_W,       // Store Conditional Word
    AMOSWAP_W,  // Atomic Memory Operation: Swap Word
    AMOADD_W,   // Atomic Memory Operation: Add Word
    AMOXOR_W,   // Atomic Memory Operation: XOR Word
    AMOAND_W,   // Atomic Memory Operation: AND Word
    AMOOR_W,    // Atomic Memory Operation: OR Word
    AMOMIN_W,   // Atomic Memory Operation: Min Word
    AMOMAX_W,   // Atomic Memory Operation: Max Word
    AMOMINU_W,  // Atomic Memory Operation: Min Unsigned Word
    AMOMAXU_W,  // Atomic Memory Operation: Max Unsigned Word
    
    // Memory ordering
    FENCE,      // Memory fence
    
    UNKNOWN     // Invalid/unimplemented
};

// Forward declaration of DecodedInstruction - defined after InstructionType
struct DecodedInstruction;

// Extended instruction information for decoder
struct ExtendedInstructionInfo {
    InstructionType type;       // Instruction type for execution
    bool is_compressed;         // True for 16-bit compressed instructions
    bool is_branch;            // True for branch/jump instructions
    bool accesses_memory;      // True for load/store instructions
    bool modifies_register;    // True if instruction writes to rd
    
    ExtendedInstructionInfo() 
        : type(InstructionType::UNKNOWN), is_compressed(false), 
          is_branch(false), accesses_memory(false), modifies_register(false) {}
};

class InstructionDecoder {
public:
    InstructionDecoder();
    ~InstructionDecoder();
    
    // Main decoding interface
    Result<DecodedInstruction> decode(u32 instruction);
    
    // Utility functions
    static bool is_compressed_instruction(u32 instruction);
    static u32 get_instruction_length(u32 instruction);
    static const char* get_instruction_name(u32 opcode, u32 funct3, u32 funct7);
    
    // Statistics
    u64 get_instructions_decoded() const { return instructions_decoded_; }
    u64 get_compressed_instructions_decoded() const { return compressed_decoded_; }

private:
    // Instruction counting for statistics
    u64 instructions_decoded_;
    u64 compressed_decoded_;
    
    // Decoding dispatch methods
    Result<DecodedInstruction> decode_r_type(u32 instruction, InstructionType type = InstructionType::ALU);
    Result<DecodedInstruction> decode_i_type(u32 instruction, InstructionType type = InstructionType::ALU);
    Result<DecodedInstruction> decode_s_type(u32 instruction, InstructionType type = InstructionType::STORE);
    Result<DecodedInstruction> decode_b_type(u32 instruction, InstructionType type = InstructionType::BRANCH);
    Result<DecodedInstruction> decode_u_type(u32 instruction, InstructionType type = InstructionType::ALU);
    Result<DecodedInstruction> decode_j_type(u32 instruction, InstructionType type = InstructionType::JUMP);
    Result<DecodedInstruction> decode_compressed_instruction(u16 instruction);

    // Specific instruction decoders
    Result<DecodedInstruction> decode_branch_instruction(u32 instruction);
    Result<DecodedInstruction> decode_load_instruction(u32 instruction);
    Result<DecodedInstruction> decode_store_instruction(u32 instruction);
    Result<DecodedInstruction> decode_arithmetic_instruction(u32 instruction);
    Result<DecodedInstruction> decode_immediate_instruction(u32 instruction);
    Result<DecodedInstruction> decode_system_instruction(u32 instruction);
    Result<DecodedInstruction> decode_atomic_instruction(u32 instruction);
    
    // Immediate extraction helpers
    i32 extract_i_immediate(u32 instruction);
    i32 extract_s_immediate(u32 instruction);
    i32 extract_b_immediate(u32 instruction);
    i32 extract_u_immediate(u32 instruction);
    i32 extract_j_immediate(u32 instruction);
    
    // Compressed instruction helpers
    Result<DecodedInstruction> decode_compressed_quadrant_0(u16 instruction);
    Result<DecodedInstruction> decode_compressed_quadrant_1(u16 instruction);
    Result<DecodedInstruction> decode_compressed_quadrant_2(u16 instruction);
    
    // Instruction classification
    InstructionType classify_instruction(u32 opcode, u32 funct3, u32 funct7);
    const char* get_mnemonic(u32 opcode, u32 funct3, u32 funct7);
    
    // Lookup tables for fast decoding
    static const std::unordered_map<u32, const char*> opcode_names_;
    static const std::unordered_map<u32, InstructionType> opcode_types_;
};

} // namespace m5tab5::emulator