#include "emulator/cpu/instruction_decoder.hpp"
#include "emulator/cpu/instruction_set.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("InstructionDecoder");

InstructionDecoder::InstructionDecoder() {
    COMPONENT_LOG_DEBUG("InstructionDecoder created");
}

InstructionDecoder::~InstructionDecoder() {
    COMPONENT_LOG_DEBUG("InstructionDecoder destroyed");
}

Result<DecodedInstruction> InstructionDecoder::decode(u32 instruction) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    
    // Extract opcode (bits 6:0)
    decoded.opcode = instruction & 0x7F;
    
    // Check for compressed instructions (16-bit)
    if ((instruction & 0x3) != 0x3) {
        return decode_compressed_instruction(instruction & 0xFFFF);
    }
    
    // Decode based on opcode
    switch (decoded.opcode) {
        case 0x37:  // LUI
            return decode_u_type(instruction, InstructionType::LUI);
            
        case 0x17:  // AUIPC
            return decode_u_type(instruction, InstructionType::AUIPC);
            
        case 0x6F:  // JAL
            return decode_j_type(instruction, InstructionType::JAL);
            
        case 0x67:  // JALR
            return decode_i_type(instruction, InstructionType::JALR);
            
        case 0x63:  // Branch instructions
            return decode_branch_instruction(instruction);
            
        case 0x03:  // Load instructions
            return decode_load_instruction(instruction);
            
        case 0x23:  // Store instructions
            return decode_store_instruction(instruction);
            
        case 0x13:  // Immediate arithmetic
            return decode_immediate_instruction(instruction);
            
        case 0x33:  // Register arithmetic
            return decode_arithmetic_instruction(instruction);
            
        case 0x0F:  // FENCE
            return decode_i_type(instruction, InstructionType::FENCE);
            
        case 0x73:  // System instructions
            return decode_system_instruction(instruction);
            
        case 0x2F:  // Atomic instructions (RV32A)
            return decode_atomic_instruction(instruction);
            
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Unknown opcode: 0x" + std::to_string(decoded.opcode)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_r_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.rd = (instruction >> 7) & 0x1F;
    decoded.funct3 = (instruction >> 12) & 0x7;
    decoded.rs1 = (instruction >> 15) & 0x1F;
    decoded.rs2 = (instruction >> 20) & 0x1F;
    decoded.funct7 = (instruction >> 25) & 0x7F;
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_i_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.rd = (instruction >> 7) & 0x1F;
    decoded.funct3 = (instruction >> 12) & 0x7;
    decoded.rs1 = (instruction >> 15) & 0x1F;
    
    // Sign-extend 12-bit immediate
    i32 imm = static_cast<i32>(instruction) >> 20;
    decoded.immediate = imm;
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_s_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.funct3 = (instruction >> 12) & 0x7;
    decoded.rs1 = (instruction >> 15) & 0x1F;
    decoded.rs2 = (instruction >> 20) & 0x1F;
    
    // Construct 12-bit immediate from bits [11:5] and [4:0]
    i32 imm = ((instruction >> 7) & 0x1F) |  // bits [4:0]
              ((instruction >> 20) & 0xFE0);  // bits [11:5]
    
    // Sign extend
    if (imm & 0x800) {
        imm |= 0xFFFFF000;
    }
    decoded.immediate = imm;
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_b_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.funct3 = (instruction >> 12) & 0x7;
    decoded.rs1 = (instruction >> 15) & 0x1F;
    decoded.rs2 = (instruction >> 20) & 0x1F;
    
    // Construct 13-bit immediate (bit 0 is always 0)
    i32 imm = ((instruction >> 7) & 0x1E) |    // bits [4:1]
              ((instruction >> 20) & 0x7E0) |   // bits [10:5]
              ((instruction << 4) & 0x800) |    // bit [11]
              ((instruction >> 19) & 0x1000);   // bit [12]
    
    // Sign extend
    if (imm & 0x1000) {
        imm |= 0xFFFFE000;
    }
    decoded.immediate = imm;
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_u_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.rd = (instruction >> 7) & 0x1F;
    
    // 20-bit immediate in upper bits
    decoded.immediate = static_cast<i32>(instruction & 0xFFFFF000);
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_j_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.rd = (instruction >> 7) & 0x1F;
    
    // Construct 21-bit immediate (bit 0 is always 0)
    i32 imm = ((instruction >> 20) & 0x7FE) |    // bits [10:1]
              ((instruction >> 9) & 0x800) |      // bit [11]
              (instruction & 0xFF000) |           // bits [19:12]
              ((instruction >> 11) & 0x100000);   // bit [20]
    
    // Sign extend
    if (imm & 0x100000) {
        imm |= 0xFFE00000;
    }
    decoded.immediate = imm;
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_branch_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    
    InstructionType type;
    switch (funct3) {
        case 0x0: type = InstructionType::BEQ; break;
        case 0x1: type = InstructionType::BNE; break;
        case 0x4: type = InstructionType::BLT; break;
        case 0x5: type = InstructionType::BGE; break;
        case 0x6: type = InstructionType::BLTU; break;
        case 0x7: type = InstructionType::BGEU; break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid branch funct3: " + std::to_string(funct3)));
    }
    
    return decode_b_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_load_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    
    InstructionType type;
    switch (funct3) {
        case 0x0: type = InstructionType::LB; break;
        case 0x1: type = InstructionType::LH; break;
        case 0x2: type = InstructionType::LW; break;
        case 0x4: type = InstructionType::LBU; break;
        case 0x5: type = InstructionType::LHU; break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid load funct3: " + std::to_string(funct3)));
    }
    
    return decode_i_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_store_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    
    InstructionType type;
    switch (funct3) {
        case 0x0: type = InstructionType::SB; break;
        case 0x1: type = InstructionType::SH; break;
        case 0x2: type = InstructionType::SW; break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid store funct3: " + std::to_string(funct3)));
    }
    
    return decode_s_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_immediate_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    u8 funct7 = (instruction >> 25) & 0x7F;
    
    InstructionType type;
    switch (funct3) {
        case 0x0: type = InstructionType::ADDI; break;
        case 0x2: type = InstructionType::SLTI; break;
        case 0x3: type = InstructionType::SLTIU; break;
        case 0x4: type = InstructionType::XORI; break;
        case 0x6: type = InstructionType::ORI; break;
        case 0x7: type = InstructionType::ANDI; break;
        case 0x1: type = InstructionType::SLLI; break;
        case 0x5:
            if (funct7 == 0x00) {
                type = InstructionType::SRLI;
            } else if (funct7 == 0x20) {
                type = InstructionType::SRAI;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid immediate shift funct7: " + std::to_string(funct7)));
            }
            break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid immediate arithmetic funct3: " + std::to_string(funct3)));
    }
    
    return decode_i_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_arithmetic_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    u8 funct7 = (instruction >> 25) & 0x7F;
    
    InstructionType type;
    switch (funct3) {
        case 0x0:
            if (funct7 == 0x00) {
                type = InstructionType::ADD;
            } else if (funct7 == 0x20) {
                type = InstructionType::SUB;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MUL;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid ADD/SUB funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x1:
            if (funct7 == 0x00) {
                type = InstructionType::SLL;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULH;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLL funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x2:
            if (funct7 == 0x00) {
                type = InstructionType::SLT;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULHSU;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLT funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x3:
            if (funct7 == 0x00) {
                type = InstructionType::SLTU;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULHU;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLTU funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x4:
            if (funct7 == 0x00) {
                type = InstructionType::XOR;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::DIV;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid XOR funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x5:
            if (funct7 == 0x00) {
                type = InstructionType::SRL;
            } else if (funct7 == 0x20) {
                type = InstructionType::SRA;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::DIVU;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid shift funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x6:
            if (funct7 == 0x00) {
                type = InstructionType::OR;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::REM;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid OR funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x7:
            if (funct7 == 0x00) {
                type = InstructionType::AND;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::REMU;
            } else {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid AND funct7: " + std::to_string(funct7)));
            }
            break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid register arithmetic funct3: " + std::to_string(funct3)));
    }
    
    return decode_r_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_system_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    
    if (funct3 == 0x0) {
        u32 imm = instruction >> 20;
        if (imm == 0x000) {
            return decode_i_type(instruction, InstructionType::ECALL);
        } else if (imm == 0x001) {
            return decode_i_type(instruction, InstructionType::EBREAK);
        } else if (imm == 0x302) {
            return decode_i_type(instruction, InstructionType::MRET);
        } else {
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid system instruction immediate: " + std::to_string(imm)));
        }
    } else if (funct3 == 0x1) {
        return decode_i_type(instruction, InstructionType::CSRRW);
    } else if (funct3 == 0x2) {
        return decode_i_type(instruction, InstructionType::CSRRS);
    } else if (funct3 == 0x3) {
        return decode_i_type(instruction, InstructionType::CSRRC);
    } else if (funct3 == 0x5) {
        return decode_i_type(instruction, InstructionType::CSRRWI);
    } else if (funct3 == 0x6) {
        return decode_i_type(instruction, InstructionType::CSRRSI);
    } else if (funct3 == 0x7) {
        return decode_i_type(instruction, InstructionType::CSRRCI);
    } else {
        return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
            "Invalid system instruction funct3: " + std::to_string(funct3)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_atomic_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    u8 funct5 = (instruction >> 27) & 0x1F;
    
    if (funct3 != 0x2) {  // Only 32-bit atomics supported
        return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
            "Invalid atomic instruction width"));
    }
    
    InstructionType type;
    switch (funct5) {
        case 0x02: type = InstructionType::LR_W; break;
        case 0x03: type = InstructionType::SC_W; break;
        case 0x01: type = InstructionType::AMOSWAP_W; break;
        case 0x00: type = InstructionType::AMOADD_W; break;
        case 0x04: type = InstructionType::AMOXOR_W; break;
        case 0x0C: type = InstructionType::AMOAND_W; break;
        case 0x08: type = InstructionType::AMOOR_W; break;
        case 0x10: type = InstructionType::AMOMIN_W; break;
        case 0x14: type = InstructionType::AMOMAX_W; break;
        case 0x18: type = InstructionType::AMOMINU_W; break;
        case 0x1C: type = InstructionType::AMOMAXU_W; break;
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid atomic funct5: " + std::to_string(funct5)));
    }
    
    return decode_r_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_compressed_instruction(u16 instruction) {
    // RV32C compressed instruction decoding
    u8 opcode = instruction & 0x3;
    
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.compressed = true;
    
    switch (opcode) {
        case 0x0:  // C0 - Stack pointer based loads/stores and arithmetic
            return decode_compressed_quadrant_0(instruction);
        case 0x1:  // C1 - Control flow and integer computation
            return decode_compressed_quadrant_1(instruction);
        case 0x2:  // C2 - Stack pointer based loads/stores and register moves
            return decode_compressed_quadrant_2(instruction);
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid compressed instruction opcode: " + std::to_string(opcode)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_compressed_quadrant_0(u16 instruction) {
    u8 funct3 = (instruction >> 13) & 0x7;
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.compressed = true;
    decoded.valid = true;
    decoded.format = InstructionFormat::COMPRESSED;
    decoded.opcode = instruction & 0x3;
    
    switch (funct3) {
        case 0x0: {  // C.ADDI4SPN - Add immediate to stack pointer
            u16 imm = ((instruction >> 7) & 0x30) |   // imm[5:4]
                      ((instruction >> 1) & 0x3C0) |  // imm[9:6]
                      ((instruction << 4) & 0x4) |    // imm[2]
                      ((instruction << 1) & 0x8);     // imm[3]
            if (imm == 0) {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "C.ADDI4SPN with zero immediate is reserved"));
            }
            decoded.type = InstructionType::ADDI;
            decoded.rd = ((instruction >> 2) & 0x7) + 8;  // rd' + 8
            decoded.rs1 = 2;  // sp
            decoded.immediate = imm;
            break;
        }
        case 0x2: {  // C.LW - Load word
            u16 imm = ((instruction << 1) & 0x40) |    // imm[6]
                      ((instruction >> 7) & 0x38) |   // imm[5:3]
                      ((instruction << 4) & 0x4);     // imm[2]
            decoded.type = InstructionType::LW;
            decoded.rd = ((instruction >> 2) & 0x7) + 8;   // rd' + 8
            decoded.rs1 = ((instruction >> 7) & 0x7) + 8;  // rs1' + 8
            decoded.immediate = imm;
            break;
        }
        case 0x6: {  // C.SW - Store word
            u16 imm = ((instruction << 1) & 0x40) |    // imm[6]
                      ((instruction >> 7) & 0x38) |   // imm[5:3]
                      ((instruction << 4) & 0x4);     // imm[2]
            decoded.type = InstructionType::SW;
            decoded.rs1 = ((instruction >> 7) & 0x7) + 8;  // rs1' + 8
            decoded.rs2 = ((instruction >> 2) & 0x7) + 8;  // rs2' + 8
            decoded.immediate = imm;
            break;
        }
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid C0 quadrant instruction: funct3=" + std::to_string(funct3)));
    }
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_compressed_quadrant_1(u16 instruction) {
    u8 funct3 = (instruction >> 13) & 0x7;
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.compressed = true;
    decoded.valid = true;
    decoded.format = InstructionFormat::COMPRESSED;
    decoded.opcode = instruction & 0x3;
    
    switch (funct3) {
        case 0x0: {  // C.ADDI - Add immediate
            i32 imm = ((instruction >> 7) & 0x20) |   // imm[5]
                      ((instruction >> 2) & 0x1F);    // imm[4:0]
            // Sign extend
            if (imm & 0x20) imm |= 0xFFFFFFC0;
            decoded.type = InstructionType::ADDI;
            decoded.rd = (instruction >> 7) & 0x1F;
            decoded.rs1 = decoded.rd;  // Same as rd
            decoded.immediate = imm;
            break;
        }
        case 0x1: {  // C.JAL - Jump and link (RV32 only)
            i32 imm = ((instruction << 3) & 0x800) |    // imm[11]
                      ((instruction >> 7) & 0x10) |     // imm[4]
                      ((instruction >> 1) & 0x300) |    // imm[9:8]
                      ((instruction << 1) & 0x400) |    // imm[10]
                      ((instruction >> 1) & 0x40) |     // imm[6]
                      ((instruction << 3) & 0x80) |     // imm[7]
                      ((instruction >> 2) & 0xE) |      // imm[3:1]
                      ((instruction << 2) & 0x20);      // imm[5]
            // Sign extend
            if (imm & 0x800) imm |= 0xFFFFF000;
            decoded.type = InstructionType::JAL;
            decoded.rd = 1;  // ra
            decoded.immediate = imm;
            break;
        }
        case 0x2: {  // C.LI - Load immediate
            i32 imm = ((instruction >> 7) & 0x20) |   // imm[5]
                      ((instruction >> 2) & 0x1F);    // imm[4:0]
            // Sign extend
            if (imm & 0x20) imm |= 0xFFFFFFC0;
            decoded.type = InstructionType::ADDI;
            decoded.rd = (instruction >> 7) & 0x1F;
            decoded.rs1 = 0;  // x0
            decoded.immediate = imm;
            break;
        }
        case 0x3: {  // C.LUI / C.ADDI16SP
            u8 rd = (instruction >> 7) & 0x1F;
            if (rd == 2) {  // C.ADDI16SP
                i32 imm = ((instruction << 3) & 0x200) |    // imm[9]
                          ((instruction << 1) & 0x40) |     // imm[6]
                          ((instruction << 4) & 0x180) |    // imm[8:7]
                          ((instruction >> 2) & 0x10) |     // imm[4]
                          ((instruction << 3) & 0x20);      // imm[5]
                // Sign extend
                if (imm & 0x200) imm |= 0xFFFFFE00;
                if (imm == 0) {
                    return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                        "C.ADDI16SP with zero immediate is reserved"));
                }
                decoded.type = InstructionType::ADDI;
                decoded.rd = 2;  // sp
                decoded.rs1 = 2;  // sp
                decoded.immediate = imm;
            } else {  // C.LUI
                i32 imm = ((instruction << 5) & 0x20000) |  // imm[17]
                          ((instruction << 10) & 0x1F000);  // imm[16:12]
                // Sign extend
                if (imm & 0x20000) imm |= 0xFFFC0000;
                if (imm == 0 || rd == 0) {
                    return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                        "C.LUI with zero immediate or rd=x0 is reserved"));
                }
                decoded.type = InstructionType::LUI;
                decoded.rd = rd;
                decoded.immediate = imm;
            }
            break;
        }
        case 0x4: {  // Misc-ALU operations
            u8 funct2 = (instruction >> 10) & 0x3;
            u8 rd_rs1_p = ((instruction >> 7) & 0x7) + 8;
            
            switch (funct2) {
                case 0x0: {  // C.SRLI
                    u8 shamt = (instruction >> 2) & 0x1F;
                    decoded.type = InstructionType::SRLI;
                    decoded.rd = rd_rs1_p;
                    decoded.rs1 = rd_rs1_p;
                    decoded.immediate = shamt;
                    break;
                }
                case 0x1: {  // C.SRAI
                    u8 shamt = (instruction >> 2) & 0x1F;
                    decoded.type = InstructionType::SRAI;
                    decoded.rd = rd_rs1_p;
                    decoded.rs1 = rd_rs1_p;
                    decoded.immediate = shamt;
                    break;
                }
                case 0x2: {  // C.ANDI
                    i32 imm = ((instruction >> 7) & 0x20) |   // imm[5]
                              ((instruction >> 2) & 0x1F);    // imm[4:0]
                    // Sign extend
                    if (imm & 0x20) imm |= 0xFFFFFFC0;
                    decoded.type = InstructionType::ANDI;
                    decoded.rd = rd_rs1_p;
                    decoded.rs1 = rd_rs1_p;
                    decoded.immediate = imm;
                    break;
                }
                case 0x3: {  // Register-register operations
                    u8 funct2_low = (instruction >> 5) & 0x3;
                    u8 rs2_p = ((instruction >> 2) & 0x7) + 8;
                    
                    switch (funct2_low) {
                        case 0x0:  // C.SUB
                            decoded.type = InstructionType::SUB;
                            break;
                        case 0x1:  // C.XOR
                            decoded.type = InstructionType::XOR;
                            break;
                        case 0x2:  // C.OR
                            decoded.type = InstructionType::OR;
                            break;
                        case 0x3:  // C.AND
                            decoded.type = InstructionType::AND;
                            break;
                    }
                    decoded.rd = rd_rs1_p;
                    decoded.rs1 = rd_rs1_p;
                    decoded.rs2 = rs2_p;
                    break;
                }
            }
            break;
        }
        case 0x5: {  // C.J - Jump
            i32 imm = ((instruction << 3) & 0x800) |    // imm[11]
                      ((instruction >> 7) & 0x10) |     // imm[4]
                      ((instruction >> 1) & 0x300) |    // imm[9:8]
                      ((instruction << 1) & 0x400) |    // imm[10]
                      ((instruction >> 1) & 0x40) |     // imm[6]
                      ((instruction << 3) & 0x80) |     // imm[7]
                      ((instruction >> 2) & 0xE) |      // imm[3:1]
                      ((instruction << 2) & 0x20);      // imm[5]
            // Sign extend
            if (imm & 0x800) imm |= 0xFFFFF000;
            decoded.type = InstructionType::JAL;
            decoded.rd = 0;  // x0
            decoded.immediate = imm;
            break;
        }
        case 0x6: {  // C.BEQZ - Branch if equal to zero
            i32 imm = ((instruction >> 4) & 0x100) |    // imm[8]
                      ((instruction << 1) & 0xC0) |     // imm[7:6]
                      ((instruction << 3) & 0x20) |     // imm[5]
                      ((instruction >> 7) & 0x18) |     // imm[4:3]
                      ((instruction >> 2) & 0x6);       // imm[2:1]
            // Sign extend
            if (imm & 0x100) imm |= 0xFFFFFE00;
            decoded.type = InstructionType::BEQ;
            decoded.rs1 = ((instruction >> 7) & 0x7) + 8;  // rs1' + 8
            decoded.rs2 = 0;  // x0
            decoded.immediate = imm;
            break;
        }
        case 0x7: {  // C.BNEZ - Branch if not equal to zero
            i32 imm = ((instruction >> 4) & 0x100) |    // imm[8]
                      ((instruction << 1) & 0xC0) |     // imm[7:6]
                      ((instruction << 3) & 0x20) |     // imm[5]
                      ((instruction >> 7) & 0x18) |     // imm[4:3]
                      ((instruction >> 2) & 0x6);       // imm[2:1]
            // Sign extend
            if (imm & 0x100) imm |= 0xFFFFFE00;
            decoded.type = InstructionType::BNE;
            decoded.rs1 = ((instruction >> 7) & 0x7) + 8;  // rs1' + 8
            decoded.rs2 = 0;  // x0
            decoded.immediate = imm;
            break;
        }
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid C1 quadrant instruction: funct3=" + std::to_string(funct3)));
    }
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_compressed_quadrant_2(u16 instruction) {
    u8 funct3 = (instruction >> 13) & 0x7;
    DecodedInstruction decoded{};
    decoded.raw_instruction = instruction;
    decoded.compressed = true;
    decoded.valid = true;
    decoded.format = InstructionFormat::COMPRESSED;
    decoded.opcode = instruction & 0x3;
    
    switch (funct3) {
        case 0x0: {  // C.SLLI - Shift left logical immediate
            u8 shamt = (instruction >> 2) & 0x1F;
            u8 rd = (instruction >> 7) & 0x1F;
            if (rd == 0) {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "C.SLLI with rd=x0 is reserved"));
            }
            decoded.type = InstructionType::SLLI;
            decoded.rd = rd;
            decoded.rs1 = rd;
            decoded.immediate = shamt;
            break;
        }
        case 0x2: {  // C.LWSP - Load word from stack pointer
            u16 imm = ((instruction << 4) & 0xC0) |    // imm[7:6]
                      ((instruction >> 7) & 0x20) |    // imm[5]
                      ((instruction >> 2) & 0x1C);     // imm[4:2]
            u8 rd = (instruction >> 7) & 0x1F;
            if (rd == 0) {
                return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "C.LWSP with rd=x0 is reserved"));
            }
            decoded.type = InstructionType::LW;
            decoded.rd = rd;
            decoded.rs1 = 2;  // sp
            decoded.immediate = imm;
            break;
        }
        case 0x4: {  // C.JR / C.MV / C.EBREAK / C.JALR / C.ADD
            u8 rs1 = (instruction >> 7) & 0x1F;
            u8 rs2 = (instruction >> 2) & 0x1F;
            
            if ((instruction >> 12) & 0x1) {  // bit 12 = 1
                if (rs2 == 0) {
                    if (rs1 == 0) {  // C.EBREAK
                        decoded.type = InstructionType::EBREAK;
                    } else {  // C.JALR
                        decoded.type = InstructionType::JALR;
                        decoded.rd = 1;  // ra
                        decoded.rs1 = rs1;
                        decoded.immediate = 0;
                    }
                } else {  // C.ADD
                    decoded.type = InstructionType::ADD;
                    decoded.rd = rs1;
                    decoded.rs1 = rs1;
                    decoded.rs2 = rs2;
                }
            } else {  // bit 12 = 0
                if (rs2 == 0) {  // C.JR
                    if (rs1 == 0) {
                        return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                            "C.JR with rs1=x0 is reserved"));
                    }
                    decoded.type = InstructionType::JALR;
                    decoded.rd = 0;  // x0
                    decoded.rs1 = rs1;
                    decoded.immediate = 0;
                } else {  // C.MV
                    decoded.type = InstructionType::ADD;
                    decoded.rd = rs1;
                    decoded.rs1 = 0;  // x0
                    decoded.rs2 = rs2;
                }
            }
            break;
        }
        case 0x6: {  // C.SWSP - Store word to stack pointer
            u16 imm = ((instruction >> 1) & 0xC0) |    // imm[7:6]
                      ((instruction >> 7) & 0x3C);     // imm[5:2]
            decoded.type = InstructionType::SW;
            decoded.rs1 = 2;  // sp
            decoded.rs2 = (instruction >> 2) & 0x1F;
            decoded.immediate = imm;
            break;
        }
        default:
            return unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid C2 quadrant instruction: funct3=" + std::to_string(funct3)));
    }
    
    return decoded;
}

}  // namespace m5tab5::emulator