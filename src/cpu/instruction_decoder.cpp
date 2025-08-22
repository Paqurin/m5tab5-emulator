#include "emulator/cpu/instruction_decoder.hpp"
#include "emulator/utils/logging.hpp"

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
    decoded.raw = instruction;
    
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
            return decode_immediate_arithmetic(instruction);
            
        case 0x33:  // Register arithmetic
            return decode_register_arithmetic(instruction);
            
        case 0x0F:  // FENCE
            return decode_i_type(instruction, InstructionType::FENCE);
            
        case 0x73:  // System instructions
            return decode_system_instruction(instruction);
            
        case 0x2F:  // Atomic instructions (RV32A)
            return decode_atomic_instruction(instruction);
            
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Unknown opcode: 0x" + std::to_string(decoded.opcode)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_r_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw = instruction;
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
    decoded.raw = instruction;
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
    decoded.raw = instruction;
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
    decoded.raw = instruction;
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
    decoded.raw = instruction;
    decoded.type = type;
    decoded.opcode = instruction & 0x7F;
    decoded.rd = (instruction >> 7) & 0x1F;
    
    // 20-bit immediate in upper bits
    decoded.immediate = static_cast<i32>(instruction & 0xFFFFF000);
    
    return decoded;
}

Result<DecodedInstruction> InstructionDecoder::decode_j_type(u32 instruction, InstructionType type) {
    DecodedInstruction decoded{};
    decoded.raw = instruction;
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
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid store funct3: " + std::to_string(funct3)));
    }
    
    return decode_s_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_immediate_arithmetic(u32 instruction) {
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
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid immediate shift funct7: " + std::to_string(funct7)));
            }
            break;
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid immediate arithmetic funct3: " + std::to_string(funct3)));
    }
    
    return decode_i_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_register_arithmetic(u32 instruction) {
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
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid ADD/SUB funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x1:
            if (funct7 == 0x00) {
                type = InstructionType::SLL;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULH;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLL funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x2:
            if (funct7 == 0x00) {
                type = InstructionType::SLT;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULHSU;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLT funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x3:
            if (funct7 == 0x00) {
                type = InstructionType::SLTU;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::MULHU;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid SLTU funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x4:
            if (funct7 == 0x00) {
                type = InstructionType::XOR;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::DIV;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid shift funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x6:
            if (funct7 == 0x00) {
                type = InstructionType::OR;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::REM;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid OR funct7: " + std::to_string(funct7)));
            }
            break;
        case 0x7:
            if (funct7 == 0x00) {
                type = InstructionType::AND;
            } else if (funct7 == 0x01) {  // RV32M
                type = InstructionType::REMU;
            } else {
                return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                    "Invalid AND funct7: " + std::to_string(funct7)));
            }
            break;
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
        return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
            "Invalid system instruction funct3: " + std::to_string(funct3)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_atomic_instruction(u32 instruction) {
    u8 funct3 = (instruction >> 12) & 0x7;
    u8 funct5 = (instruction >> 27) & 0x1F;
    
    if (funct3 != 0x2) {  // Only 32-bit atomics supported
        return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
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
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid atomic funct5: " + std::to_string(funct5)));
    }
    
    return decode_r_type(instruction, type);
}

Result<DecodedInstruction> InstructionDecoder::decode_compressed_instruction(u16 instruction) {
    // RV32C compressed instruction decoding
    u8 opcode = instruction & 0x3;
    u8 funct3 = (instruction >> 13) & 0x7;
    
    DecodedInstruction decoded{};
    decoded.raw = instruction;
    decoded.compressed = true;
    
    switch (opcode) {
        case 0x0:  // C0 - Stack pointer based loads/stores and arithmetic
            return decode_c0_instruction(instruction);
        case 0x1:  // C1 - Control flow and integer computation
            return decode_c1_instruction(instruction);
        case 0x2:  // C2 - Stack pointer based loads/stores and register moves
            return decode_c2_instruction(instruction);
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid compressed instruction opcode: " + std::to_string(opcode)));
    }
}

Result<DecodedInstruction> InstructionDecoder::decode_c0_instruction(u16 instruction) {
    // Placeholder for C0 quadrant instructions
    return std::unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
        "C0 compressed instructions not yet implemented"));
}

Result<DecodedInstruction> InstructionDecoder::decode_c1_instruction(u16 instruction) {
    // Placeholder for C1 quadrant instructions
    return std::unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
        "C1 compressed instructions not yet implemented"));
}

Result<DecodedInstruction> InstructionDecoder::decode_c2_instruction(u16 instruction) {
    // Placeholder for C2 quadrant instructions
    return std::unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
        "C2 compressed instructions not yet implemented"));
}

const char* InstructionDecoder::get_instruction_name(InstructionType type) {
    switch (type) {
        // RV32I Base Instructions
        case InstructionType::LUI: return "LUI";
        case InstructionType::AUIPC: return "AUIPC";
        case InstructionType::JAL: return "JAL";
        case InstructionType::JALR: return "JALR";
        case InstructionType::BEQ: return "BEQ";
        case InstructionType::BNE: return "BNE";
        case InstructionType::BLT: return "BLT";
        case InstructionType::BGE: return "BGE";
        case InstructionType::BLTU: return "BLTU";
        case InstructionType::BGEU: return "BGEU";
        case InstructionType::LB: return "LB";
        case InstructionType::LH: return "LH";
        case InstructionType::LW: return "LW";
        case InstructionType::LBU: return "LBU";
        case InstructionType::LHU: return "LHU";
        case InstructionType::SB: return "SB";
        case InstructionType::SH: return "SH";
        case InstructionType::SW: return "SW";
        case InstructionType::ADDI: return "ADDI";
        case InstructionType::SLTI: return "SLTI";
        case InstructionType::SLTIU: return "SLTIU";
        case InstructionType::XORI: return "XORI";
        case InstructionType::ORI: return "ORI";
        case InstructionType::ANDI: return "ANDI";
        case InstructionType::SLLI: return "SLLI";
        case InstructionType::SRLI: return "SRLI";
        case InstructionType::SRAI: return "SRAI";
        case InstructionType::ADD: return "ADD";
        case InstructionType::SUB: return "SUB";
        case InstructionType::SLL: return "SLL";
        case InstructionType::SLT: return "SLT";
        case InstructionType::SLTU: return "SLTU";
        case InstructionType::XOR: return "XOR";
        case InstructionType::SRL: return "SRL";
        case InstructionType::SRA: return "SRA";
        case InstructionType::OR: return "OR";
        case InstructionType::AND: return "AND";
        case InstructionType::FENCE: return "FENCE";
        case InstructionType::ECALL: return "ECALL";
        case InstructionType::EBREAK: return "EBREAK";
        
        // RV32M Multiplication/Division
        case InstructionType::MUL: return "MUL";
        case InstructionType::MULH: return "MULH";
        case InstructionType::MULHSU: return "MULHSU";
        case InstructionType::MULHU: return "MULHU";
        case InstructionType::DIV: return "DIV";
        case InstructionType::DIVU: return "DIVU";
        case InstructionType::REM: return "REM";
        case InstructionType::REMU: return "REMU";
        
        // RV32A Atomic
        case InstructionType::LR_W: return "LR.W";
        case InstructionType::SC_W: return "SC.W";
        case InstructionType::AMOSWAP_W: return "AMOSWAP.W";
        case InstructionType::AMOADD_W: return "AMOADD.W";
        case InstructionType::AMOXOR_W: return "AMOXOR.W";
        case InstructionType::AMOAND_W: return "AMOAND.W";
        case InstructionType::AMOOR_W: return "AMOOR.W";
        case InstructionType::AMOMIN_W: return "AMOMIN.W";
        case InstructionType::AMOMAX_W: return "AMOMAX.W";
        case InstructionType::AMOMINU_W: return "AMOMINU.W";
        case InstructionType::AMOMAXU_W: return "AMOMAXU.W";
        
        // System
        case InstructionType::CSRRW: return "CSRRW";
        case InstructionType::CSRRS: return "CSRRS";
        case InstructionType::CSRRC: return "CSRRC";
        case InstructionType::CSRRWI: return "CSRRWI";
        case InstructionType::CSRRSI: return "CSRRSI";
        case InstructionType::CSRRCI: return "CSRRCI";
        case InstructionType::MRET: return "MRET";
        
        default: return "UNKNOWN";
    }
}

}  // namespace m5tab5::emulator