#include "emulator/cpu/cpu_core.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("InstructionExecution");

Result<void> CpuCore::execute_branch_instruction(const DecodedInstruction& decoded) {
    u32 rs1_val = register_file_.read_gp_register(decoded.rs1);
    u32 rs2_val = register_file_.read_gp_register(decoded.rs2);
    bool take_branch = false;
    
    switch (decoded.type) {
        case InstructionType::BEQ:
            take_branch = (rs1_val == rs2_val);
            break;
            
        case InstructionType::BNE:
            take_branch = (rs1_val != rs2_val);
            break;
            
        case InstructionType::BLT:
            take_branch = (static_cast<i32>(rs1_val) < static_cast<i32>(rs2_val));
            break;
            
        case InstructionType::BGE:
            take_branch = (static_cast<i32>(rs1_val) >= static_cast<i32>(rs2_val));
            break;
            
        case InstructionType::BLTU:
            take_branch = (rs1_val < rs2_val);
            break;
            
        case InstructionType::BGEU:
            take_branch = (rs1_val >= rs2_val);
            break;
            
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid branch instruction"));
    }
    
    if (take_branch) {
        u32 target = register_file_.get_pc() - 4 + decoded.immediate;  // PC-4 because we already incremented
        register_file_.set_pc(target);
    }
    
    return {};
}

Result<void> CpuCore::execute_load_instruction(const DecodedInstruction& decoded) {
    u32 base = register_file_.read_gp_register(decoded.rs1);
    u32 address = base + decoded.immediate;
    
    switch (decoded.type) {
        case InstructionType::LB: {
            auto result = memory_controller_->read_u8(address);
            if (!result) return std::unexpected(result.error());
            
            // Sign extend 8-bit to 32-bit
            i32 value = static_cast<i8>(result.value());
            register_file_.write_gp_register(decoded.rd, static_cast<u32>(value));
            break;
        }
        
        case InstructionType::LH: {
            auto result = memory_controller_->read_u16(address);
            if (!result) return std::unexpected(result.error());
            
            // Sign extend 16-bit to 32-bit
            i32 value = static_cast<i16>(result.value());
            register_file_.write_gp_register(decoded.rd, static_cast<u32>(value));
            break;
        }
        
        case InstructionType::LW: {
            auto result = memory_controller_->read_u32(address);
            if (!result) return std::unexpected(result.error());
            
            register_file_.write_gp_register(decoded.rd, result.value());
            break;
        }
        
        case InstructionType::LBU: {
            auto result = memory_controller_->read_u8(address);
            if (!result) return std::unexpected(result.error());
            
            // Zero extend 8-bit to 32-bit
            register_file_.write_gp_register(decoded.rd, static_cast<u32>(result.value()));
            break;
        }
        
        case InstructionType::LHU: {
            auto result = memory_controller_->read_u16(address);
            if (!result) return std::unexpected(result.error());
            
            // Zero extend 16-bit to 32-bit
            register_file_.write_gp_register(decoded.rd, static_cast<u32>(result.value()));
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid load instruction"));
    }
    
    return {};
}

Result<void> CpuCore::execute_store_instruction(const DecodedInstruction& decoded) {
    u32 base = register_file_.read_gp_register(decoded.rs1);
    u32 address = base + decoded.immediate;
    u32 value = register_file_.read_gp_register(decoded.rs2);
    
    switch (decoded.type) {
        case InstructionType::SB: {
            auto result = memory_controller_->write_u8(address, static_cast<u8>(value));
            if (!result) return std::unexpected(result.error());
            break;
        }
        
        case InstructionType::SH: {
            auto result = memory_controller_->write_u16(address, static_cast<u16>(value));
            if (!result) return std::unexpected(result.error());
            break;
        }
        
        case InstructionType::SW: {
            auto result = memory_controller_->write_u32(address, value);
            if (!result) return std::unexpected(result.error());
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid store instruction"));
    }
    
    return {};
}

Result<void> CpuCore::execute_immediate_arithmetic(const DecodedInstruction& decoded) {
    u32 rs1_val = register_file_.read_gp_register(decoded.rs1);
    u32 result = 0;
    
    switch (decoded.type) {
        case InstructionType::ADDI:
            result = rs1_val + decoded.immediate;
            break;
            
        case InstructionType::SLTI:
            result = (static_cast<i32>(rs1_val) < decoded.immediate) ? 1 : 0;
            break;
            
        case InstructionType::SLTIU:
            result = (rs1_val < static_cast<u32>(decoded.immediate)) ? 1 : 0;
            break;
            
        case InstructionType::XORI:
            result = rs1_val ^ decoded.immediate;
            break;
            
        case InstructionType::ORI:
            result = rs1_val | decoded.immediate;
            break;
            
        case InstructionType::ANDI:
            result = rs1_val & decoded.immediate;
            break;
            
        case InstructionType::SLLI: {
            u8 shamt = decoded.immediate & 0x1F;  // Only lower 5 bits
            result = rs1_val << shamt;
            break;
        }
        
        case InstructionType::SRLI: {
            u8 shamt = decoded.immediate & 0x1F;  // Only lower 5 bits
            result = rs1_val >> shamt;
            break;
        }
        
        case InstructionType::SRAI: {
            u8 shamt = decoded.immediate & 0x1F;  // Only lower 5 bits
            result = static_cast<u32>(static_cast<i32>(rs1_val) >> shamt);
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid immediate arithmetic instruction"));
    }
    
    register_file_.write_gp_register(decoded.rd, result);
    return {};
}

Result<void> CpuCore::execute_register_arithmetic(const DecodedInstruction& decoded) {
    u32 rs1_val = register_file_.read_gp_register(decoded.rs1);
    u32 rs2_val = register_file_.read_gp_register(decoded.rs2);
    u32 result = 0;
    
    switch (decoded.type) {
        case InstructionType::ADD:
            result = rs1_val + rs2_val;
            break;
            
        case InstructionType::SUB:
            result = rs1_val - rs2_val;
            break;
            
        case InstructionType::SLL: {
            u8 shamt = rs2_val & 0x1F;  // Only lower 5 bits
            result = rs1_val << shamt;
            break;
        }
        
        case InstructionType::SLT:
            result = (static_cast<i32>(rs1_val) < static_cast<i32>(rs2_val)) ? 1 : 0;
            break;
            
        case InstructionType::SLTU:
            result = (rs1_val < rs2_val) ? 1 : 0;
            break;
            
        case InstructionType::XOR:
            result = rs1_val ^ rs2_val;
            break;
            
        case InstructionType::SRL: {
            u8 shamt = rs2_val & 0x1F;  // Only lower 5 bits
            result = rs1_val >> shamt;
            break;
        }
        
        case InstructionType::SRA: {
            u8 shamt = rs2_val & 0x1F;  // Only lower 5 bits
            result = static_cast<u32>(static_cast<i32>(rs1_val) >> shamt);
            break;
        }
        
        case InstructionType::OR:
            result = rs1_val | rs2_val;
            break;
            
        case InstructionType::AND:
            result = rs1_val & rs2_val;
            break;
            
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid register arithmetic instruction"));
    }
    
    register_file_.write_gp_register(decoded.rd, result);
    return {};
}

Result<void> CpuCore::execute_multiplication_division(const DecodedInstruction& decoded) {
    u32 rs1_val = register_file_.read_gp_register(decoded.rs1);
    u32 rs2_val = register_file_.read_gp_register(decoded.rs2);
    u32 result = 0;
    
    switch (decoded.type) {
        case InstructionType::MUL: {
            i64 product = static_cast<i64>(static_cast<i32>(rs1_val)) * 
                         static_cast<i64>(static_cast<i32>(rs2_val));
            result = static_cast<u32>(product);
            break;
        }
        
        case InstructionType::MULH: {
            i64 product = static_cast<i64>(static_cast<i32>(rs1_val)) * 
                         static_cast<i64>(static_cast<i32>(rs2_val));
            result = static_cast<u32>(product >> 32);
            break;
        }
        
        case InstructionType::MULHSU: {
            i64 product = static_cast<i64>(static_cast<i32>(rs1_val)) * 
                         static_cast<i64>(rs2_val);
            result = static_cast<u32>(product >> 32);
            break;
        }
        
        case InstructionType::MULHU: {
            u64 product = static_cast<u64>(rs1_val) * static_cast<u64>(rs2_val);
            result = static_cast<u32>(product >> 32);
            break;
        }
        
        case InstructionType::DIV: {
            if (rs2_val == 0) {
                result = 0xFFFFFFFF;  // Division by zero result
            } else if (rs1_val == 0x80000000 && rs2_val == 0xFFFFFFFF) {
                result = 0x80000000;  // Overflow case
            } else {
                result = static_cast<u32>(static_cast<i32>(rs1_val) / static_cast<i32>(rs2_val));
            }
            break;
        }
        
        case InstructionType::DIVU: {
            if (rs2_val == 0) {
                result = 0xFFFFFFFF;  // Division by zero result
            } else {
                result = rs1_val / rs2_val;
            }
            break;
        }
        
        case InstructionType::REM: {
            if (rs2_val == 0) {
                result = rs1_val;  // Remainder of division by zero
            } else if (rs1_val == 0x80000000 && rs2_val == 0xFFFFFFFF) {
                result = 0;  // Overflow case
            } else {
                result = static_cast<u32>(static_cast<i32>(rs1_val) % static_cast<i32>(rs2_val));
            }
            break;
        }
        
        case InstructionType::REMU: {
            if (rs2_val == 0) {
                result = rs1_val;  // Remainder of division by zero
            } else {
                result = rs1_val % rs2_val;
            }
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid multiplication/division instruction"));
    }
    
    register_file_.write_gp_register(decoded.rd, result);
    return {};
}

Result<void> CpuCore::execute_csr_instruction(const DecodedInstruction& decoded) {
    u16 csr_addr = decoded.immediate & 0xFFF;
    u32 rd_val = 0;
    u32 rs1_val = register_file_.read_gp_register(decoded.rs1);
    
    // Read current CSR value
    auto current_val_result = register_file_.read_csr(csr_addr);
    if (!current_val_result) {
        return std::unexpected(current_val_result.error());
    }
    
    u32 current_val = current_val_result.value();
    rd_val = current_val;  // Always write old value to rd
    
    u32 new_val = current_val;
    
    switch (decoded.type) {
        case InstructionType::CSRRW:
            new_val = rs1_val;
            break;
            
        case InstructionType::CSRRS:
            if (decoded.rs1 != 0) {  // Only write if rs1 != 0
                new_val = current_val | rs1_val;
            }
            break;
            
        case InstructionType::CSRRC:
            if (decoded.rs1 != 0) {  // Only write if rs1 != 0
                new_val = current_val & ~rs1_val;
            }
            break;
            
        case InstructionType::CSRRWI:
            new_val = decoded.rs1;  // rs1 field contains immediate
            break;
            
        case InstructionType::CSRRSI:
            if (decoded.rs1 != 0) {  // Only write if immediate != 0
                new_val = current_val | decoded.rs1;
            }
            break;
            
        case InstructionType::CSRRCI:
            if (decoded.rs1 != 0) {  // Only write if immediate != 0
                new_val = current_val & ~decoded.rs1;
            }
            break;
            
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid CSR instruction"));
    }
    
    // Write new value to CSR
    if (new_val != current_val) {
        auto write_result = register_file_.write_csr(csr_addr, new_val);
        if (!write_result) {
            return std::unexpected(write_result.error());
        }
    }
    
    // Write old value to rd
    register_file_.write_gp_register(decoded.rd, rd_val);
    
    return {};
}

Result<void> CpuCore::execute_atomic_instruction(const DecodedInstruction& decoded) {
    // Atomic instructions (RV32A extension)
    u32 address = register_file_.read_gp_register(decoded.rs1);
    u32 rs2_val = register_file_.read_gp_register(decoded.rs2);
    
    // Check address alignment for atomic operations
    if (address & 0x3) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Atomic operation on misaligned address"));
    }
    
    switch (decoded.type) {
        case InstructionType::LR_W: {
            // Load Reserved
            auto result = memory_controller_->read_u32(address);
            if (!result) return std::unexpected(result.error());
            
            register_file_.write_gp_register(decoded.rd, result.value());
            // TODO: Set reservation for this address
            break;
        }
        
        case InstructionType::SC_W: {
            // Store Conditional
            // TODO: Check if reservation is valid
            auto write_result = memory_controller_->write_u32(address, rs2_val);
            if (!write_result) {
                register_file_.write_gp_register(decoded.rd, 1);  // Failure
                return std::unexpected(write_result.error());
            }
            
            register_file_.write_gp_register(decoded.rd, 0);  // Success
            break;
        }
        
        case InstructionType::AMOSWAP_W: {
            // Atomic Swap
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            auto write_result = memory_controller_->write_u32(address, rs2_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOADD_W: {
            // Atomic Add
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = current_result.value() + rs2_val;
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOXOR_W: {
            // Atomic XOR
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = current_result.value() ^ rs2_val;
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOAND_W: {
            // Atomic AND
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = current_result.value() & rs2_val;
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOOR_W: {
            // Atomic OR
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = current_result.value() | rs2_val;
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOMIN_W: {
            // Atomic Min (signed)
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            i32 current_signed = static_cast<i32>(current_result.value());
            i32 rs2_signed = static_cast<i32>(rs2_val);
            u32 new_val = static_cast<u32>(std::min(current_signed, rs2_signed));
            
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOMAX_W: {
            // Atomic Max (signed)
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            i32 current_signed = static_cast<i32>(current_result.value());
            i32 rs2_signed = static_cast<i32>(rs2_val);
            u32 new_val = static_cast<u32>(std::max(current_signed, rs2_signed));
            
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOMINU_W: {
            // Atomic Min (unsigned)
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = std::min(current_result.value(), rs2_val);
            
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        case InstructionType::AMOMAXU_W: {
            // Atomic Max (unsigned)
            auto current_result = memory_controller_->read_u32(address);
            if (!current_result) return std::unexpected(current_result.error());
            
            u32 new_val = std::max(current_result.value(), rs2_val);
            
            auto write_result = memory_controller_->write_u32(address, new_val);
            if (!write_result) return std::unexpected(write_result.error());
            
            register_file_.write_gp_register(decoded.rd, current_result.value());
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Invalid atomic instruction"));
    }
    
    return {};
}

}  // namespace m5tab5::emulator