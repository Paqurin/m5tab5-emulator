#include "emulator/cpu/cpu_core.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("CpuCore");

CpuCore::CpuCore(CoreId core_id)
    : core_id_(core_id),
      state_(CpuState::STOPPED),
      memory_controller_(nullptr),
      frequency_(400000000),  // 400MHz default
      cycles_executed_(0),
      instructions_executed_(0) {
    
    COMPONENT_LOG_DEBUG("CpuCore {} created", static_cast<int>(core_id_));
}

CpuCore::~CpuCore() {
    if (state_ != CpuState::STOPPED) {
        stop();
    }
    COMPONENT_LOG_DEBUG("CpuCore {} destroyed", static_cast<int>(core_id_));
}

Result<void> CpuCore::initialize(const Configuration& config, MemoryController& memory_controller) {
    if (state_ != CpuState::STOPPED) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "CPU core already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing CPU core {}", static_cast<int>(core_id_));
    
    memory_controller_ = &memory_controller;
    
    // Set frequency based on core type
    if (core_id_ == CoreId::LP_CORE) {
        frequency_ = config.get_lp_cpu_frequency();
    } else {
        frequency_ = config.get_main_cpu_frequency();
    }
    
    // Initialize register file
    register_file_.reset();
    
    // Set initial PC based on core type
    if (core_id_ == CoreId::LP_CORE) {
        register_file_.set_pc(0x50000000);  // LP core reset vector
    } else {
        register_file_.set_pc(0x10000000);  // Main core reset vector (Flash start)
    }
    
    // Set hart ID in mhartid CSR
    register_file_.write_csr(CSR_MHARTID, static_cast<u32>(core_id_));
    
    // Initialize instruction decoder
    decoder_ = std::make_unique<InstructionDecoder>();
    
    // Reset statistics
    cycles_executed_ = 0;
    instructions_executed_ = 0;
    
    state_ = CpuState::HALTED;
    COMPONENT_LOG_INFO("CPU core {} initialized successfully", static_cast<int>(core_id_));
    
    return {};
}

Result<void> CpuCore::start() {
    if (state_ != CpuState::HALTED) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "CPU core must be halted to start"));
    }
    
    COMPONENT_LOG_INFO("Starting CPU core {}", static_cast<int>(core_id_));
    state_ = CpuState::RUNNING;
    
    return {};
}

Result<void> CpuCore::stop() {
    if (state_ == CpuState::STOPPED) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Stopping CPU core {}", static_cast<int>(core_id_));
    state_ = CpuState::STOPPED;
    
    return {};
}

Result<void> CpuCore::pause() {
    if (state_ != CpuState::RUNNING) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "CPU core must be running to pause"));
    }
    
    COMPONENT_LOG_DEBUG("Pausing CPU core {}", static_cast<int>(core_id_));
    state_ = CpuState::PAUSED;
    
    return {};
}

Result<void> CpuCore::resume() {
    if (state_ != CpuState::PAUSED) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "CPU core must be paused to resume"));
    }
    
    COMPONENT_LOG_DEBUG("Resuming CPU core {}", static_cast<int>(core_id_));
    state_ = CpuState::RUNNING;
    
    return {};
}

Result<void> CpuCore::reset() {
    COMPONENT_LOG_INFO("Resetting CPU core {}", static_cast<int>(core_id_));
    
    // Reset register file
    register_file_.reset();
    
    // Set initial PC
    if (core_id_ == CoreId::LP_CORE) {
        register_file_.set_pc(0x50000000);
    } else {
        register_file_.set_pc(0x10000000);
    }
    
    // Reset statistics
    cycles_executed_ = 0;
    instructions_executed_ = 0;
    
    state_ = CpuState::HALTED;
    
    return {};
}

Result<Cycles> CpuCore::execute_cycles(Cycles max_cycles) {
    if (state_ != CpuState::RUNNING) {
        return 0;
    }
    
    Cycles cycles_remaining = max_cycles;
    Cycles cycles_executed = 0;
    
    while (cycles_remaining > 0 && state_ == CpuState::RUNNING) {
        // Execute one instruction
        auto result = execute_single_instruction();
        if (!result) {
            if (result.error().code() == ErrorCode::CPU_HALT) {
                state_ = CpuState::HALTED;
                break;
            }
            return std::unexpected(result.error());
        }
        
        Cycles instruction_cycles = result.value();
        cycles_executed += instruction_cycles;
        cycles_remaining -= instruction_cycles;
        
        // Update global cycle count
        cycles_executed_ += instruction_cycles;
        register_file_.increment_cycle_count();
    }
    
    return cycles_executed;
}

Result<Cycles> CpuCore::execute_single_instruction() {
    if (!memory_controller_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Memory controller not initialized"));
    }
    
    // Fetch instruction
    u32 pc = register_file_.get_pc();
    auto instruction_result = fetch_instruction(pc);
    if (!instruction_result) {
        return std::unexpected(instruction_result.error());
    }
    
    u32 instruction = instruction_result.value();
    
    // Decode instruction
    auto decoded_result = decoder_->decode(instruction);
    if (!decoded_result) {
        return std::unexpected(decoded_result.error());
    }
    
    DecodedInstruction decoded = decoded_result.value();
    
    COMPONENT_LOG_TRACE("Core {} PC=0x{:08X}: {} (0x{:08X})",
                       static_cast<int>(core_id_), pc,
                       InstructionDecoder::get_instruction_name(decoded.type),
                       instruction);
    
    // Execute instruction
    auto execute_result = execute_instruction(decoded);
    if (!execute_result) {
        return std::unexpected(execute_result.error());
    }
    
    // Update instruction count
    instructions_executed_++;
    register_file_.increment_instruction_count();
    
    // Most instructions take 1 cycle, some take more
    Cycles cycles = get_instruction_cycles(decoded.type);
    
    return cycles;
}

CpuState CpuCore::get_state() const {
    return state_;
}

CoreId CpuCore::get_core_id() const {
    return core_id_;
}

u32 CpuCore::get_frequency() const {
    return frequency_;
}

Cycles CpuCore::get_cycles_executed() const {
    return cycles_executed_;
}

u64 CpuCore::get_instructions_executed() const {
    return instructions_executed_;
}

const RegisterFile& CpuCore::get_register_file() const {
    return register_file_;
}

RegisterFile& CpuCore::get_register_file() {
    return register_file_;
}

Result<u32> CpuCore::fetch_instruction(u32 address) {
    // Check instruction alignment
    if (address & 0x1) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Instruction fetch from misaligned address: 0x" + std::to_string(address)));
    }
    
    // Check if this is a compressed instruction
    auto first_half = memory_controller_->read_u16(address);
    if (!first_half) {
        return std::unexpected(first_half.error());
    }
    
    u16 first_half_val = first_half.value();
    
    // Check if it's a 32-bit instruction (bits [1:0] == 11)
    if ((first_half_val & 0x3) == 0x3) {
        // 32-bit instruction - read the full 32 bits
        auto full_instruction = memory_controller_->read_u32(address);
        if (!full_instruction) {
            return std::unexpected(full_instruction.error());
        }
        
        // Advance PC by 4
        register_file_.set_pc(address + 4);
        return full_instruction.value();
    } else {
        // 16-bit compressed instruction
        // Advance PC by 2
        register_file_.set_pc(address + 2);
        return static_cast<u32>(first_half_val);
    }
}

Result<void> CpuCore::execute_instruction(const DecodedInstruction& decoded) {
    switch (decoded.type) {
        // Load Upper Immediate
        case InstructionType::LUI:
            register_file_.write_gp_register(decoded.rd, decoded.immediate);
            break;
            
        // Add Upper Immediate to PC
        case InstructionType::AUIPC:
            register_file_.write_gp_register(decoded.rd, 
                register_file_.get_pc() - 4 + decoded.immediate);  // PC-4 because we already incremented
            break;
            
        // Jump and Link
        case InstructionType::JAL:
            register_file_.write_gp_register(decoded.rd, register_file_.get_pc());
            register_file_.set_pc(register_file_.get_pc() - 4 + decoded.immediate);
            break;
            
        // Jump and Link Register
        case InstructionType::JALR: {
            u32 target = (register_file_.read_gp_register(decoded.rs1) + decoded.immediate) & ~1;
            register_file_.write_gp_register(decoded.rd, register_file_.get_pc());
            register_file_.set_pc(target);
            break;
        }
        
        // Branch instructions
        case InstructionType::BEQ:
        case InstructionType::BNE:
        case InstructionType::BLT:
        case InstructionType::BGE:
        case InstructionType::BLTU:
        case InstructionType::BGEU:
            RETURN_IF_ERROR(execute_branch_instruction(decoded));
            break;
            
        // Load instructions
        case InstructionType::LB:
        case InstructionType::LH:
        case InstructionType::LW:
        case InstructionType::LBU:
        case InstructionType::LHU:
            RETURN_IF_ERROR(execute_load_instruction(decoded));
            break;
            
        // Store instructions
        case InstructionType::SB:
        case InstructionType::SH:
        case InstructionType::SW:
            RETURN_IF_ERROR(execute_store_instruction(decoded));
            break;
            
        // Immediate arithmetic
        case InstructionType::ADDI:
        case InstructionType::SLTI:
        case InstructionType::SLTIU:
        case InstructionType::XORI:
        case InstructionType::ORI:
        case InstructionType::ANDI:
        case InstructionType::SLLI:
        case InstructionType::SRLI:
        case InstructionType::SRAI:
            RETURN_IF_ERROR(execute_immediate_arithmetic(decoded));
            break;
            
        // Register arithmetic
        case InstructionType::ADD:
        case InstructionType::SUB:
        case InstructionType::SLL:
        case InstructionType::SLT:
        case InstructionType::SLTU:
        case InstructionType::XOR:
        case InstructionType::SRL:
        case InstructionType::SRA:
        case InstructionType::OR:
        case InstructionType::AND:
            RETURN_IF_ERROR(execute_register_arithmetic(decoded));
            break;
            
        // RV32M Multiplication/Division
        case InstructionType::MUL:
        case InstructionType::MULH:
        case InstructionType::MULHSU:
        case InstructionType::MULHU:
        case InstructionType::DIV:
        case InstructionType::DIVU:
        case InstructionType::REM:
        case InstructionType::REMU:
            RETURN_IF_ERROR(execute_multiplication_division(decoded));
            break;
            
        // System instructions
        case InstructionType::ECALL:
            return std::unexpected(MAKE_ERROR(CPU_EXCEPTION, "ECALL instruction"));
            
        case InstructionType::EBREAK:
            return std::unexpected(MAKE_ERROR(CPU_BREAKPOINT_HIT, "EBREAK instruction"));
            
        case InstructionType::CSRRW:
        case InstructionType::CSRRS:
        case InstructionType::CSRRC:
        case InstructionType::CSRRWI:
        case InstructionType::CSRRSI:
        case InstructionType::CSRRCI:
            RETURN_IF_ERROR(execute_csr_instruction(decoded));
            break;
            
        // Atomic instructions (RV32A)
        case InstructionType::LR_W:
        case InstructionType::SC_W:
        case InstructionType::AMOSWAP_W:
        case InstructionType::AMOADD_W:
        case InstructionType::AMOXOR_W:
        case InstructionType::AMOAND_W:
        case InstructionType::AMOOR_W:
        case InstructionType::AMOMIN_W:
        case InstructionType::AMOMAX_W:
        case InstructionType::AMOMINU_W:
        case InstructionType::AMOMAXU_W:
            RETURN_IF_ERROR(execute_atomic_instruction(decoded));
            break;
            
        case InstructionType::FENCE:
            // Memory fence - for now just a NOP in our emulator
            break;
            
        default:
            return std::unexpected(MAKE_ERROR(CPU_INVALID_INSTRUCTION,
                "Unimplemented instruction: " + 
                std::string(InstructionDecoder::get_instruction_name(decoded.type))));
    }
    
    return {};
}

Cycles CpuCore::get_instruction_cycles(InstructionType type) const {
    // Most instructions take 1 cycle
    // Some instructions take more cycles (multiplication, division, memory access)
    switch (type) {
        case InstructionType::MUL:
        case InstructionType::MULH:
        case InstructionType::MULHSU:
        case InstructionType::MULHU:
            return 3;  // Multiplication takes 3 cycles
            
        case InstructionType::DIV:
        case InstructionType::DIVU:
        case InstructionType::REM:
        case InstructionType::REMU:
            return 32;  // Division takes many cycles
            
        case InstructionType::LB:
        case InstructionType::LH:
        case InstructionType::LW:
        case InstructionType::LBU:
        case InstructionType::LHU:
        case InstructionType::SB:
        case InstructionType::SH:
        case InstructionType::SW:
            return 2;  // Memory access takes 2 cycles
            
        default:
            return 1;  // Most instructions take 1 cycle
    }
}

}  // namespace m5tab5::emulator