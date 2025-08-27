#include "emulator/cpu/cpu_core.hpp"
// #include "emulator/utils/logging.hpp" // Temporarily disabled
#include "emulator/utils/error.hpp"
#include "emulator/cpu/instruction_decoder.hpp"
#include "emulator/cpu/syscall_interface.hpp"
#include <algorithm>

namespace m5tab5::emulator {

// DECLARE_LOGGER("CpuCore"); // Temporarily disabled

CpuCore::CpuCore(const CoreConfig& config, MemoryInterface& memory)
    : config_(config),
      memory_(memory),
      registers_(),
      pc_(0),
      cycle_count_(0) {
    
    // COMPONENT_LOG_DEBUG("CpuCore created");
}

CpuCore::~CpuCore() {
    // COMPONENT_LOG_DEBUG("CpuCore destroyed");
}

EmulatorError CpuCore::initialize() {
    // COMPONENT_LOG_INFO("Initializing CPU core");
    
    // Initialize register file
    registers_.reset();
    
    // Set initial PC based on core type
    if (config_.type == CoreType::LPCore) {
        pc_ = 0x50000000;  // LP core reset vector
    } else {
        pc_ = 0x10000000;  // Main core reset vector (Flash start)
    }
    
    cycle_count_ = 0;
    
    // COMPONENT_LOG_INFO("CPU core initialized successfully");
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::reset() {
    // COMPONENT_LOG_INFO("Resetting CPU core");
    
    // Reset register file
    registers_.reset();
    
    // Set initial PC to ESP32-P4 Boot ROM reset vector
    if (config_.type == CoreType::LPCore) {
        pc_ = 0x50000000;  // LP Core starts at different address
    } else {
        pc_ = 0x40000080;  // ESP32-P4 Boot ROM reset vector
    }
    
    cycle_count_ = 0;
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::step() {
    // Execute one instruction
    return execute(1);
}

EmulatorError CpuCore::execute(uint32_t num_instructions) {
    for (uint32_t i = 0; i < num_instructions; ++i) {
        // Fetch instruction
        uint32_t instruction;
        EmulatorError result = loadMemory(pc_, 4, instruction);
        if (result != EmulatorError::Success) {
            return result;
        }
        
        // Decode instruction
        InstructionDecoder decoder;
        auto decode_result = decoder.decode(instruction);
        if (!decode_result) {
            // COMPONENT_LOG_ERROR("Failed to decode instruction");
            return EmulatorError::InvalidOperation;
        }
        
        // Execute instruction (placeholder)
        EmulatorError exec_result = execute(decode_result.value());
        if (exec_result != EmulatorError::Success) {
            return exec_result;
        }
        
        // Update PC
        pc_ += 4;
        cycle_count_++;
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeCycles(uint32_t num_cycles) {
    // Simplified: assume 1 instruction per cycle
    return execute(num_cycles);
}

EmulatorError CpuCore::triggerInterrupt(uint32_t interrupt_id) {
    // COMPONENT_LOG_DEBUG("Interrupt triggered");
    // TODO: Implement interrupt handling
    return EmulatorError::NotImplemented;
}

EmulatorError CpuCore::setInterruptHandler(uint32_t interrupt_id, Address handler_address) {
    // COMPONENT_LOG_DEBUG("Setting interrupt handler");
    // TODO: Implement interrupt handler setup
    return EmulatorError::NotImplemented;
}

bool CpuCore::isInterruptPending() const {
    // TODO: Check for pending interrupts
    return false;
}

void CpuCore::resetPerformanceCounters() {
    perf_counters_ = {};
}

EmulatorError CpuCore::setBreakpoint(Address address) {
    breakpoints_.insert(address);
    return EmulatorError::Success;
}

EmulatorError CpuCore::removeBreakpoint(Address address) {
    breakpoints_.erase(address);
    return EmulatorError::Success;
}

bool CpuCore::hasBreakpoint(Address address) const {
    return breakpoints_.find(address) != breakpoints_.end();
}

uint32_t CpuCore::getCoreId() const {
    return static_cast<uint32_t>(config_.type);
}

void CpuCore::setSystemCallInterface(std::shared_ptr<SystemCallInterface> syscall_interface) {
    syscall_interface_ = syscall_interface;
}

EmulatorError CpuCore::handleSystemCall() {
    if (!syscall_interface_) {
        // COMPONENT_LOG_ERROR("No system call interface available");
        return EmulatorError::NotImplemented;
    }
    
    // Build system call context from RISC-V registers
    SystemCallInterface::SystemCallContext context;
    
    // System call number from a7 register (x17)
    context.syscall_number = registers_.read(17);
    
    // Arguments from a0-a6 registers (x10-x16) 
    for (int i = 0; i < 7; i++) {
        context.args[i] = registers_.read(10 + i);
    }
    
    // Current PC and core information
    context.return_pc = pc_;
    context.caller_privilege = 0; // Assume user mode for now
    context.calling_core_id = getCoreId();
    
    // Execute system call
    int32_t result = syscall_interface_->handle_ecall(context);
    
    // Store result in a0 register (x10)
    registers_.write(10, static_cast<uint32_t>(result));
    
    return EmulatorError::Success;
}

// Private methods

EmulatorError CpuCore::fetch(PipelineStage& stage) {
    // TODO: Implement instruction fetch
    return EmulatorError::NotImplemented;
}

EmulatorError CpuCore::decode(const PipelineStage& stage, DecodedInstruction& decoded) {
    // TODO: Implement instruction decode
    return EmulatorError::NotImplemented;
}

EmulatorError CpuCore::execute(const DecodedInstruction& decoded) {
    // Update performance counters
    perf_counters_.instructions_executed++;
    registers_.incrementInstructionCounter();
    
    // Execute instruction based on type
    switch (decoded.type) {
        // Arithmetic with immediate
        case InstructionType::ADDI:
            return executeADDI(decoded);
        case InstructionType::SLTI:
            return executeSLTI(decoded);
        case InstructionType::SLTIU:
            return executeSLTIU(decoded);
        case InstructionType::XORI:
            return executeXORI(decoded);
        case InstructionType::ORI:
            return executeORI(decoded);
        case InstructionType::ANDI:
            return executeANDI(decoded);
        case InstructionType::SLLI:
            return executeSLLI(decoded);
        case InstructionType::SRLI:
            return executeSRLI(decoded);
        case InstructionType::SRAI:
            return executeSRAI(decoded);
            
        // Register arithmetic
        case InstructionType::ADD:
            return executeADD(decoded);
        case InstructionType::SUB:
            return executeSUB(decoded);
        case InstructionType::SLL:
            return executeSLL(decoded);
        case InstructionType::SLT:
            return executeSLT(decoded);
        case InstructionType::SLTU:
            return executeSLTU(decoded);
        case InstructionType::XOR:
            return executeXOR(decoded);
        case InstructionType::SRL:
            return executeSRL(decoded);
        case InstructionType::SRA:
            return executeSRA(decoded);
        case InstructionType::OR:
            return executeOR(decoded);
        case InstructionType::AND:
            return executeAND(decoded);
            
        // Upper immediate
        case InstructionType::LUI:
            return executeLUI(decoded);
        case InstructionType::AUIPC:
            return executeAUIPC(decoded);
            
        // Jumps
        case InstructionType::JAL:
            return executeJAL(decoded);
        case InstructionType::JALR:
            return executeJALR(decoded);
            
        // Branches
        case InstructionType::BEQ:
            return executeBEQ(decoded);
        case InstructionType::BNE:
            return executeBNE(decoded);
        case InstructionType::BLT:
            return executeBLT(decoded);
        case InstructionType::BGE:
            return executeBGE(decoded);
        case InstructionType::BLTU:
            return executeBLTU(decoded);
        case InstructionType::BGEU:
            return executeBGEU(decoded);
            
        // Loads
        case InstructionType::LB:
            return executeLB(decoded);
        case InstructionType::LH:
            return executeLH(decoded);
        case InstructionType::LW:
            return executeLW(decoded);
        case InstructionType::LBU:
            return executeLBU(decoded);
        case InstructionType::LHU:
            return executeLHU(decoded);
            
        // Stores
        case InstructionType::SB:
            return executeSB(decoded);
        case InstructionType::SH:
            return executeSH(decoded);
        case InstructionType::SW:
            return executeSW(decoded);
            
        // System
        case InstructionType::ECALL:
            return handleSystemCall();
        case InstructionType::EBREAK:
            return executeEBREAK(decoded);
            
        default:
            // COMPONENT_LOG_ERROR("Unimplemented instruction type: {}", static_cast<int>(decoded.type));
            return EmulatorError::NotImplemented;
    }
}

EmulatorError CpuCore::writeback(const DecodedInstruction& decoded) {
    // TODO: Implement register writeback
    return EmulatorError::NotImplemented;
}

EmulatorError CpuCore::loadMemory(Address address, uint32_t size, uint32_t& value) {
    // Use memory interface based on size
    if (size == 4) {
        return memory_.read32(address, value);
    } else if (size == 2) {
        uint16_t val16;
        EmulatorError result = memory_.read16(address, val16);
        value = val16;
        return result;
    } else if (size == 1) {
        uint8_t val8;
        EmulatorError result = memory_.read8(address, val8);
        value = val8;
        return result;
    } else {
        // Use block read for arbitrary sizes
        std::vector<uint8_t> data(size);
        EmulatorError result = memory_.readBlock(address, data.data(), size);
        
        if (result != EmulatorError::Success) {
            return result;
        }
        
        // Convert bytes to value (little endian)
        value = 0;
        for (uint32_t i = 0; i < size && i < 4; ++i) {
            value |= (static_cast<uint32_t>(data[i]) << (i * 8));
        }
        
        return EmulatorError::Success;
    }
}

EmulatorError CpuCore::storeMemory(Address address, uint32_t size, uint32_t value) {
    // Use memory interface based on size
    if (size == 4) {
        return memory_.write32(address, value);
    } else if (size == 2) {
        return memory_.write16(address, static_cast<uint16_t>(value));
    } else if (size == 1) {
        return memory_.write8(address, static_cast<uint8_t>(value));
    } else {
        // Use block write for arbitrary sizes
        std::vector<uint8_t> data(size);
        for (uint32_t i = 0; i < size; ++i) {
            data[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
        }
        
        return memory_.writeBlock(address, data.data(), size);
    }
}

EmulatorError CpuCore::handleException(ExceptionType type, uint32_t cause) {
    // TODO: Implement exception handling
    return EmulatorError::NotImplemented;
}

EmulatorError CpuCore::handleTrap(TrapType type) {
    // TODO: Implement trap handling
    return EmulatorError::NotImplemented;
}

bool CpuCore::predictBranch(Address pc) {
    // Simple always-not-taken predictor
    return false;
}

void CpuCore::updateBranchPredictor(Address pc, bool taken) {
    // Simple 2-bit branch predictor
    uint8_t index = (pc >> 2) & 0xFF;  // Use PC bits [9:2] as index
    uint8_t& counter = branch_predictor_[index];
    
    if (taken) {
        if (counter < 3) counter++;  // Saturate at 3
    } else {
        if (counter > 0) counter--;  // Saturate at 0
    }
}

// Arithmetic Immediate Instructions
EmulatorError CpuCore::executeADDI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t result = rs1_val + static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLTI(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    int32_t imm = decoded.immediate;
    uint32_t result = (rs1_val < imm) ? 1 : 0;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLTIU(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t imm = static_cast<uint32_t>(decoded.immediate);
    uint32_t result = (rs1_val < imm) ? 1 : 0;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeXORI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t result = rs1_val ^ static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeORI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t result = rs1_val | static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeANDI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t result = rs1_val & static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLLI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t shamt = decoded.immediate & 0x1F;  // Only use lower 5 bits
    uint32_t result = rs1_val << shamt;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSRLI(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t shamt = decoded.immediate & 0x1F;  // Only use lower 5 bits
    uint32_t result = rs1_val >> shamt;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSRAI(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    uint32_t shamt = decoded.immediate & 0x1F;  // Only use lower 5 bits
    int32_t result = rs1_val >> shamt;
    registers_.write(decoded.rd, static_cast<uint32_t>(result));
    return EmulatorError::Success;
}

// Register Arithmetic Instructions
EmulatorError CpuCore::executeADD(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = rs1_val + rs2_val;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSUB(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = rs1_val - rs2_val;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLL(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t shamt = rs2_val & 0x1F;  // Only use lower 5 bits
    uint32_t result = rs1_val << shamt;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLT(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    int32_t rs2_val = static_cast<int32_t>(registers_.read(decoded.rs2));
    uint32_t result = (rs1_val < rs2_val) ? 1 : 0;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSLTU(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = (rs1_val < rs2_val) ? 1 : 0;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeXOR(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = rs1_val ^ rs2_val;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSRL(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t shamt = rs2_val & 0x1F;  // Only use lower 5 bits
    uint32_t result = rs1_val >> shamt;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeSRA(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t shamt = rs2_val & 0x1F;  // Only use lower 5 bits
    int32_t result = rs1_val >> shamt;
    registers_.write(decoded.rd, static_cast<uint32_t>(result));
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeOR(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = rs1_val | rs2_val;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeAND(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    uint32_t result = rs1_val & rs2_val;
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

// Upper Immediate Instructions
EmulatorError CpuCore::executeLUI(const DecodedInstruction& decoded) {
    uint32_t result = static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeAUIPC(const DecodedInstruction& decoded) {
    uint32_t result = pc_ + static_cast<uint32_t>(decoded.immediate);
    registers_.write(decoded.rd, result);
    return EmulatorError::Success;
}

// Jump Instructions
EmulatorError CpuCore::executeJAL(const DecodedInstruction& decoded) {
    // Save return address (PC + 4)
    registers_.write(decoded.rd, pc_ + 4);
    
    // Jump to target
    pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented by 4 after execute
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeJALR(const DecodedInstruction& decoded) {
    // Save return address (PC + 4)
    uint32_t return_addr = pc_ + 4;
    
    // Calculate target address
    uint32_t target = registers_.read(decoded.rs1) + decoded.immediate;
    target &= ~1;  // Clear LSB for alignment
    
    // Set PC and return address
    pc_ = target - 4;  // -4 because pc_ will be incremented by 4 after execute
    registers_.write(decoded.rd, return_addr);
    
    return EmulatorError::Success;
}

// Branch Instructions
EmulatorError CpuCore::executeBEQ(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    
    if (rs1_val == rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeBNE(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    
    if (rs1_val != rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeBLT(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    int32_t rs2_val = static_cast<int32_t>(registers_.read(decoded.rs2));
    
    if (rs1_val < rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeBGE(const DecodedInstruction& decoded) {
    int32_t rs1_val = static_cast<int32_t>(registers_.read(decoded.rs1));
    int32_t rs2_val = static_cast<int32_t>(registers_.read(decoded.rs2));
    
    if (rs1_val >= rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeBLTU(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    
    if (rs1_val < rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeBGEU(const DecodedInstruction& decoded) {
    uint32_t rs1_val = registers_.read(decoded.rs1);
    uint32_t rs2_val = registers_.read(decoded.rs2);
    
    if (rs1_val >= rs2_val) {
        pc_ = pc_ + decoded.immediate - 4;  // -4 because pc_ will be incremented
        updateBranchPredictor(pc_, true);
        perf_counters_.branch_predictions++;
    } else {
        updateBranchPredictor(pc_, false);
    }
    
    return EmulatorError::Success;
}

// Load Instructions
EmulatorError CpuCore::executeLB(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    uint32_t value;
    EmulatorError result = loadMemory(addr, 1, value);
    if (result != EmulatorError::Success) {
        return result;
    }
    
    // Sign extend byte to 32 bits
    int32_t signed_value = static_cast<int8_t>(value & 0xFF);
    registers_.write(decoded.rd, static_cast<uint32_t>(signed_value));
    perf_counters_.memory_accesses++;
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeLH(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    
    // Check alignment
    if (addr & 1) {
        return EmulatorError::InvalidAddress;
    }
    
    uint32_t value;
    EmulatorError result = loadMemory(addr, 2, value);
    if (result != EmulatorError::Success) {
        return result;
    }
    
    // Sign extend halfword to 32 bits
    int32_t signed_value = static_cast<int16_t>(value & 0xFFFF);
    registers_.write(decoded.rd, static_cast<uint32_t>(signed_value));
    perf_counters_.memory_accesses++;
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeLW(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    
    // Check alignment
    if (addr & 3) {
        return EmulatorError::InvalidAddress;
    }
    
    uint32_t value;
    EmulatorError result = loadMemory(addr, 4, value);
    if (result != EmulatorError::Success) {
        return result;
    }
    
    registers_.write(decoded.rd, value);
    perf_counters_.memory_accesses++;
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeLBU(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    uint32_t value;
    EmulatorError result = loadMemory(addr, 1, value);
    if (result != EmulatorError::Success) {
        return result;
    }
    
    // Zero extend byte to 32 bits
    registers_.write(decoded.rd, value & 0xFF);
    perf_counters_.memory_accesses++;
    
    return EmulatorError::Success;
}

EmulatorError CpuCore::executeLHU(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    
    // Check alignment
    if (addr & 1) {
        return EmulatorError::InvalidAddress;
    }
    
    uint32_t value;
    EmulatorError result = loadMemory(addr, 2, value);
    if (result != EmulatorError::Success) {
        return result;
    }
    
    // Zero extend halfword to 32 bits
    registers_.write(decoded.rd, value & 0xFFFF);
    perf_counters_.memory_accesses++;
    
    return EmulatorError::Success;
}

// Store Instructions
EmulatorError CpuCore::executeSB(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    uint32_t value = registers_.read(decoded.rs2) & 0xFF;
    
    EmulatorError result = storeMemory(addr, 1, value);
    if (result == EmulatorError::Success) {
        perf_counters_.memory_accesses++;
    }
    
    return result;
}

EmulatorError CpuCore::executeSH(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    
    // Check alignment
    if (addr & 1) {
        return EmulatorError::InvalidAddress;
    }
    
    uint32_t value = registers_.read(decoded.rs2) & 0xFFFF;
    
    EmulatorError result = storeMemory(addr, 2, value);
    if (result == EmulatorError::Success) {
        perf_counters_.memory_accesses++;
    }
    
    return result;
}

EmulatorError CpuCore::executeSW(const DecodedInstruction& decoded) {
    uint32_t addr = registers_.read(decoded.rs1) + decoded.immediate;
    
    // Check alignment
    if (addr & 3) {
        return EmulatorError::InvalidAddress;
    }
    
    uint32_t value = registers_.read(decoded.rs2);
    
    EmulatorError result = storeMemory(addr, 4, value);
    if (result == EmulatorError::Success) {
        perf_counters_.memory_accesses++;
    }
    
    return result;
}

// System Instructions
EmulatorError CpuCore::executeEBREAK(const DecodedInstruction& decoded) {
    // Trigger breakpoint exception - for now just indicate an exception occurred
    // In a real implementation this would trigger a debug trap
    // COMPONENT_LOG_DEBUG("EBREAK instruction executed at PC: 0x{:08X}", pc_);
    return EmulatorError::InvalidOperation;
}

}  // namespace m5tab5::emulator