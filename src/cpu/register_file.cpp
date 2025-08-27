#include "emulator/cpu/register_file.hpp"
#include "emulator/utils/logging.hpp"
#include <spdlog/fmt/fmt.h>

namespace m5tab5::emulator {

DECLARE_LOGGER("RegisterFile");

RegisterFile::RegisterFile() {
    reset();
    COMPONENT_LOG_DEBUG("RegisterFile created");
}

RegisterFile::~RegisterFile() {
    COMPONENT_LOG_DEBUG("RegisterFile destroyed");
}

void RegisterFile::reset() {
    // Clear all general-purpose registers
    gp_registers_.fill(0);
    
    // Clear floating-point registers if enabled
    if (has_floating_point_) {
        fp_registers_.fill(0);
    }
    
    // Initialize CSRs with default values
    initializeCSRs();
    
    // Reset performance counters
    cycle_counter_ = 0;
    instruction_counter_ = 0;
    trap_level_ = 0;
    
    COMPONENT_LOG_DEBUG("RegisterFile reset completed");
}

u32 RegisterFile::read(u8 reg_num) const {
    // x0 always reads as 0 (hardwired zero register)
    if (reg_num == 0) {
        return 0;
    }
    
    // Validate register number
    if (reg_num >= 32) {
        COMPONENT_LOG_ERROR("Invalid register number: {}", reg_num);
        return 0;
    }
    
    return gp_registers_[reg_num];
}

void RegisterFile::write(u8 reg_num, u32 value) {
    // x0 is hardwired to 0 and cannot be written
    if (reg_num == 0) {
        return;
    }
    
    // Validate register number
    if (reg_num >= 32) {
        COMPONENT_LOG_ERROR("Invalid register number: {}", reg_num);
        return;
    }
    
    gp_registers_[reg_num] = value;
    COMPONENT_LOG_TRACE("GP register x{} = 0x{:08X}", reg_num, value);
}

u32 RegisterFile::readCSR(u32 csr) const {
    return handleCSRRead(csr);
}

void RegisterFile::writeCSR(u32 csr, u32 value) {
    if (!isCSRWritable(csr)) {
        COMPONENT_LOG_WARN("Attempt to write read-only CSR: 0x{:03X}", csr);
        return;
    }
    
    csr_registers_[csr] = value;
    handleCSRWrite(csr, value);
    COMPONENT_LOG_TRACE("CSR 0x{:03X} = 0x{:08X}", csr, value);
}

bool RegisterFile::isValidCSR(u32 csr) const {
    // Check if CSR is in valid range and accessible at current privilege level
    return isCSRReadable(csr) || isCSRWritable(csr);
}

u32 RegisterFile::readFloat(u8 reg) const {
    if (!has_floating_point_) {
        COMPONENT_LOG_ERROR("Floating-point not enabled");
        return 0;
    }
    
    if (reg >= 32) {
        COMPONENT_LOG_ERROR("Invalid floating-point register: {}", reg);
        return 0;
    }
    
    return fp_registers_[reg];
}

void RegisterFile::writeFloat(u8 reg, u32 value) {
    if (!has_floating_point_) {
        COMPONENT_LOG_ERROR("Floating-point not enabled");
        return;
    }
    
    if (reg >= 32) {
        COMPONENT_LOG_ERROR("Invalid floating-point register: {}", reg);
        return;
    }
    
    fp_registers_[reg] = value;
    COMPONENT_LOG_TRACE("FP register f{} = 0x{:08X}", reg, value);
}

void RegisterFile::pushStack(u32 value) {
    u32 sp = getStackPointer();
    sp -= 4;  // Decrement stack pointer
    setStackPointer(sp);
    // Note: Actual memory write would be handled by CPU core
}

u32 RegisterFile::popStack() {
    u32 sp = getStackPointer();
    u32 value = 0;  // Note: Actual memory read would be handled by CPU core
    sp += 4;  // Increment stack pointer
    setStackPointer(sp);
    return value;
}

std::string RegisterFile::getRegisterName(u8 reg) const {
    static const std::array<const char*, 32> names = {
        "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
        "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
        "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
        "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
    };
    
    if (reg < 32) {
        return names[reg];
    }
    return "invalid";
}

std::string RegisterFile::getCSRName(u32 csr) const {
    switch (csr) {
        case MSTATUS: return "mstatus";
        case MISA: return "misa";
        case MIE: return "mie";
        case MTVEC: return "mtvec";
        case MSCRATCH: return "mscratch";
        case MEPC: return "mepc";
        case MCAUSE: return "mcause";
        case MTVAL: return "mtval";
        case MIP: return "mip";
        case MCYCLE: return "mcycle";
        case MINSTRET: return "minstret";
        case MCYCLEH: return "mcycleh";
        case MINSTRETH: return "minstreth";
        case CYCLE: return "cycle";
        case INSTRET: return "instret";
        case CYCLEH: return "cycleh";
        case INSTRETH: return "instreth";
        default: return "unknown";
    }
}

std::string RegisterFile::dump() const {
    std::string result = "Register File:\n";
    
    // General-purpose registers
    for (int i = 0; i < 32; i += 4) {
        result += fmt::format("x{:2d}-x{:2d}: {:08x} {:08x} {:08x} {:08x}\n",
            i, i+3, gp_registers_[i], gp_registers_[i+1], 
            gp_registers_[i+2], gp_registers_[i+3]);
    }
    
    // Performance counters
    result += fmt::format("Cycles: {}, Instructions: {}\n", 
        cycle_counter_, instruction_counter_);
    
    return result;
}

void RegisterFile::enterTrap(const TrapInfo& trap_info) {
    // Save current PC in MEPC
    writeCSR(MEPC, trap_info.pc);
    
    // Set cause in MCAUSE
    u32 cause = trap_info.cause;
    if (trap_info.interrupt) {
        cause |= 0x80000000;  // Set interrupt bit
    }
    writeCSR(MCAUSE, cause);
    
    // Set trap value in MTVAL
    writeCSR(MTVAL, trap_info.value);
    
    // Update privilege level and disable interrupts
    u32 mstatus = readCSR(MSTATUS);
    mstatus &= ~0x8;  // Clear MIE (Machine Interrupt Enable)
    writeCSR(MSTATUS, mstatus);
    
    trap_level_++;
    COMPONENT_LOG_DEBUG("Entered trap: cause=0x{:08X}, pc=0x{:08X}", cause, trap_info.pc);
}

void RegisterFile::exitTrap() {
    if (trap_level_ > 0) {
        trap_level_--;
        
        // Restore interrupt enable bit
        u32 mstatus = readCSR(MSTATUS);
        mstatus |= 0x8;  // Set MIE
        writeCSR(MSTATUS, mstatus);
        
        COMPONENT_LOG_DEBUG("Exited trap");
    }
}

bool RegisterFile::isCSRReadable(u32 csr) const {
    // All implemented CSRs are readable in machine mode
    switch (csr) {
        case MSTATUS:
        case MISA:
        case MIE:
        case MTVEC:
        case MSCRATCH:
        case MEPC:
        case MCAUSE:
        case MTVAL:
        case MIP:
        case MCYCLE:
        case MINSTRET:
        case MCYCLEH:
        case MINSTRETH:
        case CYCLE:
        case INSTRET:
        case CYCLEH:
        case INSTRETH:
            return true;
        default:
            return false;
    }
}

bool RegisterFile::isCSRWritable(u32 csr) const {
    // Most CSRs are writable, except some read-only performance counters
    switch (csr) {
        case CYCLE:
        case INSTRET:
        case CYCLEH:
        case INSTRETH:
            return false;  // Read-only user-level counters
        default:
            return isCSRReadable(csr);
    }
}

u32 RegisterFile::getPrivilegeLevel() const {
    // Currently only machine mode (3) is implemented
    return 3;
}

void RegisterFile::initializeCSRs() {
    // Initialize CSRs with ESP32-P4 specific values
    csr_registers_[MSTATUS] = 0x00001800;  // MPP=11 (Machine mode)
    csr_registers_[MISA] = 0x40101105;     // RV32IMAC
    csr_registers_[MIE] = 0;
    csr_registers_[MTVEC] = 0x40000000;    // Boot ROM vector
    csr_registers_[MSCRATCH] = 0;
    csr_registers_[MEPC] = 0;
    csr_registers_[MCAUSE] = 0;
    csr_registers_[MTVAL] = 0;
    csr_registers_[MIP] = 0;
    csr_registers_[MCYCLE] = 0;
    csr_registers_[MINSTRET] = 0;
    csr_registers_[MCYCLEH] = 0;
    csr_registers_[MINSTRETH] = 0;
}

void RegisterFile::handleCSRWrite(u32 csr, u32 value) {
    // Handle side effects of CSR writes
    switch (csr) {
        case MCYCLE:
            cycle_counter_ = (cycle_counter_ & 0xFFFFFFFF00000000ULL) | value;
            break;
        case MCYCLEH:
            cycle_counter_ = (cycle_counter_ & 0x00000000FFFFFFFFULL) | 
                           (static_cast<u64>(value) << 32);
            break;
        case MINSTRET:
            instruction_counter_ = (instruction_counter_ & 0xFFFFFFFF00000000ULL) | value;
            break;
        case MINSTRETH:
            instruction_counter_ = (instruction_counter_ & 0x00000000FFFFFFFFULL) | 
                                 (static_cast<u64>(value) << 32);
            break;
        default:
            // No special side effects for other CSRs
            break;
    }
}

u32 RegisterFile::handleCSRRead(u32 csr) const {
    // Handle special read behavior for some CSRs
    switch (csr) {
        case MCYCLE:
        case CYCLE:
            return static_cast<u32>(cycle_counter_ & 0xFFFFFFFF);
        case MCYCLEH:
        case CYCLEH:
            return static_cast<u32>(cycle_counter_ >> 32);
        case MINSTRET:
        case INSTRET:
            return static_cast<u32>(instruction_counter_ & 0xFFFFFFFF);
        case MINSTRETH:
        case INSTRETH:
            return static_cast<u32>(instruction_counter_ >> 32);
        default:
            // Return stored value for other CSRs
            auto it = csr_registers_.find(csr);
            return (it != csr_registers_.end()) ? it->second : 0;
    }
}

}  // namespace m5tab5::emulator