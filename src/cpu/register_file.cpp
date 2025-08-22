#include "emulator/cpu/register_file.hpp"
#include "emulator/utils/logging.hpp"
#include <array>

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
    // Initialize all general-purpose registers to 0
    std::fill(gp_registers_.begin(), gp_registers_.end(), 0);
    
    // Initialize CSRs to default values
    csr_registers_.clear();
    
    // Set up essential CSRs with default values
    set_csr(CSR_MISA, 0x40001100);  // RV32IMAC
    set_csr(CSR_MVENDORID, 0x0000);  // Non-commercial implementation
    set_csr(CSR_MARCHID, 0x0000);
    set_csr(CSR_MIMPID, 0x0001);
    set_csr(CSR_MHARTID, 0x0000);  // Hart 0
    
    // Machine status register
    set_csr(CSR_MSTATUS, 0x00001800);  // MPP=11 (machine mode)
    
    // Machine trap-vector base address
    set_csr(CSR_MTVEC, 0x00000000);
    
    // Performance counters
    set_csr(CSR_MCYCLE, 0);
    set_csr(CSR_MCYCLEH, 0);
    set_csr(CSR_MINSTRET, 0);
    set_csr(CSR_MINSTRETH, 0);
    
    COMPONENT_LOG_DEBUG("RegisterFile reset completed");
}

u32 RegisterFile::read_gp_register(u8 reg_num) const {
    if (reg_num >= GP_REGISTER_COUNT) {
        COMPONENT_LOG_WARN("Attempt to read invalid GP register: {}", reg_num);
        return 0;
    }
    
    // x0 always reads as 0
    if (reg_num == 0) {
        return 0;
    }
    
    return gp_registers_[reg_num];
}

void RegisterFile::write_gp_register(u8 reg_num, u32 value) {
    if (reg_num >= GP_REGISTER_COUNT) {
        COMPONENT_LOG_WARN("Attempt to write invalid GP register: {}", reg_num);
        return;
    }
    
    // x0 is hardwired to 0 and cannot be written
    if (reg_num == 0) {
        return;
    }
    
    gp_registers_[reg_num] = value;
    COMPONENT_LOG_TRACE("GP register x{} = 0x{:08X}", reg_num, value);
}

Result<u32> RegisterFile::read_csr(u16 csr_addr) const {
    // Check CSR access permissions based on privilege level
    u8 privilege_level = (csr_addr >> 8) & 0x3;
    if (privilege_level > current_privilege_level_) {
        return std::unexpected(MAKE_ERROR(CPU_PRIVILEGE_VIOLATION,
            "Insufficient privilege to access CSR 0x" + std::to_string(csr_addr)));
    }
    
    auto it = csr_registers_.find(csr_addr);
    if (it != csr_registers_.end()) {
        return it->second;
    }
    
    // Handle special CSRs that need dynamic values
    switch (csr_addr) {
        case CSR_CYCLE:
            return static_cast<u32>(get_csr(CSR_MCYCLE));
            
        case CSR_CYCLEH:
            return static_cast<u32>(get_csr(CSR_MCYCLE) >> 32);
            
        case CSR_TIME:
            // Time CSR typically reads from a memory-mapped timer
            return 0;  // TODO: Implement timer integration
            
        case CSR_TIMEH:
            return 0;  // TODO: Implement timer integration
            
        case CSR_INSTRET:
            return static_cast<u32>(get_csr(CSR_MINSTRET));
            
        case CSR_INSTRETH:
            return static_cast<u32>(get_csr(CSR_MINSTRET) >> 32);
            
        default:
            COMPONENT_LOG_WARN("Read from unimplemented CSR: 0x{:03X}", csr_addr);
            return 0;
    }
}

Result<void> RegisterFile::write_csr(u16 csr_addr, u32 value) {
    // Check CSR access permissions
    u8 privilege_level = (csr_addr >> 8) & 0x3;
    if (privilege_level > current_privilege_level_) {
        return std::unexpected(MAKE_ERROR(CPU_PRIVILEGE_VIOLATION,
            "Insufficient privilege to access CSR 0x" + std::to_string(csr_addr)));
    }
    
    // Check if CSR is read-only
    bool is_readonly = ((csr_addr >> 10) & 0x3) == 0x3;
    if (is_readonly) {
        return std::unexpected(MAKE_ERROR(CPU_PRIVILEGE_VIOLATION,
            "Attempt to write read-only CSR 0x" + std::to_string(csr_addr)));
    }
    
    // Handle special CSRs with side effects
    switch (csr_addr) {
        case CSR_MSTATUS:
            // Mask writable bits in MSTATUS
            value &= 0x807FF9BB;  // Mask for writable bits
            break;
            
        case CSR_MTVEC:
            // MTVEC must be aligned based on mode
            if ((value & 0x3) == 0x1) {  // Vectored mode
                value &= ~0x3F;  // 64-byte alignment
            } else {  // Direct mode
                value &= ~0x3;   // 4-byte alignment
            }
            break;
            
        case CSR_MCYCLE:
        case CSR_MCYCLEH:
        case CSR_MINSTRET:
        case CSR_MINSTRETH:
            // Performance counters are writable in machine mode
            break;
            
        case CSR_CYCLE:
        case CSR_CYCLEH:
        case CSR_TIME:
        case CSR_TIMEH:
        case CSR_INSTRET:
        case CSR_INSTRETH:
            // These are read-only shadows, ignore writes
            return {};
            
        default:
            // Check if this is a valid CSR address range
            if (csr_addr >= 0xC00 && csr_addr <= 0xC1F) {
                // Read-only user counters, ignore writes
                return {};
            }
            break;
    }
    
    set_csr(csr_addr, value);
    COMPONENT_LOG_TRACE("CSR 0x{:03X} = 0x{:08X}", csr_addr, value);
    
    return {};
}

u32 RegisterFile::get_pc() const {
    return pc_;
}

void RegisterFile::set_pc(u32 pc) {
    pc_ = pc;
    COMPONENT_LOG_TRACE("PC = 0x{:08X}", pc);
}

void RegisterFile::increment_pc(i32 offset) {
    pc_ += offset;
    COMPONENT_LOG_TRACE("PC incremented by {} to 0x{:08X}", offset, pc_);
}

void RegisterFile::increment_cycle_count() {
    u64 mcycle = get_csr(CSR_MCYCLE);
    mcycle++;
    set_csr(CSR_MCYCLE, static_cast<u32>(mcycle));
    set_csr(CSR_MCYCLEH, static_cast<u32>(mcycle >> 32));
}

void RegisterFile::increment_instruction_count() {
    u64 minstret = get_csr(CSR_MINSTRET);
    minstret++;
    set_csr(CSR_MINSTRET, static_cast<u32>(minstret));
    set_csr(CSR_MINSTRETH, static_cast<u32>(minstret >> 32));
}

const char* RegisterFile::get_register_name(u8 reg_num) {
    static const char* register_names[32] = {
        "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
        "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
        "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
        "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
    };
    
    if (reg_num < 32) {
        return register_names[reg_num];
    }
    return "invalid";
}

const char* RegisterFile::get_csr_name(u16 csr_addr) {
    switch (csr_addr) {
        case CSR_USTATUS: return "ustatus";
        case CSR_UIE: return "uie";
        case CSR_UTVEC: return "utvec";
        case CSR_USCRATCH: return "uscratch";
        case CSR_UEPC: return "uepc";
        case CSR_UCAUSE: return "ucause";
        case CSR_UTVAL: return "utval";
        case CSR_UIP: return "uip";
        case CSR_CYCLE: return "cycle";
        case CSR_TIME: return "time";
        case CSR_INSTRET: return "instret";
        case CSR_CYCLEH: return "cycleh";
        case CSR_TIMEH: return "timeh";
        case CSR_INSTRETH: return "instreth";
        case CSR_SSTATUS: return "sstatus";
        case CSR_SEDELEG: return "sedeleg";
        case CSR_SIDELEG: return "sideleg";
        case CSR_SIE: return "sie";
        case CSR_STVEC: return "stvec";
        case CSR_SCOUNTEREN: return "scounteren";
        case CSR_SSCRATCH: return "sscratch";
        case CSR_SEPC: return "sepc";
        case CSR_SCAUSE: return "scause";
        case CSR_STVAL: return "stval";
        case CSR_SIP: return "sip";
        case CSR_SATP: return "satp";
        case CSR_MVENDORID: return "mvendorid";
        case CSR_MARCHID: return "marchid";
        case CSR_MIMPID: return "mimpid";
        case CSR_MHARTID: return "mhartid";
        case CSR_MSTATUS: return "mstatus";
        case CSR_MISA: return "misa";
        case CSR_MEDELEG: return "medeleg";
        case CSR_MIDELEG: return "mideleg";
        case CSR_MIE: return "mie";
        case CSR_MTVEC: return "mtvec";
        case CSR_MCOUNTEREN: return "mcounteren";
        case CSR_MSCRATCH: return "mscratch";
        case CSR_MEPC: return "mepc";
        case CSR_MCAUSE: return "mcause";
        case CSR_MTVAL: return "mtval";
        case CSR_MIP: return "mip";
        case CSR_MCYCLE: return "mcycle";
        case CSR_MINSTRET: return "minstret";
        case CSR_MCYCLEH: return "mcycleh";
        case CSR_MINSTRETH: return "minstreth";
        default: return "unknown";
    }
}

void RegisterFile::dump_state() const {
    COMPONENT_LOG_INFO("=== Register File State ===");
    COMPONENT_LOG_INFO("PC: 0x{:08X}", pc_);
    
    // Dump GP registers
    for (int i = 0; i < 32; i += 4) {
        COMPONENT_LOG_INFO("x{:2d}({:4s}): 0x{:08X}  x{:2d}({:4s}): 0x{:08X}  x{:2d}({:4s}): 0x{:08X}  x{:2d}({:4s}): 0x{:08X}",
                          i, get_register_name(i), gp_registers_[i],
                          i+1, get_register_name(i+1), gp_registers_[i+1],
                          i+2, get_register_name(i+2), gp_registers_[i+2],
                          i+3, get_register_name(i+3), gp_registers_[i+3]);
    }
    
    // Dump important CSRs
    COMPONENT_LOG_INFO("Key CSRs:");
    const u16 important_csrs[] = {
        CSR_MSTATUS, CSR_MTVEC, CSR_MEPC, CSR_MCAUSE,
        CSR_MCYCLE, CSR_MINSTRET, CSR_MISA, CSR_MHARTID
    };
    
    for (u16 csr : important_csrs) {
        auto value = const_cast<RegisterFile*>(this)->read_csr(csr);
        if (value) {
            COMPONENT_LOG_INFO("  {}: 0x{:08X}", get_csr_name(csr), value.value());
        }
    }
}

u32 RegisterFile::get_csr(u16 csr_addr) const {
    auto it = csr_registers_.find(csr_addr);
    return (it != csr_registers_.end()) ? it->second : 0;
}

void RegisterFile::set_csr(u16 csr_addr, u32 value) {
    csr_registers_[csr_addr] = value;
}

}  // namespace m5tab5::emulator