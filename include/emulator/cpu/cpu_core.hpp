#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/cpu/register_file.hpp"
#include "emulator/cpu/instruction_set.hpp"
#include "emulator/memory/memory_interface.hpp"

#include <array>
#include <functional>
#include <memory>
#include <set>

namespace m5tab5::emulator {

// Forward declarations
class SystemCallInterface;

/**
 * @brief RISC-V CPU core emulation
 * 
 * Emulates the ESP32-P4 dual-core RISC-V processor with:
 * - RV32IMAC instruction set
 * - Dual-core with independent execution
 * - Memory management unit
 * - Interrupt handling
 * - Performance counters
 */
class CpuCore {
public:
    enum class CoreType {
        MainCore0,
        MainCore1,
        LPCore
    };

    struct CoreConfig {
        CoreType type;
        uint32_t frequency_hz;
        bool enable_fpu = true;
        bool enable_compressed = true;
        bool enable_atomic = true;
        bool enable_mul_div = true;
    };

    explicit CpuCore(const CoreConfig& config, MemoryInterface& memory);
    ~CpuCore();

    // Core control
    EmulatorError initialize();
    EmulatorError reset();
    
    // Execution
    EmulatorError step(); // Execute one instruction
    EmulatorError execute(uint32_t num_instructions);
    EmulatorError executeCycles(uint32_t num_cycles);

    // State access
    RegisterFile& getRegisters() { return registers_; }
    const RegisterFile& getRegisters() const { return registers_; }
    
    Address getProgramCounter() const { return pc_; }
    void setProgramCounter(Address pc) { pc_ = pc; }

    // Interrupt handling
    EmulatorError triggerInterrupt(uint32_t interrupt_id);
    EmulatorError setInterruptHandler(uint32_t interrupt_id, Address handler_address);
    bool isInterruptPending() const;

    // Performance monitoring
    struct PerformanceCounters {
        uint64_t instructions_executed = 0;
        uint64_t cycles_executed = 0;
        uint64_t cache_hits = 0;
        uint64_t cache_misses = 0;
        uint64_t branch_predictions = 0;
        uint64_t branch_mispredictions = 0;
        uint64_t memory_accesses = 0;
        uint64_t interrupts_handled = 0;
    };

    const PerformanceCounters& getPerformanceCounters() const { return perf_counters_; }
    void resetPerformanceCounters();

    // Debug support
    EmulatorError setBreakpoint(Address address);
    EmulatorError removeBreakpoint(Address address);
    bool hasBreakpoint(Address address) const;

    // System call support
    void setSystemCallInterface(std::shared_ptr<SystemCallInterface> syscall_interface);
    EmulatorError handleSystemCall();

    // Core identification
    CoreType getCoreType() const { return config_.type; }
    uint32_t getCoreId() const;

private:
    // Instruction execution pipeline
    struct PipelineStage {
        Address pc = 0;
        uint32_t instruction = 0;
        bool valid = false;
    };

    // Internal execution methods
    EmulatorError fetch(PipelineStage& stage);
    EmulatorError decode(const PipelineStage& stage, DecodedInstruction& decoded);
    EmulatorError execute(const DecodedInstruction& decoded);
    EmulatorError writeback(const DecodedInstruction& decoded);

    // Memory operations
    EmulatorError loadMemory(Address address, uint32_t size, uint32_t& value);
    EmulatorError storeMemory(Address address, uint32_t size, uint32_t value);

    // Exception handling
    EmulatorError handleException(ExceptionType type, uint32_t cause);
    EmulatorError handleTrap(TrapType type);

    // Branch prediction (simple 2-bit predictor)
    bool predictBranch(Address pc);
    void updateBranchPredictor(Address pc, bool taken);

    // Configuration and state
    CoreConfig config_;
    MemoryInterface& memory_;
    
    // CPU state
    RegisterFile registers_;
    Address pc_ = 0;
    uint32_t cycle_count_ = 0;
    
    // Pipeline stages
    std::array<PipelineStage, 5> pipeline_; // 5-stage pipeline
    uint32_t pipeline_stalls_ = 0;

    // Interrupt system
    struct InterruptState {
        bool enabled = false;
        uint32_t pending_mask = 0;
        uint32_t enabled_mask = 0;
        std::array<Address, 32> handlers = {};
    } interrupt_state_;

    // Performance monitoring
    PerformanceCounters perf_counters_;

    // Debug support
    std::set<Address> breakpoints_;
    bool debug_mode_ = false;

    // System call support
    std::shared_ptr<SystemCallInterface> syscall_interface_;

    // Branch prediction table (256 entries, 2-bit saturating counters)
    std::array<uint8_t, 256> branch_predictor_ = {};

    // Future: Add instruction set implementation here if needed
};

} // namespace m5tab5::emulator