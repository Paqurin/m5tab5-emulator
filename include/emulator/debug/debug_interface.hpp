#pragma once

#include "emulator/core/types.hpp"
#include "emulator/debug/debug_types.hpp"
#include <vector>
#include <optional>

namespace m5tab5::emulator {

// Debug symbol information
struct Symbol {
    std::string name;
    Address address;
    size_t size;
    std::string type; // "function", "variable", "object"
    std::string file;
    uint32_t line;
};

// Stack frame information
struct StackFrame {
    Address pc;
    Address sp;
    Address fp;
    std::string function_name;
    std::string file;
    uint32_t line;
    std::vector<std::pair<std::string, uint32_t>> local_variables;
};

// Disassembly line
struct DisassemblyLine {
    Address address;
    uint32_t instruction;
    std::string mnemonic;
    std::string operands;
    std::string comment;
};

// Performance profile entry
struct ProfileEntry {
    Address pc;
    std::string function_name;
    uint64_t hit_count;
    uint64_t exclusive_cycles;
    uint64_t inclusive_cycles;
};

// Performance profile report
struct ProfileReport {
    uint64_t total_cycles;
    uint64_t total_instructions;
    std::vector<ProfileEntry> functions;
    std::vector<ProfileEntry> hot_spots;
};

/**
 * @brief Abstract debug interface
 */
class DebugInterface {
public:
    virtual ~DebugInterface() = default;
    
    // Lifecycle
    virtual EmulatorError initialize() = 0;
    virtual EmulatorError shutdown() = 0;
    
    // Breakpoint management
    virtual uint32_t setBreakpoint(BreakpointType type, Address address, size_t size = 4) = 0;
    virtual EmulatorError removeBreakpoint(uint32_t breakpoint_id) = 0;
    virtual EmulatorError enableBreakpoint(uint32_t breakpoint_id, bool enabled) = 0;
    virtual std::vector<Breakpoint> getBreakpoints() const = 0;
    
    // Execution control
    virtual EmulatorError stepInstruction() = 0;
    virtual EmulatorError stepOver() = 0;
    virtual EmulatorError stepOut() = 0;
    virtual EmulatorError continueExecution() = 0;
    virtual EmulatorError pauseExecution() = 0;
    
    // Memory inspection
    virtual EmulatorError readMemory(Address address, size_t size, std::vector<uint8_t>& data) = 0;
    virtual EmulatorError writeMemory(Address address, const std::vector<uint8_t>& data) = 0;
    
    // Register access
    virtual EmulatorError readRegister(uint32_t reg_id, uint32_t& value) = 0;
    virtual EmulatorError writeRegister(uint32_t reg_id, uint32_t value) = 0;
    virtual std::vector<uint32_t> getAllRegisters() = 0;
    
    // Stack inspection
    virtual std::vector<StackFrame> getStackTrace(uint32_t max_frames = 32) = 0;
    virtual EmulatorError getStackFrame(uint32_t frame_index, StackFrame& frame) = 0;
    
    // Symbol management
    virtual EmulatorError loadSymbols(const std::string& symbol_file) = 0;
    virtual std::optional<Symbol> lookupSymbol(const std::string& name) = 0;
    virtual std::optional<Symbol> lookupSymbol(Address address) = 0;
};

} // namespace m5tab5::emulator