#pragma once

#include "emulator/core/types.hpp"
#include "emulator/debug/debug_interface.hpp"
#include "emulator/debug/debug_types.hpp"
#include "emulator/debug/gdb_server.hpp"
#include "emulator/debug/profiler.hpp"

#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <thread>
#include <atomic>
#include <shared_mutex>

namespace m5tab5::emulator {

class EmulatorCore;
class CPUCore;  
class MemoryController;

/**
 * @brief Comprehensive debugging system for the emulator
 * 
 * Features:
 * - GDB remote debugging protocol
 * - Breakpoints and watchpoints
 * - Performance profiling
 * - Memory inspection
 * - Register monitoring
 * - Execution tracing
 * - Interactive debugging console
 */
class Debugger : public DebugInterface {
public:
    struct DebugConfig {
        bool enable_gdb_server = false;
        uint16_t gdb_port = 3333;
        bool enable_profiler = false;
        bool enable_tracing = false;
        size_t trace_buffer_size = 1024 * 1024; // 1MB
        bool break_on_exception = true;
        bool break_on_interrupt = false;
        std::string log_file = "";
    };

    // Using BreakpointType and Breakpoint from debug_types.hpp

    explicit Debugger(EmulatorCore& emulator, const DebugConfig& config);
    ~Debugger();

    // DebugInterface implementation
    EmulatorError initialize() override;
    EmulatorError shutdown() override;
    
    // Breakpoint management
    uint32_t setBreakpoint(BreakpointType type, Address address, 
                          size_t size = 4) override;
    EmulatorError removeBreakpoint(uint32_t breakpoint_id) override;
    EmulatorError enableBreakpoint(uint32_t breakpoint_id, bool enabled) override;
    std::vector<Breakpoint> getBreakpoints() const override;

    // Execution control
    EmulatorError stepInstruction() override;
    EmulatorError stepOver() override;
    EmulatorError stepOut() override;
    EmulatorError continueExecution() override;
    EmulatorError pauseExecution() override;

    // Memory inspection
    EmulatorError readMemory(Address address, size_t size, 
                            std::vector<uint8_t>& data) override;
    EmulatorError writeMemory(Address address, 
                             const std::vector<uint8_t>& data) override;

    // Register access
    EmulatorError readRegister(uint32_t reg_id, uint32_t& value) override;
    EmulatorError writeRegister(uint32_t reg_id, uint32_t value) override;
    std::vector<uint32_t> getAllRegisters() override;

    // Stack inspection
    std::vector<StackFrame> getStackTrace(uint32_t max_frames = 32) override;
    EmulatorError getStackFrame(uint32_t frame_index, StackFrame& frame) override;

    // Symbol management
    EmulatorError loadSymbols(const std::string& symbol_file) override;
    std::optional<Symbol> lookupSymbol(const std::string& name) override;
    std::optional<Symbol> lookupSymbol(Address address) override;

    // Advanced debugging features
    EmulatorError setConditionalBreakpoint(Address address, 
                                          const std::string& condition);
    EmulatorError setWatchpoint(Address address, size_t size, 
                               BreakpointType type);
    
    // Execution tracing
    EmulatorError startTracing();
    EmulatorError stopTracing();
    std::vector<TraceEntry> getTrace(size_t max_entries = 1000);
    EmulatorError saveTrace(const std::string& filename);

    // Performance profiling
    Profiler& getProfiler() { return *profiler_; }
    EmulatorError startProfiling();
    EmulatorError stopProfiling();
    ProfileReport getProfileReport();

    // GDB server
    EmulatorError startGDBServer();
    EmulatorError stopGDBServer();
    bool isGDBServerRunning() const;

    // Debug console
    EmulatorError executeCommand(const std::string& command, std::string& output);
    std::vector<std::string> getAvailableCommands();

    // Event callbacks
    using BreakpointCallback = std::function<void(const Breakpoint&)>;
    using ExceptionCallback = std::function<void(uint32_t exception_code, Address pc)>;
    
    void setBreakpointCallback(BreakpointCallback callback);
    void setExceptionCallback(ExceptionCallback callback);

    // Debug information
    struct DebugInfo {
        bool is_running = false;
        bool is_paused = false;
        Address current_pc = 0;
        uint32_t current_instruction = 0;
        uint32_t active_breakpoints = 0;
        uint32_t hit_breakpoints = 0;
        bool tracing_enabled = false;
        size_t trace_entries = 0;
    };

    DebugInfo getDebugInfo() const;

    // Disassembly
    EmulatorError disassemble(Address address, size_t count,
                             std::vector<DisassemblyLine>& lines);

private:
    // Internal breakpoint handling
    bool checkBreakpoint(BreakpointType type, Address address, size_t size);
    void handleBreakpointHit(const Breakpoint& bp);
    
    // Trace management
    void addTraceEntry(Address pc, uint32_t instruction, ClockCycle cycle);
    
    // Symbol table management
    struct SymbolTable {
        std::unordered_map<std::string, Symbol> name_to_symbol;
        std::unordered_map<Address, Symbol> address_to_symbol;
    };
    
    EmulatorError parseELFSymbols(const std::string& filename);
    EmulatorError parseDWARFDebugInfo(const std::string& filename);

    // Command processing
    EmulatorError processInfoCommand(const std::vector<std::string>& args, std::string& output);
    EmulatorError processBreakCommand(const std::vector<std::string>& args, std::string& output);
    EmulatorError processMemoryCommand(const std::vector<std::string>& args, std::string& output);
    EmulatorError processRegisterCommand(const std::vector<std::string>& args, std::string& output);

    // Configuration and state
    DebugConfig config_;
    EmulatorCore& emulator_;
    
    // Debugging components
    std::unique_ptr<GDBServer> gdb_server_;
    std::unique_ptr<Profiler> profiler_;
    
    // Breakpoint management
    std::unordered_map<uint32_t, Breakpoint> breakpoints_;
    uint32_t next_breakpoint_id_ = 1;
    mutable std::mutex breakpoints_mutex_;

    // Symbol management
    SymbolTable symbol_table_;
    mutable std::shared_mutex symbols_mutex_;

    // Execution tracing (using TraceEntry from debug_types.hpp)
    
    std::vector<TraceEntry> trace_buffer_;
    size_t trace_write_index_ = 0;
    bool trace_buffer_full_ = false;
    std::atomic<bool> tracing_enabled_{false};
    mutable std::mutex trace_mutex_;

    // Debug state
    std::atomic<bool> debug_break_requested_{false};
    std::atomic<bool> single_step_mode_{false};
    Address step_over_return_pc_ = 0;
    uint32_t step_out_stack_depth_ = 0;

    // Event callbacks
    BreakpointCallback breakpoint_callback_;
    ExceptionCallback exception_callback_;

    // Command table
    std::unordered_map<std::string, std::function<EmulatorError(const std::vector<std::string>&, std::string&)>> command_table_;
    
    // Statistics
    struct DebugStatistics {
        uint64_t breakpoints_hit = 0;
        uint64_t single_steps = 0;
        uint64_t memory_reads = 0;
        uint64_t memory_writes = 0;
        uint64_t register_reads = 0;
        uint64_t register_writes = 0;
    };
    
    DebugStatistics debug_stats_;
    mutable std::mutex stats_mutex_;
};

} // namespace m5tab5::emulator