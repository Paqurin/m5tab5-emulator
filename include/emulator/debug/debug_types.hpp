#pragma once

#include "emulator/core/types.hpp"
#include <string>
#include <functional>

namespace m5tab5::emulator {

// Breakpoint types
enum class BreakpointType {
    EXECUTION,      // Break on instruction execution
    READ,          // Break on memory read
    WRITE,         // Break on memory write
    ACCESS         // Break on memory read or write
};

// Breakpoint structure 
struct Breakpoint {
    uint32_t id;
    BreakpointType type;
    Address address;
    size_t size = 4;
    bool enabled = true;
    uint32_t hit_count = 0;
    std::string condition; // Optional condition expression
    std::function<bool()> callback; // Optional callback
    
    Breakpoint() : id(0), type(BreakpointType::EXECUTION), address(0) {}
};

// Execution trace entry
struct TraceEntry {
    Address pc;
    uint32_t instruction;
    ClockCycle cycle;
    TimeStamp timestamp;
    
    TraceEntry() : pc(0), instruction(0), cycle(0) {}
};

} // namespace m5tab5::emulator