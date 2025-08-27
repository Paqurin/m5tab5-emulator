#include "emulator/cpu/cpu_core.hpp"
#include "emulator/cpu/register_file.hpp"
#include "emulator/cpu/instruction_decoder.hpp" 
#include "emulator/memory/memory_controller.hpp"
#include "emulator/config/configuration.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>

using namespace m5tab5::emulator;

// Simple memory interface for testing
class TestMemory : public MemoryInterface {
private:
    std::vector<uint8_t> memory_;
    static const uint32_t MEMORY_SIZE = 64 * 1024; // 64KB
    
public:
    TestMemory() : memory_(MEMORY_SIZE, 0) {}
    
    EmulatorError read8(Address address, uint8_t& value) override {
        if (address >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        value = memory_[address];
        return EmulatorError::Success;
    }
    
    EmulatorError read16(Address address, uint16_t& value) override {
        if (address + 1 >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        value = memory_[address] | (memory_[address + 1] << 8);
        return EmulatorError::Success;
    }
    
    EmulatorError read32(Address address, uint32_t& value) override {
        if (address + 3 >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        value = memory_[address] | (memory_[address + 1] << 8) | 
                (memory_[address + 2] << 16) | (memory_[address + 3] << 24);
        return EmulatorError::Success;
    }
    
    EmulatorError write8(Address address, uint8_t value) override {
        if (address >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        memory_[address] = value;
        return EmulatorError::Success;
    }
    
    EmulatorError write16(Address address, uint16_t value) override {
        if (address + 1 >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        memory_[address] = value & 0xFF;
        memory_[address + 1] = (value >> 8) & 0xFF;
        return EmulatorError::Success;
    }
    
    EmulatorError write32(Address address, uint32_t value) override {
        if (address + 3 >= MEMORY_SIZE) return EmulatorError::InvalidAddress;
        memory_[address] = value & 0xFF;
        memory_[address + 1] = (value >> 8) & 0xFF;
        memory_[address + 2] = (value >> 16) & 0xFF;
        memory_[address + 3] = (value >> 24) & 0xFF;
        return EmulatorError::Success;
    }
    
    bool isValidAddress(Address address) const override {
        return address < MEMORY_SIZE;
    }
    
    bool isWritableAddress(Address address) const override {
        return address < MEMORY_SIZE;
    }
    
    bool isExecutableAddress(Address address) const override {
        return address < MEMORY_SIZE;
    }
};

int main() {
    std::cout << "Testing RISC-V CPU Core Instruction Execution" << std::endl;
    
    // Create test memory interface
    TestMemory memory;
    
    // Create CPU core configuration
    CpuCore::CoreConfig config;
    config.type = CpuCore::CoreType::MainCore0;
    config.frequency_hz = 400000000; // 400MHz
    
    // Create CPU core
    CpuCore cpu(config, memory);
    
    // Initialize CPU
    if (cpu.initialize() != EmulatorError::Success) {
        std::cerr << "Failed to initialize CPU core" << std::endl;
        return 1;
    }
    
    std::cout << "CPU core initialized successfully" << std::endl;
    
    // Set up test program in memory
    uint32_t program_start = 0x1000;
    uint32_t program[] = {
        0x02A00093,  // ADDI x1, x0, 42     (li x1, 42)
        0x06408113,  // ADDI x2, x1, 100    (addi x2, x1, 100) 
        0x002081B3,  // ADD x3, x1, x2      (add x3, x1, x2)
        0x40118233   // SUB x4, x3, x1      (sub x4, x3, x1)
    };
    
    // Write program to memory
    for (size_t i = 0; i < sizeof(program)/sizeof(program[0]); i++) {
        uint32_t addr = program_start + i * 4;
        if (memory.write32(addr, program[i]) != EmulatorError::Success) {
            std::cerr << "Failed to write instruction " << i << " to memory" << std::endl;
            return 1;
        }
    }
    
    // Set PC to start of program
    cpu.setProgramCounter(program_start);
    
    std::cout << "\\nTest program loaded at 0x" << std::hex << program_start << std::endl;
    std::cout << "Instructions:" << std::endl;
    std::cout << "  ADDI x1, x0, 42   # x1 = 42" << std::endl;
    std::cout << "  ADDI x2, x1, 100  # x2 = 142" << std::endl;
    std::cout << "  ADD  x3, x1, x2   # x3 = 184" << std::endl;
    std::cout << "  SUB  x4, x3, x1   # x4 = 142" << std::endl;
    
    // Show initial register state
    std::cout << "\\nInitial register state:" << std::endl;
    auto& registers = cpu.getRegisters();
    std::cout << "x1 = " << std::dec << registers.read(1) << " (0x" << std::hex << registers.read(1) << ")" << std::endl;
    std::cout << "x2 = " << std::dec << registers.read(2) << " (0x" << std::hex << registers.read(2) << ")" << std::endl;
    std::cout << "x3 = " << std::dec << registers.read(3) << " (0x" << std::hex << registers.read(3) << ")" << std::endl;
    std::cout << "x4 = " << std::dec << registers.read(4) << " (0x" << std::hex << registers.read(4) << ")" << std::endl;
    std::cout << "PC = 0x" << std::hex << cpu.getProgramCounter() << std::endl;
    
    // Execute instructions step by step
    std::cout << "\\nExecuting instructions..." << std::endl;
    bool success = true;
    
    for (int step = 0; step < 4; step++) {
        std::cout << "\\nStep " << (step + 1) << ":" << std::endl;
        std::cout << "PC = 0x" << std::hex << cpu.getProgramCounter() << std::endl;
        
        EmulatorError result = cpu.step();
        if (result != EmulatorError::Success) {
            std::cerr << "ERROR: Execution failed at step " << (step + 1) 
                      << " with error code: " << static_cast<int>(result) << std::endl;
            success = false;
            break;
        }
        
        std::cout << "After execution:" << std::endl;
        std::cout << "  x1 = " << std::dec << registers.read(1) << " (0x" << std::hex << registers.read(1) << ")" << std::endl;
        std::cout << "  x2 = " << std::dec << registers.read(2) << " (0x" << std::hex << registers.read(2) << ")" << std::endl;
        std::cout << "  x3 = " << std::dec << registers.read(3) << " (0x" << std::hex << registers.read(3) << ")" << std::endl;
        std::cout << "  x4 = " << std::dec << registers.read(4) << " (0x" << std::hex << registers.read(4) << ")" << std::endl;
    }
    
    if (success) {
        std::cout << "\\n=== VERIFICATION ===" << std::endl;
        
        uint32_t x1 = registers.read(1);
        uint32_t x2 = registers.read(2);
        uint32_t x3 = registers.read(3);
        uint32_t x4 = registers.read(4);
        
        // Verify expected results
        std::cout << "Expected vs Actual:" << std::endl;
        std::cout << "x1: Expected 42, Got " << x1 << (x1 == 42 ? " ✓" : " ✗") << std::endl;
        std::cout << "x2: Expected 142, Got " << x2 << (x2 == 142 ? " ✓" : " ✗") << std::endl;  
        std::cout << "x3: Expected 184, Got " << x3 << (x3 == 184 ? " ✓" : " ✗") << std::endl;
        std::cout << "x4: Expected 142, Got " << x4 << (x4 == 142 ? " ✓" : " ✗") << std::endl;
        
        if (x1 == 42 && x2 == 142 && x3 == 184 && x4 == 142) {
            std::cout << "\\n🎉 SUCCESS: All instructions executed correctly!" << std::endl;
            std::cout << "✅ RISC-V instruction execution engine is working!" << std::endl;
            
            // Show performance counters
            auto perf = cpu.getPerformanceCounters();
            std::cout << "\\nPerformance:" << std::endl;
            std::cout << "Instructions executed: " << perf.instructions_executed << std::endl;
            std::cout << "Memory accesses: " << perf.memory_accesses << std::endl;
            
            return 0;
        } else {
            std::cerr << "\\n❌ FAILURE: Incorrect execution results" << std::endl;
            return 1;
        }
    }
    
    return 1;
}