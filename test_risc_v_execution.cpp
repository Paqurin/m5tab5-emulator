#include "emulator/core/emulator_core.hpp"
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/config/configuration.hpp"
#include <iostream>
#include <iomanip>

using namespace m5tab5::emulator;

int main() {
    std::cout << "Testing RISC-V Instruction Execution Engine" << std::endl;
    
    try {
        // Create default configuration
        Configuration config;
        std::cout << "Using default configuration" << std::endl;
        
        // Create emulator core
        auto emulator = std::make_unique<EmulatorCore>(config);
        
        // Initialize emulator
        auto init_result = emulator->initialize(config);
        if (!init_result.has_value()) {
            std::cerr << "Failed to initialize emulator" << std::endl;
            return 1;
        }
        
        // Get the dual core manager and CPU core
        auto dual_core = emulator->getComponent<DualCoreManager>();
        if (!dual_core) {
            std::cerr << "Failed to get dual core manager" << std::endl;
            return 1;
        }
        
        auto core0_result = dual_core->get_core(DualCoreManager::CoreId::CORE_0);
        if (!core0_result.has_value()) {
            std::cerr << "Failed to get CPU core 0" << std::endl;
            return 1;
        }
        auto core0 = core0_result.value();
        
        // Get memory controller to set up test program
        auto memory = emulator->getComponent<MemoryController>();
        if (!memory) {
            std::cerr << "Failed to get memory controller" << std::endl;
            return 1;
        }
        
        std::cout << "Setting up test RISC-V program..." << std::endl;
        
        // Create a simple RISC-V program
        // Address 0x10000000: ADDI x1, x0, 42      (li x1, 42)
        // Address 0x10000004: ADDI x2, x1, 100     (addi x2, x1, 100) 
        // Address 0x10000008: ADD x3, x1, x2       (add x3, x1, x2)
        // Address 0x1000000C: SUB x4, x3, x1       (sub x4, x3, x1)
        
        uint32_t program[] = {
            0x02A00093,  // ADDI x1, x0, 42
            0x06408113,  // ADDI x2, x1, 100  
            0x002081B3,  // ADD x3, x1, x2
            0x40118233   // SUB x4, x3, x1
        };
        
        uint32_t program_start = 0x10000000;
        
        // Write program to memory
        for (size_t i = 0; i < sizeof(program)/sizeof(program[0]); i++) {
            uint32_t addr = program_start + i * 4;
            if (memory->write32(addr, program[i]) != EmulatorError::Success) {
                std::cerr << "Failed to write instruction " << i << " to memory" << std::endl;
                return 1;
            }
        }
        
        // Set PC to start of program
        core0->setProgramCounter(program_start);
        
        std::cout << "\nInitial register state:" << std::endl;
        auto& registers = core0->getRegisters();
        std::cout << "x1 = 0x" << std::hex << std::setw(8) << std::setfill('0') << registers.read(1) << std::endl;
        std::cout << "x2 = 0x" << std::hex << std::setw(8) << std::setfill('0') << registers.read(2) << std::endl;
        std::cout << "x3 = 0x" << std::hex << std::setw(8) << std::setfill('0') << registers.read(3) << std::endl;
        std::cout << "x4 = 0x" << std::hex << std::setw(8) << std::setfill('0') << registers.read(4) << std::endl;
        std::cout << "PC = 0x" << std::hex << std::setw(8) << std::setfill('0') << core0->getProgramCounter() << std::endl;
        
        std::cout << "\nExecuting instructions..." << std::endl;
        
        // Execute each instruction step by step
        for (int i = 0; i < 4; i++) {
            std::cout << "\nStep " << (i+1) << ":" << std::endl;
            std::cout << "PC = 0x" << std::hex << std::setw(8) << std::setfill('0') << core0->getProgramCounter() << std::endl;
            
            // Execute one instruction
            EmulatorError result = core0->step();
            if (result != EmulatorError::Success) {
                std::cerr << "Execution failed at step " << (i+1) << " with error: " << static_cast<int>(result) << std::endl;
                return 1;
            }
            
            std::cout << "x1 = " << std::dec << registers.read(1) << " (0x" << std::hex << registers.read(1) << ")" << std::endl;
            std::cout << "x2 = " << std::dec << registers.read(2) << " (0x" << std::hex << registers.read(2) << ")" << std::endl;
            std::cout << "x3 = " << std::dec << registers.read(3) << " (0x" << std::hex << registers.read(3) << ")" << std::endl;
            std::cout << "x4 = " << std::dec << registers.read(4) << " (0x" << std::hex << registers.read(4) << ")" << std::endl;
        }
        
        std::cout << "\nFinal verification:" << std::endl;
        uint32_t x1 = registers.read(1);  // Should be 42
        uint32_t x2 = registers.read(2);  // Should be 142 (42 + 100)
        uint32_t x3 = registers.read(3);  // Should be 184 (42 + 142)
        uint32_t x4 = registers.read(4);  // Should be 142 (184 - 42)
        
        bool success = true;
        if (x1 != 42) {
            std::cerr << "ERROR: x1 should be 42, got " << x1 << std::endl;
            success = false;
        }
        if (x2 != 142) {
            std::cerr << "ERROR: x2 should be 142, got " << x2 << std::endl;
            success = false;
        }
        if (x3 != 184) {
            std::cerr << "ERROR: x3 should be 184, got " << x3 << std::endl;
            success = false;
        }
        if (x4 != 142) {
            std::cerr << "ERROR: x4 should be 142, got " << x4 << std::endl;
            success = false;
        }
        
        if (success) {
            std::cout << "SUCCESS: All instructions executed correctly!" << std::endl;
            std::cout << "✓ ADDI x1, x0, 42 → x1 = " << x1 << std::endl;
            std::cout << "✓ ADDI x2, x1, 100 → x2 = " << x2 << std::endl;
            std::cout << "✓ ADD x3, x1, x2 → x3 = " << x3 << std::endl;
            std::cout << "✓ SUB x4, x3, x1 → x4 = " << x4 << std::endl;
            
            // Show performance counters
            auto perf = core0->getPerformanceCounters();
            std::cout << "\nPerformance counters:" << std::endl;
            std::cout << "Instructions executed: " << perf.instructions_executed << std::endl;
            std::cout << "Cycles executed: " << perf.cycles_executed << std::endl;
            std::cout << "Memory accesses: " << perf.memory_accesses << std::endl;
            
            return 0;
        } else {
            std::cerr << "FAILURE: Instruction execution produced incorrect results" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}