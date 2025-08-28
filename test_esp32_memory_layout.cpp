#include "emulator/memory/memory_controller.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/utils/types.hpp"
#include <iostream>
#include <iomanip>

using namespace m5tab5::emulator;

int main() {
    std::cout << "ESP32-P4 Memory Layout Validation Test\n";
    std::cout << "=====================================\n\n";
    
    try {
        // Create default configuration
        Configuration config;
        // For this test, we'll use the defaults
        
        // Use default config values
        
        // Create and initialize memory controller
        MemoryController memory_controller;
        auto init_result = memory_controller.initialize(config);
        if (!init_result) {
            std::cerr << "Failed to initialize memory controller\n";
            return 1;
        }
        
        std::cout << "Memory Controller initialized successfully\n\n";
        
        // Test key memory addresses
        struct MemoryTest {
            const char* name;
            Address address;
            bool should_exist;
        };
        
        MemoryTest tests[] = {
            {"Boot ROM base", 0x40000000, true},
            {"Boot ROM reset vector", 0x40000080, true},
            {"Boot ROM end", 0x40007FFF, true},
            {"Flash XIP base", 0x42000000, true},
            {"Flash bootloader", 0x42000000, true},
            {"Flash application", 0x42001000, true},
            {"PSRAM base", 0x48000000, true},
            {"L2 SRAM base", 0x4FF00000, true},
            {"L2 SRAM end", 0x4FFBFFFF, true},
            {"Invalid address", 0x30000000, false}
        };
        
        std::cout << "Testing memory address validity:\n";
        std::cout << "Address      Region                 Valid Expected Status\n";
        std::cout << "------------ --------------------- ----- -------- ------\n";
        
        bool all_passed = true;
        for (const auto& test : tests) {
            auto valid_result = memory_controller.is_valid_address(test.address);
            bool is_valid = valid_result.has_value() && valid_result.value();
            bool passed = (is_valid == test.should_exist);
            
            std::cout << std::hex << std::setfill('0') << std::setw(8) << std::uppercase 
                     << "0x" << test.address << "   "
                     << std::left << std::setw(21) << std::setfill(' ') << test.name 
                     << (is_valid ? "YES  " : "NO   ")
                     << (test.should_exist ? "YES     " : "NO      ")
                     << (passed ? "PASS" : "FAIL") << "\n";
            
            if (!passed) all_passed = false;
        }
        
        std::cout << "\n";
        
        // Test instruction fetch from Boot ROM
        std::cout << "Testing instruction fetch from Boot ROM:\n";
        
        // Try to read from reset vector
        auto instruction_result = memory_controller.read_u32(BOOT_ROM_RESET_VECTOR);
        if (instruction_result) {
            std::cout << "Reset vector instruction: 0x" 
                     << std::hex << std::setfill('0') << std::setw(8) << std::uppercase
                     << instruction_result.value() << "\n";
            
            if (instruction_result.value() != 0) {
                std::cout << "Boot ROM contains executable code ✓\n";
            } else {
                std::cout << "Boot ROM appears empty (all zeros) ✗\n";
                all_passed = false;
            }
        } else {
            std::cout << "Failed to read from reset vector ✗\n";
            all_passed = false;
        }
        
        // Just test memory layout validation (skip CPU for now)
        std::cout << "\nSkipping CPU test - focusing on memory layout validation\n";
        // For this test, we're primarily validating the memory controller setup
        
        std::cout << "\n===========================================\n";
        if (all_passed) {
            std::cout << "All tests PASSED! ESP32-P4 memory layout is authentic.\n";
            return 0;
        } else {
            std::cout << "Some tests FAILED. Memory layout needs fixes.\n";
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}