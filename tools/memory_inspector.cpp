#include "emulator/core/emulator_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <iostream>
#include <iomanip>

using namespace m5tab5::emulator;

int main(int argc, char* argv[]) {
    std::cout << "M5Stack Tab5 Emulator - Memory Inspector\n";
    std::cout << "Version 1.0.0\n\n";
    
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <address> <length>\n";
        std::cout << "  address: Memory address to start reading from (hex)\n";
        std::cout << "  length:  Number of bytes to read\n\n";
        std::cout << "Examples:\n";
        std::cout << "  " << argv[0] << " 0x10000000 256  # Read 256 bytes from Flash base\n";
        std::cout << "  " << argv[0] << " 0x30000000 1024 # Read 1KB from SRAM base\n";
        return 1;
    }
    
    // TODO: Parse address and length, connect to emulator, read memory
    std::cout << "Memory address: " << argv[1] << "\n";
    std::cout << "Read length:    " << argv[2] << " bytes\n";
    std::cout << "Memory inspector functionality coming soon...\n";
    
    return 0;
}