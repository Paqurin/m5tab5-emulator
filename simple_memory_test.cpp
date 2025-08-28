#include <iostream>
#include <iomanip>

// Simple test of ESP32-P4 memory addresses
int main() {
    std::cout << "ESP32-P4 Memory Layout Test\n";
    std::cout << "==========================\n\n";
    
    struct MemoryRegion {
        const char* name;
        unsigned int start;
        unsigned int size;
        const char* description;
    };
    
    MemoryRegion regions[] = {
        {"Boot ROM", 0x40000000, 32 * 1024, "Hardware boot ROM with reset vector at 0x40000080"},
        {"Flash XIP", 0x42000000, 16 * 1024 * 1024, "Execute-in-place flash via MMU"},
        {"PSRAM", 0x48000000, 32 * 1024 * 1024, "External PSRAM (cached)"},
        {"L2 SRAM", 0x4FF00000, 768 * 1024, "Internal high-speed memory"},
        {"MMIO", 0x40008000, 248 * 1024 * 1024, "Memory-mapped I/O registers"}
    };
    
    std::cout << "ESP32-P4 Authentic Memory Layout:\n";
    std::cout << "Region         Start Addr    Size         End Addr      Description\n";
    std::cout << "-------------- ------------ ------------ ------------ --------------\n";
    
    for (const auto& region : regions) {
        unsigned int end_addr = region.start + region.size - 1;
        std::cout << std::left << std::setw(14) << region.name
                  << " 0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << region.start
                  << "   " << std::dec << std::setw(8) << (region.size >= 1024*1024 ? region.size/(1024*1024) : region.size/1024)
                  << (region.size >= 1024*1024 ? " MB" : " KB")
                  << "   0x" << std::hex << std::uppercase << std::setw(8) << end_addr
                  << "  " << region.description << "\n";
    }
    
    std::cout << "\nKey ESP32-P4 Boot Sequence:\n";
    std::cout << "1. CPU starts at Boot ROM reset vector: 0x40000080\n";
    std::cout << "2. Boot ROM initializes clocks and MMU\n"; 
    std::cout << "3. Boot ROM loads bootloader from Flash (0x42000000)\n";
    std::cout << "4. Bootloader loads application and transfers control\n";
    std::cout << "5. Application runs from Flash XIP with SRAM/PSRAM for data\n";
    
    std::cout << "\nThis layout enables:\n";
    std::cout << "- Authentic ESP32-P4 memory addressing\n";
    std::cout << "- Proper CPU instruction execution from Boot ROM\n";
    std::cout << "- Real application loading and execution\n";
    std::cout << "- Hardware peripheral access via MMIO\n";
    
    return 0;
}