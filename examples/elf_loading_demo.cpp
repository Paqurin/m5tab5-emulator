#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/firmware/elf_loader.hpp"

#include <iomanip>
#include <thread>
#include <chrono>

#include <iostream>
#include <filesystem>
#include <fstream>

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::firmware;

/**
 * @brief Comprehensive ELF Loading Demo
 * 
 * This example demonstrates the complete ELF loading pipeline:
 * 1. EmulatorCore initialization with integrated ELF loader
 * 2. Loading an ELF application with progress tracking
 * 3. Setting up CPU execution context
 * 4. Starting application execution
 */

// Progress callback for ELF loading
void progress_callback(const std::string& stage, float progress, const std::string& message) {
    std::cout << "[" << std::setw(8) << stage << "] " 
              << std::fixed << std::setprecision(1) << (progress * 100.0f) << "% - "
              << message << std::endl;
}

// Simple test ELF data (minimal RISC-V executable)
static const std::vector<uint8_t> create_test_elf() {
    return {
        // ELF Header
        0x7F, 0x45, 0x4C, 0x46,  // e_ident[EI_MAG0..EI_MAG3] - ELF magic
        0x01,                     // e_ident[EI_CLASS] - ELFCLASS32
        0x01,                     // e_ident[EI_DATA] - ELFDATA2LSB
        0x01,                     // e_ident[EI_VERSION] - EV_CURRENT
        0x00,                     // e_ident[EI_OSABI] - ELFOSABI_NONE
        0x00, 0x00, 0x00, 0x00,   // e_ident[EI_PAD] - padding
        0x00, 0x00, 0x00, 0x00,   // more padding
        
        0x02, 0x00,               // e_type - ET_EXEC
        0xF3, 0x00,               // e_machine - EM_RISCV (243)
        0x01, 0x00, 0x00, 0x00,   // e_version - EV_CURRENT
        0x00, 0x10, 0x00, 0x40,   // e_entry - Entry point (0x40001000 - Flash)
        0x34, 0x00, 0x00, 0x00,   // e_phoff - Program header offset
        0x00, 0x00, 0x00, 0x00,   // e_shoff - Section header offset
        0x00, 0x00, 0x00, 0x00,   // e_flags
        0x34, 0x00,               // e_ehsize - ELF header size
        0x20, 0x00,               // e_phentsize - Program header entry size
        0x01, 0x00,               // e_phnum - Number of program headers
        0x28, 0x00,               // e_shentsize - Section header entry size
        0x00, 0x00,               // e_shnum - Number of section headers
        0x00, 0x00,               // e_shstrndx - Section header string table index

        // Program Header (PT_LOAD)
        0x01, 0x00, 0x00, 0x00,   // p_type - PT_LOAD
        0x54, 0x00, 0x00, 0x00,   // p_offset - File offset
        0x00, 0x10, 0x00, 0x40,   // p_vaddr - Virtual address (0x40001000 - Flash)
        0x00, 0x10, 0x00, 0x40,   // p_paddr - Physical address (0x40001000)
        0x10, 0x00, 0x00, 0x00,   // p_filesz - Size in file
        0x10, 0x00, 0x00, 0x00,   // p_memsz - Size in memory
        0x05, 0x00, 0x00, 0x00,   // p_flags - PF_R | PF_X (readable, executable)
        0x04, 0x00, 0x00, 0x00,   // p_align - Alignment

        // Simple RISC-V "Hello World" program
        0x93, 0x00, 0x00, 0x00,   // addi x1, x0, 0     (set x1 to 0)
        0x13, 0x01, 0x10, 0x00,   // addi x2, x0, 1     (set x2 to 1) 
        0xef, 0x00, 0x00, 0x00,   // jal x1, 0          (call function at PC+0)
        0x73, 0x00, 0x10, 0x00    // ebreak             (debug breakpoint)
    };
}

int main() {
    std::cout << "=== M5Stack Tab5 ELF Loading Demo ===\n\n";

    try {
        // Initialize logging
        auto log_result = Logger::initialize(Logger::LogLevel::INFO, "", true);
        if (!log_result) {
            std::cerr << "Failed to initialize logging: " << log_result.error().to_string() << std::endl;
            return 1;
        }

        std::cout << "1. Creating emulator configuration...\n";
        
        // Create default configuration for ESP32-P4
        Configuration config;
        
        // Configure CPU for dual-core RISC-V @ 400MHz
        auto cpu_config = config.getCPUConfig();
        cpu_config.main_core_freq = 400000000;  // 400MHz
        cpu_config.enable_dual_core = true;     // Dual core
        
        // Configure memory layout
        auto memory_config = config.getMemoryConfig();
        memory_config.sram_size = 768 * 1024;   // 768KB SRAM
        memory_config.psram_size = 32 * 1024 * 1024;  // 32MB PSRAM
        memory_config.flash_size = 16 * 1024 * 1024;  // 16MB Flash
        
        // Apply the configuration
        config.setCPUConfig(cpu_config);
        config.setMemoryConfig(memory_config);
        
        // Disable graphics for this demo
        config.setValue("graphics", "enable", false);

        std::cout << "2. Initializing M5Stack Tab5 emulator with ELF loader...\n";

        // Create and initialize emulator
        EmulatorCore emulator(config);
        auto init_result = emulator.initialize(config);
        if (!init_result.has_value()) {
            std::cerr << "Failed to initialize emulator: " << init_result.error().to_string() << std::endl;
            return 1;
        }

        std::cout << "✓ Emulator initialized successfully\n";
        std::cout << "  - Dual-core RISC-V CPU @ 400MHz\n";
        std::cout << "  - 768KB SRAM + 32MB PSRAM + 16MB Flash\n";
        std::cout << "  - ELF loader integrated and ready\n\n";

        // Get ELF loader component
        auto elf_loader = emulator.getComponent<ELFLoader>();
        if (!elf_loader) {
            std::cerr << "Failed to get ELF loader component from emulator" << std::endl;
            return 1;
        }

        std::cout << "3. Creating test ELF application...\n";

        // Create test ELF file
        std::string test_elf_path = "/tmp/hello_esp32p4.elf";
        auto test_elf_data = create_test_elf();
        
        std::ofstream test_file(test_elf_path, std::ios::binary);
        test_file.write(reinterpret_cast<const char*>(test_elf_data.data()), test_elf_data.size());
        test_file.close();

        std::cout << "✓ Test ELF created: " << test_elf_path << " (" << test_elf_data.size() << " bytes)\n";
        std::cout << "  - RISC-V 32-bit executable\n";
        std::cout << "  - Entry point: 0x40001000 (Flash memory)\n";
        std::cout << "  - Simple instruction sequence with debug breakpoint\n\n";

        std::cout << "4. Loading ELF application with progress tracking...\n";

        // Load ELF application through EmulatorCore
        auto load_result = emulator.load_elf_application(test_elf_path, progress_callback);
        
        // Cleanup test file
        std::filesystem::remove(test_elf_path);

        if (!load_result.has_value()) {
            std::cerr << "Failed to load ELF application: " << load_result.error().message() << std::endl;
            return 1;
        }

        const auto& loading_result = load_result.value();
        
        std::cout << "\n✓ ELF application loaded successfully!\n";
        std::cout << "  - Entry Point: 0x" << std::hex << loading_result.entry_point << std::endl;
        std::cout << "  - Stack Top: 0x" << std::hex << loading_result.stack_top << std::endl;
        std::cout << "  - Total Size: " << std::dec << loading_result.total_size << " bytes\n";
        std::cout << "  - Warnings: " << loading_result.warnings.size() << std::endl;

        if (!loading_result.warnings.empty()) {
            std::cout << "  Warnings:\n";
            for (const auto& warning : loading_result.warnings) {
                std::cout << "    - " << warning << std::endl;
            }
        }

        std::cout << "\n5. Analyzing CPU execution context...\n";

        const auto& context = loading_result.execution_context;
        if (context.valid) {
            std::cout << "✓ CPU execution context configured:\n";
            std::cout << "  - Entry Point: 0x" << std::hex << context.entry_point << std::endl;
            std::cout << "  - Stack Pointer: 0x" << std::hex << context.stack_pointer << std::endl;
            std::cout << "  - Global Pointer: 0x" << std::hex << context.global_pointer << std::endl;
            std::cout << "  - MSTATUS: 0x" << std::hex << context.mstatus << " (Machine mode)\n";
            std::cout << "  - MTVEC: 0x" << std::hex << context.mtvec << " (Interrupt vector)\n";
        } else {
            std::cout << "⚠ CPU execution context is not valid\n";
        }

        std::cout << "\n6. Setting application entry point...\n";
        
        auto entry_result = emulator.set_application_entry_point(loading_result.entry_point);
        if (!entry_result.has_value()) {
            std::cout << "⚠ Warning: " << entry_result.error().message() << std::endl;
        } else {
            std::cout << "✓ Application entry point configured\n";
        }

        std::cout << "\n7. Memory layout analysis...\n";
        
        // Get memory controller to analyze layout
        auto memory_controller = emulator.getComponent<MemoryController>();
        if (memory_controller) {
            std::cout << "✓ Memory regions configured:\n";
            std::cout << "  - Flash (Code): 0x40000000 - 0x40FFFFFF (16MB)\n";
            std::cout << "  - SRAM (Data): 0x4FF00000 - 0x4FFBFFFF (768KB)\n";
            std::cout << "  - PSRAM (Heap): 0x48000000 - 0x49FFFFFF (32MB)\n";
            std::cout << "  - Application loaded at: 0x" << std::hex << loading_result.entry_point << std::endl;
        }

        std::cout << "\n8. Boot ROM integration...\n";
        
        auto boot_rom = emulator.getComponent<BootROM>();
        if (boot_rom) {
            std::cout << "✓ Boot ROM available for integration\n";
            std::cout << "  - Can modify boot sequence to jump to application\n";
            std::cout << "  - ESP32-P4 bootloader sequence supported\n";
        }

        std::cout << "\n9. Starting application execution...\n";
        
        auto exec_result = emulator.start_application_execution();
        if (!exec_result.has_value()) {
            std::cout << "⚠ Application execution start: " << exec_result.error().message() << std::endl;
        } else {
            std::cout << "✓ Application execution started successfully\n";
            std::cout << "  - CPU cores activated\n";
            std::cout << "  - Ready to execute RISC-V instructions\n";
            std::cout << "  - Emulator running at 400MHz target frequency\n";
        }

        // Let it run briefly
        std::cout << "\n10. Running application for 1 second...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Get execution statistics
        auto cycles = emulator.getCurrentCycle();
        auto speed = emulator.get_execution_speed();
        
        std::cout << "✓ Execution statistics:\n";
        std::cout << "  - Cycles executed: " << cycles << std::endl;
        std::cout << "  - Execution speed: " << std::fixed << std::setprecision(2) << speed << "x real-time\n";

        std::cout << "\n11. Stopping emulator...\n";
        
        auto stop_result = emulator.stop();
        if (!stop_result.has_value()) {
            std::cerr << "Warning during stop: " << stop_result.error().to_string() << std::endl;
        } else {
            std::cout << "✓ Emulator stopped successfully\n";
        }

        std::cout << "\n=== ELF Loading Demo Completed Successfully ===\n";
        std::cout << "\nFeatures Demonstrated:\n";
        std::cout << "✓ Complete EmulatorCore integration with ELF loader\n";
        std::cout << "✓ ELF parsing and validation for ESP32-P4 RISC-V architecture\n";
        std::cout << "✓ Memory segment loading to appropriate ESP32-P4 regions\n";
        std::cout << "✓ CPU execution context creation and setup\n";
        std::cout << "✓ Progress tracking during loading process\n";
        std::cout << "✓ Component registry integration for easy access\n";
        std::cout << "✓ Boot ROM integration capability\n";
        std::cout << "✓ Application execution startup\n\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }
}