/**
 * @file basic_emulator.cpp
 * @brief Basic M5Stack Tab5 Emulator example demonstrating fundamental setup and lifecycle management
 * 
 * This example shows:
 * - Configuration loading and validation
 * - Emulator creation and initialization
 * - Basic emulation loop
 * - Proper shutdown and resource cleanup
 * - Error handling patterns
 * 
 * Usage:
 *   ./basic_emulator [--config config_file] [--debug] [--help]
 * 
 * @author M5Stack Tab5 Emulator Project
 * @date 2024
 */

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logger.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

// Global flag for graceful shutdown
std::atomic<bool> running{true};

// Signal handler for graceful shutdown
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived shutdown signal, stopping emulator..." << std::endl;
        running = false;
    }
}

// Print usage information
void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]\n"
              << "Options:\n"
              << "  --config <file>   Load configuration from file (default: config/default.json)\n"
              << "  --debug           Enable debug logging\n"
              << "  --help            Show this help message\n"
              << std::endl;
}

// Parse command line arguments
struct CommandLineArgs {
    std::string config_file = "config/default.json";
    bool debug = false;
    bool show_help = false;
};

CommandLineArgs parse_args(int argc, char* argv[]) {
    CommandLineArgs args;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            args.show_help = true;
        } else if (arg == "--debug") {
            args.debug = true;
        } else if (arg == "--config" && i + 1 < argc) {
            args.config_file = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            args.show_help = true;
        }
    }
    
    return args;
}

int main(int argc, char* argv[]) {
    std::cout << "=== M5Stack Tab5 Emulator - Basic Example ===" << std::endl;
    std::cout << "Demonstrating basic emulator setup and lifecycle management" << std::endl;
    std::cout << "Press Ctrl+C to gracefully shutdown the emulator" << std::endl << std::endl;
    
    // Parse command line arguments
    auto args = parse_args(argc, argv);
    
    if (args.show_help) {
        print_usage(argv[0]);
        return 0;
    }
    
    try {
        // Set up signal handlers for graceful shutdown
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        
        // Initialize logging
        if (args.debug) {
            utils::Logger::set_level(utils::Logger::Level::DEBUG);
            std::cout << "Debug logging enabled" << std::endl;
        }
        
        std::cout << "Loading configuration from: " << args.config_file << std::endl;
        
        // Step 1: Load configuration
        auto config = config::Configuration::load(args.config_file);
        if (!config.has_value()) {
            std::cerr << "❌ Failed to load configuration from: " << args.config_file << std::endl;
            std::cerr << "   Error code: " << static_cast<int>(config.error()) << std::endl;
            std::cerr << "   Make sure the file exists and is valid JSON" << std::endl;
            return -1;
        }
        
        std::cout << "✅ Configuration loaded successfully" << std::endl;
        
        // Display some configuration details
        std::cout << "Configuration details:" << std::endl;
        std::cout << "  - Display: " << config.value().display.width << "x" 
                  << config.value().display.height << std::endl;
        std::cout << "  - CPU cores: " << config.value().cpu.core_count << std::endl;
        std::cout << "  - Memory: " << (config.value().memory.psram_size / 1024 / 1024) 
                  << " MB PSRAM" << std::endl;
        std::cout << std::endl;
        
        // Step 2: Create emulator instance
        std::cout << "Creating emulator instance..." << std::endl;
        
        auto emulator = EmulatorCore::create(config.value());
        if (!emulator.has_value()) {
            std::cerr << "❌ Failed to create emulator instance" << std::endl;
            std::cerr << "   Error code: " << static_cast<int>(emulator.error()) << std::endl;
            return -1;
        }
        
        std::cout << "✅ Emulator instance created successfully" << std::endl;
        
        // Step 3: Initialize emulator components
        std::cout << "Initializing emulator components..." << std::endl;
        
        auto init_result = emulator.value()->initialize();
        if (!init_result.has_value()) {
            std::cerr << "❌ Failed to initialize emulator" << std::endl;
            std::cerr << "   Error code: " << static_cast<int>(init_result.error()) << std::endl;
            return -1;
        }
        
        std::cout << "✅ Emulator components initialized successfully" << std::endl;
        
        // Step 4: Start emulator
        std::cout << "Starting emulator..." << std::endl;
        
        auto start_result = emulator.value()->start();
        if (!start_result.has_value()) {
            std::cerr << "❌ Failed to start emulator" << std::endl;
            std::cerr << "   Error code: " << static_cast<int>(start_result.error()) << std::endl;
            return -1;
        }
        
        std::cout << "✅ Emulator started successfully" << std::endl;
        std::cout << "🚀 M5Stack Tab5 Emulator is now running!" << std::endl;
        std::cout << "   A graphics window should appear showing the emulated display" << std::endl;
        std::cout << std::endl;
        
        // Step 5: Main emulation loop
        std::cout << "Entering main emulation loop..." << std::endl;
        
        auto last_status_check = std::chrono::steady_clock::now();
        constexpr auto STATUS_CHECK_INTERVAL = std::chrono::seconds(5);
        
        while (running) {
            // Check emulator status periodically
            auto now = std::chrono::steady_clock::now();
            if (now - last_status_check >= STATUS_CHECK_INTERVAL) {
                auto status = emulator.value()->get_status();
                if (status.has_value()) {
                    const auto& s = status.value();
                    
                    // Only print status if we're in debug mode to avoid spam
                    if (args.debug) {
                        std::cout << "📊 Emulator status: ";
                        switch (s.state) {
                            case EmulatorCore::State::STOPPED:
                                std::cout << "STOPPED";
                                running = false;  // Exit if stopped
                                break;
                            case EmulatorCore::State::INITIALIZING:
                                std::cout << "INITIALIZING";
                                break;
                            case EmulatorCore::State::RUNNING:
                                std::cout << "RUNNING";
                                break;
                            case EmulatorCore::State::PAUSED:
                                std::cout << "PAUSED";
                                break;
                            case EmulatorCore::State::ERROR:
                                std::cout << "ERROR";
                                running = false;  // Exit on error
                                break;
                        }
                        
                        std::cout << " (Uptime: " 
                                  << std::chrono::duration_cast<std::chrono::seconds>(s.uptime).count() 
                                  << "s)" << std::endl;
                    }
                    
                    // Exit if emulator is in error state
                    if (s.state == EmulatorCore::State::ERROR) {
                        std::cerr << "❌ Emulator entered error state, shutting down" << std::endl;
                        running = false;
                    }
                } else {
                    std::cerr << "⚠️  Failed to get emulator status" << std::endl;
                }
                
                last_status_check = now;
            }
            
            // Sleep to prevent busy waiting
            // In a real application, you might process events or perform other work here
            std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
        }
        
        std::cout << std::endl << "Shutting down emulator..." << std::endl;
        
        // Step 6: Stop emulator
        auto stop_result = emulator.value()->stop();
        if (!stop_result.has_value()) {
            std::cerr << "⚠️  Failed to stop emulator gracefully" << std::endl;
            std::cerr << "   Error code: " << static_cast<int>(stop_result.error()) << std::endl;
        } else {
            std::cout << "✅ Emulator stopped successfully" << std::endl;
        }
        
        // Step 7: Shutdown and cleanup
        emulator.value()->shutdown();
        std::cout << "✅ Emulator shutdown complete" << std::endl;
        
        std::cout << std::endl << "🎉 Basic emulator example completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "💥 Exception caught: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "💥 Unknown exception caught" << std::endl;
        return -1;
    }
    
    return 0;
}

/*
 * Example Output:
 * 
 * === M5Stack Tab5 Emulator - Basic Example ===
 * Demonstrating basic emulator setup and lifecycle management
 * Press Ctrl+C to gracefully shutdown the emulator
 * 
 * Loading configuration from: config/default.json
 * ✅ Configuration loaded successfully
 * Configuration details:
 *   - Display: 1280x720
 *   - CPU cores: 2
 *   - Memory: 8 MB PSRAM
 * 
 * Creating emulator instance...
 * ✅ Emulator instance created successfully
 * Initializing emulator components...
 * ✅ Emulator components initialized successfully
 * Starting emulator...
 * ✅ Emulator started successfully
 * 🚀 M5Stack Tab5 Emulator is now running!
 *    A graphics window should appear showing the emulated display
 * 
 * Entering main emulation loop...
 * ^C
 * Received shutdown signal, stopping emulator...
 * 
 * Shutting down emulator...
 * ✅ Emulator stopped successfully
 * ✅ Emulator shutdown complete
 * 
 * 🎉 Basic emulator example completed successfully!
 */