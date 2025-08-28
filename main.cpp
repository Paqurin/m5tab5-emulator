#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <thread>
#include <chrono>
#include <future>

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/shutdown_manager.hpp"
// #include "emulator/firmware/firmware_integration.hpp"  // Disabled for CLI version

using namespace m5tab5::emulator;

std::atomic<bool> shutdown_requested{false};
std::atomic<bool> signal_handled{false};
std::unique_ptr<EmulatorCore> emulator;
// std::unique_ptr<firmware::FirmwareIntegration> firmware_integration;  // Disabled for CLI version

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        // Prevent double signal handling
        bool expected = false;
        if (!signal_handled.compare_exchange_strong(expected, true)) {
            return;  // Signal already handled
        }
        
        std::cout << "\nShutdown requested...\n";
        shutdown_requested = true;
        
        // Use shutdown manager for orderly cleanup
        auto& shutdown_mgr = utils::ShutdownManager::instance();
        shutdown_mgr.request_shutdown();
        
        // Try to stop emulator quickly in signal handler (async-safe)
        if (emulator) {
            try {
                auto result = emulator->stop();
                if (!result) {
                    std::cerr << "Failed to stop emulator: " << result.error().to_string() << std::endl;
                }
            } catch (...) {
                std::cerr << "Exception during emergency stop\n";
            }
        }
    }
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "\nOptions:\n"
              << "  -c, --config <file>    Configuration file path\n"
              << "  -d, --debug            Enable debug mode\n"
              << "  -g, --gdb <port>       Enable GDB server on specified port\n"
              << "  -l, --log-level <level> Set log level (trace, debug, info, warn, error)\n"
              << "  -f, --log-file <file>  Log file path\n"
              << "  -p, --profile          Enable performance profiling\n"
              << "  -w, --firmware <file>  Load firmware ELF file\n"
              << "  --boot                 Boot loaded firmware immediately\n"
              << "  -h, --help             Show this help message\n"
              << "\nExamples:\n"
              << "  " << program_name << "                          # Run with default settings\n"
              << "  " << program_name << " -c config/debug.json     # Run with custom config\n"
              << "  " << program_name << " -d -g 3333               # Debug mode with GDB server\n"
              << "  " << program_name << " -l debug -f emulator.log # Debug logging to file\n"
              << "  " << program_name << " -w firmware.elf --boot   # Load and boot firmware\n";
}

int main(int argc, char* argv[]) {
    // Set up signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Parse command line arguments
    std::string config_file = "config/default.json";
    std::string log_level = "info";
    std::string log_file = "";
    std::string firmware_path = "";
    bool debug_mode = false;
    bool enable_profiling = false;
    bool boot_firmware = false;
    int gdb_port = 0;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                config_file = argv[++i];
            } else {
                std::cerr << "Error: Config file path required after " << arg << std::endl;
                return 1;
            }
        }
        else if (arg == "-d" || arg == "--debug") {
            debug_mode = true;
            log_level = "debug";
        }
        else if (arg == "-g" || arg == "--gdb") {
            if (i + 1 < argc) {
                gdb_port = std::stoi(argv[++i]);
                debug_mode = true;
            } else {
                std::cerr << "Error: GDB port required after " << arg << std::endl;
                return 1;
            }
        }
        else if (arg == "-l" || arg == "--log-level") {
            if (i + 1 < argc) {
                log_level = argv[++i];
            } else {
                std::cerr << "Error: Log level required after " << arg << std::endl;
                return 1;
            }
        }
        else if (arg == "-f" || arg == "--log-file") {
            if (i + 1 < argc) {
                log_file = argv[++i];
            } else {
                std::cerr << "Error: Log file path required after " << arg << std::endl;
                return 1;
            }
        }
        else if (arg == "-p" || arg == "--profile") {
            enable_profiling = true;
        }
        else if (arg == "-w" || arg == "--firmware") {
            if (i + 1 < argc) {
                firmware_path = argv[++i];
            } else {
                std::cerr << "Error: Firmware file path required after " << arg << std::endl;
                return 1;
            }
        }
        else if (arg == "--boot") {
            boot_firmware = true;
        }
        else {
            std::cerr << "Error: Unknown option " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    try {
        // Initialize logging first
        Logger::LogLevel log_level_enum = Logger::LogLevel::INFO;
        if (log_level == "trace") log_level_enum = Logger::LogLevel::TRACE;
        else if (log_level == "debug") log_level_enum = Logger::LogLevel::DEBUG_LEVEL;
        else if (log_level == "info") log_level_enum = Logger::LogLevel::INFO;
        else if (log_level == "warn") log_level_enum = Logger::LogLevel::WARN;
        else if (log_level == "error") log_level_enum = Logger::LogLevel::ERROR_LEVEL;
        else {
            std::cerr << "Error: Invalid log level '" << log_level << "'" << std::endl;
            return 1;
        }
        
        auto log_result = Logger::initialize(log_level_enum, log_file, true);
        if (!log_result) {
            std::cerr << "Failed to initialize logger: " << log_result.error().to_string() << std::endl;
            return 1;
        }
        
        LOG_INFO("M5Stack Tab5 Emulator starting...");
        LOG_INFO("Version: 1.0.0");
        LOG_INFO(std::string("Build: ") + __DATE__ + " " + __TIME__);
        
        // Load configuration
        Configuration config;
        
        // Try to load config file if it exists
        if (std::filesystem::exists(config_file)) {
            bool config_loaded = config.loadFromFile(config_file);
            if (!config_loaded) {
                LOG_ERROR("Failed to load configuration from: " + config_file);
                return 1;
            }
            LOG_INFO("Loaded configuration from: " + config_file);
        } else {
            LOG_INFO("Configuration file not found, using defaults");
        }
        
        // Override config with command line options
        if (debug_mode) {
            // Enable debugging features in config
            LOG_INFO("Debug mode enabled");
        }
        
        if (gdb_port > 0) {
            LOG_INFO("GDB server will be started on port " + std::to_string(gdb_port));
        }
        
        if (enable_profiling) {
            LOG_INFO("Performance profiling enabled");
        }
        
        if (!firmware_path.empty()) {
            LOG_INFO("Firmware file specified: " + firmware_path);
            if (boot_firmware) {
                LOG_INFO("Auto-boot firmware enabled");
            }
        }
        
        // Create and initialize emulator
        emulator = std::make_unique<EmulatorCore>(config);
        
        auto init_result = emulator->initialize(config);
        if (!init_result) {
            LOG_ERROR("Failed to initialize emulator: " + init_result.error().to_string());
            return 1;
        }
        
        LOG_INFO("Emulator initialized successfully");
        
        // Note: Firmware integration disabled in CLI version
        // For firmware loading, please use the GUI version: ./m5tab5-emulator-gui
        if (!firmware_path.empty()) {
            LOG_WARN("Firmware loading requested but not available in CLI version");
            LOG_INFO("Please use the GUI version for firmware loading: ./m5tab5-emulator-gui");
            LOG_INFO("CLI version provides basic emulator functionality only");
        }
        
        // Start emulator
        auto start_result = emulator->start();
        if (!start_result) {
            LOG_ERROR("Failed to start emulator: " + start_result.error().to_string());
            return 1;
        }
        
        LOG_INFO("M5Stack Tab5 Emulator (CLI) started - Press Ctrl+C to stop");
        LOG_INFO("For firmware loading and advanced features, use: ./m5tab5-emulator-gui");
        
        // Main loop - wait for shutdown signal
        LOG_DEBUG("Entering main loop, waiting for shutdown signal...");
        
        while (!shutdown_requested.load() && emulator->get_state() == EmulatorState::RUNNING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Print periodic status
            static int status_counter = 0;
            if (++status_counter % 100 == 0) {  // Every 10 seconds
                try {
                    LOG_INFO("Status: " + std::to_string(emulator->get_cycles_executed()) + 
                             " cycles executed, " + std::to_string(emulator->get_execution_speed()) + "x speed");
                } catch (const std::exception& e) {
                    LOG_WARN("Error getting emulator status: " + std::string(e.what()));
                }
            }
            
            // Check emulator state more frequently
            if (emulator->get_state() != EmulatorState::RUNNING) {
                LOG_INFO("Emulator state changed, exiting main loop");
                break;
            }
        }
        
        LOG_DEBUG("Main loop exited, proceeding with shutdown...");
        
        // Shutdown emulator using shutdown manager
        LOG_INFO("Shutting down emulator...");
        
        auto& shutdown_mgr = utils::ShutdownManager::instance();
        shutdown_mgr.request_shutdown();
        
        // Give components time to shutdown gracefully
        shutdown_mgr.execute_shutdown(std::chrono::milliseconds(5000));
        
        // firmware_integration disabled in CLI version
        
        // Final cleanup with timeout protection
        bool shutdown_success = false;
        try {
            auto future = std::async(std::launch::async, [&emulator]() -> Result<void> {
                return emulator->shutdown();
            });
            
            if (future.wait_for(std::chrono::milliseconds(3000)) == std::future_status::ready) {
                auto shutdown_result = future.get();
                if (!shutdown_result) {
                    LOG_WARN("Emulator did not shutdown cleanly: " + shutdown_result.error().to_string());
                } else {
                    shutdown_success = true;
                }
            } else {
                LOG_ERROR("Emulator shutdown timed out, forcing exit");
                // Don't wait for the future, just continue with cleanup
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during emulator shutdown: " + std::string(e.what()));
        }
        
        // Ensure emulator pointer is reset
        emulator.reset();
        
        LOG_INFO("Emulator shutdown completed ({})", shutdown_success ? "clean" : "forced");
        
        // Shutdown logger with timeout
        try {
            auto future = std::async(std::launch::async, []() {
                Logger::shutdown();
            });
            
            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
                std::cerr << "Logger shutdown timed out\n";
            }
        } catch (...) {
            std::cerr << "Exception during logger shutdown\n";
        }
        
        return shutdown_success ? 0 : 1;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        // firmware_integration disabled in CLI version
        if (emulator) {
            emulator->shutdown();
        }
        Logger::shutdown();
        return 1;
    }
}