#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <thread>
#include <chrono>

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"

using namespace m5tab5::emulator;

std::atomic<bool> shutdown_requested{false};
std::unique_ptr<EmulatorCore> emulator;

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutdown requested...\n";
        shutdown_requested = true;
        if (emulator) {
            auto result = emulator->stop();
            if (!result) {
                std::cerr << "Failed to stop emulator: " << result.error().to_string() << std::endl;
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
              << "  -h, --help             Show this help message\n"
              << "\nExamples:\n"
              << "  " << program_name << "                          # Run with default settings\n"
              << "  " << program_name << " -c config/debug.json     # Run with custom config\n"
              << "  " << program_name << " -d -g 3333               # Debug mode with GDB server\n"
              << "  " << program_name << " -l debug -f emulator.log # Debug logging to file\n";
}

int main(int argc, char* argv[]) {
    // Set up signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Parse command line arguments
    std::string config_file = "config/default.json";
    std::string log_level = "info";
    std::string log_file = "";
    bool debug_mode = false;
    bool enable_profiling = false;
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
        else {
            std::cerr << "Error: Unknown option " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    try {
        // Initialize logging first
        LogLevel log_level_enum = LogLevel::INFO;
        if (log_level == "trace") log_level_enum = LogLevel::TRACE;
        else if (log_level == "debug") log_level_enum = LogLevel::DEBUG_LEVEL;
        else if (log_level == "info") log_level_enum = LogLevel::INFO;
        else if (log_level == "warn") log_level_enum = LogLevel::WARN;
        else if (log_level == "error") log_level_enum = LogLevel::ERROR;
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
        
        // Create and initialize emulator
        emulator = std::make_unique<EmulatorCore>(config);
        
        auto init_result = emulator->initialize(config);
        if (!init_result) {
            LOG_ERROR("Failed to initialize emulator: " + init_result.error().to_string());
            return 1;
        }
        
        LOG_INFO("Emulator initialized successfully");
        
        // Start emulator
        auto start_result = emulator->start();
        if (!start_result) {
            LOG_ERROR("Failed to start emulator: " + start_result.error().to_string());
            return 1;
        }
        
        LOG_INFO("Emulator started - Press Ctrl+C to stop");
        
        // Main loop - wait for shutdown signal
        while (!shutdown_requested && emulator->get_state() == EmulatorState::RUNNING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Print periodic status
            static int status_counter = 0;
            if (++status_counter % 100 == 0) {  // Every 10 seconds
                LOG_INFO("Status: " + std::to_string(emulator->get_cycles_executed()) + 
                         " cycles executed, " + std::to_string(emulator->get_execution_speed()) + "x speed");
            }
        }
        
        // Shutdown emulator
        LOG_INFO("Shutting down emulator...");
        auto shutdown_result = emulator->shutdown();
        if (!shutdown_result) {
            LOG_ERROR("Failed to shutdown emulator cleanly: " + shutdown_result.error().to_string());
            return 1;
        }
        
        LOG_INFO("Emulator shutdown completed");
        Logger::shutdown();
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        if (emulator) {
            emulator->shutdown();
        }
        Logger::shutdown();
        return 1;
    }
}