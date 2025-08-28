#include "emulator/utils/logging.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <cctype>

namespace m5tab5::emulator {

bool Logger::initialized_ = false;
Logger::LogLevel Logger::current_level_ = Logger::LogLevel::INFO;

Result<void> Logger::initialize(Logger::LogLevel level, const std::string& log_file, bool enable_console) {
    try {
        if (initialized_) {
            return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING, 
                "Logger already initialized"));
        }
        
        std::vector<spdlog::sink_ptr> sinks;
        
        // Add console sink if enabled
        if (enable_console) {
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
            sinks.push_back(console_sink);
        }
        
        // Add file sink if log file specified
        if (!log_file.empty()) {
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                log_file, 1024 * 1024 * 10, 3);  // 10MB max size, 3 files
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] [%t] %v");
            sinks.push_back(file_sink);
        }
        
        if (sinks.empty()) {
            return unexpected(MAKE_ERROR(INVALID_PARAMETER,
                "At least one logging sink must be enabled"));
        }
        
        // Create main logger
        auto logger = std::make_shared<spdlog::logger>("main", sinks.begin(), sinks.end());
        logger->set_level(to_spdlog_level(level));
        logger->flush_on(spdlog::level::warn);
        
        spdlog::register_logger(logger);
        spdlog::set_default_logger(logger);
        
        current_level_ = level;
        initialized_ = true;
        
        LOG_INFO("Logger initialized with level: {}", static_cast<int>(level));
        return {};
        
    } catch (const spdlog::spdlog_ex& ex) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to initialize logger: " + std::string(ex.what())));
    } catch (const std::exception& ex) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Unexpected error during logger initialization: " + std::string(ex.what())));
    }
}

void Logger::shutdown() {
    if (initialized_) {
        LOG_INFO("Shutting down logger");
        spdlog::shutdown();
        initialized_ = false;
    }
}

std::shared_ptr<spdlog::logger> Logger::get_logger(const std::string& name) {
    if (!initialized_) {
        // Return nullptr if logger not initialized - don't auto-initialize
        return nullptr;
    }
    
    auto logger = spdlog::get(name);
    if (!logger && name != "main") {
        // Create a new logger based on the main logger's sinks
        auto main_logger = spdlog::get("main");
        if (main_logger) {
            auto sinks = main_logger->sinks();
            logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
            logger->set_level(main_logger->level());
            spdlog::register_logger(logger);
        }
    }
    
    return logger ? logger : spdlog::default_logger();
}

void Logger::set_level(Logger::LogLevel level) {
    current_level_ = level;
    if (auto logger = get_logger()) {
        logger->set_level(to_spdlog_level(level));
    }
    // Update all registered loggers
    spdlog::set_level(to_spdlog_level(level));
}

Logger::LogLevel Logger::get_level() {
    return current_level_;
}

spdlog::level::level_enum Logger::to_spdlog_level(Logger::LogLevel level) {
    switch (level) {
        case Logger::LogLevel::TRACE: return spdlog::level::trace;
        case Logger::LogLevel::DEBUG_LEVEL: return spdlog::level::debug;
        case Logger::LogLevel::INFO: return spdlog::level::info;
        case Logger::LogLevel::WARN: return spdlog::level::warn;
        case Logger::LogLevel::ERROR_LEVEL: return spdlog::level::err;
        default: return spdlog::level::info;
    }
}

Logger::LogLevel Logger::from_spdlog_level(spdlog::level::level_enum level) {
    switch (level) {
        case spdlog::level::trace: return Logger::LogLevel::TRACE;
        case spdlog::level::debug: return Logger::LogLevel::DEBUG_LEVEL;
        case spdlog::level::info: return Logger::LogLevel::INFO;
        case spdlog::level::warn: return Logger::LogLevel::WARN;
        case spdlog::level::err: return Logger::LogLevel::ERROR_LEVEL;
        default: return Logger::LogLevel::INFO;
    }
}

Logger::LogLevel Logger::from_string(const std::string& level_str) {
    std::string lower = level_str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    
    if (lower == "trace") return Logger::LogLevel::TRACE;
    else if (lower == "debug") return Logger::LogLevel::DEBUG_LEVEL;
    else if (lower == "info") return Logger::LogLevel::INFO;
    else if (lower == "warn" || lower == "warning") return Logger::LogLevel::WARN;
    else if (lower == "error" || lower == "err") return Logger::LogLevel::ERROR_LEVEL;
    else return Logger::LogLevel::INFO; // default fallback
}

}  // namespace m5tab5::emulator