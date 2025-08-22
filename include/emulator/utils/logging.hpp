#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/logger.h>
#include <memory>
#include <string>

namespace m5tab5::emulator {

enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4
};

class Logger {
public:
    static Result<void> initialize(LogLevel level = LogLevel::INFO,
                                   const std::string& log_file = "",
                                   bool enable_console = true);
    
    static void shutdown();
    
    static std::shared_ptr<spdlog::logger> get_logger(const std::string& name = "main");
    
    static void set_level(LogLevel level);
    static LogLevel get_level();
    
    template<typename... Args>
    static void trace(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->trace(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void debug(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->debug(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void info(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->info(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void warn(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->warn(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void error(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->error(format, std::forward<Args>(args)...);
        }
    }
    
private:
    static spdlog::level::level_enum to_spdlog_level(LogLevel level);
    static LogLevel from_spdlog_level(spdlog::level::level_enum level);
    
    static bool initialized_;
    static LogLevel current_level_;
};

// Convenience macros for logging
#define LOG_TRACE(...) ::m5tab5::emulator::Logger::trace(__VA_ARGS__)
#define LOG_DEBUG(...) ::m5tab5::emulator::Logger::debug(__VA_ARGS__)
#define LOG_INFO(...) ::m5tab5::emulator::Logger::info(__VA_ARGS__)
#define LOG_WARN(...) ::m5tab5::emulator::Logger::warn(__VA_ARGS__)
#define LOG_ERROR(...) ::m5tab5::emulator::Logger::error(__VA_ARGS__)

// Component-specific loggers
class ComponentLogger {
public:
    explicit ComponentLogger(const std::string& component_name)
        : component_name_(component_name) {}
    
    template<typename... Args>
    void trace(const std::string& format, Args&&... args) const {
        Logger::trace("[{}] " + format, component_name_, std::forward<Args>(args)...);
    }
    
    template<typename... Args>
    void debug(const std::string& format, Args&&... args) const {
        Logger::debug("[{}] " + format, component_name_, std::forward<Args>(args)...);
    }
    
    template<typename... Args>
    void info(const std::string& format, Args&&... args) const {
        Logger::info("[{}] " + format, component_name_, std::forward<Args>(args)...);
    }
    
    template<typename... Args>
    void warn(const std::string& format, Args&&... args) const {
        Logger::warn("[{}] " + format, component_name_, std::forward<Args>(args)...);
    }
    
    template<typename... Args>
    void error(const std::string& format, Args&&... args) const {
        Logger::error("[{}] " + format, component_name_, std::forward<Args>(args)...);
    }

private:
    std::string component_name_;
};

#define DECLARE_LOGGER(name) \
    static const ::m5tab5::emulator::ComponentLogger logger_{name}

#define COMPONENT_LOG_TRACE(...) logger_.trace(__VA_ARGS__)
#define COMPONENT_LOG_DEBUG(...) logger_.debug(__VA_ARGS__)
#define COMPONENT_LOG_INFO(...) logger_.info(__VA_ARGS__)
#define COMPONENT_LOG_WARN(...) logger_.warn(__VA_ARGS__)
#define COMPONENT_LOG_ERROR(...) logger_.error(__VA_ARGS__)

}  // namespace m5tab5::emulator