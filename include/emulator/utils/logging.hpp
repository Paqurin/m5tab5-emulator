#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/logger.h>
#include <spdlog/fmt/bundled/format.h>
#include <memory>
#include <string>

namespace m5tab5::emulator {

enum class LogLevel {
    TRACE = 0,
    DEBUG_LEVEL = 1,
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
    static LogLevel from_string(const std::string& level_str);
    
    template<typename... Args>
    static void trace(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->trace(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void trace(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->trace(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void debug(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->debug(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void debug(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->debug(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void info(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->info(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void info(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->info(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void warn(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->warn(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void warn(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->warn(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void error(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->error(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void error(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->error(format, std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void critical(const std::string& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->critical(fmt::runtime(format), std::forward<Args>(args)...);
        }
    }
    
    template<typename... Args>
    static void critical(const fmt::basic_runtime<char>& format, Args&&... args) {
        if (auto logger = get_logger()) {
            logger->critical(format, std::forward<Args>(args)...);
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
#define LOG_CRITICAL(...) ::m5tab5::emulator::Logger::critical(__VA_ARGS__)

// Component-specific loggers
class ComponentLogger {
public:
    explicit ComponentLogger(const std::string& component_name)
        : component_name_(component_name) {}
    
    template<typename... Args>
    void trace(const std::string& format, Args&&... args) const {
        if (auto logger = Logger::get_logger()) {
            if constexpr (sizeof...(args) == 0) {
                logger->trace("[{}] {}", component_name_, format);
            } else {
                logger->trace(("[{}] " + format).c_str(), component_name_, std::forward<Args>(args)...);
            }
        }
    }
    
    template<typename... Args>
    void debug(const std::string& format, Args&&... args) const {
        if (auto logger = Logger::get_logger()) {
            if constexpr (sizeof...(Args) > 0) {
                // Has arguments - use C++20 compatible approach
                logger->debug(("[{}] " + format).c_str(), component_name_, std::forward<Args>(args)...);
            } else {
                // No arguments - just print the format as message
                logger->debug("[{}] {}", component_name_, format);
            }
        }
    }
    
    template<typename... Args>
    void info(const std::string& format, Args&&... args) const {
        if (auto logger = Logger::get_logger()) {
            if constexpr (sizeof...(args) == 0) {
                logger->info("[{}] {}", component_name_, format);
            } else {
                logger->info(("[{}] " + format).c_str(), component_name_, std::forward<Args>(args)...);
            }
        }
    }
    
    template<typename... Args>
    void warn(const std::string& format, Args&&... args) const {
        if (auto logger = Logger::get_logger()) {
            if constexpr (sizeof...(args) == 0) {
                logger->warn("[{}] {}", component_name_, format);
            } else {
                logger->warn(("[{}] " + format).c_str(), component_name_, std::forward<Args>(args)...);
            }
        }
    }
    
    template<typename... Args>
    void error(const std::string& format, Args&&... args) const {
        if (auto logger = Logger::get_logger()) {
            if constexpr (sizeof...(args) == 0) {
                logger->error("[{}] {}", component_name_.c_str(), format.c_str());
            } else {
                std::string full_format = "[{}] " + format;
                logger->error(full_format.c_str(), component_name_.c_str(), std::forward<Args>(args)...);
            }
        }
    }

private:
    std::string component_name_;
};

#define DECLARE_LOGGER(name) \
    static constexpr const char* logger_name_ = name

#define COMPONENT_LOG_TRACE(fmt, ...) do { } while(0)
#define COMPONENT_LOG_DEBUG(fmt, ...) do { } while(0)
#define COMPONENT_LOG_INFO(fmt, ...) do { } while(0)
#define COMPONENT_LOG_WARN(fmt, ...) do { } while(0)
#define COMPONENT_LOG_ERROR(fmt, ...) do { } while(0)

}  // namespace m5tab5::emulator