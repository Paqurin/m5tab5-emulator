#pragma once

#include <string>
#include <string_view>
#include <source_location>
#include <expected>
#include <system_error>

namespace m5tab5::emulator {

enum class ErrorCode {
    SUCCESS = 0,
    
    // Configuration errors
    CONFIG_INVALID_FORMAT = 1000,
    CONFIG_MISSING_FIELD = 1001,
    CONFIG_INVALID_VALUE = 1002,
    CONFIG_FILE_NOT_FOUND = 1003,
    
    // Memory errors
    MEMORY_INVALID_ADDRESS = 2000,
    MEMORY_ACCESS_VIOLATION = 2001,
    MEMORY_ALIGNMENT_ERROR = 2002,
    MEMORY_OUT_OF_BOUNDS = 2003,
    MEMORY_ALLOCATION_FAILED = 2004,
    
    // CPU errors
    CPU_INVALID_INSTRUCTION = 3000,
    CPU_PRIVILEGE_VIOLATION = 3001,
    CPU_BREAKPOINT_HIT = 3002,
    CPU_EXCEPTION = 3003,
    CPU_HALT = 3004,
    
    // Peripheral errors
    PERIPHERAL_NOT_FOUND = 4000,
    PERIPHERAL_INIT_FAILED = 4001,
    PERIPHERAL_BUSY = 4002,
    PERIPHERAL_TIMEOUT = 4003,
    PERIPHERAL_INVALID_CONFIG = 4004,
    
    // I/O errors
    IO_INVALID_PORT = 5000,
    IO_READ_FAILED = 5001,
    IO_WRITE_FAILED = 5002,
    IO_DEVICE_NOT_READY = 5003,
    
    // System errors
    SYSTEM_NOT_INITIALIZED = 6000,
    SYSTEM_ALREADY_RUNNING = 6001,
    SYSTEM_SHUTDOWN_FAILED = 6002,
    SYSTEM_RESOURCE_EXHAUSTED = 6003,
    
    // Plugin errors
    PLUGIN_NOT_FOUND = 7000,
    PLUGIN_LOAD_FAILED = 7001,
    PLUGIN_INVALID_INTERFACE = 7002,
    PLUGIN_VERSION_MISMATCH = 7003,
    
    // Generic errors
    INVALID_PARAMETER = 8000,
    NOT_IMPLEMENTED = 8001,
    OPERATION_FAILED = 8002,
    TIMEOUT = 8003,
    ABORTED = 8004
};

class Error {
public:
    Error(ErrorCode code, 
          std::string_view message = {},
          std::source_location location = std::source_location::current())
        : code_(code), message_(message), location_(location) {}
    
    Error(ErrorCode code,
          std::string&& message,
          std::source_location location = std::source_location::current())
        : code_(code), message_(std::move(message)), location_(location) {}
    
    ErrorCode code() const noexcept { return code_; }
    const std::string& message() const noexcept { return message_; }
    const std::source_location& location() const noexcept { return location_; }
    
    std::string to_string() const;
    
    bool operator==(const Error& other) const noexcept {
        return code_ == other.code_;
    }
    
    bool operator==(ErrorCode code) const noexcept {
        return code_ == code;
    }

private:
    ErrorCode code_;
    std::string message_;
    std::source_location location_;
};

template<typename T>
using Result = std::expected<T, Error>;

using VoidResult = Result<void>;

#define RETURN_IF_ERROR(expr) \
    do { \
        auto result = (expr); \
        if (!result) { \
            return std::unexpected(result.error()); \
        } \
    } while (0)

#define ASSIGN_OR_RETURN(var, expr) \
    auto result_##__LINE__ = (expr); \
    if (!result_##__LINE__) { \
        return std::unexpected(result_##__LINE__.error()); \
    } \
    var = std::move(result_##__LINE__.value())

#define MAKE_ERROR(code, message) \
    Error(ErrorCode::code, message)

#define MAKE_SIMPLE_ERROR(code) \
    Error(ErrorCode::code)

const char* error_code_to_string(ErrorCode code) noexcept;

}  // namespace m5tab5::emulator