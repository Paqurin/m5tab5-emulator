#pragma once

#include <string>
#include <string_view>
#include <system_error>
#include <variant>
#include <optional>

// C++20 compatibility - std::expected is C++23
#if __cplusplus >= 202302L
#include <expected>
#else
#include <source_location>
#endif

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
    MEMORY_ACCESS_ERROR = 2005,
    
    // Cache errors
    CACHE_MISS = 2100,
    CACHE_FULL = 2101,
    
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
    SYSTEM_BUSY = 6004,
    SYSTEM_SHUTDOWN = 6005,
    
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
    ABORTED = 8004,
    INVALID_OPERATION = 8005,
    INVALID_STATE = 8006,
    BUFFER_OVERFLOW = 8007,
    INSUFFICIENT_DATA = 8008,
    TIMEOUT_ERROR = 8009,
    FILE_ERROR = 8010,
    ALREADY_INITIALIZED = 8011,
    INVALID_ARGUMENT = 8012,
    DEVICE_ERROR = 8013,
    NOT_INITIALIZED = 8014,
    NO_DATA_AVAILABLE = 8015,
    OPERATION_ABORTED = 8016,
    PROCESSING_ERROR = 8017
};

class Error {
public:
    explicit Error(ErrorCode code, 
                   std::source_location location = std::source_location::current())
        : code_(code), message_(), location_(location) {}
    
    Error(ErrorCode code,
          const std::string& message,
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

// C++20 compatible implementation of std::expected (must come before Result typedef)
#if __cplusplus < 202302L
template<typename T>
class unexpected {
private:
    T error_;

public:
    constexpr explicit unexpected(T&& error) : error_(std::move(error)) {}
    constexpr explicit unexpected(const T& error) : error_(error) {}
    
    constexpr const T& value() const& { return error_; }
    constexpr T& value() & { return error_; }
    constexpr T&& value() && { return std::move(error_); }
    constexpr const T&& value() const&& { return std::move(error_); }
};

template<typename T, typename E>
class expected {
private:
    std::variant<T, E> data_;

public:
    constexpr expected() = default;
    constexpr expected(const T& value) : data_(value) {}
    constexpr expected(T&& value) : data_(std::move(value)) {}
    constexpr expected(const unexpected<E>& unexp) : data_(unexp.value()) {}
    constexpr expected(unexpected<E>&& unexp) : data_(std::move(unexp.value())) {}
    
    constexpr bool has_value() const { return std::holds_alternative<T>(data_); }
    constexpr explicit operator bool() const { return has_value(); }
    
    constexpr const T& value() const& { return std::get<T>(data_); }
    constexpr T& value() & { return std::get<T>(data_); }
    constexpr T&& value() && { return std::get<T>(std::move(data_)); }
    constexpr const T&& value() const&& { return std::get<T>(std::move(data_)); }
    
    constexpr const E& error() const& { return std::get<E>(data_); }
    constexpr E& error() & { return std::get<E>(data_); }
    constexpr E&& error() && { return std::get<E>(std::move(data_)); }
    constexpr const E&& error() const&& { return std::get<E>(std::move(data_)); }
    
    constexpr const T& operator*() const& { return value(); }
    constexpr T& operator*() & { return value(); }
    constexpr T&& operator*() && { return std::move(value()); }
    constexpr const T&& operator*() const&& { return std::move(value()); }
    
    constexpr const T* operator->() const { return &value(); }
    constexpr T* operator->() { return &value(); }
};

// Specialization for void
template<typename E>
class expected<void, E> {
private:
    std::optional<E> error_;

public:
    constexpr expected() = default;
    constexpr expected(const unexpected<E>& unexp) : error_(unexp.value()) {}
    constexpr expected(unexpected<E>&& unexp) : error_(std::move(unexp.value())) {}
    
    constexpr bool has_value() const { return !error_.has_value(); }
    constexpr explicit operator bool() const { return has_value(); }
    
    constexpr void value() const { 
        if (error_.has_value()) {
            throw std::runtime_error("Expected contains error");
        }
    }
    
    constexpr const E& error() const& { return error_.value(); }
    constexpr E& error() & { return error_.value(); }
    constexpr E&& error() && { return std::move(error_.value()); }
    constexpr const E&& error() const&& { return std::move(error_.value()); }
};
#endif // __cplusplus < 202302L

#if __cplusplus >= 202302L
template<typename T>
using Result = std::expected<T, Error>;
#else
template<typename T>  
using Result = expected<T, Error>;
#endif

using VoidResult = Result<void>;

#if __cplusplus >= 202302L
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
#else
#define RETURN_IF_ERROR(expr) \
    do { \
        auto result = (expr); \
        if (!result) { \
            return unexpected(result.error()); \
        } \
    } while (0)

#define ASSIGN_OR_RETURN(var, expr) \
    auto result_##__LINE__ = (expr); \
    if (!result_##__LINE__) { \
        return unexpected(result_##__LINE__.error()); \
    } \
    var = std::move(result_##__LINE__.value())
#endif

#define MAKE_ERROR(code, message) \
    Error(ErrorCode::code, message)

#define MAKE_SIMPLE_ERROR(code) \
    Error(ErrorCode::code)

// Helper function for making errors
inline Error make_error(ErrorCode code, const std::string& message) {
    return Error(code, message);
}

const char* error_code_to_string(ErrorCode code) noexcept;

}  // namespace m5tab5::emulator