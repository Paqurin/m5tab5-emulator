#include "emulator/utils/error.hpp"
#include <sstream>
#include <format>

namespace m5tab5::emulator {

std::string Error::to_string() const {
    std::ostringstream oss;
    oss << "[" << error_code_to_string(code_) << "]";
    
    if (!message_.empty()) {
        oss << " " << message_;
    }
    
    oss << " (at " << location_.file_name() 
        << ":" << location_.line() 
        << ":" << location_.column() 
        << " in " << location_.function_name() << ")";
    
    return oss.str();
}

const char* error_code_to_string(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::SUCCESS:
            return "SUCCESS";
            
        // Configuration errors
        case ErrorCode::CONFIG_INVALID_FORMAT:
            return "CONFIG_INVALID_FORMAT";
        case ErrorCode::CONFIG_MISSING_FIELD:
            return "CONFIG_MISSING_FIELD";
        case ErrorCode::CONFIG_INVALID_VALUE:
            return "CONFIG_INVALID_VALUE";
        case ErrorCode::CONFIG_FILE_NOT_FOUND:
            return "CONFIG_FILE_NOT_FOUND";
            
        // Memory errors
        case ErrorCode::MEMORY_INVALID_ADDRESS:
            return "MEMORY_INVALID_ADDRESS";
        case ErrorCode::MEMORY_ACCESS_VIOLATION:
            return "MEMORY_ACCESS_VIOLATION";
        case ErrorCode::MEMORY_ALIGNMENT_ERROR:
            return "MEMORY_ALIGNMENT_ERROR";
        case ErrorCode::MEMORY_OUT_OF_BOUNDS:
            return "MEMORY_OUT_OF_BOUNDS";
        case ErrorCode::MEMORY_ALLOCATION_FAILED:
            return "MEMORY_ALLOCATION_FAILED";
            
        // CPU errors
        case ErrorCode::CPU_INVALID_INSTRUCTION:
            return "CPU_INVALID_INSTRUCTION";
        case ErrorCode::CPU_PRIVILEGE_VIOLATION:
            return "CPU_PRIVILEGE_VIOLATION";
        case ErrorCode::CPU_BREAKPOINT_HIT:
            return "CPU_BREAKPOINT_HIT";
        case ErrorCode::CPU_EXCEPTION:
            return "CPU_EXCEPTION";
        case ErrorCode::CPU_HALT:
            return "CPU_HALT";
            
        // Peripheral errors
        case ErrorCode::PERIPHERAL_NOT_FOUND:
            return "PERIPHERAL_NOT_FOUND";
        case ErrorCode::PERIPHERAL_INIT_FAILED:
            return "PERIPHERAL_INIT_FAILED";
        case ErrorCode::PERIPHERAL_BUSY:
            return "PERIPHERAL_BUSY";
        case ErrorCode::PERIPHERAL_TIMEOUT:
            return "PERIPHERAL_TIMEOUT";
        case ErrorCode::PERIPHERAL_INVALID_CONFIG:
            return "PERIPHERAL_INVALID_CONFIG";
            
        // I/O errors
        case ErrorCode::IO_INVALID_PORT:
            return "IO_INVALID_PORT";
        case ErrorCode::IO_READ_FAILED:
            return "IO_READ_FAILED";
        case ErrorCode::IO_WRITE_FAILED:
            return "IO_WRITE_FAILED";
        case ErrorCode::IO_DEVICE_NOT_READY:
            return "IO_DEVICE_NOT_READY";
            
        // System errors
        case ErrorCode::SYSTEM_NOT_INITIALIZED:
            return "SYSTEM_NOT_INITIALIZED";
        case ErrorCode::SYSTEM_ALREADY_RUNNING:
            return "SYSTEM_ALREADY_RUNNING";
        case ErrorCode::SYSTEM_SHUTDOWN_FAILED:
            return "SYSTEM_SHUTDOWN_FAILED";
        case ErrorCode::SYSTEM_RESOURCE_EXHAUSTED:
            return "SYSTEM_RESOURCE_EXHAUSTED";
            
        // Plugin errors
        case ErrorCode::PLUGIN_NOT_FOUND:
            return "PLUGIN_NOT_FOUND";
        case ErrorCode::PLUGIN_LOAD_FAILED:
            return "PLUGIN_LOAD_FAILED";
        case ErrorCode::PLUGIN_INVALID_INTERFACE:
            return "PLUGIN_INVALID_INTERFACE";
        case ErrorCode::PLUGIN_VERSION_MISMATCH:
            return "PLUGIN_VERSION_MISMATCH";
            
        // Generic errors
        case ErrorCode::INVALID_PARAMETER:
            return "INVALID_PARAMETER";
        case ErrorCode::NOT_IMPLEMENTED:
            return "NOT_IMPLEMENTED";
        case ErrorCode::OPERATION_FAILED:
            return "OPERATION_FAILED";
        case ErrorCode::TIMEOUT:
            return "TIMEOUT";
        case ErrorCode::ABORTED:
            return "ABORTED";
            
        default:
            return "UNKNOWN_ERROR";
    }
}

}  // namespace m5tab5::emulator