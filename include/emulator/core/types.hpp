#pragma once

#include <cstdint>
#include <memory>
#include <functional>
#include <chrono>

namespace m5tab5::emulator {

// Core types
using Address = uint32_t;
using RegisterValue = uint32_t;
using ClockCycle = uint64_t;
using TimeStamp = std::chrono::high_resolution_clock::time_point;

// Standard integer types
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

// Memory layout constants
constexpr size_t FLASH_SIZE = 16 * 1024 * 1024;    // 16MB
constexpr size_t PSRAM_SIZE = 32 * 1024 * 1024;    // 32MB
constexpr size_t SRAM_SIZE = 768 * 1024;           // 768KB
constexpr size_t CACHE_SIZE = 8 * 1024;            // 8KB

constexpr Address FLASH_BASE = 0x10000000;
constexpr Address PSRAM_BASE = 0x20000000;
constexpr Address SRAM_BASE = 0x30000000;
constexpr Address PERIPHERAL_BASE = 0x40000000;

// CPU specifications
constexpr uint32_t CPU_FREQ_HZ = 400 * 1000 * 1000; // 400MHz
constexpr uint32_t LP_CORE_FREQ_HZ = 40 * 1000 * 1000; // 40MHz

// Display specifications
constexpr uint32_t DISPLAY_WIDTH = 1280;
constexpr uint32_t DISPLAY_HEIGHT = 720;
constexpr uint32_t DISPLAY_BPP = 24; // RGB888

// GPIO configuration
constexpr uint32_t GPIO_PIN_COUNT = 55;

// Error handling
enum class EmulatorError {
    Success = 0,
    InvalidAddress,
    InvalidOperation,
    HardwareFault,
    ConfigurationError,
    ResourceNotAvailable,
    RESOURCE_BUSY,
    TimeoutError,
    NotImplemented,
    MemoryAccessError
};

// Forward declarations
class EmulatorCore;
class MemoryController;
class PeripheralManager;
class DisplayController;
class AudioController;

} // namespace m5tab5::emulator