#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <chrono>

namespace m5tab5::emulator {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using size_t = std::size_t;
using usize = std::size_t;

using Address = u32;
using PhysicalAddress = u32;
using VirtualAddress = u32;

constexpr u32 INVALID_ADDRESS = 0xFFFFFFFF;

using Cycles = u64;
using Frequency = u32;

using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::nanoseconds;

enum class CoreId : u8 {
    CORE_0 = 0,
    CORE_1 = 1,
    LP_CORE = 2,
    INVALID = 0xFF
};

// InterruptType moved to peripherals/interrupt_controller.hpp for comprehensive definitions

enum class MemoryAccessType : u8 {
    READ = 0,
    WRITE = 1,
    EXECUTE = 2,
    READ_WRITE = 3,
    INVALID = 0xFF
};

enum class GpioMode : u8 {
    INPUT = 0,
    OUTPUT = 1,
    INPUT_PULLUP = 2,
    INPUT_PULLDOWN = 3,
    OUTPUT_OPEN_DRAIN = 4,
    FUNCTION = 5,
    ANALOG = 6,
    INVALID = 0xFF
};

struct MemoryLayout {
    Address start_address;
    size_t size;
    bool writable;
    bool executable;
    bool cacheable;
    
    constexpr Address end_address() const {
        return start_address + size - 1;
    }
    
    constexpr bool contains(Address addr) const {
        return addr >= start_address && addr <= end_address();
    }
};

constexpr MemoryLayout FLASH_LAYOUT{
    .start_address = 0x10000000,
    .size = 16 * 1024 * 1024,  // 16MB
    .writable = false,
    .executable = true,
    .cacheable = true
};

constexpr MemoryLayout PSRAM_LAYOUT{
    .start_address = 0x20000000,
    .size = 32 * 1024 * 1024,  // 32MB
    .writable = true,
    .executable = false,
    .cacheable = true
};

constexpr MemoryLayout SRAM_LAYOUT{
    .start_address = 0x30000000,
    .size = 768 * 1024,  // 768KB
    .writable = true,
    .executable = true,
    .cacheable = true
};

constexpr MemoryLayout MMIO_LAYOUT{
    .start_address = 0x40000000,
    .size = 256 * 1024 * 1024,  // 256MB
    .writable = true,
    .executable = false,
    .cacheable = false
};

// Communication Interface Base Addresses
constexpr Address I2C0_BASE_ADDR = 0x40013000;
constexpr Address I2C1_BASE_ADDR = 0x40014000;
constexpr Address SPI0_BASE_ADDR = 0x40008000;
constexpr Address SPI1_BASE_ADDR = 0x40009000;
constexpr Address SPI2_BASE_ADDR = 0x4000A000;
constexpr Address UART0_BASE_ADDR = 0x40000000;
constexpr Address UART1_BASE_ADDR = 0x40001000;
constexpr Address UART2_BASE_ADDR = 0x40002000;
constexpr Address UART3_BASE_ADDR = 0x40003000;
constexpr Address UART4_BASE_ADDR = 0x40004000;

// PWM and Timer Base Addresses
constexpr Address PWM0_BASE_ADDR = 0x40010000;
constexpr Address PWM1_BASE_ADDR = 0x40011000;
constexpr Address PWM2_BASE_ADDR = 0x40012000;

// ADC Base Addresses
constexpr Address ADC0_BASE_ADDR = 0x40020000;
constexpr Address ADC1_BASE_ADDR = 0x40020400;

// Pin Mux Base Address
constexpr Address PIN_MUX_BASE_ADDR = 0x40030000;

template<typename T>
using UniquePtr = std::unique_ptr<T>;

template<typename T>
using SharedPtr = std::shared_ptr<T>;

template<typename T>
using WeakPtr = std::weak_ptr<T>;

template<typename T, typename... Args>
constexpr auto make_unique(Args&&... args) {
    return std::make_unique<T>(std::forward<Args>(args)...);
}

template<typename T, typename... Args>
constexpr auto make_shared(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace m5tab5::emulator