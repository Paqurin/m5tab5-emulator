# M5Stack Tab5 Emulator - Developer Guide

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Development Environment](#development-environment)
3. [Code Organization](#code-organization)
4. [Component Development](#component-development)
5. [Testing Strategy](#testing-strategy)
6. [Performance Considerations](#performance-considerations)
7. [Debugging Techniques](#debugging-techniques)
8. [Contributing Guidelines](#contributing-guidelines)
9. [Advanced Topics](#advanced-topics)

## Architecture Overview

### High-Level Architecture

The M5Stack Tab5 Emulator follows a modular, component-based architecture designed for maintainability, testability, and performance:

```
┌─────────────────────────────────────────────────────┐
│                 Application Layer                   │
├─────────────────────────────────────────────────────┤
│                 EmulatorCore                        │
│           (Orchestration & Lifecycle)               │
├─────────────────────────────────────────────────────┤
│  CPUCore  │  MemCtrl  │  Graphics  │  Peripherals   │
│           │           │            │                 │
│  - RISC-V │  - PSRAM  │  - Display │  - GPIO        │
│  - Dual   │  - Cache  │  - Touch   │  - I2C/SPI     │
│    Core   │  - DMA    │  - Render  │  - UART        │
├─────────────────────────────────────────────────────┤
│        Connectivity       │       Audio Pipeline     │
│                           │                         │
│  - Wi-Fi (802.11)        │  - ES8388 Codec         │
│  - Bluetooth (BLE/Classic)│  - Dual Microphones     │
│  - USB (Host/Device/OTG)  │  - Audio Processing     │
│  - RS-485 (Modbus RTU)   │  - Beamforming          │
├─────────────────────────────────────────────────────┤
│              Debug & Profiling Layer                │
│                                                     │
│  - Performance Profiler    - Resource Monitor       │
│  - Regression Detector     - Optimization Analyzer  │
├─────────────────────────────────────────────────────┤
│                 System Layer                        │
│                                                     │
│  - Configuration   - Error Handling   - Logging     │
│  - Threading       - Memory Pool      - Utilities   │
└─────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Modularity**: Each component is self-contained with well-defined interfaces
2. **Thread Safety**: All components support concurrent access with proper synchronization
3. **Error Handling**: Comprehensive error handling using `std::expected` pattern
4. **Performance**: Optimized for real-time operation with minimal overhead
5. **Testability**: Components are designed for unit and integration testing
6. **Extensibility**: Easy to add new peripherals and features

### Component Communication

Components communicate through:
- **Memory-Mapped I/O (MMIO)**: Hardware registers are mapped to memory addresses
- **Interrupt System**: Components can trigger and handle interrupts
- **Event Queues**: Asynchronous event processing for UI and network events
- **Shared State**: Thread-safe shared state for cross-component coordination

## Development Environment

### Required Tools

- **Compiler**: GCC 9+ or Clang 10+ with C++20 support
- **Build System**: CMake 3.16+
- **Version Control**: Git 2.25+
- **Libraries**: SDL2, spdlog, Google Test

### Development Dependencies

```bash
# Ubuntu/Debian
sudo apt install clang-format clang-tidy cppcheck valgrind
sudo apt install gdb lldb strace perf

# Development tools
sudo apt install cmake-curses-gui ninja-build ccache
```

### IDE Configuration

#### VS Code
Recommended extensions:
- C/C++ Extension Pack
- CMake Tools
- GitLens
- Doxygen Documentation Generator

#### CLion
Import the CMake project directly. CLion will automatically configure build targets and debugging.

### Build Configuration

#### Debug Build
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_PROFILING=ON -DENABLE_TESTING=ON ..
make -j$(nproc)
```

#### Release Build
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTING=OFF ..
make -j$(nproc)
```

#### Development Build (with all debugging tools)
```bash
cmake -DCMAKE_BUILD_TYPE=Debug \
      -DENABLE_PROFILING=ON \
      -DENABLE_TESTING=ON \
      -DENABLE_SANITIZERS=ON \
      -DENABLE_STATIC_ANALYSIS=ON ..
make -j$(nproc)
```

## Code Organization

### Directory Structure

```
include/emulator/
├── core/                    # Core emulator systems
│   ├── emulator_core.hpp   # Main orchestration
│   ├── component_manager.hpp
│   └── lifecycle_manager.hpp
├── cpu/                     # RISC-V CPU implementation
│   ├── cpu_core.hpp        # Dual-core CPU
│   ├── instruction_decoder.hpp
│   ├── register_file.hpp
│   └── interrupt_controller.hpp
├── memory/                  # Memory subsystem
│   ├── memory_controller.hpp
│   ├── cache_controller.hpp
│   ├── dma_controller.hpp
│   └── memory_protection.hpp
├── graphics/                # Display and input
│   ├── graphics_engine.hpp
│   ├── display_controller.hpp
│   ├── framebuffer.hpp
│   └── touch_controller.hpp
├── peripherals/             # Hardware peripherals
│   ├── gpio_controller.hpp
│   ├── i2c_controller.hpp
│   ├── spi_controller.hpp
│   ├── uart_controller.hpp
│   ├── pwm_controller.hpp
│   └── adc_controller.hpp
├── connectivity/            # Network and communication
│   ├── wifi_controller.hpp
│   ├── bluetooth_controller.hpp
│   ├── usb_controller.hpp
│   └── rs485_controller.hpp
├── audio/                   # Audio processing
│   ├── audio_pipeline.hpp
│   ├── codec_es8388.hpp
│   └── microphone_array.hpp
├── sensors/                 # Sensor simulation
│   ├── imu_bmi270.hpp
│   ├── camera_sc2356.hpp
│   └── sensor_fusion.hpp
├── debug/                   # Debugging and profiling
│   ├── profiler.hpp
│   ├── resource_monitor.hpp
│   ├── regression_detector.hpp
│   └── optimization_analyzer.hpp
├── config/                  # Configuration system
│   └── configuration.hpp
└── utils/                   # Utilities
    ├── types.hpp
    ├── error.hpp
    ├── logger.hpp
    └── threading.hpp
```

### Naming Conventions

- **Classes**: PascalCase (`GPIOController`, `MemoryController`)
- **Functions**: snake_case (`digital_write`, `get_status`)
- **Variables**: snake_case (`pin_count`, `current_state`)
- **Constants**: UPPER_SNAKE_CASE (`MAX_PIN_COUNT`, `DEFAULT_FREQUENCY`)
- **Namespaces**: lowercase (`emulator::core`, `emulator::peripherals`)

### Header Organization

All public headers follow this structure:

```cpp
#pragma once

// System includes
#include <chrono>
#include <memory>
#include <vector>

// Project includes
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

namespace emulator::peripherals {

class GPIOController {
public:
    // Public types and enums
    enum class Mode { INPUT, OUTPUT };
    
    // Static factory methods
    static expected<std::unique_ptr<GPIOController>, utils::ErrorCode> create();
    
    // Core interface
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Functional interface
    // ... rest of public interface
    
private:
    // Private implementation
    // ... private members
};

} // namespace emulator::peripherals
```

## Component Development

### Creating a New Component

1. **Define the Interface**: Start with a clear header file defining the public API
2. **Implement Core Logic**: Focus on the essential functionality first
3. **Add Error Handling**: Use `std::expected` for all operations that can fail
4. **Ensure Thread Safety**: Add appropriate synchronization primitives
5. **Write Tests**: Create unit tests covering all public functionality
6. **Add Documentation**: Document the API and usage patterns

### Example: Creating a New Sensor Component

```cpp
// include/emulator/sensors/temperature_sensor.hpp
#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <memory>
#include <chrono>
#include <functional>

namespace emulator::sensors {

class TemperatureSensor {
public:
    struct TemperatureReading {
        double celsius;
        double fahrenheit;
        std::chrono::high_resolution_clock::time_point timestamp;
        double accuracy;  // ±degrees
    };
    
    using ReadingCallback = std::function<void(const TemperatureReading&)>;
    
    static expected<std::unique_ptr<TemperatureSensor>, utils::ErrorCode> create(
        double min_temp = -40.0, double max_temp = 125.0);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Sensor operations
    expected<TemperatureReading, utils::ErrorCode> read_temperature();
    expected<void, utils::ErrorCode> set_reading_callback(ReadingCallback callback);
    expected<void, utils::ErrorCode> start_continuous_reading(std::chrono::milliseconds interval);
    void stop_continuous_reading();
    
    // Simulation controls
    void set_simulated_temperature(double celsius);
    void simulate_temperature_drift(double rate_per_second);
    void add_noise(double noise_level);
    
private:
    TemperatureSensor(double min_temp, double max_temp);
    
    void continuous_reading_loop();
    double generate_temperature_reading();
    
    double min_temperature_;
    double max_temperature_;
    double current_temperature_;
    double drift_rate_;
    double noise_level_;
    
    std::atomic<bool> is_reading_;
    std::thread reading_thread_;
    std::chrono::milliseconds reading_interval_;
    
    ReadingCallback reading_callback_;
    mutable std::mutex reading_mutex_;
};

} // namespace emulator::sensors
```

### Implementation Guidelines

#### Error Handling
Always use `std::expected` for operations that can fail:

```cpp
expected<void, utils::ErrorCode> GPIOController::configure_pin(uint8_t pin, Mode mode) {
    if (pin >= MAX_PIN_COUNT) {
        return utils::unexpected(utils::ErrorCode::INVALID_PARAMETER);
    }
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    pins_[pin].mode = mode;
    pins_[pin].configured = true;
    
    return {};  // Success
}
```

#### Thread Safety
Use appropriate synchronization:

```cpp
class GPIOController {
private:
    mutable std::shared_mutex pins_mutex_;  // For read-heavy operations
    mutable std::mutex interrupt_mutex_;    // For interrupt registration
    
public:
    bool digital_read(uint8_t pin) const {
        std::shared_lock<std::shared_mutex> lock(pins_mutex_);
        return pins_[pin].state;
    }
    
    void digital_write(uint8_t pin, bool state) {
        std::unique_lock<std::shared_mutex> lock(pins_mutex_);
        pins_[pin].state = state;
    }
};
```

#### Performance Optimization
- Use appropriate data structures for the use case
- Minimize dynamic allocation in hot paths
- Consider cache locality for frequently accessed data
- Use atomic operations for simple state variables

```cpp
// Good: cache-friendly structure
struct PinState {
    std::atomic<bool> state;
    std::atomic<Mode> mode;
    uint8_t interrupt_trigger;
    uint8_t pull_mode;
    // Pack related data together
};

// Avoid: scattered data access
class BadGPIOController {
    std::vector<bool> pin_states_;        // Separate vector
    std::vector<Mode> pin_modes_;         // Separate vector
    std::map<uint8_t, InterruptCallback> callbacks_;  // Expensive lookups
};
```

## Testing Strategy

### Test Organization

```
tests/
├── unit/                    # Unit tests
│   ├── test_gpio_controller.cpp
│   ├── test_i2c_controller.cpp
│   └── test_memory_controller.cpp
├── integration/             # Integration tests
│   ├── test_peripheral_coordination.cpp
│   ├── test_connectivity_stack.cpp
│   └── test_audio_pipeline.cpp
├── performance/             # Performance tests
│   ├── test_cpu_performance.cpp
│   ├── test_memory_bandwidth.cpp
│   └── test_graphics_throughput.cpp
├── regression/              # Regression tests
│   └── test_stability.cpp
└── fixtures/                # Test fixtures and utilities
    ├── test_fixtures.hpp
    └── mock_components.hpp
```

### Unit Testing Guidelines

Use Google Test framework with the following patterns:

```cpp
#include <gtest/gtest.h>
#include "emulator/peripherals/gpio_controller.hpp"
#include "tests/fixtures/test_fixtures.hpp"

class GPIOControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto result = peripherals::GPIOController::create();
        ASSERT_TRUE(result.has_value());
        gpio_ = std::move(result.value());
        
        auto init_result = gpio_->initialize();
        ASSERT_TRUE(init_result.has_value());
    }
    
    void TearDown() override {
        if (gpio_) {
            gpio_->shutdown();
        }
    }
    
    std::unique_ptr<peripherals::GPIOController> gpio_;
};

TEST_F(GPIOControllerTest, ConfigurePinAsOutput) {
    // Test pin configuration
    auto result = gpio_->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
    EXPECT_TRUE(result.has_value());
    
    // Test invalid pin
    auto invalid_result = gpio_->configure_pin(255, peripherals::GPIOController::Mode::OUTPUT);
    EXPECT_FALSE(invalid_result.has_value());
    EXPECT_EQ(invalid_result.error(), utils::ErrorCode::INVALID_PARAMETER);
}

TEST_F(GPIOControllerTest, DigitalWriteAndRead) {
    // Configure pin
    ASSERT_TRUE(gpio_->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT).has_value());
    
    // Write high
    EXPECT_TRUE(gpio_->digital_write(2, true).has_value());
    
    // Configure as input to read back
    ASSERT_TRUE(gpio_->configure_pin(2, peripherals::GPIOController::Mode::INPUT).has_value());
    
    auto read_result = gpio_->digital_read(2);
    ASSERT_TRUE(read_result.has_value());
    EXPECT_TRUE(read_result.value());
}

TEST_F(GPIOControllerTest, InterruptHandling) {
    bool interrupt_fired = false;
    uint8_t interrupt_pin = 0;
    bool interrupt_state = false;
    
    auto callback = [&](uint8_t pin, bool state) {
        interrupt_fired = true;
        interrupt_pin = pin;
        interrupt_state = state;
    };
    
    // Configure interrupt
    ASSERT_TRUE(gpio_->configure_pin(0, peripherals::GPIOController::Mode::INPUT).has_value());
    ASSERT_TRUE(gpio_->attach_interrupt(0, 
        peripherals::GPIOController::InterruptTrigger::RISING_EDGE, 
        callback).has_value());
    
    // Simulate pin state change
    gpio_->simulate_pin_change(0, true);
    
    // Wait for interrupt processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_TRUE(interrupt_fired);
    EXPECT_EQ(interrupt_pin, 0);
    EXPECT_TRUE(interrupt_state);
}
```

### Integration Testing

Test component interactions:

```cpp
TEST_F(IntegrationTest, GPIOToI2CWorkflow) {
    // Setup GPIO and I2C controllers
    auto gpio = create_gpio_controller();
    auto i2c = create_i2c_controller();
    
    // Configure GPIO pin to trigger I2C transaction
    gpio->configure_pin(5, GPIOController::Mode::INPUT);
    
    bool i2c_transaction_completed = false;
    auto gpio_callback = [&](uint8_t pin, bool state) {
        if (pin == 5 && !state) {  // Button press (active low)
            // Trigger I2C read from sensor
            auto result = i2c->read(0x68, 6);  // BMI270 accelerometer
            i2c_transaction_completed = result.has_value();
        }
    };
    
    gpio->attach_interrupt(5, GPIOController::InterruptTrigger::FALLING_EDGE, gpio_callback);
    
    // Simulate button press
    gpio->simulate_pin_change(5, false);
    
    // Wait for processing
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(i2c_transaction_completed);
}
```

### Performance Testing

Measure performance characteristics:

```cpp
TEST_F(PerformanceTest, GPIOPerformance) {
    constexpr size_t NUM_OPERATIONS = 100000;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i < NUM_OPERATIONS; ++i) {
        gpio_->digital_write(2, i % 2 == 0);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double ops_per_second = (NUM_OPERATIONS * 1e9) / duration.count();
    
    // Should achieve at least 1M operations per second
    EXPECT_GT(ops_per_second, 1000000.0);
    
    std::cout << "GPIO operations per second: " << ops_per_second << std::endl;
}
```

## Performance Considerations

### CPU-Intensive Operations

- Use compiler optimizations (`-O3`, `-march=native`)
- Minimize dynamic allocation in hot paths
- Use appropriate data structures (vectors vs maps vs unordered_maps)
- Consider CPU cache effects

### Memory Management

- Pre-allocate buffers where possible
- Use object pools for frequently created/destroyed objects
- Minimize memory fragmentation
- Monitor memory usage with built-in profiling tools

### Threading

- Use thread-local storage for per-thread data
- Minimize lock contention
- Use lock-free data structures where appropriate
- Balance thread count with CPU cores

### I/O Operations

- Use asynchronous I/O where possible
- Batch operations when feasible
- Implement proper buffering strategies

## Debugging Techniques

### Built-in Profiling

Use the emulator's profiling system:

```cpp
#include "emulator/debug/profiler.hpp"

void debug_example() {
    auto profiler = debug::Profiler::create();
    profiler.value()->initialize();
    profiler.value()->start_profiling("debug_session");
    
    {
        PROFILE_SCOPE(*profiler.value(), "critical_section");
        // Your code here
    }
    
    PROFILE_FUNCTION(*profiler.value());  // Profile entire function
    
    auto report = profiler.value()->generate_performance_report();
    std::cout << report.value() << std::endl;
}
```

### Memory Debugging

Use Valgrind for memory debugging:

```bash
# Build with debug symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

# Run with Valgrind
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all \
         --track-origins=yes ./m5tab5-emulator
```

### Address Sanitizer

Enable AddressSanitizer for runtime error detection:

```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_SANITIZERS=ON ..
make
./m5tab5-emulator
```

### GDB Debugging

Debug with GDB:

```bash
gdb ./m5tab5-emulator
(gdb) set args --config ../config/debug.json
(gdb) break EmulatorCore::initialize
(gdb) run
```

Useful GDB commands for multi-threaded debugging:
```bash
(gdb) info threads              # List all threads
(gdb) thread 2                  # Switch to thread 2
(gdb) thread apply all bt       # Backtrace all threads
```

## Contributing Guidelines

### Code Style

We use clang-format with the following style:

```bash
# Format all source files
find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

### Static Analysis

Run static analysis before submitting:

```bash
# Run clang-tidy
clang-tidy src/**/*.cpp -- -Iinclude -std=c++20

# Run cppcheck
cppcheck --enable=all --std=c++20 -Iinclude src/
```

### Pull Request Process

1. **Create Feature Branch**: `git checkout -b feature/amazing-feature`
2. **Write Tests**: Ensure new functionality has appropriate test coverage
3. **Run Full Test Suite**: `make test`
4. **Check Performance**: Run performance tests if applicable
5. **Update Documentation**: Update relevant documentation
6. **Submit PR**: Include clear description and test results

### Commit Message Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

Example:
```
feat(gpio): add interrupt support for all pins

- Implement interrupt registration system
- Add interrupt trigger configuration  
- Support multiple callbacks per pin
- Include comprehensive unit tests

Fixes #123
```

## Advanced Topics

### Custom Peripheral Development

To add a new peripheral:

1. **Create Header File**: Define the peripheral interface in `include/emulator/peripherals/`
2. **Implement Core Logic**: Create implementation in `src/peripherals/`
3. **Register with Core**: Add to `EmulatorCore` component registry
4. **Memory Mapping**: Add MMIO register definitions
5. **Add Tests**: Create comprehensive test suite

### RISC-V Instruction Extension

To add new RISC-V instructions:

1. **Define Opcodes**: Add to instruction decoder
2. **Implement Logic**: Add execution logic in CPU core
3. **Update Assembler**: Add assembly support if needed
4. **Test Coverage**: Add instruction-level tests

### Performance Optimization

For critical performance paths:

1. **Profile First**: Use built-in profiler to identify bottlenecks
2. **Micro-benchmarks**: Create focused performance tests
3. **Algorithm Analysis**: Consider algorithmic improvements
4. **Hardware Utilization**: Optimize for target hardware

### Multi-core Synchronization

When adding components that interact with multiple CPU cores:

1. **Cache Coherency**: Implement proper cache invalidation
2. **Memory Barriers**: Use appropriate synchronization primitives
3. **Lock-free Design**: Consider lock-free algorithms for high-performance paths
4. **Deadlock Prevention**: Use consistent lock ordering

---

This developer guide provides the foundation for contributing to the M5Stack Tab5 Emulator. For specific API details, see the [API Reference](api/README.md), and for usage examples, see the [User Guide](user-guide.md).