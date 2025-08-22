# M5Stack Tab5 Emulator - Claude Assistant Integration

This file contains project-specific information and commands for Claude Code assistant to effectively work with the M5Stack Tab5 Emulator project.

## Project Overview

The M5Stack Tab5 Emulator is a comprehensive hardware emulator for the M5Stack Tab5 device, featuring authentic ESP32-P4 simulation with full peripheral support. It's designed for Linux operation with modern C++20 architecture.

**Key Technologies**: C++20, CMake, SDL2, RISC-V emulation, Real-time graphics, Multi-threading

## Quick Commands Reference

### Build Commands
```bash
# Development build with all features
mkdir build-dev && cd build-dev
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTING=ON -DENABLE_PROFILING=ON -DBUILD_EXAMPLES=ON ..
make -j$(nproc)

# Release build optimized
mkdir build-release && cd build-release  
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native" ..
make -j$(nproc)

# Quick rebuild
make -j$(nproc)

# Clean rebuild
make clean && make -j$(nproc)
```

### Testing Commands
```bash
# Run all tests
make test

# Run specific test categories
./tests/unit_tests
./tests/integration_tests  
./tests/performance_tests

# Run tests with specific filters
./tests/unit_tests --gtest_filter="GPIO*"
./tests/performance_tests --benchmark_filter=".*Performance.*"

# Memory testing with Valgrind
valgrind --tool=memcheck --leak-check=full ./tests/unit_tests
```

### Development Commands
```bash
# Format all code
find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Static analysis
clang-tidy src/**/*.cpp -- -Iinclude -std=c++20

# Code quality check
cppcheck --enable=all --std=c++20 -Iinclude src/

# Run emulator with debugging
./m5tab5-emulator --debug --log-level debug --config config/development.json

# Run with profiling
./m5tab5-emulator --profile --config config/performance.json
```

### Git Commands
```bash
# Standard development workflow
git checkout develop
git pull origin develop
git checkout -b feature/new-feature
# ... make changes ...
git add . && git commit -m "feat(component): description"
git push origin feature/new-feature

# Create release
git checkout -b release/v1.1.0
# ... update version ...
git checkout main && git merge release/v1.1.0
git tag v1.1.0 && git push origin main && git push origin v1.1.0
```

## Project Structure

```
m5tab5-emulator/
├── include/emulator/          # Public headers
│   ├── core/                  # Core emulation engine
│   ├── cpu/                   # RISC-V CPU implementation  
│   ├── memory/                # Memory subsystem
│   ├── graphics/              # Display and touch
│   ├── peripherals/           # GPIO, I2C, SPI, UART, etc.
│   ├── connectivity/          # Wi-Fi, Bluetooth, USB, RS-485
│   ├── audio/                 # Audio processing
│   ├── sensors/               # IMU, camera simulation
│   ├── debug/                 # Profiling and debugging
│   ├── config/                # Configuration system
│   └── utils/                 # Utilities and helpers
├── src/                       # Implementation files
├── tests/                     # Test suite (unit/integration/performance)
├── examples/                  # Usage examples  
├── docs/                      # Documentation
├── config/                    # Configuration files
└── scripts/                   # Build and utility scripts
```

## Key Components and APIs

### EmulatorCore (Main orchestrator)
```cpp
#include "emulator/core/emulator_core.hpp"

auto config = config::Configuration::load("config/default.json");
auto emulator = EmulatorCore::create(config.value());
emulator.value()->initialize();
emulator.value()->start();
```

### GPIO Controller
```cpp
#include "emulator/peripherals/gpio_controller.hpp"

auto gpio = emulator->get_component<peripherals::GPIOController>();
gpio.value()->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
gpio.value()->digital_write(2, true);
```

### I2C Controller  
```cpp
#include "emulator/peripherals/i2c_controller.hpp"

auto i2c = emulator->get_component<peripherals::I2CController>();
i2c.value()->configure(400000, peripherals::I2CController::Mode::MASTER);
auto result = i2c.value()->read(0x68, 6);  // Read from BMI270
```

### Graphics Engine
```cpp
#include "emulator/graphics/graphics_engine.hpp"

auto graphics = emulator->get_component<graphics::GraphicsEngine>();
graphics.value()->clear_screen(0x000000);
graphics.value()->draw_rectangle(100, 100, 200, 150, 0xFF0000);
graphics.value()->present();
```

## Common Development Patterns

### Error Handling Pattern
```cpp
auto result = component->operation();
if (!result.has_value()) {
    LOG_ERROR("Operation failed: {}", static_cast<int>(result.error()));
    return result.error();
}
// Use result.value() safely
```

### Component Creation Pattern
```cpp
class MyComponent {
public:
    static expected<std::unique_ptr<MyComponent>, utils::ErrorCode> create();
    expected<void, utils::ErrorCode> initialize();
    void shutdown();
private:
    MyComponent() = default;  // Private constructor
};
```

### Thread-Safe Component Pattern
```cpp
class ThreadSafeComponent {
private:
    mutable std::mutex state_mutex_;
    std::atomic<bool> is_running_{false};
    
public:
    void operation() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // Thread-safe operation
    }
};
```

### Profiling Pattern
```cpp
#include "emulator/debug/profiler.hpp"

{
    PROFILE_SCOPE(profiler, "expensive_operation");
    // Your code here
}
```

## Configuration Files

### Main Configuration (config/default.json)
- Display settings (1280x720, SDL2 options)
- CPU configuration (dual-core RISC-V, 400MHz)
- Memory settings (8MB PSRAM, caching)  
- Peripheral configurations (GPIO, I2C, SPI, UART)
- Connectivity options (Wi-Fi, Bluetooth, USB)
- Audio settings (ES8388 codec, microphone array)
- Sensor configurations (BMI270 IMU, SC2356 camera)

### Development Configuration (config/development.json)  
- Debug logging enabled
- Profiling enabled
- Extended timeouts
- Development-friendly settings

### Performance Configuration (config/performance.json)
- Optimized for benchmarking
- Minimal logging
- High-performance settings

## Testing Strategy

### Unit Tests (`tests/unit/`)
- Component-level testing
- Mock dependencies
- Comprehensive API coverage
- Performance micro-benchmarks

### Integration Tests (`tests/integration/`)  
- Multi-component coordination
- Real-world scenarios
- Cross-peripheral workflows
- Connectivity stack testing

### Performance Tests (`tests/performance/`)
- Throughput benchmarks
- Latency measurements  
- Resource usage analysis
- Regression detection

## Debugging Information

### Common Issues
1. **SDL2 not found**: Install libsdl2-dev packages
2. **C++20 errors**: Use GCC 10+ or Clang 12+
3. **Graphics not working**: Check X11/Wayland setup
4. **High CPU usage**: Use performance configuration

### Debug Builds
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_SANITIZERS=ON ..
```

### Memory Debugging
```bash
valgrind --tool=memcheck --leak-check=full ./m5tab5-emulator
```

### Performance Analysis
```bash
perf record ./m5tab5-emulator --config config/performance.json
perf report
```

## Documentation Structure

- **README.md**: Main project overview and quick start
- **docs/USAGE.md**: Comprehensive usage documentation  
- **docs/DEVELOPMENT.md**: Development environment and workflows
- **docs/user-guide.md**: User-focused tutorial and guides
- **docs/developer-guide.md**: Architecture and contribution guidelines
- **docs/api/README.md**: Complete API reference
- **examples/README.md**: Usage examples and tutorials

## Build Targets Reference

```bash
# Core targets
make m5tab5-emulator      # Main executable
make examples             # Build all examples
make test                 # Run test suite
make docs                 # Generate documentation

# Quality targets  
make format               # Format source code
make lint                 # Static analysis
make coverage             # Code coverage report

# Packaging targets
make package              # Create distribution
make install              # Install system-wide
```

## Important Files for Code Generation

When creating new components or modifying existing ones, these are key template/reference files:

- `include/emulator/peripherals/gpio_controller.hpp` - Component interface pattern
- `src/peripherals/gpio_controller.cpp` - Component implementation pattern  
- `tests/unit/test_gpio_controller.cpp` - Unit test pattern
- `tests/integration/test_peripheral_coordination.cpp` - Integration test pattern
- `examples/gpio_control.cpp` - Usage example pattern

## Dependencies

**Required Libraries**:
- SDL2 (graphics and input)
- spdlog (logging)
- fmt (string formatting)  
- Google Test (testing framework)

**Build Tools**:
- CMake 3.20+
- GCC 10+ or Clang 12+
- Make or Ninja

**Development Tools** (optional):
- clang-format (code formatting)
- clang-tidy (static analysis)
- cppcheck (additional static analysis)
- valgrind (memory debugging)
- perf (performance profiling)

## Performance Characteristics

**Target Performance**:
- GPIO operations: >1M ops/sec
- I2C transactions: >10K transactions/sec  
- Graphics rendering: 60 FPS at 1280x720
- Memory usage: <100MB typical
- CPU usage: <50% on modern hardware

**Optimization Areas**:
- Memory allocation patterns
- Cache-friendly data structures
- Lock-free algorithms for hot paths
- SIMD usage for audio/graphics processing

This file serves as a comprehensive reference for Claude Code assistant when working on the M5Stack Tab5 Emulator project.