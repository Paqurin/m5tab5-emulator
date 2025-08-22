# M5Stack Tab5 Emulator - Development Documentation

## Table of Contents

1. [Development Environment Setup](#development-environment-setup)
2. [Project Structure](#project-structure)
3. [Build System](#build-system)
4. [Development Workflow](#development-workflow)
5. [Testing Framework](#testing-framework)
6. [Performance Analysis](#performance-analysis)
7. [Debugging Techniques](#debugging-techniques)
8. [Code Quality](#code-quality)
9. [Contributing Guidelines](#contributing-guidelines)
10. [Release Management](#release-management)

## Development Environment Setup

### Prerequisites

- **Operating System**: Linux (Ubuntu 20.04+, Fedora 34+, Arch Linux)
- **Compiler**: GCC 10+ or Clang 12+ with full C++20 support
- **Build Tools**: CMake 3.20+, Ninja (optional), Make
- **Version Control**: Git 2.30+

### Required Development Libraries

#### Ubuntu/Debian
```bash
# Core build tools
sudo apt update
sudo apt install build-essential cmake ninja-build pkg-config git

# SDL2 development libraries
sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev libsdl2-mixer-dev

# Logging and utilities
sudo apt install libspdlog-dev libfmt-dev libjsoncpp-dev

# Testing framework
sudo apt install libgtest-dev libgmock-dev

# Development tools
sudo apt install clang-format clang-tidy cppcheck valgrind
sudo apt install gdb lldb strace perf doxygen graphviz

# Optional: Advanced debugging
sudo apt install rr-debugger heaptrack massif-visualizer
```

#### Fedora/RHEL/CentOS
```bash
# Core build tools
sudo dnf install gcc-c++ cmake ninja-build pkgconfig git

# SDL2 development
sudo dnf install SDL2-devel SDL2_image-devel SDL2_ttf-devel SDL2_mixer-devel

# Libraries
sudo dnf install spdlog-devel fmt-devel jsoncpp-devel

# Testing
sudo dnf install gtest-devel gmock-devel

# Development tools
sudo dnf install clang-tools-extra cppcheck valgrind
sudo dnf install gdb lldb strace perf doxygen graphviz
```

### IDE Configuration

#### Visual Studio Code
Recommended extensions and settings:

```json
{
  "extensions": [
    "ms-vscode.cpptools-extension-pack",
    "ms-vscode.cmake-tools",
    "vadimcn.vscode-lldb",
    "xaver.clang-format",
    "notskm.clang-tidy"
  ],
  "settings": {
    "C_Cpp.default.cppStandard": "c++20",
    "C_Cpp.default.compilerPath": "/usr/bin/g++-10",
    "cmake.configureOnOpen": true,
    "files.associations": {
      "*.hpp": "cpp",
      "*.tpp": "cpp"
    },
    "clang-format.executable": "/usr/bin/clang-format-12"
  }
}
```

#### CLion
Project configuration:
- **Language Standard**: C++20
- **Compiler**: GCC 10+ or Clang 12+
- **CMake Options**: `-DENABLE_TESTING=ON -DENABLE_PROFILING=ON`
- **Code Style**: Use provided `.clang-format`

### Development Build Configuration

Create a development build script:

```bash
#!/bin/bash
# scripts/dev-build.sh

set -e

BUILD_DIR="build-dev"
CMAKE_BUILD_TYPE="Debug"

# Clean previous build
rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

# Configure with all development features
cmake \
  -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DENABLE_TESTING=ON \
  -DENABLE_PROFILING=ON \
  -DBUILD_EXAMPLES=ON \
  -DENABLE_SANITIZERS=ON \
  -DENABLE_STATIC_ANALYSIS=ON \
  -DENABLE_COVERAGE=ON \
  -GNinja \
  ..

# Build
ninja -j$(nproc)

echo "Development build complete!"
echo "Run tests: ninja test"
echo "Run examples: ./examples/basic_emulator"
```

## Project Structure

### Source Code Organization

```
m5tab5-emulator/
├── include/emulator/           # Public header files
│   ├── core/                   # Core emulation engine
│   │   ├── emulator_core.hpp
│   │   ├── component_manager.hpp
│   │   └── lifecycle_manager.hpp
│   ├── cpu/                    # RISC-V CPU implementation
│   │   ├── cpu_core.hpp
│   │   ├── instruction_set.hpp
│   │   └── registers.hpp
│   ├── memory/                 # Memory subsystem
│   │   ├── memory_controller.hpp
│   │   ├── cache_controller.hpp
│   │   └── dma_controller.hpp
│   ├── peripherals/            # Hardware peripherals
│   │   ├── gpio_controller.hpp
│   │   ├── i2c_controller.hpp
│   │   ├── spi_controller.hpp
│   │   └── uart_controller.hpp
│   ├── graphics/               # Graphics and display
│   │   ├── graphics_engine.hpp
│   │   ├── display_controller.hpp
│   │   └── touch_controller.hpp
│   ├── connectivity/           # Network and communication
│   │   ├── wifi_controller.hpp
│   │   ├── bluetooth_controller.hpp
│   │   ├── usb_controller.hpp
│   │   └── rs485_controller.hpp
│   ├── audio/                  # Audio processing
│   │   ├── audio_pipeline.hpp
│   │   ├── codec_es8388.hpp
│   │   └── microphone_array.hpp
│   ├── sensors/                # Sensor simulation
│   │   ├── imu_bmi270.hpp
│   │   ├── camera_sc2356.hpp
│   │   └── sensor_fusion.hpp
│   ├── debug/                  # Debugging and profiling
│   │   ├── profiler.hpp
│   │   ├── resource_monitor.hpp
│   │   ├── regression_detector.hpp
│   │   └── optimization_analyzer.hpp
│   ├── config/                 # Configuration system
│   │   └── configuration.hpp
│   └── utils/                  # Utilities and helpers
│       ├── types.hpp
│       ├── error.hpp
│       ├── logger.hpp
│       └── threading.hpp
├── src/                        # Implementation files
│   ├── core/                   # Mirror of include structure
│   ├── cpu/
│   ├── memory/
│   ├── peripherals/
│   ├── graphics/
│   ├── connectivity/
│   ├── audio/
│   ├── sensors/
│   ├── debug/
│   ├── config/
│   └── utils/
├── tests/                      # Test suite
│   ├── unit/                   # Unit tests
│   ├── integration/            # Integration tests
│   ├── performance/            # Performance benchmarks
│   ├── regression/             # Regression tests
│   └── fixtures/               # Test utilities
├── examples/                   # Usage examples
├── docs/                       # Documentation
├── config/                     # Configuration files
├── scripts/                    # Build and utility scripts
└── tools/                      # Development tools
```

### Header File Structure

Standard header template:

```cpp
#pragma once

// System includes (alphabetical)
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// Project includes (alphabetical)
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"

namespace emulator::component_name {

/**
 * @brief Brief description of the class
 * 
 * Detailed description explaining the purpose, usage,
 * and any important implementation details.
 * 
 * @example
 * auto component = ComponentName::create();
 * component.value()->initialize();
 * 
 * @thread_safety Thread-safe/Not thread-safe
 * @since Version 1.0.0
 */
class ComponentName {
public:
    // Public types and constants
    enum class State { IDLE, RUNNING, ERROR };
    static constexpr size_t MAX_BUFFER_SIZE = 4096;
    
    // Factory methods
    static expected<std::unique_ptr<ComponentName>, utils::ErrorCode> create();
    
    // Core interface
    expected<void, utils::ErrorCode> initialize();
    expected<void, utils::ErrorCode> start();
    expected<void, utils::ErrorCode> stop();
    void shutdown();
    
    // Component-specific methods
    // ...
    
    // Non-copyable, moveable
    ComponentName(const ComponentName&) = delete;
    ComponentName& operator=(const ComponentName&) = delete;
    ComponentName(ComponentName&&) = default;
    ComponentName& operator=(ComponentName&&) = default;
    
private:
    explicit ComponentName();
    
    // Private implementation
    // ...
};

} // namespace emulator::component_name
```

## Build System

### CMake Configuration

Main CMake structure (`CMakeLists.txt`):

```cmake
cmake_minimum_required(VERSION 3.20)
project(m5tab5-emulator
    VERSION 1.0.0
    DESCRIPTION "M5Stack Tab5 Hardware Emulator"
    LANGUAGES CXX
)

# Build options
option(ENABLE_TESTING "Enable unit testing" OFF)
option(ENABLE_PROFILING "Enable performance profiling" OFF)
option(BUILD_EXAMPLES "Build example applications" OFF)
option(ENABLE_SANITIZERS "Enable runtime sanitizers" OFF)
option(ENABLE_STATIC_ANALYSIS "Enable static analysis" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# Warning flags
add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(src)

if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
```

### Build Targets

```bash
# Core targets
make m5tab5-emulator          # Main executable
make emulator_core            # Core library
make emulator_peripherals     # Peripherals library

# Development targets
make test                     # Run all tests
make examples                 # Build all examples
make docs                     # Generate documentation

# Quality targets
make format                   # Format source code
make lint                     # Run static analysis
make coverage                 # Generate coverage report

# Packaging targets
make package                  # Create distribution package
make install                  # Install system-wide
```

### Custom Build Scripts

Create utility scripts in `scripts/`:

```bash
#!/bin/bash
# scripts/quick-test.sh - Run specific test categories
set -e

BUILD_DIR=${1:-build-dev}

echo "Running unit tests..."
$BUILD_DIR/tests/unit_tests

echo "Running integration tests..."
$BUILD_DIR/tests/integration_tests

echo "Running performance benchmarks..."
$BUILD_DIR/tests/performance_tests --benchmark_min_time=1
```

## Development Workflow

### Git Workflow

#### Branch Strategy
```bash
# Main branches
main                    # Stable release branch
develop                 # Development integration branch

# Feature branches
feature/gpio-interrupts     # New feature development
feature/audio-enhancement   # Another feature

# Support branches
hotfix/critical-bug-fix     # Critical fixes
release/v1.1.0             # Release preparation
```

#### Development Process
```bash
# 1. Create feature branch
git checkout develop
git pull origin develop
git checkout -b feature/my-new-feature

# 2. Development cycle
# ... make changes ...
git add .
git commit -m "feat(gpio): implement interrupt handling

- Add interrupt registration system
- Support multiple trigger types
- Include comprehensive unit tests
- Update documentation

Fixes #123"

# 3. Push and create PR
git push origin feature/my-new-feature
# Create pull request on GitHub

# 4. After review and merge
git checkout develop
git pull origin develop
git branch -d feature/my-new-feature
```

### Code Review Process

#### Pre-commit Checklist
- [ ] Code follows style guidelines (run `make format`)
- [ ] All tests pass (`make test`)
- [ ] Static analysis clean (`make lint`)
- [ ] Documentation updated
- [ ] No compiler warnings
- [ ] Performance impact assessed

#### Pull Request Template
```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## Testing
- [ ] Unit tests added/updated
- [ ] Integration tests added/updated
- [ ] Manual testing performed

## Performance Impact
- [ ] No performance impact
- [ ] Performance improvement (include benchmark results)
- [ ] Performance regression acceptable (justify)

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] Tests added/updated and passing
```

## Testing Framework

### Test Categories

#### Unit Tests
```cpp
// tests/unit/test_gpio_controller.cpp
#include <gtest/gtest.h>
#include "emulator/peripherals/gpio_controller.hpp"
#include "tests/fixtures/mock_emulator.hpp"

class GPIOControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        emulator_ = std::make_unique<MockEmulator>();
        auto result = peripherals::GPIOController::create();
        ASSERT_TRUE(result.has_value());
        gpio_ = std::move(result.value());
        ASSERT_TRUE(gpio_->initialize().has_value());
    }
    
    void TearDown() override {
        gpio_->shutdown();
    }
    
    std::unique_ptr<MockEmulator> emulator_;
    std::unique_ptr<peripherals::GPIOController> gpio_;
};

TEST_F(GPIOControllerTest, ConfigurePinAsOutput) {
    auto result = gpio_->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
    EXPECT_TRUE(result.has_value());
    
    auto mode = gpio_->get_pin_mode(2);
    EXPECT_TRUE(mode.has_value());
    EXPECT_EQ(mode.value(), peripherals::GPIOController::Mode::OUTPUT);
}

TEST_F(GPIOControllerTest, DigitalWriteRead) {
    ASSERT_TRUE(gpio_->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT).has_value());
    ASSERT_TRUE(gpio_->digital_write(2, true).has_value());
    
    // Configure another pin to read the state
    ASSERT_TRUE(gpio_->configure_pin(3, peripherals::GPIOController::Mode::INPUT).has_value());
    gpio_->simulate_pin_connection(2, 3);  // Simulate physical connection
    
    auto state = gpio_->digital_read(3);
    ASSERT_TRUE(state.has_value());
    EXPECT_TRUE(state.value());
}
```

#### Integration Tests
```cpp
// tests/integration/test_peripheral_coordination.cpp
#include <gtest/gtest.h>
#include "emulator/core/emulator_core.hpp"
#include "tests/fixtures/integration_fixture.hpp"

class PeripheralCoordinationTest : public IntegrationFixture {
protected:
    void SetUp() override {
        IntegrationFixture::SetUp();
        
        gpio_ = emulator_->get_component<peripherals::GPIOController>().value();
        i2c_ = emulator_->get_component<peripherals::I2CController>().value();
    }
    
    std::shared_ptr<peripherals::GPIOController> gpio_;
    std::shared_ptr<peripherals::I2CController> i2c_;
};

TEST_F(PeripheralCoordinationTest, GPIOTriggeredI2CTransaction) {
    // Setup: GPIO pin triggers I2C sensor read
    bool i2c_transaction_completed = false;
    
    auto gpio_callback = [&](uint8_t pin, bool state) {
        if (pin == 5 && !state) {  // Button press
            auto result = i2c_->read(0x68, 6);  // Read accelerometer
            i2c_transaction_completed = result.has_value();
        }
    };
    
    // Configure GPIO interrupt
    gpio_->configure_pin(5, peripherals::GPIOController::Mode::INPUT);
    gpio_->attach_interrupt(5, peripherals::GPIOController::InterruptTrigger::FALLING_EDGE, gpio_callback);
    
    // Configure I2C with virtual sensor
    i2c_->configure(400000, peripherals::I2CController::Mode::MASTER);
    i2c_->add_virtual_device(0x68);  // BMI270 IMU
    
    // Trigger the workflow
    gpio_->simulate_pin_change(5, false);
    
    // Wait for processing
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(i2c_transaction_completed);
}
```

#### Performance Tests
```cpp
// tests/performance/test_gpio_performance.cpp
#include <benchmark/benchmark.h>
#include "emulator/peripherals/gpio_controller.hpp"

class GPIOPerformanceFixture : public benchmark::Fixture {
public:
    void SetUp(const ::benchmark::State& state) override {
        auto result = peripherals::GPIOController::create();
        gpio_ = std::move(result.value());
        gpio_->initialize();
        gpio_->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
    }
    
    void TearDown(const ::benchmark::State& state) override {
        gpio_->shutdown();
    }
    
protected:
    std::unique_ptr<peripherals::GPIOController> gpio_;
};

BENCHMARK_F(GPIOPerformanceFixture, DigitalWritePerformance)(benchmark::State& state) {
    bool pin_state = false;
    for (auto _ : state) {
        gpio_->digital_write(2, pin_state);
        pin_state = !pin_state;
    }
    
    state.SetItemsProcessed(state.iterations());
    state.counters["ops_per_sec"] = benchmark::Counter(
        state.iterations(), benchmark::Counter::kIsRate);
}

BENCHMARK_REGISTER_F(GPIOPerformanceFixture, DigitalWritePerformance)
    ->Unit(benchmark::kNanosecond)
    ->MinTime(1.0);
```

### Running Tests

```bash
# Run all tests
make test

# Run specific test categories
./tests/unit_tests
./tests/integration_tests
./tests/performance_tests

# Run with specific filters
./tests/unit_tests --gtest_filter="GPIO*"
./tests/performance_tests --benchmark_filter="GPIO.*"

# Run with coverage
make coverage
```

## Performance Analysis

### Built-in Profiling

```cpp
#include "emulator/debug/profiler.hpp"

void performance_critical_function() {
    auto profiler = debug::Profiler::create();
    profiler.value()->initialize();
    profiler.value()->start_profiling("performance_session");
    
    {
        PROFILE_SCOPE(*profiler.value(), "critical_operation");
        // Your performance-critical code here
    }
    
    auto metrics = profiler.value()->get_performance_metrics();
    // Analyze results
}
```

### External Profiling Tools

#### perf (Linux)
```bash
# Profile CPU usage
perf record ./m5tab5-emulator --config config/performance.json
perf report

# Profile specific functions
perf record -g --call-graph=dwarf ./m5tab5-emulator
perf report --stdio

# Memory bandwidth analysis
perf stat -e cache-misses,cache-references ./m5tab5-emulator
```

#### Valgrind
```bash
# Memory profiling
valgrind --tool=massif ./m5tab5-emulator
ms_print massif.out.XXX

# Cache profiling  
valgrind --tool=cachegrind ./m5tab5-emulator
cg_annotate cachegrind.out.XXX
```

#### Intel VTune (if available)
```bash
# Hotspot analysis
vtune -collect hotspots -result-dir vtune_results ./m5tab5-emulator

# Memory access analysis
vtune -collect memory-access -result-dir vtune_memory ./m5tab5-emulator
```

### Performance Benchmarking

```bash
# Automated benchmark suite
scripts/run-benchmarks.sh

# Continuous performance monitoring
scripts/performance-regression-test.sh baseline_results current_results

# Memory usage analysis
scripts/analyze-memory-usage.sh
```

## Debugging Techniques

### GDB Integration

Create `.gdbinit` for the project:

```gdb
# .gdbinit
set print pretty on
set print object on
set print static-members on
set print vtbl on
set print demangle on
set demangle-style gnu-v3

# Custom commands for emulator debugging
define print-emulator-state
    print *((EmulatorCore*)$arg0)
end

define print-gpio-state  
    print *((peripherals::GPIOController*)$arg0)
end

# Set breakpoints for common debugging scenarios
break EmulatorCore::initialize
break peripherals::GPIOController::digital_write
```

Usage:
```bash
gdb ./m5tab5-emulator
(gdb) set args --config config/debug.json
(gdb) run
(gdb) print-emulator-state emulator
```

### Advanced Debugging

#### Record and Replay (rr)
```bash
# Record execution
rr record ./m5tab5-emulator --config config/debug.json

# Replay for debugging
rr replay
(gdb) break main
(gdb) continue
(gdb) reverse-continue  # Go backwards in time!
```

#### Core Dump Analysis
```bash
# Enable core dumps
ulimit -c unlimited

# Analyze crash
gdb ./m5tab5-emulator core.XXXX
(gdb) bt
(gdb) info registers
(gdb) x/20i $pc-40
```

### Logging and Diagnostics

```cpp
#include "emulator/utils/logger.hpp"

// Different log levels
LOG_TRACE("Detailed trace information: {}", detailed_info);
LOG_DEBUG("Debug information: pin={}, state={}", pin, state);
LOG_INFO("Informational message: emulator initialized");
LOG_WARN("Warning: high CPU usage detected: {}%", cpu_usage);
LOG_ERROR("Error: failed to initialize component: {}", error_code);

// Conditional logging
LOG_DEBUG_IF(enable_debug, "Debug info: {}", debug_data);

// Performance logging
{
    auto timer = utils::ScopedTimer("expensive_operation");
    // expensive operation
} // Automatically logs execution time
```

## Code Quality

### Static Analysis

#### Clang-Tidy Configuration
Create `.clang-tidy`:

```yaml
---
Checks: >
  -*,
  bugprone-*,
  cert-*,
  clang-analyzer-*,
  cppcoreguidelines-*,
  google-*,
  hicpp-*,
  misc-*,
  modernize-*,
  performance-*,
  portability-*,
  readability-*,
  -google-readability-todo,
  -misc-non-private-member-variables-in-classes,
  -readability-magic-numbers

HeaderFilterRegex: 'include/emulator/.*'
WarningsAsErrors: true
FormatStyle: file
```

#### Code Formatting
`.clang-format`:

```yaml
---
Language: Cpp
BasedOnStyle: Google
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100
AccessModifierOffset: -2
IndentAccessModifiers: false
AlignAfterOpenBracket: Align
AlignConsecutiveDeclarations: false
AlignConsecutiveAssignments: false
AlignTrailingComments: true
AllowAllParametersOfDeclarationOnNextLine: false
AllowShortBlocksOnASingleLine: false
AllowShortFunctionsOnASingleLine: InlineOnly
AlwaysBreakAfterReturnType: None
AlwaysBreakBeforeMultilineStrings: false
BreakBeforeBinaryOperators: None
BreakBeforeBraces: Attach
BreakConstructorInitializers: BeforeColon
ConstructorInitializerAllOnOneLineOrOnePerLine: true
Cpp11BracedListStyle: true
SpaceAfterCStyleCast: false
SpaceBeforeAssignmentOperators: true
SpaceBeforeParens: ControlStatements
SpaceInEmptyParentheses: false
SpacesInAngles: false
SpacesInCStyleCastParentheses: false
SpacesInParentheses: false
SpacesInSquareBrackets: false
Standard: Cpp11
```

### Quality Automation

#### Pre-commit Hook
Create `.git/hooks/pre-commit`:

```bash
#!/bin/bash
set -e

echo "Running pre-commit quality checks..."

# Check formatting
echo "Checking code formatting..."
if ! scripts/check-format.sh; then
    echo "Code formatting issues found. Run 'make format' to fix."
    exit 1
fi

# Run static analysis
echo "Running static analysis..."
if ! scripts/static-analysis.sh; then
    echo "Static analysis issues found."
    exit 1
fi

# Run unit tests
echo "Running unit tests..."
if ! make test-unit; then
    echo "Unit tests failed."
    exit 1
fi

echo "All pre-commit checks passed!"
```

#### Continuous Integration
`.github/workflows/ci.yml`:

```yaml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Install dependencies
      run: |
        sudo apt update
        sudo apt install -y build-essential cmake libsdl2-dev libspdlog-dev
        
    - name: Configure
      run: |
        cmake -DENABLE_TESTING=ON -DENABLE_STATIC_ANALYSIS=ON .
        
    - name: Build
      run: make -j$(nproc)
      
    - name: Run tests
      run: make test
      
    - name: Static analysis
      run: make lint
      
    - name: Upload coverage
      uses: codecov/codecov-action@v3
```

## Contributing Guidelines

### Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally
3. **Set up development environment** following this guide  
4. **Create a feature branch** from `develop`
5. **Make your changes** following code standards
6. **Test thoroughly** with unit and integration tests
7. **Submit a pull request** with detailed description

### Code Standards

#### Naming Conventions
- **Classes**: `PascalCase` (e.g., `GPIOController`)
- **Functions**: `snake_case` (e.g., `digital_write()`)
- **Variables**: `snake_case` (e.g., `pin_count`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `MAX_PIN_COUNT`)
- **Namespaces**: `lowercase` (e.g., `emulator::peripherals`)

#### Error Handling
Always use `std::expected` for operations that can fail:

```cpp
expected<void, utils::ErrorCode> MyClass::risky_operation() {
    if (precondition_failed()) {
        return unexpected(utils::ErrorCode::INVALID_STATE);
    }
    
    // Success case
    return {};
}
```

#### Documentation
Use Doxygen comments for public APIs:

```cpp
/**
 * @brief Configure a GPIO pin mode
 * 
 * @param pin Pin number (0-54)
 * @param mode Desired pin mode
 * @return Success or error code
 * 
 * @throws None
 * @thread_safety Thread-safe
 * @since Version 1.0.0
 */
expected<void, utils::ErrorCode> configure_pin(uint8_t pin, Mode mode);
```

### Review Process

All changes must go through code review:

1. **Automated checks** must pass (CI/CD)
2. **Manual review** by project maintainers
3. **Testing** validation in development environment
4. **Documentation** updates if needed
5. **Performance** impact assessment for critical paths

## Release Management

### Version Strategy

We follow [Semantic Versioning](https://semver.org/):
- **Major** (1.0.0): Breaking changes
- **Minor** (1.1.0): New features, backward compatible
- **Patch** (1.1.1): Bug fixes, backward compatible

### Release Process

```bash
# 1. Create release branch
git checkout develop
git checkout -b release/v1.1.0

# 2. Update version numbers
scripts/update-version.sh 1.1.0

# 3. Run full test suite
make clean
make test-all
make benchmark
make static-analysis

# 4. Update changelog
# Edit CHANGELOG.md

# 5. Create release commit
git add .
git commit -m "chore: prepare release v1.1.0"

# 6. Merge to main
git checkout main
git merge release/v1.1.0

# 7. Tag release
git tag -a v1.1.0 -m "Release version 1.1.0"

# 8. Push to origin
git push origin main
git push origin v1.1.0

# 9. Create GitHub release
gh release create v1.1.0 --generate-notes
```

### Distribution

```bash
# Create distribution package
make package

# Install system-wide
sudo make install

# Create Docker image
docker build -t m5stack/tab5-emulator:v1.1.0 .
```

---

This development documentation provides a comprehensive guide for contributing to the M5Stack Tab5 Emulator project. For usage instructions, see [USAGE.md](USAGE.md), and for API details, see the [API Reference](api/README.md).