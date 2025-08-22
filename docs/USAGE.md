# M5Stack Tab5 Emulator - Usage Documentation

## Table of Contents

1. [Quick Start](#quick-start)
2. [Installation Guide](#installation-guide)
3. [Configuration](#configuration)
4. [Basic Operations](#basic-operations)
5. [Component Usage](#component-usage)
6. [Command Line Interface](#command-line-interface)
7. [Development Workflow](#development-workflow)
8. [Troubleshooting](#troubleshooting)

## Quick Start

### Minimum Requirements

- **OS**: Linux (Ubuntu 20.04+ recommended)
- **Compiler**: GCC 9+ or Clang 10+ with C++20 support
- **Build Tools**: CMake 3.16+, Make
- **Libraries**: SDL2, SDL2_image, SDL2_ttf, spdlog

### 5-Minute Setup

```bash
# 1. Clone repository
git clone https://github.com/Paqurin/m5tab5-emulator.git
cd m5tab5-emulator

# 2. Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install build-essential cmake pkg-config
sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev
sudo apt install libspdlog-dev

# 3. Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# 4. Run
./m5tab5-emulator
```

## Installation Guide

### System Dependencies

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install build-essential cmake pkg-config git
sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev
sudo apt install libspdlog-dev libfmt-dev
```

#### Fedora/CentOS
```bash
sudo dnf install gcc-c++ cmake pkgconfig git
sudo dnf install SDL2-devel SDL2_image-devel SDL2_ttf-devel
sudo dnf install spdlog-devel fmt-devel
```

#### Arch Linux
```bash
sudo pacman -S base-devel cmake pkgconfig git
sudo pacman -S sdl2 sdl2_image sdl2_ttf
sudo pacman -S spdlog fmt
```

### Build Configuration

#### Development Build
```bash
cmake -DCMAKE_BUILD_TYPE=Debug \
      -DENABLE_TESTING=ON \
      -DENABLE_PROFILING=ON \
      -DBUILD_EXAMPLES=ON ..
make -j$(nproc)
```

#### Release Build
```bash
cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_TESTING=OFF \
      -DENABLE_PROFILING=OFF ..
make -j$(nproc)
```

#### Performance Build
```bash
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-O3 -march=native -DNDEBUG" \
      -DENABLE_TESTING=OFF ..
make -j$(nproc)
```

### Verification

After building, verify the installation:

```bash
# Check emulator binary
./m5tab5-emulator --help

# Run basic tests
make test

# Run examples (if built)
./examples/basic_emulator
```

## Configuration

### Configuration Files

The emulator uses JSON configuration files located in `config/`:

- **`default.json`**: Standard configuration for general use
- **`development.json`**: Development settings with debugging enabled
- **`performance.json`**: Optimized for performance testing
- **`minimal.json`**: Minimal resource usage configuration

### Configuration Structure

```json
{
  "emulator": {
    "name": "M5Stack Tab5 Emulator",
    "version": "1.0.0"
  },
  "display": {
    "width": 1280,
    "height": 720,
    "fullscreen": false,
    "vsync": true,
    "window_title": "M5Stack Tab5 Emulator"
  },
  "cpu": {
    "architecture": "riscv32",
    "core_count": 2,
    "clock_frequency": 400000000,
    "enable_interrupts": true,
    "instruction_cache_size": 32768,
    "data_cache_size": 32768
  },
  "memory": {
    "psram_size": 8388608,
    "internal_ram_size": 512000,
    "enable_cache": true,
    "cache_size": 32768,
    "enable_dma": true
  },
  "peripherals": {
    "gpio": {
      "pin_count": 55,
      "enable_interrupts": true,
      "default_pull": "none"
    },
    "i2c": {
      "frequency": 400000,
      "enable_pullup": true,
      "timeout_ms": 1000
    },
    "spi": {
      "frequency": 8000000,
      "mode": 0,
      "bit_order": "msb_first"
    },
    "uart": {
      "baud_rate": 115200,
      "data_bits": 8,
      "stop_bits": 1,
      "parity": "none"
    }
  },
  "connectivity": {
    "wifi": {
      "enabled": true,
      "simulation_mode": true,
      "default_networks": [
        {
          "ssid": "TestNetwork",
          "rssi": -50,
          "security": "WPA2"
        }
      ]
    },
    "bluetooth": {
      "enabled": true,
      "classic_enabled": true,
      "ble_enabled": true,
      "device_name": "M5Stack Tab5"
    }
  },
  "audio": {
    "sample_rate": 44100,
    "channels": 2,
    "buffer_size": 1024,
    "enable_microphone": true,
    "microphone_array_size": 2
  },
  "sensors": {
    "imu": {
      "type": "BMI270",
      "sampling_rate": 100,
      "enable_fusion": true
    },
    "camera": {
      "type": "SC2356",
      "resolution": "1920x1080",
      "fps": 30
    }
  },
  "debug": {
    "log_level": "info",
    "enable_profiling": false,
    "profiling_interval_ms": 1000,
    "enable_resource_monitoring": false
  }
}
```

### Custom Configuration

Create a custom configuration file:

```bash
cp config/default.json config/my_config.json
# Edit config/my_config.json as needed
./m5tab5-emulator --config config/my_config.json
```

## Basic Operations

### Starting the Emulator

```bash
# Basic startup
./m5tab5-emulator

# With custom configuration
./m5tab5-emulator --config config/development.json

# With debug logging
./m5tab5-emulator --debug --log-level debug

# With profiling enabled
./m5tab5-emulator --profile

# Fullscreen mode
./m5tab5-emulator --fullscreen
```

### Runtime Controls

While the emulator is running:

- **Mouse**: Simulate touch input on the display
- **Keyboard Shortcuts**:
  - `Ctrl+C`: Graceful shutdown
  - `F11`: Toggle fullscreen
  - `Ctrl+R`: Reset emulator
  - `Ctrl+P`: Toggle pause
  - `Ctrl+D`: Toggle debug overlay
  - `Ctrl+S`: Take screenshot

### Stopping the Emulator

The emulator can be stopped in several ways:

1. **Graceful shutdown**: Press `Ctrl+C`
2. **Close window**: Click the X button on the graphics window
3. **API call**: Use the emulator API to stop programmatically

## Component Usage

### GPIO Operations

```cpp
#include "emulator/peripherals/gpio_controller.hpp"

// Get GPIO controller
auto gpio = emulator->get_component<peripherals::GPIOController>();

// Configure pin as output
gpio.value()->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);

// Write to pin
gpio.value()->digital_write(2, true);

// Configure pin as input with pullup
gpio.value()->configure_pin(0, peripherals::GPIOController::Mode::INPUT);
gpio.value()->set_pull_mode(0, peripherals::GPIOController::Pull::UP);

// Read from pin
auto state = gpio.value()->digital_read(0);
```

### I2C Communication

```cpp
#include "emulator/peripherals/i2c_controller.hpp"

// Get I2C controller
auto i2c = emulator->get_component<peripherals::I2CController>();

// Configure I2C
i2c.value()->configure(400000, peripherals::I2CController::Mode::MASTER);

// Write data
std::vector<uint8_t> data = {0x01, 0x02, 0x03};
auto result = i2c.value()->write(0x68, data);

// Read data  
auto read_result = i2c.value()->read(0x68, 6);
```

### Wi-Fi Operations

```cpp
#include "emulator/connectivity/wifi_controller.hpp"

// Get Wi-Fi controller
auto wifi = emulator->get_component<connectivity::WiFiController>();

// Scan for networks
auto networks = wifi.value()->scan_networks();

// Connect to network
connectivity::WiFiController::StationConfig config;
config.ssid = "MyNetwork";
config.password = "password";
config.security = connectivity::WiFiController::SecurityType::WPA2;

wifi.value()->configure_station(config);
auto connect_result = wifi.value()->connect();
```

### Graphics and Touch

```cpp
#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/graphics/touch_controller.hpp"

// Get graphics engine
auto graphics = emulator->get_component<graphics::GraphicsEngine>();

// Clear screen and draw
graphics.value()->clear_screen(0x000000);  // Black
graphics.value()->draw_rectangle(100, 100, 200, 150, 0xFF0000);  // Red rectangle
graphics.value()->present();

// Get touch controller
auto touch = emulator->get_component<graphics::TouchController>();

// Set touch callback
auto callback = [](const graphics::TouchController::TouchEvent& event) {
    std::cout << "Touch at (" << event.x << ", " << event.y << ")" << std::endl;
};
touch.value()->set_event_callback(callback);
```

## Command Line Interface

### Available Options

```
Usage: m5tab5-emulator [OPTIONS]

Options:
  --config <file>           Load configuration from file
  --debug                   Enable debug logging
  --log-level <level>       Set log level (trace, debug, info, warn, error)
  --profile                 Enable performance profiling
  --fullscreen             Start in fullscreen mode
  --width <pixels>         Override display width
  --height <pixels>        Override display height
  --vsync                  Enable vertical synchronization
  --no-vsync              Disable vertical synchronization
  --headless              Run without graphics (for testing)
  --version               Show version information
  --help                  Show this help message

Examples:
  m5tab5-emulator                                    # Basic run
  m5tab5-emulator --config config/development.json  # Development mode
  m5tab5-emulator --debug --log-level trace         # Maximum debugging
  m5tab5-emulator --profile --fullscreen            # Performance testing
  m5tab5-emulator --headless                        # Testing without UI
```

### Environment Variables

```bash
# Override configuration file location
export M5TAB5_CONFIG="config/my_config.json"

# Set log level
export M5TAB5_LOG_LEVEL="debug"

# Enable profiling
export M5TAB5_PROFILE="1"

# Set log file location
export M5TAB5_LOG_FILE="logs/emulator.log"
```

## Development Workflow

### Setting Up Development Environment

```bash
# Clone repository
git clone https://github.com/Paqurin/m5tab5-emulator.git
cd m5tab5-emulator

# Install development dependencies
sudo apt install clang-format clang-tidy cppcheck valgrind gdb

# Configure development build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug \
      -DENABLE_TESTING=ON \
      -DENABLE_PROFILING=ON \
      -DBUILD_EXAMPLES=ON \
      -DENABLE_SANITIZERS=ON ..

# Build with all development tools
make -j$(nproc)
```

### Running Tests

```bash
# Run all tests
make test

# Run specific test category
./tests/unit_tests
./tests/integration_tests
./tests/performance_tests

# Run tests with Valgrind
valgrind --tool=memcheck --leak-check=full ./tests/unit_tests
```

### Development Commands

```bash
# Format code
find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Run static analysis
clang-tidy src/**/*.cpp -- -Iinclude -std=c++20

# Run code quality checks
cppcheck --enable=all --std=c++20 -Iinclude src/

# Generate documentation
doxygen docs/Doxyfile

# Profile performance
./m5tab5-emulator --profile --config config/performance.json
```

### Debugging

```bash
# Debug with GDB
gdb ./m5tab5-emulator
(gdb) set args --config config/development.json --debug
(gdb) run

# Debug with Address Sanitizer
cmake -DENABLE_SANITIZERS=ON ..
make
./m5tab5-emulator

# Debug memory issues
valgrind --tool=memcheck --leak-check=full \
         --show-leak-kinds=all --track-origins=yes \
         ./m5tab5-emulator
```

## Troubleshooting

### Common Issues

#### Build Issues

**Problem**: CMake fails to find SDL2
```bash
# Solution: Install SDL2 development packages
sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev

# Or manually set SDL2 path
cmake -DSDL2_DIR=/path/to/sdl2 ..
```

**Problem**: Compiler errors about C++20 features
```bash
# Solution: Use a newer compiler
sudo apt install gcc-10 g++-10
cmake -DCMAKE_CXX_COMPILER=g++-10 ..
```

#### Runtime Issues

**Problem**: Emulator window doesn't appear
```bash
# Check if X11 forwarding is enabled (if using SSH)
ssh -X username@hostname

# Check SDL2 video driver
export SDL_VIDEODRIVER=x11  # or wayland

# Run with debug logging
./m5tab5-emulator --debug --log-level debug
```

**Problem**: Audio not working
```bash
# Check audio permissions
groups $USER  # Should include 'audio' group

# Test audio device
aplay /usr/share/sounds/alsa/Front_Left.wav

# Try different audio driver
export SDL_AUDIODRIVER=pulse  # or alsa
```

**Problem**: High CPU usage
```bash
# Use performance configuration
./m5tab5-emulator --config config/performance.json

# Disable profiling
./m5tab5-emulator --no-profile

# Check for background processes
top -p $(pgrep m5tab5-emulator)
```

#### Configuration Issues

**Problem**: Configuration file not found
```bash
# Check file exists and is readable
ls -la config/default.json

# Use absolute path
./m5tab5-emulator --config $(pwd)/config/default.json

# Validate JSON syntax
python3 -m json.tool config/default.json
```

### Diagnostic Commands

```bash
# System information
./m5tab5-emulator --version
lscpu
lsmem
lsusb
lspci

# Dependencies check
ldd ./m5tab5-emulator
pkg-config --modversion sdl2

# Log analysis
tail -f logs/emulator.log
grep ERROR logs/emulator.log
```

### Getting Help

1. **Check Documentation**: Review [User Guide](user-guide.md) and [Developer Guide](developer-guide.md)
2. **Search Issues**: Check [GitHub Issues](https://github.com/Paqurin/m5tab5-emulator/issues)
3. **Enable Debug Logging**: Run with `--debug --log-level debug`
4. **Create Issue**: If problem persists, create a [new issue](https://github.com/Paqurin/m5tab5-emulator/issues/new)

### Performance Optimization

```bash
# Profile the application
./m5tab5-emulator --profile --config config/performance.json

# Monitor resources
htop
iotop
nethogs

# Optimize build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native" ..

# Use dedicated graphics
export __NV_PRIME_RENDER_OFFLOAD=1  # For NVIDIA Optimus
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

---

For more detailed information, see:
- [API Reference](api/README.md)
- [Developer Guide](developer-guide.md)
- [Examples](../examples/README.md)