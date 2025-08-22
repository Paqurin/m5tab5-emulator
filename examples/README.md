# M5Stack Tab5 Emulator - Examples

This directory contains practical examples demonstrating various aspects of the M5Stack Tab5 Emulator. Each example is self-contained and includes detailed comments explaining the concepts and implementation.

## Examples Overview

| Example | Description | Components Used | Difficulty |
|---------|-------------|----------------|------------|
| [Basic Emulator](#basic-emulator) | Simple emulator setup and lifecycle | EmulatorCore, Configuration | Beginner |
| [GPIO Control](#gpio-control) | GPIO operations and interrupts | GPIOController | Beginner |
| [I2C Sensor Communication](#i2c-sensor-communication) | I2C device interaction | I2CController, BMI270 | Beginner |
| [Touch Interface](#touch-interface) | Multi-touch handling | TouchController, GraphicsEngine | Intermediate |
| [Audio Processing](#audio-processing) | Audio playback and recording | AudioPipeline, ES8388 | Intermediate |
| [Wi-Fi Connectivity](#wi-fi-connectivity) | Network operations | WiFiController | Intermediate |
| [Sensor Fusion](#sensor-fusion) | IMU data processing | SensorFusion, BMI270 | Advanced |
| [Performance Profiling](#performance-profiling) | Profiling and optimization | Profiler, ResourceMonitor | Advanced |
| [Complete Application](#complete-application) | Full-featured demo app | All components | Advanced |

## Building and Running Examples

### Prerequisites

Ensure the emulator is built with examples enabled:

```bash
cd build
cmake -DBUILD_EXAMPLES=ON ..
make -j$(nproc)
```

### Running Examples

Each example is built as a separate executable:

```bash
# List available examples
ls examples/

# Run a specific example
./examples/basic_emulator
./examples/gpio_control
./examples/touch_interface
# ... etc
```

## Example Descriptions

### Basic Emulator

**File**: `examples/basic_emulator.cpp`

Demonstrates the fundamental emulator setup, initialization, and lifecycle management. This is the starting point for understanding how to create and manage an emulator instance.

**Key Concepts**:
- Configuration loading
- Emulator creation and initialization
- Basic lifecycle management
- Error handling patterns

**Usage**:
```bash
./examples/basic_emulator --config ../config/development.json
```

### GPIO Control

**File**: `examples/gpio_control.cpp`

Shows how to configure GPIO pins, perform digital I/O operations, and handle interrupts. Includes examples of pin multiplexing and interrupt-driven programming.

**Key Concepts**:
- Pin configuration (input/output modes)
- Digital read/write operations
- Interrupt registration and handling
- Pull-up/pull-down resistor configuration

**Usage**:
```bash
./examples/gpio_control
```

**Example Output**:
```
Configuring GPIO pins...
Pin 2 configured as OUTPUT
Pin 0 configured as INPUT with PULLUP
Starting LED blink sequence...
LED ON (Pin 2: HIGH)
LED OFF (Pin 2: LOW)
Button interrupt registered on Pin 0
Waiting for button press... (Press Ctrl+C to exit)
```

### I2C Sensor Communication

**File**: `examples/i2c_sensor.cpp`

Demonstrates I2C communication with simulated sensors, including device detection, register read/write operations, and data processing.

**Key Concepts**:
- I2C bus configuration
- Device scanning and detection
- Register-level communication
- Sensor data interpretation

**Usage**:
```bash
./examples/i2c_sensor
```

### Touch Interface

**File**: `examples/touch_interface.cpp`

Shows how to handle touch input events, implement gesture recognition, and create interactive UI elements.

**Key Concepts**:
- Touch event handling
- Multi-touch support
- Coordinate transformation
- Simple gesture recognition

**Usage**:
```bash
./examples/touch_interface
```

**Features Demonstrated**:
- Single and multi-touch detection
- Touch pressure sensitivity
- Gesture recognition (tap, swipe, pinch)
- Touch coordinate calibration

### Audio Processing

**File**: `examples/audio_processing.cpp`

Illustrates audio playback, recording, and processing using the ES8388 codec and dual microphone array.

**Key Concepts**:
- Audio configuration and initialization
- Playback callback implementation
- Recording and audio capture
- Basic audio processing (filters, effects)

**Usage**:
```bash
./examples/audio_processing
```

**Audio Features**:
- Sine wave generation
- Audio file playback
- Microphone recording
- Real-time audio effects

### Wi-Fi Connectivity

**File**: `examples/wifi_connectivity.cpp`

Demonstrates Wi-Fi operations including network scanning, connection management, and basic networking.

**Key Concepts**:
- Network scanning and discovery
- Connection establishment
- IP configuration
- Basic HTTP client operations

**Usage**:
```bash
./examples/wifi_connectivity
```

### Sensor Fusion

**File**: `examples/sensor_fusion.cpp`

Advanced example showing IMU sensor fusion, orientation estimation, and motion detection.

**Key Concepts**:
- Multi-sensor data integration
- Kalman filtering
- Quaternion-based orientation
- Motion gesture recognition

**Usage**:
```bash
./examples/sensor_fusion
```

**Sensor Fusion Features**:
- 6-DOF orientation estimation
- Activity classification
- Gesture recognition
- Sensor calibration

### Performance Profiling

**File**: `examples/performance_profiling.cpp`

Shows how to use the built-in profiling tools to measure and optimize performance.

**Key Concepts**:
- Performance measurement
- Resource monitoring
- Bottleneck identification
- Optimization techniques

**Usage**:
```bash
./examples/performance_profiling --profile
```

### Complete Application

**File**: `examples/complete_application.cpp`

A comprehensive example that combines multiple subsystems into a functional application, demonstrating best practices for real-world usage.

**Key Concepts**:
- Multi-component coordination
- State management
- Event-driven architecture
- Error recovery

**Usage**:
```bash
./examples/complete_application --config ../config/complete_app.json
```

## Building Custom Examples

### Example Template

Use this template to create your own examples:

```cpp
#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    std::cout << "=== My Custom Example ===" << std::endl;
    
    try {
        // Load configuration
        auto config = config::Configuration::load("config/default.json");
        if (!config.has_value()) {
            std::cerr << "Failed to load configuration" << std::endl;
            return -1;
        }
        
        // Create emulator
        auto emulator = EmulatorCore::create(config.value());
        if (!emulator.has_value()) {
            std::cerr << "Failed to create emulator" << std::endl;
            return -1;
        }
        
        // Initialize
        if (!emulator.value()->initialize().has_value()) {
            std::cerr << "Failed to initialize emulator" << std::endl;
            return -1;
        }
        
        std::cout << "Emulator initialized successfully" << std::endl;
        
        // Your custom logic here
        // ...
        
        // Cleanup
        emulator.value()->shutdown();
        std::cout << "Example completed successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
```

### Adding to Build System

Add your example to `examples/CMakeLists.txt`:

```cmake
# Add your custom example
add_executable(my_custom_example my_custom_example.cpp)
target_link_libraries(my_custom_example 
    emulator_core 
    emulator_config 
    ${SDL2_LIBRARIES}
)
target_include_directories(my_custom_example PRIVATE ${CMAKE_SOURCE_DIR}/include)
```

## Common Patterns and Best Practices

### Error Handling Pattern

```cpp
auto result = component->some_operation();
if (!result.has_value()) {
    std::cerr << "Operation failed: " << static_cast<int>(result.error()) << std::endl;
    // Handle error appropriately
    return -1;
}
// Use result.value() safely
```

### Resource Management Pattern

```cpp
class ExampleApplication {
private:
    std::unique_ptr<EmulatorCore> emulator_;
    
public:
    bool initialize() {
        auto config = config::Configuration::load("config/default.json");
        if (!config.has_value()) return false;
        
        auto emulator = EmulatorCore::create(config.value());
        if (!emulator.has_value()) return false;
        
        emulator_ = std::move(emulator.value());
        return emulator_->initialize().has_value();
    }
    
    ~ExampleApplication() {
        if (emulator_) {
            emulator_->shutdown();
        }
    }
};
```

### Asynchronous Operation Pattern

```cpp
void setup_async_operations() {
    // Use callbacks for asynchronous events
    auto touch_callback = [this](const TouchEvent& event) {
        handle_touch_event(event);
    };
    
    touch_controller_->set_event_callback(touch_callback);
    
    // Use futures for long-running operations
    auto future = std::async(std::launch::async, [this]() {
        return perform_long_operation();
    });
    
    // Continue with other work...
    
    // Wait for completion when needed
    auto result = future.get();
}
```

## Troubleshooting Examples

### Common Issues

1. **Emulator fails to initialize**:
   - Check configuration file syntax
   - Verify SDL2 libraries are installed
   - Ensure sufficient system resources

2. **No graphics output**:
   - Verify display configuration
   - Check SDL2 video driver setup
   - Try different display modes

3. **Touch input not working**:
   - Ensure SDL2 supports touch input
   - Check touch controller configuration
   - Verify event callback registration

4. **Audio issues**:
   - Check audio device permissions
   - Verify SDL2 audio backend
   - Test with different buffer sizes

### Debug Mode

Run examples with debug output:

```bash
./examples/gpio_control --log-level debug
```

### Profiling Examples

Profile example performance:

```bash
./examples/performance_profiling --profile --log-level info
```

## Contributing Examples

We welcome new examples! When contributing:

1. **Follow the template structure**
2. **Include comprehensive comments**
3. **Test on multiple configurations**
4. **Update this README with your example**
5. **Add appropriate CMake targets**

For detailed contribution guidelines, see [CONTRIBUTING.md](../CONTRIBUTING.md).

---

For more information about the emulator API, see the [API Reference](../docs/api/README.md). For general usage instructions, see the [User Guide](../docs/user-guide.md).