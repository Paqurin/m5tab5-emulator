# M5Stack Tab5 Emulator - User Guide

## Table of Contents

1. [Getting Started](#getting-started)
2. [Configuration](#configuration)
3. [Basic Usage](#basic-usage)
4. [Working with Peripherals](#working-with-peripherals)
5. [Graphics and Touch](#graphics-and-touch)
6. [Connectivity](#connectivity)
7. [Audio System](#audio-system)
8. [Debugging and Profiling](#debugging-and-profiling)
9. [Troubleshooting](#troubleshooting)
10. [Performance Tips](#performance-tips)

## Getting Started

### Installation

Before using the M5Stack Tab5 Emulator, ensure you have all required dependencies installed:

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install build-essential cmake pkg-config
sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev
sudo apt install libspdlog-dev libgtest-dev
```

#### Building the Emulator
```bash
# Clone and build
git clone https://github.com/Paqurin/m5tab5-emulator.git
cd m5tab5-emulator
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### First Run

Run the emulator with default settings:
```bash
./m5tab5-emulator
```

You should see a 1280×720 window representing the M5Stack Tab5 display.

## Configuration

### Configuration Files

The emulator uses JSON configuration files located in the `config/` directory:

- `default.json` - Basic configuration for general use
- `development.json` - Extended debugging and profiling options
- `performance.json` - Optimized for performance testing

### Basic Configuration Structure

```json
{
  "display": {
    "width": 1280,
    "height": 720,
    "fullscreen": false,
    "vsync": true
  },
  "cpu": {
    "core_count": 2,
    "clock_frequency": 400000000,
    "enable_interrupts": true
  },
  "memory": {
    "psram_size": 8388608,
    "enable_cache": true,
    "cache_size": 32768
  },
  "peripherals": {
    "gpio_pins": 55,
    "i2c_frequency": 400000,
    "spi_frequency": 8000000
  }
}
```

### Command Line Options

```bash
./m5tab5-emulator [OPTIONS]

Options:
  --config <file>      Load specific configuration file
  --debug              Enable debug logging
  --profile            Enable performance profiling
  --log-level <level>  Set log level (trace, debug, info, warn, error)
  --fullscreen         Start in fullscreen mode
  --help               Show help message
```

## Basic Usage

### Creating an Emulator Instance

```cpp
#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"

int main() {
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
    
    // Initialize and start
    if (!emulator.value()->initialize().has_value()) {
        std::cerr << "Failed to initialize emulator" << std::endl;
        return -1;
    }
    
    if (!emulator.value()->start().has_value()) {
        std::cerr << "Failed to start emulator" << std::endl;
        return -1;
    }
    
    return 0;
}
```

### Emulator Lifecycle

1. **Creation** - Create emulator with configuration
2. **Initialization** - Initialize all components and allocate resources
3. **Start** - Begin emulation loop and component threads
4. **Running** - Emulator processes instructions and handles events
5. **Stop** - Gracefully stop all components
6. **Shutdown** - Clean up resources and exit

## Working with Peripherals

### GPIO Operations

The M5Stack Tab5 has 55 GPIO pins with various functions:

```cpp
#include "emulator/peripherals/gpio_controller.hpp"

void gpio_example(std::shared_ptr<peripherals::GPIOController> gpio) {
    // Configure pin as output
    gpio->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
    
    // Set pin high
    gpio->digital_write(2, true);
    
    // Configure pin as input with pullup
    gpio->configure_pin(0, peripherals::GPIOController::Mode::INPUT);
    gpio->set_pull_mode(0, peripherals::GPIOController::Pull::UP);
    
    // Read pin state
    auto state = gpio->digital_read(0);
    if (state.has_value()) {
        std::cout << "Pin 0 state: " << state.value() << std::endl;
    }
    
    // Setup interrupt
    auto callback = [](uint8_t pin, bool state) {
        std::cout << "Pin " << (int)pin << " changed to " << state << std::endl;
    };
    
    gpio->attach_interrupt(0, peripherals::GPIOController::InterruptTrigger::CHANGE, callback);
}
```

### I2C Communication

The emulator supports multiple I2C interfaces at up to 400kHz:

```cpp
#include "emulator/peripherals/i2c_controller.hpp"

void i2c_example(std::shared_ptr<peripherals::I2CController> i2c) {
    // Configure I2C as master at 400kHz
    i2c->configure(400000, peripherals::I2CController::Mode::MASTER);
    
    // Scan for devices
    auto devices = i2c->scan_bus();
    if (devices.has_value()) {
        std::cout << "Found " << devices.value().size() << " I2C devices" << std::endl;
        for (auto addr : devices.value()) {
            std::cout << "  Device at address 0x" << std::hex << (int)addr << std::endl;
        }
    }
    
    // Write to device
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    auto write_result = i2c->write(0x68, data);
    
    // Read from device
    auto read_result = i2c->read(0x68, 3);
    if (read_result.has_value()) {
        std::cout << "Read " << read_result.value().size() << " bytes" << std::endl;
    }
}
```

### SPI Communication

SPI interface supports up to 8MHz operation:

```cpp
#include "emulator/peripherals/spi_controller.hpp"

void spi_example(std::shared_ptr<peripherals::SPIController> spi) {
    // Configure SPI
    peripherals::SPIController::Config config;
    config.frequency = 8000000;  // 8MHz
    config.mode = peripherals::SPIController::Mode::MODE_0;
    config.bit_order = peripherals::SPIController::BitOrder::MSB_FIRST;
    
    spi->configure(config);
    
    // Transfer data
    std::vector<uint8_t> tx_data = {0xAA, 0xBB, 0xCC};
    auto rx_data = spi->transfer(tx_data);
    
    if (rx_data.has_value()) {
        std::cout << "SPI transfer successful" << std::endl;
    }
}
```

## Graphics and Touch

### Display System

The emulator provides a 1280×720 display with hardware acceleration:

```cpp
#include "emulator/graphics/graphics_engine.hpp"

void graphics_example(std::shared_ptr<graphics::GraphicsEngine> gfx) {
    // Get framebuffer dimensions
    auto fb_info = gfx->get_framebuffer_info();
    if (fb_info.has_value()) {
        std::cout << "Display: " << fb_info.value().width 
                  << "x" << fb_info.value().height << std::endl;
    }
    
    // Clear screen to blue
    gfx->clear_screen(0x0000FF);
    
    // Draw rectangle
    gfx->draw_rectangle(100, 100, 200, 150, 0xFF0000);
    
    // Draw text (if font system is available)
    gfx->draw_text("Hello, M5Stack!", 50, 50, 0xFFFFFF);
    
    // Present the frame
    gfx->present();
}
```

### Touch Input

The GT911 touch controller supports up to 5 simultaneous touch points:

```cpp
#include "emulator/graphics/touch_controller.hpp"

void touch_example(std::shared_ptr<graphics::TouchController> touch) {
    // Set up touch event callback
    auto touch_callback = [](const graphics::TouchController::TouchEvent& event) {
        switch (event.type) {
            case graphics::TouchController::TouchEventType::PRESS:
                std::cout << "Touch press at (" << event.x << ", " << event.y << ")" << std::endl;
                break;
            case graphics::TouchController::TouchEventType::RELEASE:
                std::cout << "Touch release at (" << event.x << ", " << event.y << ")" << std::endl;
                break;
            case graphics::TouchController::TouchEventType::MOVE:
                std::cout << "Touch move to (" << event.x << ", " << event.y << ")" << std::endl;
                break;
        }
    };
    
    touch->set_event_callback(touch_callback);
    
    // Enable multi-touch
    touch->enable_multitouch(true);
    
    // Set touch sensitivity
    touch->set_sensitivity(0.8);
}
```

## Connectivity

### Wi-Fi

The emulator provides 802.11 Wi-Fi simulation with WPA3 security:

```cpp
#include "emulator/connectivity/wifi_controller.hpp"

void wifi_example(std::shared_ptr<connectivity::WiFiController> wifi) {
    // Add simulated networks
    wifi->add_simulated_network("MyNetwork", -45, 
                               connectivity::WiFiController::SecurityType::WPA2);
    wifi->add_simulated_network("OpenNetwork", -60, 
                               connectivity::WiFiController::SecurityType::NONE);
    
    // Scan for networks
    auto networks = wifi->scan_networks();
    if (networks.has_value()) {
        std::cout << "Found " << networks.value().size() << " networks:" << std::endl;
        for (const auto& network : networks.value()) {
            std::cout << "  " << network.ssid << " (RSSI: " << network.rssi << ")" << std::endl;
        }
    }
    
    // Connect to network
    connectivity::WiFiController::StationConfig config;
    config.ssid = "MyNetwork";
    config.password = "mypassword";
    config.security = connectivity::WiFiController::SecurityType::WPA2;
    
    wifi->configure_station(config);
    
    auto result = wifi->connect();
    if (result.has_value()) {
        std::cout << "Connected to Wi-Fi!" << std::endl;
        
        // Get connection info
        auto ip_info = wifi->get_ip_info();
        if (ip_info.has_value()) {
            std::cout << "IP: " << ip_info.value().ip_address << std::endl;
            std::cout << "Gateway: " << ip_info.value().gateway << std::endl;
        }
    }
}
```

### Bluetooth

Bluetooth Classic and BLE support with GATT operations:

```cpp
#include "emulator/connectivity/bluetooth_controller.hpp"

void bluetooth_example(std::shared_ptr<connectivity::BluetoothController> bt) {
    // Enable Bluetooth
    bt->enable();
    
    // Set device name
    bt->set_device_name("M5Stack Tab5 Emulator");
    
    // Start advertising (BLE)
    connectivity::BluetoothController::BLEConfig ble_config;
    ble_config.advertising_interval = 100;  // 100ms
    ble_config.connectable = true;
    
    bt->configure_ble(ble_config);
    bt->start_advertising();
    
    // Scan for devices
    auto scan_result = bt->start_scan(std::chrono::seconds(10));
    if (scan_result.has_value()) {
        auto devices = bt->get_scanned_devices();
        if (devices.has_value()) {
            for (const auto& device : devices.value()) {
                std::cout << "Found device: " << device.name 
                          << " (" << device.address << ")" << std::endl;
            }
        }
    }
}
```

## Audio System

### Audio Playback and Recording

The ES8388 codec provides high-quality audio processing:

```cpp
#include "emulator/audio/audio_pipeline.hpp"

void audio_example(std::shared_ptr<audio::AudioPipeline> audio) {
    // Configure audio
    audio::AudioPipeline::AudioConfig config;
    config.sample_rate = 44100;
    config.channels = 2;
    config.bit_depth = 16;
    config.buffer_size = 1024;
    
    audio->configure(config);
    
    // Set up playback callback
    auto playback_callback = [](float* buffer, size_t frame_count, size_t channels) {
        // Generate a simple sine wave
        static double phase = 0.0;
        const double frequency = 440.0;  // A4 note
        
        for (size_t i = 0; i < frame_count; ++i) {
            float sample = 0.3f * std::sin(phase);
            for (size_t ch = 0; ch < channels; ++ch) {
                buffer[i * channels + ch] = sample;
            }
            phase += 2.0 * M_PI * frequency / 44100.0;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        }
    };
    
    audio->set_playback_callback(playback_callback);
    
    // Start audio
    audio->start_playback();
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    audio->stop_playback();
}
```

## Debugging and Profiling

### Performance Profiling

The emulator includes comprehensive profiling tools:

```cpp
#include "emulator/debug/profiler.hpp"

void profiling_example() {
    auto profiler = debug::Profiler::create();
    if (!profiler.has_value()) return;
    
    profiler.value()->initialize();
    profiler.value()->start_profiling("performance_test");
    
    // Profile a code section
    {
        PROFILE_SCOPE(*profiler.value(), "expensive_operation");
        
        // Your code here
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Profile a function
    PROFILE_FUNCTION(*profiler.value());
    
    // Stop profiling and get results
    profiler.value()->stop_profiling();
    
    auto metrics = profiler.value()->get_performance_metrics();
    if (metrics.has_value()) {
        std::cout << "Operations/sec: " << metrics.value().operations_per_second << std::endl;
        std::cout << "Memory usage: " << metrics.value().memory_usage_bytes << " bytes" << std::endl;
    }
    
    // Generate detailed report
    auto report = profiler.value()->generate_performance_report();
    if (report.has_value()) {
        std::cout << report.value() << std::endl;
    }
}
```

### Resource Monitoring

Monitor system resources in real-time:

```cpp
#include "emulator/debug/resource_monitor.hpp"

void monitoring_example() {
    auto monitor = debug::ResourceMonitor::create();
    if (!monitor.has_value()) return;
    
    monitor.value()->initialize();
    monitor.value()->start_monitoring();
    
    // Set up alert callback
    auto alert_callback = [](const debug::ResourceMonitor::ResourceAlert& alert) {
        std::cout << "Resource alert: " << alert.message << std::endl;
    };
    
    monitor.value()->set_alert_callback(alert_callback);
    
    // Set thresholds
    monitor.value()->set_cpu_threshold(80.0);      // 80% CPU usage
    monitor.value()->set_memory_threshold(1024*1024*1024);  // 1GB memory
    
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // Get current metrics
    auto metrics = monitor.value()->get_current_metrics();
    if (metrics.has_value()) {
        std::cout << "CPU usage: " << metrics.value().cpu_usage_percent << "%" << std::endl;
        std::cout << "Memory usage: " << metrics.value().memory_usage_bytes / 1024 / 1024 << " MB" << std::endl;
    }
    
    monitor.value()->stop_monitoring();
}
```

## Troubleshooting

### Common Issues

#### Emulator Won't Start
- **Check dependencies**: Ensure all SDL2 libraries are installed
- **Verify configuration**: Check JSON syntax in configuration files
- **Check permissions**: Ensure write access to log directories

#### Graphics Issues
- **Black screen**: Check if SDL2 video drivers are properly installed
- **Poor performance**: Try disabling vsync or reducing resolution
- **Touch not working**: Verify SDL2 event handling is enabled

#### Audio Problems
- **No sound**: Check audio device permissions and SDL2 audio drivers
- **Audio glitches**: Increase buffer size in audio configuration
- **High latency**: Reduce buffer size but may cause audio dropouts

#### Connectivity Issues
- **Wi-Fi simulation not working**: Ensure network simulation is enabled in config
- **Bluetooth pairing fails**: Check if virtual devices are properly configured

### Debug Logging

Enable detailed logging for troubleshooting:

```bash
./m5tab5-emulator --log-level debug --debug
```

Log files are written to `logs/` directory with timestamps.

### Performance Issues

If the emulator runs slowly:

1. **Disable debugging**: Remove `--debug` flag
2. **Optimize configuration**: Use `config/performance.json`
3. **Reduce graphics quality**: Lower resolution or disable effects
4. **Check system resources**: Monitor CPU and memory usage

## Performance Tips

### Optimization Guidelines

1. **CPU Usage**
   - Use release builds (`-O3` optimization)
   - Disable unnecessary debug features
   - Limit background thread activity

2. **Memory Management**
   - Configure appropriate PSRAM size
   - Enable memory caching
   - Monitor for memory leaks

3. **Graphics Performance**
   - Use hardware acceleration when available
   - Optimize framebuffer operations
   - Consider lower resolution for better performance

4. **Real-time Performance**
   - Set appropriate thread priorities
   - Use dedicated CPU cores for emulation
   - Minimize system interruptions

### Benchmarking

Run performance benchmarks:

```bash
cd build
./tests/performance_tests
```

This will generate detailed performance reports and identify bottlenecks.

---

For more detailed technical information, see the [Developer Guide](developer-guide.md) and [API Reference](api/README.md).