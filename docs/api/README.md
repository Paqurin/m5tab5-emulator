# M5Stack Tab5 Emulator API Reference

Complete API documentation for all emulator components and systems.

## Table of Contents

1. [Core Systems](#core-systems)
2. [CPU Emulation](#cpu-emulation)
3. [Memory Management](#memory-management)
4. [Graphics & Display](#graphics--display)
5. [Peripherals](#peripherals)
6. [Connectivity](#connectivity)
7. [Audio Processing](#audio-processing)
8. [Debugging & Profiling](#debugging--profiling)
9. [Configuration](#configuration)
10. [Error Handling](#error-handling)

---

## Core Systems

### EmulatorCore

Main orchestrator for the entire emulation system.

#### Class Definition
```cpp
class EmulatorCore {
public:
    enum class State {
        UNINITIALIZED,
        INITIALIZED, 
        RUNNING,
        PAUSED,
        STOPPED,
        SHUTDOWN,
        ERROR
    };
    
    struct Status {
        State state;
        std::chrono::high_resolution_clock::time_point start_time;
        std::chrono::milliseconds uptime;
        double cpu_usage_percent;
        size_t memory_usage_bytes;
        uint32_t frame_count;
        double frames_per_second;
    };

    static expected<std::unique_ptr<EmulatorCore>, utils::ErrorCode> create(
        std::shared_ptr<config::Configuration> configuration);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    utils::expected<void, utils::ErrorCode> start();
    utils::expected<void, utils::ErrorCode> stop();
    utils::expected<void, utils::ErrorCode> pause();
    utils::expected<void, utils::ErrorCode> resume();
    
    expected<Status, utils::ErrorCode> get_status() const;
    expected<void, utils::ErrorCode> reset();
};
```

#### Usage Example
```cpp
// Create emulator with configuration
auto config = config::Configuration::load("config.json");
auto emulator = EmulatorCore::create(config.value());

// Initialize and start
emulator.value()->initialize();
emulator.value()->start();

// Check status
auto status = emulator.value()->get_status();
std::cout << "FPS: " << status.value().frames_per_second << std::endl;
```

#### Component Access Methods
```cpp
// Get system components
std::shared_ptr<memory::MemoryController> get_memory_controller();
std::vector<std::shared_ptr<cpu::CPUCore>> get_cpu_cores();
std::shared_ptr<graphics::GraphicsEngine> get_graphics_engine();
std::shared_ptr<peripherals::GPIOController> get_gpio_controller();
std::shared_ptr<peripherals::I2CController> get_i2c_controller(uint8_t index);
std::shared_ptr<connectivity::WiFiController> get_wifi_controller();
std::shared_ptr<connectivity::BluetoothController> get_bluetooth_controller();
```

---

## CPU Emulation

### CPUCore

RISC-V RV32IMAC processor core implementation.

#### Class Definition
```cpp
class CPUCore {
public:
    enum class State {
        IDLE,
        RUNNING, 
        PAUSED,
        STOPPED,
        ERROR
    };

    static expected<std::unique_ptr<CPUCore>, utils::ErrorCode> create(
        uint32_t core_id, std::shared_ptr<memory::MemoryController> memory_controller);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    utils::expected<void, utils::ErrorCode> start();
    utils::expected<void, utils::ErrorCode> stop();
    utils::expected<void, utils::ErrorCode> pause();
    utils::expected<void, utils::ErrorCode> resume();
    
    // Execution
    utils::expected<void, utils::ErrorCode> step();
    utils::expected<void, utils::ErrorCode> run_for_cycles(uint64_t cycles);
    
    // Register access
    expected<uint32_t, utils::ErrorCode> get_register(uint8_t reg_id) const;
    utils::expected<void, utils::ErrorCode> set_register(uint8_t reg_id, uint32_t value);
    
    // Program counter
    uint32_t get_pc() const;
    utils::expected<void, utils::ErrorCode> set_pc(uint32_t address);
    utils::expected<void, utils::ErrorCode> increment_pc();
    
    // Interrupts
    utils::expected<void, utils::ErrorCode> trigger_interrupt(uint32_t interrupt_id);
    expected<uint32_t, utils::ErrorCode> get_pending_interrupts() const;
    
    // Performance metrics
    uint64_t get_cycle_count() const;
    uint64_t get_instruction_count() const;
    State get_state() const;
    uint32_t get_core_id() const;
};
```

#### Register Access Example
```cpp
auto cpu = CPUCore::create(0, memory_controller);

// Set registers
cpu->set_register(1, 0x12345678);  // x1 = 0x12345678
cpu->set_register(2, 0x87654321);  // x2 = 0x87654321

// Read register
auto value = cpu->get_register(1);
std::cout << "x1 = 0x" << std::hex << value.value() << std::endl;

// Execute instructions
cpu->step();  // Execute one instruction
cpu->run_for_cycles(1000);  // Run for 1000 cycles
```

#### Interrupt Handling Example
```cpp
// Trigger timer interrupt
cpu->trigger_interrupt(7);

// Check pending interrupts
auto pending = cpu->get_pending_interrupts();
if (pending.value() & (1 << 7)) {
    std::cout << "Timer interrupt pending" << std::endl;
}
```

---

## Memory Management

### MemoryController

Centralized memory management with cache coherency and DMA support.

#### Class Definition
```cpp
class MemoryController {
public:
    struct Statistics {
        uint64_t total_reads;
        uint64_t total_writes;
        uint64_t cache_hits;
        uint64_t cache_misses;
        uint64_t dma_transfers;
        size_t current_usage_bytes;
        size_t peak_usage_bytes;
    };

    static expected<std::unique_ptr<MemoryController>, utils::ErrorCode> create();
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Basic memory access
    expected<uint8_t, utils::ErrorCode> read8(uint32_t address) const;
    expected<uint16_t, utils::ErrorCode> read16(uint32_t address) const;
    expected<uint32_t, utils::ErrorCode> read32(uint32_t address) const;
    
    utils::expected<void, utils::ErrorCode> write8(uint32_t address, uint8_t value);
    utils::expected<void, utils::ErrorCode> write16(uint32_t address, uint16_t value);
    utils::expected<void, utils::ErrorCode> write32(uint32_t address, uint32_t value);
    
    // Bulk operations
    expected<std::vector<uint8_t>, utils::ErrorCode> read_bulk(
        uint32_t address, size_t length) const;
    utils::expected<void, utils::ErrorCode> write_bulk(
        uint32_t address, const std::vector<uint8_t>& data);
    
    // DMA operations
    utils::expected<void, utils::ErrorCode> dma_transfer(
        uint32_t src_address, uint32_t dst_address, size_t length);
    
    // Memory mapping
    utils::expected<void, utils::ErrorCode> map_region(
        uint32_t virtual_addr, uint32_t physical_addr, size_t size);
    utils::expected<void, utils::ErrorCode> unmap_region(uint32_t virtual_addr);
    
    // Statistics and monitoring
    expected<Statistics, utils::ErrorCode> get_statistics() const;
    void reset_statistics();
};
```

#### Basic Memory Access Example
```cpp
auto memory = MemoryController::create();

// Write different data sizes
memory->write8(0x20000000, 0xAB);          // Write byte
memory->write16(0x20000004, 0x1234);       // Write halfword  
memory->write32(0x20000008, 0xDEADBEEF);   // Write word

// Read data back
auto byte_val = memory->read8(0x20000000);
auto half_val = memory->read16(0x20000004);
auto word_val = memory->read32(0x20000008);

std::cout << "Byte: 0x" << std::hex << (int)byte_val.value() << std::endl;
std::cout << "Half: 0x" << std::hex << half_val.value() << std::endl;
std::cout << "Word: 0x" << std::hex << word_val.value() << std::endl;
```

#### Bulk Transfer Example
```cpp
// Prepare test data
std::vector<uint8_t> test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};

// Write bulk data
memory->write_bulk(0x20001000, test_data);

// Read it back
auto read_data = memory->read_bulk(0x20001000, test_data.size());
assert(read_data.value() == test_data);

// DMA transfer
memory->dma_transfer(0x20001000, 0x20002000, test_data.size());
```

---

## Graphics & Display

### GraphicsEngine

Unified graphics system with SDL2 rendering and touch input.

#### Class Definition
```cpp
class GraphicsEngine {
public:
    struct DisplayConfig {
        uint32_t width = 1280;
        uint32_t height = 720; 
        uint32_t refresh_rate = 60;
        bool vsync_enabled = true;
        bool double_buffering = true;
    };

    static expected<std::unique_ptr<GraphicsEngine>, utils::ErrorCode> create(
        const DisplayConfig& config);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Rendering
    utils::expected<void, utils::ErrorCode> clear_screen(const Color& color);
    utils::expected<void, utils::ErrorCode> present();
    utils::expected<void, utils::ErrorCode> set_pixel(uint32_t x, uint32_t y, const Color& color);
    
    // Framebuffer access
    std::shared_ptr<Framebuffer> get_framebuffer();
    std::shared_ptr<TouchInput> get_touch_input();
    
    // Performance metrics
    expected<RenderingStats, utils::ErrorCode> get_rendering_stats() const;
    uint32_t get_frame_count() const;
    double get_fps() const;
};
```

#### Framebuffer

Direct pixel access for high-performance rendering.

```cpp
class Framebuffer {
public:
    utils::expected<void, utils::ErrorCode> set_pixel(uint32_t x, uint32_t y, const Color& color);
    expected<Color, utils::ErrorCode> get_pixel(uint32_t x, uint32_t y) const;
    
    utils::expected<void, utils::ErrorCode> fill_rectangle(
        uint32_t x, uint32_t y, uint32_t width, uint32_t height, const Color& color);
    
    utils::expected<void, utils::ErrorCode> copy_region(
        uint32_t src_x, uint32_t src_y, uint32_t dst_x, uint32_t dst_y,
        uint32_t width, uint32_t height);
    
    uint32_t get_width() const;
    uint32_t get_height() const;
    size_t get_pixel_count() const;
};
```

#### TouchInput

GT911 touch controller emulation with multi-touch support.

```cpp
class TouchInput {
public:
    struct TouchPoint {
        uint32_t x, y;
        uint8_t id;
        bool pressed;
        float pressure;
    };

    struct TouchEvent {
        TouchPoint point;
        enum class Type { PRESS, RELEASE, MOVE } type;
        std::chrono::high_resolution_clock::time_point timestamp;
    };

    expected<std::vector<TouchEvent>, utils::ErrorCode> get_touch_events();
    expected<std::vector<TouchPoint>, utils::ErrorCode> get_active_touches() const;
    
    void simulate_touch_event(const TouchPoint& point);
    void simulate_multi_touch(const std::vector<TouchPoint>& points);
    
    bool is_calibrated() const;
    utils::expected<void, utils::ErrorCode> calibrate(
        const std::vector<std::pair<TouchPoint, TouchPoint>>& calibration_points);
};
```

#### Graphics Usage Example
```cpp
// Create graphics engine
GraphicsEngine::DisplayConfig config;
config.width = 1280;
config.height = 720;
config.refresh_rate = 60;

auto graphics = GraphicsEngine::create(config);
graphics->initialize();

// Get framebuffer for direct pixel access
auto framebuffer = graphics->get_framebuffer();

// Draw a red rectangle
Color red = {255, 0, 0, 255};
framebuffer->fill_rectangle(100, 100, 200, 150, red);

// Get touch input
auto touch = graphics->get_touch_input();

// Process touch events
auto events = touch->get_touch_events();
for (const auto& event : events.value()) {
    if (event.type == TouchInput::TouchEvent::Type::PRESS) {
        std::cout << "Touch at (" << event.point.x << ", " << event.point.y << ")" << std::endl;
    }
}

// Present frame
graphics->present();
```

---

## Peripherals

### GPIOController

55-pin GPIO system with interrupt support and pin multiplexing.

#### Class Definition
```cpp
class GPIOController {
public:
    enum class Mode {
        INPUT,
        OUTPUT,
        INPUT_PULLUP,
        INPUT_PULLDOWN,
        OPEN_DRAIN,
        ANALOG
    };
    
    enum class Pull {
        NONE,
        UP,
        DOWN
    };
    
    enum class InterruptTrigger {
        RISING_EDGE,
        FALLING_EDGE,
        BOTH_EDGES,
        LOW_LEVEL,
        HIGH_LEVEL
    };

    using InterruptCallback = std::function<void(uint8_t pin, bool state)>;

    static expected<std::unique_ptr<GPIOController>, utils::ErrorCode> create(
        std::shared_ptr<memory::MemoryController> memory_controller);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Pin configuration
    utils::expected<void, utils::ErrorCode> configure_pin(uint8_t pin, Mode mode);
    expected<Mode, utils::ErrorCode> get_pin_mode(uint8_t pin) const;
    
    utils::expected<void, utils::ErrorCode> set_pull_mode(uint8_t pin, Pull pull);
    expected<Pull, utils::ErrorCode> get_pull_mode(uint8_t pin) const;
    
    // Digital I/O
    utils::expected<void, utils::ErrorCode> digital_write(uint8_t pin, bool state);
    expected<bool, utils::ErrorCode> digital_read(uint8_t pin) const;
    
    // Interrupt handling
    utils::expected<void, utils::ErrorCode> attach_interrupt(
        uint8_t pin, InterruptTrigger trigger, InterruptCallback callback);
    utils::expected<void, utils::ErrorCode> detach_interrupt(uint8_t pin);
    
    // Pin validation
    bool is_pin_available(uint8_t pin) const;
    expected<std::vector<uint8_t>, utils::ErrorCode> get_available_pins() const;
    
    // Simulation helpers
    void simulate_external_input(uint8_t pin, bool state);
};
```

#### GPIO Usage Example
```cpp
auto gpio = GPIOController::create(memory_controller);

// Configure LED pin as output
gpio->configure_pin(2, GPIOController::Mode::OUTPUT);

// Blink LED
for (int i = 0; i < 5; ++i) {
    gpio->digital_write(2, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    gpio->digital_write(2, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// Configure button with interrupt
gpio->configure_pin(0, GPIOController::Mode::INPUT);
gpio->set_pull_mode(0, GPIOController::Pull::UP);

auto button_callback = [](uint8_t pin, bool state) {
    std::cout << "Button " << (state ? "released" : "pressed") << std::endl;
};

gpio->attach_interrupt(0, GPIOController::InterruptTrigger::BOTH_EDGES, button_callback);
```

### I2CController

I2C communication with device simulation and transaction support.

#### Class Definition
```cpp
class I2CController {
public:
    enum class Mode {
        MASTER,
        SLAVE
    };

    struct Configuration {
        uint32_t clock_frequency;
        Mode mode;
        uint8_t slave_address;  // Only used in slave mode
    };

    static expected<std::unique_ptr<I2CController>, utils::ErrorCode> create(
        uint8_t controller_id, std::shared_ptr<memory::MemoryController> memory_controller);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    utils::expected<void, utils::ErrorCode> configure(
        uint32_t clock_frequency, Mode mode, uint8_t slave_address = 0);
    expected<Configuration, utils::ErrorCode> get_configuration() const;
    
    // Device management
    void add_virtual_device(uint8_t address);
    void remove_virtual_device(uint8_t address);
    expected<std::vector<uint8_t>, utils::ErrorCode> scan_devices();
    
    // Register operations
    utils::expected<void, utils::ErrorCode> write_register(
        uint8_t device_address, uint8_t register_address, uint8_t value);
    expected<uint8_t, utils::ErrorCode> read_register(
        uint8_t device_address, uint8_t register_address);
    
    utils::expected<void, utils::ErrorCode> write_registers(
        uint8_t device_address, uint8_t register_address, const std::vector<uint8_t>& values);
    expected<std::vector<uint8_t>, utils::ErrorCode> read_registers(
        uint8_t device_address, uint8_t register_address, size_t count);
    
    // Transaction support
    utils::expected<void, utils::ErrorCode> begin_transaction(uint8_t device_address);
    utils::expected<void, utils::ErrorCode> end_transaction();
    utils::expected<void, utils::ErrorCode> repeated_start(uint8_t device_address);
    
    utils::expected<void, utils::ErrorCode> write_byte(uint8_t value);
    expected<uint8_t, utils::ErrorCode> read_byte();
    
    // Bulk operations
    utils::expected<void, utils::ErrorCode> write_bulk(
        uint8_t device_address, const std::vector<uint8_t>& data);
    expected<std::vector<uint8_t>, utils::ErrorCode> read_bulk(
        uint8_t device_address, size_t count);
    
    // Advanced features
    utils::expected<void, utils::ErrorCode> reset_bus();
    void enable_clock_stretching(uint8_t device_address, bool enabled);
    void simulate_bus_error();
};
```

#### I2C Usage Example
```cpp
auto i2c = I2CController::create(0, memory_controller);

// Configure as master at 400kHz
i2c->configure(400000, I2CController::Mode::MASTER);

// Add virtual BMI270 IMU
i2c->add_virtual_device(0x68);

// Read chip ID
auto chip_id = i2c->read_register(0x68, 0x00);
std::cout << "BMI270 Chip ID: 0x" << std::hex << (int)chip_id.value() << std::endl;

// Read accelerometer data (6 bytes)
auto accel_data = i2c->read_registers(0x68, 0x0C, 6);
if (accel_data.has_value() && accel_data.value().size() >= 6) {
    int16_t accel_x = (accel_data.value()[1] << 8) | accel_data.value()[0];
    int16_t accel_y = (accel_data.value()[3] << 8) | accel_data.value()[2];
    int16_t accel_z = (accel_data.value()[5] << 8) | accel_data.value()[4];
    
    std::cout << "Acceleration: X=" << accel_x << " Y=" << accel_y << " Z=" << accel_z << std::endl;
}

// Transaction example
i2c->begin_transaction(0x68);
i2c->write_byte(0x7C);  // Power control register
i2c->write_byte(0x0E);  // Enable accelerometer and gyroscope
i2c->end_transaction();
```

---

## Connectivity

### WiFiController

802.11 Wi-Fi simulation with station/AP modes and security protocols.

#### Class Definition
```cpp
class WiFiController {
public:
    enum class State {
        IDLE,
        SCANNING,
        CONNECTING,
        CONNECTED,
        AP_STARTED,
        DISCONNECTED,
        ERROR
    };
    
    enum class SecurityType {
        NONE,
        WEP,
        WPA,
        WPA2,
        WPA3
    };
    
    enum class PowerMode {
        ACTIVE,
        POWER_SAVE,
        PERFORMANCE
    };

    struct StationConfig {
        std::string ssid;
        std::string password;
        SecurityType security;
        bool auto_reconnect = true;
    };
    
    struct APConfig {
        std::string ssid;
        std::string password;
        uint8_t channel = 1;
        SecurityType security = SecurityType::WPA2;
        uint8_t max_connections = 4;
        bool hidden = false;
    };
    
    struct NetworkInfo {
        std::string ssid;
        std::string bssid;
        int8_t rssi;
        uint8_t channel;
        SecurityType security;
        bool hidden;
    };
    
    struct IPInfo {
        std::string ip_address;
        std::string subnet_mask;
        std::string gateway;
        std::string dns_primary;
        std::string dns_secondary;
    };
    
    struct Status {
        State state;
        bool connected;
        std::string connected_ssid;
        int8_t rssi;
        uint32_t ip_address;
        std::chrono::milliseconds connection_duration;
    };

    static expected<std::unique_ptr<WiFiController>, utils::ErrorCode> create(
        std::shared_ptr<memory::MemoryController> memory_controller);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Station mode
    utils::expected<void, utils::ErrorCode> configure_station(const StationConfig& config);
    expected<StationConfig, utils::ErrorCode> get_station_config() const;
    
    utils::expected<void, utils::ErrorCode> connect();
    utils::expected<void, utils::ErrorCode> disconnect();
    
    expected<std::vector<NetworkInfo>, utils::ErrorCode> scan_networks();
    
    // Access Point mode
    utils::expected<void, utils::ErrorCode> configure_ap(const APConfig& config);
    expected<APConfig, utils::ErrorCode> get_ap_config() const;
    
    utils::expected<void, utils::ErrorCode> start_ap();
    utils::expected<void, utils::ErrorCode> stop_ap();
    
    // Network information
    expected<Status, utils::ErrorCode> get_status() const;
    expected<IPInfo, utils::ErrorCode> get_ip_info() const;
    expected<int8_t, utils::ErrorCode> get_rssi() const;
    
    // Power management
    utils::expected<void, utils::ErrorCode> set_power_mode(PowerMode mode);
    expected<PowerMode, utils::ErrorCode> get_power_mode() const;
    
    // MAC address
    expected<std::string, utils::ErrorCode> get_mac_address() const;
    utils::expected<void, utils::ErrorCode> set_mac_address(const std::string& mac);
    
    // Performance measurement
    struct ThroughputMeasurement {
        float download_mbps;
        float upload_mbps;
        std::chrono::milliseconds measurement_duration;
    };
    
    expected<ThroughputMeasurement, utils::ErrorCode> measure_throughput();
    expected<std::chrono::milliseconds, utils::ErrorCode> measure_latency(const std::string& host);
    
    // Simulation helpers
    void add_simulated_network(const std::string& ssid, int8_t rssi, SecurityType security, bool wps_enabled = false);
    void remove_simulated_network(const std::string& ssid);
    void simulate_connection_drop();
    
    // Callbacks
    using ConnectCallback = std::function<void()>;
    using DisconnectCallback = std::function<void()>;
    
    utils::expected<void, utils::ErrorCode> set_connect_callback(ConnectCallback callback);
    utils::expected<void, utils::ErrorCode> set_disconnect_callback(DisconnectCallback callback);
};
```

#### Wi-Fi Usage Example
```cpp
auto wifi = WiFiController::create(memory_controller);

// Add some simulated networks
wifi->add_simulated_network("HomeNetwork", -45, WiFiController::SecurityType::WPA2);
wifi->add_simulated_network("OfficeWiFi", -60, WiFiController::SecurityType::WPA3);

// Scan for networks
auto networks = wifi->scan_networks();
for (const auto& network : networks.value()) {
    std::cout << "SSID: " << network.ssid 
              << ", RSSI: " << (int)network.rssi << "dBm"
              << ", Security: " << (int)network.security << std::endl;
}

// Configure and connect
WiFiController::StationConfig config;
config.ssid = "HomeNetwork";
config.password = "mypassword123";
config.security = WiFiController::SecurityType::WPA2;

wifi->configure_station(config);

auto result = wifi->connect();
if (result.has_value()) {
    std::cout << "Connected to Wi-Fi!" << std::endl;
    
    // Get IP information
    auto ip_info = wifi->get_ip_info();
    std::cout << "IP: " << ip_info.value().ip_address << std::endl;
    std::cout << "Gateway: " << ip_info.value().gateway << std::endl;
    
    // Measure performance
    auto throughput = wifi->measure_throughput();
    std::cout << "Download: " << throughput.value().download_mbps << " Mbps" << std::endl;
    std::cout << "Upload: " << throughput.value().upload_mbps << " Mbps" << std::endl;
}
```

### BluetoothController

Bluetooth Classic and BLE with GATT operations and device management.

#### Class Definition
```cpp
class BluetoothController {
public:
    enum class State {
        DISABLED,
        ENABLING,
        ENABLED,
        DISCOVERING,
        CONNECTING,
        CONNECTED,
        ERROR
    };
    
    enum class DeviceClass {
        SMARTPHONE,
        TABLET,
        LAPTOP,
        DESKTOP,
        HEADPHONES,
        SPEAKER,
        KEYBOARD,
        MOUSE,
        GAMEPAD,
        SENSOR,
        UNKNOWN
    };

    struct DeviceInfo {
        std::string name;
        std::string address;  // MAC address
        DeviceClass device_class;
        int8_t rssi;
        bool paired;
        bool connected;
        bool supports_ble;
        std::vector<std::string> services;
    };
    
    struct Status {
        State state;
        bool enabled;
        bool discoverable;
        bool discovering;
        uint8_t connected_device_count;
        uint8_t max_connections;
    };

    static expected<std::unique_ptr<BluetoothController>, utils::ErrorCode> create(
        std::shared_ptr<memory::MemoryController> memory_controller);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Basic control
    utils::expected<void, utils::ErrorCode> enable();
    utils::expected<void, utils::ErrorCode> disable();
    expected<Status, utils::ErrorCode> get_status() const;
    
    // Discovery
    utils::expected<void, utils::ErrorCode> start_discovery();
    utils::expected<void, utils::ErrorCode> stop_discovery();
    expected<std::vector<DeviceInfo>, utils::ErrorCode> get_discovered_devices() const;
    
    // Connection management
    utils::expected<void, utils::ErrorCode> connect_device(const std::string& address);
    utils::expected<void, utils::ErrorCode> disconnect_device(const std::string& address);
    expected<std::vector<DeviceInfo>, utils::ErrorCode> get_connected_devices() const;
    
    // Pairing
    utils::expected<void, utils::ErrorCode> pair_device(const std::string& address, const std::string& pin = "");
    utils::expected<void, utils::ErrorCode> unpair_device(const std::string& address);
    expected<std::vector<DeviceInfo>, utils::ErrorCode> get_paired_devices() const;
    
    // Data transfer
    utils::expected<void, utils::ErrorCode> send_data(
        const std::string& address, const std::vector<uint8_t>& data);
    expected<std::vector<uint8_t>, utils::ErrorCode> receive_data(const std::string& address);
    
    // BLE GATT operations
    expected<std::vector<std::string>, utils::ErrorCode> discover_services(const std::string& address);
    expected<std::vector<uint8_t>, utils::ErrorCode> read_characteristic(
        const std::string& address, const std::string& service_uuid, const std::string& char_uuid);
    utils::expected<void, utils::ErrorCode> write_characteristic(
        const std::string& address, const std::string& service_uuid, 
        const std::string& char_uuid, const std::vector<uint8_t>& data);
    
    // Configuration
    expected<std::string, utils::ErrorCode> get_local_address() const;
    utils::expected<void, utils::ErrorCode> set_discoverable(bool discoverable, uint32_t timeout_seconds = 0);
    utils::expected<void, utils::ErrorCode> set_device_name(const std::string& name);
    
    // Simulation helpers
    void add_simulated_device(const std::string& name, const std::string& address, DeviceClass device_class);
    void remove_simulated_device(const std::string& address);
    void simulate_device_connection(const std::string& address);
    void simulate_device_disconnection(const std::string& address);
    
    // Callbacks
    using DeviceDiscoveredCallback = std::function<void(const DeviceInfo&)>;
    using DeviceConnectedCallback = std::function<void(const DeviceInfo&)>;
    using DataReceivedCallback = std::function<void(const std::string&, const std::vector<uint8_t>&)>;
    
    utils::expected<void, utils::ErrorCode> set_device_discovered_callback(DeviceDiscoveredCallback callback);
    utils::expected<void, utils::ErrorCode> set_device_connected_callback(DeviceConnectedCallback callback);
    utils::expected<void, utils::ErrorCode> set_data_received_callback(DataReceivedCallback callback);
};
```

#### Bluetooth Usage Example
```cpp
auto bluetooth = BluetoothController::create(memory_controller);

// Enable Bluetooth
bluetooth->enable();

// Add some simulated devices
bluetooth->add_simulated_device("iPhone 14", "12:34:56:78:9A:BC", BluetoothController::DeviceClass::SMARTPHONE);
bluetooth->add_simulated_device("AirPods Pro", "AA:BB:CC:DD:EE:FF", BluetoothController::DeviceClass::HEADPHONES);

// Start device discovery
bluetooth->start_discovery();

// Wait a bit for discovery
std::this_thread::sleep_for(std::chrono::milliseconds(2000));

// Get discovered devices
auto devices = bluetooth->get_discovered_devices();
for (const auto& device : devices.value()) {
    std::cout << "Found: " << device.name 
              << " (" << device.address << ")"
              << " RSSI: " << (int)device.rssi << "dBm" << std::endl;
}

// Connect to a device
if (!devices.value().empty()) {
    const auto& device = devices.value()[0];
    auto connect_result = bluetooth->connect_device(device.address);
    
    if (connect_result.has_value()) {
        std::cout << "Connected to " << device.name << std::endl;
        
        // Send some data
        std::vector<uint8_t> message = {'H', 'e', 'l', 'l', 'o'};
        bluetooth->send_data(device.address, message);
        
        // For BLE devices, discover services
        if (device.supports_ble) {
            auto services = bluetooth->discover_services(device.address);
            for (const auto& service : services.value()) {
                std::cout << "Service: " << service << std::endl;
            }
        }
    }
}

bluetooth->stop_discovery();
```

---

## Audio Processing

### AudioPipeline

SDL2-based audio processing with ES8388 codec simulation.

#### Class Definition
```cpp
class AudioPipeline {
public:
    struct AudioFormat {
        uint32_t sample_rate = 48000;
        uint16_t bit_depth = 16;
        uint8_t channels = 2;
        uint32_t buffer_size = 1024;
    };
    
    struct AudioStats {
        uint64_t samples_played;
        uint64_t samples_recorded;
        uint32_t buffer_underruns;
        uint32_t buffer_overruns;
        double cpu_usage_percent;
        std::chrono::milliseconds latency;
    };

    static expected<std::unique_ptr<AudioPipeline>, utils::ErrorCode> create(
        const AudioFormat& format);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    utils::expected<void, utils::ErrorCode> start_playback();
    utils::expected<void, utils::ErrorCode> stop_playback();
    utils::expected<void, utils::ErrorCode> start_recording();
    utils::expected<void, utils::ErrorCode> stop_recording();
    
    // Audio data
    utils::expected<void, utils::ErrorCode> write_audio_data(const std::vector<int16_t>& samples);
    expected<std::vector<int16_t>, utils::ErrorCode> read_audio_data();
    
    // Volume control
    utils::expected<void, utils::ErrorCode> set_playback_volume(float volume);  // 0.0-1.0
    utils::expected<void, utils::ErrorCode> set_recording_gain(float gain);     // 0.0-2.0
    
    expected<float, utils::ErrorCode> get_playback_volume() const;
    expected<float, utils::ErrorCode> get_recording_gain() const;
    
    // Format and configuration
    expected<AudioFormat, utils::ErrorCode> get_format() const;
    expected<AudioStats, utils::ErrorCode> get_statistics() const;
    
    // Callbacks
    using AudioCallback = std::function<void(const std::vector<int16_t>&)>;
    
    utils::expected<void, utils::ErrorCode> set_playback_callback(AudioCallback callback);
    utils::expected<void, utils::ErrorCode> set_recording_callback(AudioCallback callback);
};
```

#### Audio Usage Example
```cpp
// Create audio pipeline
AudioPipeline::AudioFormat format;
format.sample_rate = 48000;
format.bit_depth = 16;
format.channels = 2;

auto audio = AudioPipeline::create(format);

// Set up recording callback
auto recording_callback = [](const std::vector<int16_t>& samples) {
    // Process recorded audio data
    std::cout << "Recorded " << samples.size() << " samples" << std::endl;
};

audio->set_recording_callback(recording_callback);

// Start recording
audio->start_recording();

// Generate and play a sine wave
audio->start_playback();

const int duration_seconds = 2;
const int total_samples = format.sample_rate * duration_seconds * format.channels;
std::vector<int16_t> sine_wave;

for (int i = 0; i < total_samples; i += format.channels) {
    double time = (double)i / (format.sample_rate * format.channels);
    int16_t sample = (int16_t)(sin(2.0 * M_PI * 440.0 * time) * 16384); // 440Hz tone
    
    sine_wave.push_back(sample); // Left channel
    sine_wave.push_back(sample); // Right channel
}

// Play the generated audio
audio->write_audio_data(sine_wave);

// Get statistics
auto stats = audio->get_statistics();
std::cout << "Samples played: " << stats.value().samples_played << std::endl;
std::cout << "CPU usage: " << stats.value().cpu_usage_percent << "%" << std::endl;
```

---

## Debugging & Profiling

### Profiler

High-precision performance profiling with component breakdown.

#### Class Definition
```cpp
class Profiler {
public:
    struct PerformanceMetrics {
        double cpu_usage_percent;
        size_t memory_usage_bytes;
        size_t peak_memory_bytes;
        double operations_per_second;
        double average_latency_ns;
        double min_latency_ns;
        double max_latency_ns;
        size_t total_operations;
        std::chrono::nanoseconds total_execution_time;
        std::unordered_map<std::string, double> component_breakdown;
    };
    
    struct ComponentStats {
        std::string name;
        size_t call_count;
        std::chrono::nanoseconds total_time;
        std::chrono::nanoseconds min_time;
        std::chrono::nanoseconds max_time;
        std::chrono::nanoseconds average_time;
        size_t memory_allocated;
        size_t memory_freed;
        double cpu_percentage;
    };

    static expected<std::unique_ptr<Profiler>, utils::ErrorCode> create();
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    void start_profiling(const std::string& session_name);
    void stop_profiling();
    void pause_profiling();
    void resume_profiling();
    
    // Scope-based profiling
    class ProfileScope {
    public:
        ProfileScope(Profiler& profiler, const std::string& name, 
                    const std::string& component = "", const std::string& function = "");
        ~ProfileScope();
        
        void add_memory_allocation(size_t bytes);
        void add_memory_deallocation(size_t bytes);
    };
    
    ProfileScope create_scope(const std::string& name, const std::string& component = "", 
                             const std::string& function = "");
    
    // Memory tracking
    void record_memory_allocation(size_t bytes, const std::string& component = "");
    void record_memory_deallocation(size_t bytes, const std::string& component = "");
    void record_custom_metric(const std::string& name, double value);
    
    // Results
    expected<PerformanceMetrics, utils::ErrorCode> get_performance_metrics() const;
    expected<std::vector<ComponentStats>, utils::ErrorCode> get_component_stats() const;
    
    // Reports
    expected<std::string, utils::ErrorCode> generate_performance_report() const;
    expected<void, utils::ErrorCode> export_profile_data(const std::string& filename) const;
    
    // Configuration
    void set_sampling_rate(double samples_per_second);
    void set_memory_tracking_enabled(bool enabled);
    void set_component_filter(const std::vector<std::string>& components);
    
    bool is_profiling() const;
};

// Convenience macros
#define PROFILE_SCOPE(profiler, name) \
    auto _prof_scope = profiler.create_scope(name, __FILE__, __FUNCTION__)

#define PROFILE_FUNCTION(profiler) \
    auto _prof_scope = profiler.create_scope(__FUNCTION__, __FILE__, __FUNCTION__)

#define PROFILE_COMPONENT(profiler, component, name) \
    auto _prof_scope = profiler.create_scope(name, component, __FUNCTION__)
```

#### Profiling Usage Example
```cpp
auto profiler = Profiler::create();
profiler->initialize();

// Start profiling session
profiler->start_profiling("performance_test");

// Profile a function with automatic scope
{
    PROFILE_SCOPE(*profiler, "expensive_computation");
    
    // Simulate expensive work
    for (int i = 0; i < 1000000; ++i) {
        double result = sin(i) * cos(i);
        (void)result; // Prevent optimization
    }
}

// Profile with component information
{
    PROFILE_COMPONENT(*profiler, "graphics", "render_frame");
    
    // Simulate graphics work
    std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60fps
}

// Manual memory tracking
profiler->record_memory_allocation(1024 * 1024, "buffer_allocation"); // 1MB
std::this_thread::sleep_for(std::chrono::milliseconds(100));
profiler->record_memory_deallocation(1024 * 1024, "buffer_allocation");

// Stop profiling and get results
profiler->stop_profiling();

auto metrics = profiler->get_performance_metrics();
std::cout << "Operations/sec: " << metrics.value().operations_per_second << std::endl;
std::cout << "Average latency: " << metrics.value().average_latency_ns << "ns" << std::endl;
std::cout << "Memory usage: " << metrics.value().memory_usage_bytes << " bytes" << std::endl;

auto component_stats = profiler->get_component_stats();
for (const auto& stat : component_stats.value()) {
    std::cout << "Component: " << stat.name 
              << ", Calls: " << stat.call_count
              << ", CPU: " << stat.cpu_percentage << "%" << std::endl;
}

// Generate detailed report
auto report = profiler->generate_performance_report();
std::cout << report.value() << std::endl;

// Export data for analysis
profiler->export_profile_data("profile_data.json");
```

---

## Configuration

### Configuration

JSON-based configuration system with validation and hot-reloading.

#### Class Definition
```cpp
class Configuration {
public:
    static expected<std::shared_ptr<Configuration>, utils::ErrorCode> load(
        const std::string& filename);
    static expected<std::shared_ptr<Configuration>, utils::ErrorCode> create_from_string(
        const std::string& json_content);
    
    // Value getters with defaults
    expected<int, utils::ErrorCode> get_int(const std::string& key) const;
    expected<double, utils::ErrorCode> get_double(const std::string& key) const;
    expected<bool, utils::ErrorCode> get_bool(const std::string& key) const;
    expected<std::string, utils::ErrorCode> get_string(const std::string& key) const;
    
    int get_int(const std::string& key, int default_value) const;
    double get_double(const std::string& key, double default_value) const;
    bool get_bool(const std::string& key, bool default_value) const;
    std::string get_string(const std::string& key, const std::string& default_value) const;
    
    // Array getters
    expected<std::vector<int>, utils::ErrorCode> get_int_array(const std::string& key) const;
    expected<std::vector<double>, utils::ErrorCode> get_double_array(const std::string& key) const;
    expected<std::vector<bool>, utils::ErrorCode> get_bool_array(const std::string& key) const;
    expected<std::vector<std::string>, utils::ErrorCode> get_string_array(const std::string& key) const;
    
    // Value setters
    utils::expected<void, utils::ErrorCode> set_int(const std::string& key, int value);
    utils::expected<void, utils::ErrorCode> set_double(const std::string& key, double value);
    utils::expected<void, utils::ErrorCode> set_bool(const std::string& key, bool value);
    utils::expected<void, utils::ErrorCode> set_string(const std::string& key, const std::string& value);
    
    // Validation
    using ValidationRule = std::function<bool(const nlohmann::json&)>;
    void add_validation_rule(const std::string& key, ValidationRule rule);
    expected<bool, utils::ErrorCode> validate() const;
    
    // File operations
    utils::expected<void, utils::ErrorCode> save(const std::string& filename) const;
    utils::expected<void, utils::ErrorCode> reload();
    
    // Merging and environment overrides
    utils::expected<void, utils::ErrorCode> merge(const Configuration& other);
    void enable_environment_overrides(const std::string& prefix);
    
    // Change watching
    using ChangeCallback = std::function<void(const std::string& key, const nlohmann::json& value)>;
    utils::expected<void, utils::ErrorCode> watch_changes(ChangeCallback callback);
    void stop_watching();
    
    // Key existence
    bool has_key(const std::string& key) const;
    expected<std::vector<std::string>, utils::ErrorCode> get_all_keys() const;
};
```

#### Configuration Usage Example
```cpp
// Load configuration from file
auto config = Configuration::load("config/development.json");

// Get values with defaults
int core_count = config->get_int("cpu.core_count", 2);
bool debug_enabled = config->get_bool("emulator.enable_debugging", false);
std::string log_level = config->get_string("emulator.log_level", "info");

// Get arrays
auto gpio_pins = config->get_int_array("peripherals.gpio_pins");
auto i2c_addresses = config->get_string_array("peripherals.i2c_addresses");

// Set values
config->set_int("display.width", 1280);
config->set_int("display.height", 720);
config->set_bool("graphics.vsync_enabled", true);

// Add validation rules
config->add_validation_rule("cpu.core_count", [](const auto& value) {
    return value.is_number_integer() && value.get<int>() >= 1 && value.get<int>() <= 8;
});

config->add_validation_rule("display.refresh_rate", [](const auto& value) {
    return value.is_number_integer() && value.get<int>() >= 30 && value.get<int>() <= 144;
});

// Validate configuration
auto validation_result = config->validate();
if (!validation_result.value()) {
    std::cerr << "Configuration validation failed!" << std::endl;
}

// Enable environment variable overrides
config->enable_environment_overrides("M5TAB5_EMULATOR");
// Now M5TAB5_EMULATOR_CPU_CORE_COUNT=4 will override cpu.core_count

// Watch for changes
auto change_callback = [](const std::string& key, const auto& value) {
    std::cout << "Configuration changed: " << key << " = " << value << std::endl;
};
config->watch_changes(change_callback);

// Save modified configuration
config->save("config/modified.json");
```

---

## Error Handling

### Error System

Comprehensive error handling with source location tracking and error chaining.

#### Types
```cpp
enum class ErrorCode {
    SUCCESS = 0,
    
    // General errors
    INVALID_PARAMETER,
    OUT_OF_MEMORY,
    INVALID_STATE,
    TIMEOUT,
    CANCELLED,
    
    // File I/O errors
    FILE_NOT_FOUND,
    FILE_READ_ERROR,
    FILE_WRITE_ERROR,
    PARSE_ERROR,
    
    // Hardware errors
    DEVICE_NOT_FOUND,
    DEVICE_BUSY,
    COMMUNICATION_ERROR,
    HARDWARE_ERROR,
    CALIBRATION_ERROR,
    
    // Memory errors
    INVALID_ADDRESS,
    ACCESS_VIOLATION,
    ALIGNMENT_ERROR,
    MEMORY_ERROR,
    
    // Network errors
    NETWORK_ERROR,
    CONNECTION_FAILED,
    AUTHENTICATION_FAILED,
    PROTOCOL_ERROR,
    
    // System errors
    SYSTEM_ERROR,
    RESOURCE_EXHAUSTED,
    PERMISSION_DENIED,
    OPERATION_NOT_SUPPORTED,
    
    // Custom application errors
    VALIDATION_FAILED,
    CONFIGURATION_ERROR,
    INITIALIZATION_FAILED,
    SHUTDOWN_ERROR
};

enum class ErrorCategory {
    SUCCESS,
    INVALID_INPUT,
    SYSTEM,
    HARDWARE,
    NETWORK,
    SECURITY,
    RESOURCE,
    CONFIGURATION,
    UNKNOWN
};

class Error {
public:
    enum class Severity {
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };

    static Error create(ErrorCode code, const std::string& message, 
                       const std::string& context = "",
                       std::source_location location = std::source_location::current());
    
    ErrorCode code() const;
    const std::string& message() const;
    const std::string& context() const;
    const std::source_location& location() const;
    Severity severity() const;
    
    Error with_context(const std::string& context) const;
    Error with_severity(Severity severity) const;
    Error with_cause(std::shared_ptr<Error> cause) const;
    
    std::shared_ptr<Error> cause() const;
    std::string to_string() const;
};

// Expected type for error handling
template<typename T, typename E>
class expected {
public:
    expected(T value);
    expected(E error);
    
    bool has_value() const;
    bool has_error() const;
    
    T& value();
    const T& value() const;
    E& error();
    const E& error() const;
    
    T value_or(T default_value) const;
    
    template<typename F>
    auto transform(F&& func) -> expected<decltype(func(value())), E>;
    
    template<typename F>
    auto and_then(F&& func) -> decltype(func(value()));
    
    template<typename F>
    auto or_else(F&& func) -> expected<T, E>;
};

// Convenience macros
#define TRY(var, expr) \
    auto _temp_result = (expr); \
    if (!_temp_result.has_value()) { \
        return utils::expected<decltype(_temp_result.value()), decltype(_temp_result.error())>::error(_temp_result.error()); \
    } \
    auto var = _temp_result.value()

#define RETURN_IF_ERROR(expr) \
    do { \
        auto _result = (expr); \
        if (!_result.has_value()) { \
            return utils::expected<void, utils::ErrorCode>::error(_result.error()); \
        } \
    } while(0)
```

#### Error Handling Usage Example
```cpp
// Function that can fail
expected<int, ErrorCode> divide(int a, int b) {
    if (b == 0) {
        return expected<int, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
    }
    return expected<int, ErrorCode>(a / b);
}

// Using TRY macro for error propagation
expected<double, ErrorCode> complex_calculation(int x, int y, int z) {
    TRY(result1, divide(x, y));
    TRY(result2, divide(result1, z));
    
    return expected<double, ErrorCode>(sqrt(result2));
}

// Error handling with transform and chaining
auto result = divide(10, 2)
    .transform([](int x) { return x * 2; })  // Transform success value
    .and_then([](int x) -> expected<std::string, ErrorCode> {
        return expected<std::string, ErrorCode>(std::to_string(x));
    })
    .or_else([](ErrorCode error) -> expected<std::string, ErrorCode> {
        return expected<std::string, ErrorCode>("default_value");
    });

if (result.has_value()) {
    std::cout << "Result: " << result.value() << std::endl;
} else {
    std::cout << "Error: " << error_code_to_string(result.error()) << std::endl;
}

// Creating detailed errors
auto error = Error::create(ErrorCode::COMMUNICATION_ERROR, "I2C transaction failed")
                .with_context("Device 0x68")
                .with_severity(Error::Severity::ERROR);

std::cout << error.to_string() << std::endl;

// Error registry for tracking
ErrorRegistry registry;
registry.register_error(error);

auto metrics = registry.get_metrics();
std::cout << "Total errors: " << metrics.total_errors << std::endl;
```

---

For complete implementation details and additional examples, see:
- [User Guide](../user-guide.md)
- [Developer Guide](../developer-guide.md)  
- [Examples Directory](../../examples/)
- [Test Suite](../../tests/)

This API reference covers the core functionality. Many classes have additional methods and configuration options not shown here for brevity. Refer to the header files in `include/emulator/` for complete method signatures and documentation.