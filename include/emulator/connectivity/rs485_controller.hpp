#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/gpio_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <array>
#include <string>

namespace m5tab5::emulator {

enum class RS485Mode : u8 {
    DISABLED = 0,
    HALF_DUPLEX = 1,    // Most common RS-485 mode
    FULL_DUPLEX = 2,    // Four-wire RS-485
    MULTI_DROP = 3      // Multi-point configuration
};

enum class RS485BaudRate : u32 {
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_230400 = 230400,
    BAUD_460800 = 460800,
    BAUD_921600 = 921600,
    BAUD_1000000 = 1000000,
    BAUD_2000000 = 2000000
};

enum class RS485DataBits : u8 {
    DATA_5 = 5,
    DATA_6 = 6,
    DATA_7 = 7,
    DATA_8 = 8,
    DATA_9 = 9
};

enum class RS485Parity : u8 {
    NONE = 0,
    EVEN = 1,
    ODD = 2,
    MARK = 3,   // Always 1
    SPACE = 4   // Always 0
};

enum class RS485StopBits : u8 {
    STOP_1 = 1,
    STOP_1_5 = 15,  // 1.5 stop bits
    STOP_2 = 2
};

enum class RS485FlowControl : u8 {
    NONE = 0,
    RTS_CTS = 1,        // Hardware flow control
    XON_XOFF = 2,       // Software flow control
    AUTO_DIRECTION = 3   // Automatic direction control
};

enum class RS485ErrorType : u8 {
    NONE = 0,
    FRAMING = 1,        // Invalid stop bit
    PARITY = 2,         // Parity mismatch
    OVERRUN = 3,        // Buffer overflow
    BREAK = 4,          // Break condition detected
    NOISE = 5,          // Line noise
    COLLISION = 6,      // Bus collision (multi-drop)
    TIMEOUT = 7,        // Communication timeout
    CRC = 8,            // CRC error (if enabled)
    ADDRESS = 9         // Invalid address (multi-drop)
};

enum class RS485EventType : u8 {
    DATA_RECEIVED = 0,
    DATA_SENT = 1,
    ERROR_OCCURRED = 2,
    BREAK_DETECTED = 3,
    COLLISION_DETECTED = 4,
    BUS_IDLE = 5,
    BUS_BUSY = 6,
    BUFFER_FULL = 7,
    TIMEOUT = 8,
    ADDRESS_MATCH = 9,
    DIRECTION_CHANGED = 10
};

enum class RS485Protocol : u8 {
    RAW = 0,            // Raw data transmission
    MODBUS_RTU = 1,     // Modbus RTU protocol
    MODBUS_ASCII = 2,   // Modbus ASCII protocol
    PROFIBUS = 3,       // Profibus protocol
    CAN_OVER_RS485 = 4, // CAN protocol over RS-485
    CUSTOM = 5          // Custom protocol
};

enum class RS485AddressMode : u8 {
    DISABLED = 0,
    ADDRESS_9BIT = 1,   // 9th bit indicates address
    ADDRESS_BYTE = 2,   // First byte is address
    MULTICAST = 3       // Multicast addressing
};

struct RS485Config {
    RS485Mode mode = RS485Mode::HALF_DUPLEX;
    RS485BaudRate baud_rate = RS485BaudRate::BAUD_115200;
    RS485DataBits data_bits = RS485DataBits::DATA_8;
    RS485Parity parity = RS485Parity::NONE;
    RS485StopBits stop_bits = RS485StopBits::STOP_1;
    RS485FlowControl flow_control = RS485FlowControl::AUTO_DIRECTION;
    
    // Bus characteristics
    bool termination_enabled = true;
    bool bias_resistors_enabled = true;
    u8 driver_enable_polarity = 1;  // 1 = active high, 0 = active low
    u16 turnaround_delay_us = 100;  // Delay between RX and TX
    u16 inter_frame_delay_us = 1000; // Minimum delay between frames
    
    // Protocol settings
    RS485Protocol protocol = RS485Protocol::RAW;
    RS485AddressMode address_mode = RS485AddressMode::DISABLED;
    u8 station_address = 1;         // Local station address
    bool crc_enabled = false;
    bool auto_retry_enabled = false;
    u8 max_retries = 3;
    
    // Buffer settings
    u16 tx_buffer_size = 256;
    u16 rx_buffer_size = 256;
    u16 timeout_ms = 1000;
    bool echo_suppression = true;   // Suppress own transmissions
};

struct RS485Frame {
    u8 address = 0;                 // Station address (if used)
    std::vector<u8> data;           // Frame data
    u16 crc = 0;                    // CRC checksum (if enabled)
    std::chrono::steady_clock::time_point timestamp;
    RS485ErrorType error = RS485ErrorType::NONE;
    u8 signal_quality = 100;       // Signal quality percentage
    i8 rssi_dbm = 0;               // Signal strength
    bool is_broadcast = false;      // Broadcast frame
    bool is_valid = true;
};

struct RS485BusState {
    bool is_busy = false;
    bool collision_detected = false;
    u8 active_stations = 0;
    std::chrono::steady_clock::time_point last_activity;
    float bus_load_percentage = 0.0f;
    u8 noise_level = 0;            // 0-100 percentage
    bool termination_present = true;
    float cable_length_m = 10.0f;
};

struct RS485Statistics {
    u64 frames_sent = 0;
    u64 frames_received = 0;
    u64 bytes_sent = 0;
    u64 bytes_received = 0;
    u64 framing_errors = 0;
    u64 parity_errors = 0;
    u64 overrun_errors = 0;
    u64 collision_errors = 0;
    u64 timeout_errors = 0;
    u64 crc_errors = 0;
    u64 retransmissions = 0;
    u64 break_conditions = 0;
    
    float error_rate = 0.0f;
    float throughput_bps = 0.0f;
    float peak_throughput_bps = 0.0f;
    u32 average_response_time_ms = 0;
    u8 bus_utilization_percent = 0;
    u32 power_consumption_mw = 0;
    
    std::chrono::steady_clock::time_point last_reset;
};

struct RS485Event {
    RS485EventType type;
    RS485ErrorType error;
    std::vector<u8> data;
    u8 address;
    std::chrono::steady_clock::time_point timestamp;
    u32 event_data;
};

// Modbus specific structures
struct ModbusFrame {
    u8 slave_address;
    u8 function_code;
    std::vector<u8> data;
    u16 crc;
    bool is_exception = false;
    u8 exception_code = 0;
};

struct ModbusRegister {
    u16 address;
    u16 value;
    bool readable = true;
    bool writable = true;
};

using RS485EventCallback = std::function<void(const RS485Event& event)>;
using RS485DataCallback = std::function<void(const RS485Frame& frame)>;
using RS485ErrorCallback = std::function<void(RS485ErrorType error, const std::string& message)>;

class RS485Controller {
public:
    static constexpr u8 MAX_STATIONS = 32;
    static constexpr u16 MAX_FRAME_SIZE = 256;
    static constexpr u16 MIN_FRAME_SIZE = 1;
    static constexpr u16 MAX_CABLE_LENGTH_M = 1200;
    static constexpr u32 MAX_BAUD_RATE = 2000000;
    static constexpr u8 BROADCAST_ADDRESS = 0x00;
    
    RS485Controller();
    ~RS485Controller();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller,
                           GPIOController* gpio_controller);
    Result<void> shutdown();

    // Configuration
    Result<void> configure(const RS485Config& config);
    Result<RS485Config> get_configuration() const;
    Result<void> set_baud_rate(RS485BaudRate baud_rate);
    Result<void> set_data_format(RS485DataBits data_bits, RS485Parity parity, RS485StopBits stop_bits);
    Result<void> set_station_address(u8 address);
    Result<void> enable_termination(bool enable);
    
    // Communication control
    Result<void> enable(bool enable);
    Result<bool> is_enabled() const;
    Result<void> set_direction(bool transmit); // true = TX, false = RX
    Result<bool> get_direction() const;
    Result<void> flush_buffers();
    Result<void> reset_bus();
    
    // Data transmission
    Result<void> send_frame(const RS485Frame& frame);
    Result<void> send_data(const std::vector<u8>& data, u8 address = 0);
    Result<void> send_broadcast(const std::vector<u8>& data);
    Result<bool> is_tx_complete() const;
    Result<size_t> get_tx_queue_size() const;
    
    // Data reception
    Result<RS485Frame> receive_frame(u32 timeout_ms = 0);
    Result<std::vector<u8>> receive_data(size_t max_length = 0, u32 timeout_ms = 0);
    Result<size_t> get_rx_queue_size() const;
    Result<size_t> get_available_bytes() const;
    
    // Bus management
    Result<RS485BusState> get_bus_state() const;
    Result<void> detect_collision();
    Result<void> send_break(u32 duration_ms = 100);
    Result<bool> wait_for_bus_idle(u32 timeout_ms = 1000);
    Result<std::vector<u8>> scan_bus(); // Scan for active stations
    
    // Error handling
    Result<RS485ErrorType> get_last_error() const;
    Result<void> clear_errors();
    Result<std::vector<RS485ErrorType>> get_error_history() const;
    
    // Protocol support
    Result<void> set_protocol(RS485Protocol protocol);
    Result<RS485Protocol> get_protocol() const;
    
    // Modbus RTU support
    Result<void> modbus_send_request(u8 slave_address, u8 function_code, const std::vector<u8>& data);
    Result<ModbusFrame> modbus_receive_response(u32 timeout_ms = 1000);
    Result<void> modbus_send_response(u8 slave_address, u8 function_code, const std::vector<u8>& data);
    Result<void> modbus_send_exception(u8 slave_address, u8 function_code, u8 exception_code);
    Result<void> modbus_set_register(u16 address, u16 value);
    Result<u16> modbus_get_register(u16 address) const;
    
    // Advanced features
    Result<void> enable_echo_suppression(bool enable);
    Result<void> set_signal_conditioning(u8 slew_rate, u8 drive_strength);
    Result<void> calibrate_timing();
    Result<void> perform_line_test();
    Result<i8> measure_signal_strength();
    
    // Diagnostics
    Result<void> enable_loopback_test(bool enable);
    Result<void> inject_error(RS485ErrorType error_type);
    Result<void> simulate_noise(u8 noise_level_percent);
    Result<void> simulate_collision();
    
    // Event handling
    Result<void> set_event_callback(RS485EventCallback callback);
    Result<void> set_data_callback(RS485DataCallback callback);
    Result<void> set_error_callback(RS485ErrorCallback callback);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    const RS485Statistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct RS485Hardware {
        bool enabled = false;
        bool direction_tx = false;  // true = TX, false = RX
        u8 de_pin = 0;             // Driver Enable pin
        u8 re_pin = 0;             // Receiver Enable pin
        bool termination_enabled = true;
        bool bias_enabled = true;
        u8 slew_rate = 2;          // 0-7 scale
        u8 drive_strength = 4;     // 0-7 scale
        i8 differential_voltage_mv = 200; // Typical RS-485 voltage
    };

    struct RS485Transceiver {
        std::queue<RS485Frame> tx_queue;
        std::queue<RS485Frame> rx_queue;
        bool tx_in_progress = false;
        bool collision_detected = false;
        std::chrono::steady_clock::time_point tx_start_time;
        std::chrono::steady_clock::time_point last_rx_time;
        u16 current_tx_bytes = 0;
        u16 current_rx_bytes = 0;
        RS485Frame current_tx_frame;
        std::vector<u8> rx_buffer;
        size_t rx_buffer_pos = 0;
    };

    struct RS485BusSimulator {
        std::vector<u8> active_stations;
        std::map<u8, std::chrono::steady_clock::time_point> station_activity;
        bool bus_busy = false;
        std::chrono::steady_clock::time_point bus_idle_start;
        u8 current_transmitter = 0;
        float cable_attenuation_db = 0.5f;
        u8 ambient_noise_level = 5;
        bool termination_mismatch = false;
        std::queue<RS485Frame> bus_frames; // Frames on the bus
    };

    struct RS485ProtocolHandler {
        RS485Protocol current_protocol = RS485Protocol::RAW;
        std::map<u16, ModbusRegister> modbus_registers;
        u8 modbus_slave_address = 1;
        bool modbus_master_mode = false;
        std::chrono::steady_clock::time_point last_modbus_request;
        u8 pending_function_code = 0;
        std::vector<u8> pending_data;
    };

    struct RS485ErrorHandler {
        RS485ErrorType last_error = RS485ErrorType::NONE;
        std::vector<RS485ErrorType> error_history;
        size_t max_error_history = 100;
        bool echo_suppression_enabled = true;
        u8 noise_injection_level = 0;
        bool loopback_enabled = false;
        std::chrono::steady_clock::time_point last_error_time;
    };

    struct RS485Timing {
        u32 bit_time_us = 0;           // Time per bit in microseconds
        u32 char_time_us = 0;          // Time per character
        u32 frame_gap_us = 0;          // Inter-frame gap
        u32 turnaround_time_us = 100;  // TX to RX turnaround
        std::chrono::steady_clock::time_point last_char_time;
        std::chrono::steady_clock::time_point frame_timeout_start;
        bool in_frame = false;
    };

    // Core operations
    void simulate_bus_activity();
    void simulate_data_transmission();
    void simulate_data_reception();
    void simulate_collisions();
    void simulate_noise_and_errors();
    void process_tx_queue();
    void process_rx_queue();
    
    // Frame processing
    void transmit_frame(const RS485Frame& frame);
    bool receive_frame_byte(u8 byte);
    void complete_frame_reception();
    bool validate_frame(const RS485Frame& frame);
    void handle_frame_error(RS485ErrorType error);
    
    // Protocol handling
    void process_modbus_frame(const RS485Frame& frame);
    ModbusFrame parse_modbus_frame(const std::vector<u8>& data);
    std::vector<u8> build_modbus_frame(const ModbusFrame& frame);
    u16 calculate_modbus_crc(const std::vector<u8>& data);
    bool verify_modbus_crc(const std::vector<u8>& data, u16 crc);
    
    // Bus management
    void update_bus_state();
    void detect_active_stations();
    void handle_collision_detection();
    void calculate_bus_timing();
    void update_direction_control();
    
    // Error simulation and handling
    void inject_framing_error();
    void inject_parity_error();
    void inject_noise();
    bool should_inject_error();
    void log_error(RS485ErrorType error, const std::string& message);
    
    // Signal processing
    i8 calculate_signal_strength(float distance_m, u8 noise_level);
    u8 calculate_signal_quality(i8 rssi, u8 noise);
    bool detect_collision_voltage();
    void update_differential_signaling();
    
    // Utility functions
    u16 calculate_crc16(const std::vector<u8>& data);
    void add_crc_to_frame(RS485Frame& frame);
    bool verify_frame_crc(const RS485Frame& frame);
    std::vector<u8> escape_control_characters(const std::vector<u8>& data);
    std::vector<u8> unescape_control_characters(const std::vector<u8>& data);
    
    void trigger_rs485_event(RS485EventType type, RS485ErrorType error = RS485ErrorType::NONE,
                            const std::vector<u8>& data = {}, u8 address = 0);
    void update_statistics();
    u32 calculate_power_consumption();
    
    bool initialized_;
    RS485Config config_;
    
    InterruptController* interrupt_controller_;
    GPIOController* gpio_controller_;
    
    // Hardware simulation
    RS485Hardware hardware_;
    RS485Transceiver transceiver_;
    RS485BusSimulator bus_simulator_;
    RS485ProtocolHandler protocol_handler_;
    RS485ErrorHandler error_handler_;
    RS485Timing timing_;
    
    // Callback interfaces
    RS485EventCallback event_callback_;
    RS485DataCallback data_callback_;
    RS485ErrorCallback error_callback_;
    
    // Background processing
    std::unique_ptr<std::thread> rs485_thread_;
    std::atomic<bool> thread_running_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    RS485Statistics statistics_;
    
    mutable std::mutex rs485_mutex_;
};

}  // namespace m5tab5::emulator