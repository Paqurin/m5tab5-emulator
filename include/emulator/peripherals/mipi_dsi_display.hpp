#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <atomic>
#include <array>

namespace m5tab5::emulator {

enum class MIPIDataType : u8 {
    VSYNC_START = 0x01,          // V-Sync Start
    VSYNC_END = 0x11,            // V-Sync End  
    HSYNC_START = 0x21,          // H-Sync Start
    HSYNC_END = 0x31,            // H-Sync End
    COLOR_MODE_OFF = 0x02,       // Color Mode Off
    COLOR_MODE_ON = 0x12,        // Color Mode On
    SHUTDOWN_PERIPHERAL = 0x22,   // Shutdown Peripheral
    TURN_ON_PERIPHERAL = 0x32,    // Turn On Peripheral
    GENERIC_SHORT_WRITE_0 = 0x03, // Generic Short Write, no parameters
    GENERIC_SHORT_WRITE_1 = 0x13, // Generic Short Write, 1 parameter
    GENERIC_SHORT_WRITE_2 = 0x23, // Generic Short Write, 2 parameters
    GENERIC_READ_0 = 0x04,       // Generic Read, no parameters
    GENERIC_READ_1 = 0x14,       // Generic Read, 1 parameter  
    GENERIC_READ_2 = 0x24,       // Generic Read, 2 parameters
    DCS_SHORT_WRITE_0 = 0x05,    // DCS Short Write, no parameters
    DCS_SHORT_WRITE_1 = 0x15,    // DCS Short Write, 1 parameter
    DCS_READ = 0x06,             // DCS Read
    SET_MAXIMUM_RETURN_PACKET = 0x37, // Set Maximum Return Packet Size
    NULL_PACKET = 0x09,          // Null Packet
    BLANKING_PACKET = 0x19,      // Blanking Packet
    GENERIC_LONG_WRITE = 0x29,   // Generic Long Write
    DCS_LONG_WRITE = 0x39,       // DCS Long Write
    LOOSELY_PACKED_PIXEL_STREAM_20 = 0x0C, // Loosely Packed Pixel Stream, 20-bit
    PACKED_PIXEL_STREAM_24 = 0x0D,    // Packed Pixel Stream, 24-bit
    PACKED_PIXEL_STREAM_16 = 0x0E,    // Packed Pixel Stream, 16-bit
    PACKED_PIXEL_STREAM_18 = 0x1E,    // Packed Pixel Stream, 18-bit
    PACKED_PIXEL_STREAM_24_2 = 0x2E,  // Packed Pixel Stream, 24-bit (variant)
    PACKED_PIXEL_STREAM_30 = 0x0D     // Packed Pixel Stream, 30-bit
};

enum class DSIMode : u8 {
    VIDEO_MODE = 0,              // Video mode operation
    COMMAND_MODE = 1             // Command mode operation
};

enum class VideoMode : u8 {
    NON_BURST_SYNC_PULSES = 0,   // Non-burst mode with sync pulses
    NON_BURST_SYNC_EVENTS = 1,   // Non-burst mode with sync events
    BURST_MODE = 2               // Burst mode
};

enum class ColorFormat : u8 {
    RGB565 = 0,                  // 16-bit RGB565
    RGB666_PACKED = 1,           // 18-bit RGB666 packed
    RGB666_LOOSELY_PACKED = 2,   // 18-bit RGB666 loosely packed
    RGB888 = 3                   // 24-bit RGB888
};

enum class DSIInterruptType : u16 {
    ACK_WITH_ERROR = 0x0001,     // Acknowledge with error
    SYNC_TIMEOUT = 0x0002,       // Synchronization timeout
    LOW_POWER_TIMEOUT = 0x0004,  // Low power timeout
    HIGH_SPEED_TIMEOUT = 0x0008, // High speed timeout
    FALSE_CONTROL = 0x0010,      // False control error
    CONTENTION_DETECTED = 0x0020, // Contention detected
    ECC_SINGLE_BIT = 0x0040,     // ECC single bit error
    ECC_MULTI_BIT = 0x0080,      // ECC multi-bit error
    CHECKSUM_ERROR = 0x0100,     // Checksum error
    DSI_DATA_TYPE_ERROR = 0x0200, // DSI data type not recognized
    VC_ID_INVALID = 0x0400,      // Virtual channel ID invalid
    INVALID_TX_LENGTH = 0x0800,  // Invalid transmission length
    PROTOCOL_VIOLATION = 0x1000, // Protocol violation
    GENERIC_WRITE_FIFO_FULL = 0x2000, // Generic write FIFO full
    GENERIC_READ_FIFO_EMPTY = 0x4000, // Generic read FIFO empty
    HIGH_SPEED_TX_TIMEOUT = 0x8000   // High speed transmission timeout
};

struct MIPITimingConfig {
    u32 pixel_clock_khz = 74250;        // Pixel clock frequency (74.25 MHz for 1280x720@60Hz)
    u32 hs_clock_khz = 445500;          // High-speed clock frequency (445.5 MHz)
    u32 lp_clock_khz = 10000;           // Low-power clock frequency (10 MHz)
    
    // Video timing parameters (1280x720@60Hz)
    u16 h_active = 1280;                // Horizontal active pixels
    u16 h_front_porch = 110;            // Horizontal front porch
    u16 h_back_porch = 220;             // Horizontal back porch  
    u16 h_sync_width = 40;              // Horizontal sync width
    
    u16 v_active = 720;                 // Vertical active lines
    u16 v_front_porch = 5;              // Vertical front porch
    u16 v_back_porch = 20;              // Vertical back porch
    u16 v_sync_width = 5;               // Vertical sync width
    
    // DSI specific timing
    u8 hs_prepare = 6;                  // HS prepare time (ns * 4)
    u8 hs_zero = 10;                    // HS zero time (ns * 4)
    u8 hs_trail = 8;                    // HS trail time (ns * 4)
    u8 hs_exit = 7;                     // HS exit time (ns * 4)
    u8 lpx = 4;                         // Low-power transmit time (ns * 4)
    u8 clk_prepare = 4;                 // Clock prepare time (ns * 4)
    u8 clk_zero = 20;                   // Clock zero time (ns * 4)
    u8 clk_post = 10;                   // Clock post time (ns * 4)
    u8 clk_trail = 4;                   // Clock trail time (ns * 4)
};

struct DSIPacket {
    u8 data_type;                       // MIPI data type
    u8 virtual_channel;                 // Virtual channel ID (0-3)
    u16 word_count;                     // Word count for long packets
    std::vector<u8> payload;            // Packet payload
    u16 checksum;                       // Checksum for long packets
    u8 ecc;                             // ECC for header
    std::chrono::steady_clock::time_point timestamp;
};

struct DisplayStatistics {
    u64 frames_transmitted = 0;
    u64 packets_transmitted = 0;
    u64 bytes_transmitted = 0;
    u64 sync_errors = 0;
    u64 ecc_errors = 0;
    u64 checksum_errors = 0;
    u64 timeout_errors = 0;
    u64 protocol_violations = 0;
    u64 vsync_interrupts = 0;
    double average_fps = 0.0;
    double pixel_throughput_mbps = 0.0;
    u32 current_line = 0;
    bool in_blanking_period = false;
};

class MIPIDSIDisplay {
public:
    static constexpr u8 MAX_VIRTUAL_CHANNELS = 4;
    static constexpr u16 MAX_PACKET_SIZE = 65535;
    static constexpr u32 DISPLAY_WIDTH = 1280;
    static constexpr u32 DISPLAY_HEIGHT = 720;
    static constexpr u32 REFRESH_RATE_HZ = 60;
    static constexpr u32 FIFO_SIZE = 2048;
    
    MIPIDSIDisplay();
    ~MIPIDSIDisplay();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller,
                           Framebuffer* framebuffer);
    Result<void> shutdown();

    // DSI Configuration
    Result<void> configure_dsi(DSIMode mode, VideoMode video_mode, ColorFormat color_format);
    Result<void> configure_timing(const MIPITimingConfig& timing_config);
    Result<void> configure_lanes(u8 num_data_lanes); // 1, 2, or 4 lanes
    Result<void> set_virtual_channel(u8 channel_id);
    Result<void> enable_ecc_correction(bool enable);
    Result<void> enable_crc_check(bool enable);
    
    // Display Control
    Result<void> start_display();
    Result<void> stop_display();
    Result<void> enter_sleep_mode();
    Result<void> exit_sleep_mode();
    Result<void> set_display_brightness(u8 brightness); // 0-255
    Result<void> set_color_enhancement(bool enable);
    
    // Command Mode Operations  
    Result<void> send_dcs_command(u8 command, const std::vector<u8>& parameters = {});
    Result<std::vector<u8>> read_dcs_command(u8 command, size_t read_length);
    Result<void> send_generic_command(const std::vector<u8>& command);
    Result<std::vector<u8>> read_generic_command(const std::vector<u8>& command, size_t read_length);
    
    // Video Mode Operations
    Result<void> update_frame_data(const std::vector<u8>& frame_data);
    Result<void> set_frame_rate(u32 fps);
    Result<void> enable_vsync_interrupt(bool enable);
    Result<void> enable_hsync_interrupt(bool enable);
    
    // Low-level Packet Interface
    Result<void> send_short_packet(MIPIDataType data_type, u8 data0 = 0, u8 data1 = 0);
    Result<void> send_long_packet(MIPIDataType data_type, const std::vector<u8>& payload);
    Result<DSIPacket> receive_packet();
    
    // Error Handling and Recovery
    Result<void> reset_dsi_controller();
    Result<void> flush_tx_fifo();
    Result<void> flush_rx_fifo();
    Result<u16> get_error_status();
    Result<void> clear_errors();
    
    // Power Management
    Result<void> enter_ulps(); // Ultra Low Power State
    Result<void> exit_ulps();
    Result<void> set_power_mode(bool low_power);
    
    // Calibration and Test
    Result<bool> perform_phy_calibration();
    Result<bool> test_pattern_generate(u8 pattern_type);
    Result<void> enable_bist(bool enable); // Built-In Self Test
    
    // Interrupt Handling
    Result<void> enable_interrupt(DSIInterruptType interrupt_type);
    Result<void> disable_interrupt(DSIInterruptType interrupt_type);
    Result<u16> get_interrupt_status();
    Result<void> clear_interrupt(DSIInterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_display_on() const { return display_enabled_; }
    DSIMode get_dsi_mode() const { return dsi_mode_; }
    ColorFormat get_color_format() const { return color_format_; }
    const DisplayStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct DSIController {
        // Control registers
        u32 dsi_control = 0x00000001;      // DSI control register
        u32 dsi_configuration = 0x00000000; // DSI configuration
        u32 dsi_command_mode = 0x00000000;  // Command mode configuration
        u32 dsi_video_mode = 0x00000003;    // Video mode configuration
        u32 dsi_timeout1 = 0x00000000;      // Timeout register 1
        u32 dsi_timeout2 = 0x00000000;      // Timeout register 2
        
        // Clock and timing
        u32 dsi_clk_configuration = 0x00000000; // Clock configuration
        u32 dsi_phy_timing1 = 0x00000000;   // PHY timing register 1
        u32 dsi_phy_timing2 = 0x00000000;   // PHY timing register 2
        u32 dsi_video_timing1 = 0x00000000; // Video timing register 1
        u32 dsi_video_timing2 = 0x00000000; // Video timing register 2
        u32 dsi_video_timing3 = 0x00000000; // Video timing register 3
        
        // FIFO and packet handling
        u32 dsi_generic_header = 0x00000000; // Generic packet header
        u32 dsi_generic_payload = 0x00000000; // Generic packet payload
        u32 dsi_command_header = 0x00000000;  // Command packet header
        u32 dsi_command_payload = 0x00000000; // Command packet payload
        u32 dsi_rx_fifo = 0x00000000;        // RX FIFO data
        
        // Status and interrupt
        u32 dsi_status = 0x00000000;         // DSI status register
        u32 dsi_interrupt_enable = 0x00000000; // Interrupt enable
        u32 dsi_interrupt_status = 0x00000000; // Interrupt status
        u32 dsi_error_status = 0x00000000;    // Error status
        
        // PHY control  
        u32 dsi_phy_control = 0x00000000;    // PHY control register
        u32 dsi_phy_status = 0x00000001;     // PHY status register
    };

    struct VideoTiming {
        u32 total_width;                     // Total horizontal width
        u32 total_height;                    // Total vertical height
        u32 active_width;                    // Active horizontal width
        u32 active_height;                   // Active vertical height
        std::chrono::nanoseconds line_time;  // Time per line
        std::chrono::nanoseconds frame_time; // Time per frame
        std::chrono::nanoseconds pixel_time; // Time per pixel
    };

    struct PacketProcessor {
        std::queue<DSIPacket> tx_fifo;
        std::queue<DSIPacket> rx_fifo;
        std::queue<DSIPacket> pending_packets;
        DSIPacket current_packet;
        bool packet_in_progress = false;
        size_t bytes_sent = 0;
        std::chrono::steady_clock::time_point packet_start_time;
    };

    // Core display operations
    void process_display_pipeline();
    void generate_video_stream();
    void process_command_queue();
    void handle_blanking_periods();
    
    // Packet processing
    void encode_packet(DSIPacket& packet);
    void decode_packet(DSIPacket& packet);
    u8 calculate_ecc(u8 byte1, u8 byte2, u8 byte3);
    u16 calculate_crc16(const std::vector<u8>& data);
    bool verify_packet_integrity(const DSIPacket& packet);
    
    // Video mode implementation
    void send_vsync_start();
    void send_vsync_end();
    void send_hsync_start();
    void send_hsync_end();
    void send_pixel_data(const std::vector<u8>& pixel_data);
    void send_blanking_packet();
    
    // Command mode implementation
    void process_dcs_command(u8 command, const std::vector<u8>& parameters);
    void process_generic_command(const std::vector<u8>& command);
    std::vector<u8> read_display_register(u8 register_addr);
    
    // Color format conversion
    std::vector<u8> convert_rgb888_to_rgb565(const std::vector<u8>& rgb888_data);
    std::vector<u8> convert_rgb888_to_rgb666(const std::vector<u8>& rgb888_data, bool packed);
    std::vector<u8> extract_pixel_data_from_framebuffer();
    
    // Timing and synchronization
    void update_video_timing();
    void wait_for_hsync();
    void wait_for_vsync();
    bool is_in_blanking_period();
    void trigger_vsync_interrupt();
    void trigger_hsync_interrupt();
    
    // Error handling
    void handle_transmission_error(DSIInterruptType error_type);
    void recover_from_error();
    void log_protocol_violation(const std::string& violation);
    
    // PHY layer simulation
    void simulate_phy_layer();
    void calibrate_phy_timing();
    void adjust_clock_skew();
    void simulate_signal_integrity_issues();
    
    // Power management
    void enter_low_power_mode();
    void exit_low_power_mode();
    void manage_clock_gating();
    
    // Test and debug features
    void generate_test_pattern(u8 pattern_type);
    void run_built_in_self_test();
    void measure_signal_timing();
    
    void trigger_interrupt(DSIInterruptType interrupt_type);
    void update_statistics();
    
    bool initialized_;
    bool display_enabled_;
    bool in_sleep_mode_;
    bool low_power_mode_;
    
    DSIMode dsi_mode_;
    VideoMode video_mode_;
    ColorFormat color_format_;
    u8 num_data_lanes_;
    u8 virtual_channel_;
    
    DSIController controller_regs_;
    MIPITimingConfig timing_config_;
    VideoTiming video_timing_;
    PacketProcessor packet_processor_;
    
    InterruptController* interrupt_controller_;
    Framebuffer* framebuffer_;
    
    // Timing control
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_frame_start_;
    std::chrono::steady_clock::time_point last_line_start_;
    u32 current_line_;
    u32 current_pixel_;
    bool in_active_region_;
    
    DisplayStatistics statistics_;
    
    mutable std::mutex dsi_mutex_;
};

}  // namespace m5tab5::emulator