#include "emulator/peripherals/mipi_dsi_display.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>

namespace m5tab5::emulator {

DECLARE_LOGGER("MIPI_DSI_Display");

MIPIDSIDisplay::MIPIDSIDisplay()
    : initialized_(false),
      display_enabled_(false),
      in_sleep_mode_(false),
      low_power_mode_(false),
      dsi_mode_(DSIMode::VIDEO_MODE),
      video_mode_(VideoMode::NON_BURST_SYNC_PULSES),
      color_format_(ColorFormat::RGB888),
      num_data_lanes_(4),
      virtual_channel_(0),
      interrupt_controller_(nullptr),
      framebuffer_(nullptr),
      current_line_(0),
      current_pixel_(0),
      in_active_region_(false) {
    
    // Initialize timing configuration for 1280x720@60Hz
    timing_config_.pixel_clock_khz = 74250;
    timing_config_.h_active = 1280;
    timing_config_.h_front_porch = 110;
    timing_config_.h_back_porch = 220;
    timing_config_.h_sync_width = 40;
    timing_config_.v_active = 720;
    timing_config_.v_front_porch = 5;
    timing_config_.v_back_porch = 20;
    timing_config_.v_sync_width = 5;
    
    COMPONENT_LOG_DEBUG("MIPI-DSI display controller created");
}

MIPIDSIDisplay::~MIPIDSIDisplay() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("MIPI-DSI display controller destroyed");
}

Result<void> MIPIDSIDisplay::initialize(const Configuration& config,
                                       InterruptController* interrupt_controller,
                                       Framebuffer* framebuffer) {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "MIPI-DSI display already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing MIPI-DSI display controller");
    
    interrupt_controller_ = interrupt_controller;
    framebuffer_ = framebuffer;
    
    // Initialize controller registers
    controller_regs_ = {};
    controller_regs_.dsi_control = 0x00000001; // Enable DSI
    controller_regs_.dsi_phy_status = 0x00000001; // PHY ready
    
    // Calculate video timing
    update_video_timing();
    
    // Clear packet processor
    packet_processor_ = {};
    
    // Clear statistics
    statistics_ = {};
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_frame_start_ = last_update_;
    last_line_start_ = last_update_;
    
    initialized_ = true;
    COMPONENT_LOG_INFO("MIPI-DSI display controller initialized successfully");
    
    return {};
}

Result<void> MIPIDSIDisplay::shutdown() {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down MIPI-DSI display controller");
    
    // Stop display
    display_enabled_ = false;
    controller_regs_.dsi_control = 0x00000000;
    
    // Clear packet queues
    while (!packet_processor_.tx_fifo.empty()) packet_processor_.tx_fifo.pop();
    while (!packet_processor_.rx_fifo.empty()) packet_processor_.rx_fifo.pop();
    while (!packet_processor_.pending_packets.empty()) packet_processor_.pending_packets.pop();
    
    interrupt_controller_ = nullptr;
    framebuffer_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("MIPI-DSI display controller shutdown completed");
    return {};
}

Result<void> MIPIDSIDisplay::configure_dsi(DSIMode mode, VideoMode video_mode, ColorFormat color_format) {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "MIPI-DSI display not initialized"));
    }
    
    dsi_mode_ = mode;
    video_mode_ = video_mode;
    color_format_ = color_format;
    
    // Update configuration registers
    controller_regs_.dsi_configuration = 0x00000000;
    controller_regs_.dsi_configuration |= (static_cast<u32>(mode) << 0);
    controller_regs_.dsi_configuration |= (static_cast<u32>(video_mode) << 2);
    controller_regs_.dsi_configuration |= (static_cast<u32>(color_format) << 4);
    controller_regs_.dsi_configuration |= (num_data_lanes_ << 8);
    
    COMPONENT_LOG_INFO("MIPI-DSI configured: mode={}, video_mode={}, color_format={}",
                      static_cast<u8>(mode), static_cast<u8>(video_mode), static_cast<u8>(color_format));
    
    return {};
}

Result<void> MIPIDSIDisplay::start_display() {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "MIPI-DSI display not initialized"));
    }
    
    display_enabled_ = true;
    in_sleep_mode_ = false;
    controller_regs_.dsi_control |= 0x00000002; // Enable display
    controller_regs_.dsi_status |= 0x00000001; // Display on
    
    COMPONENT_LOG_INFO("MIPI-DSI display started");
    return {};
}

Result<void> MIPIDSIDisplay::stop_display() {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "MIPI-DSI display not initialized"));
    }
    
    display_enabled_ = false;
    controller_regs_.dsi_control &= ~0x00000002; // Disable display
    controller_regs_.dsi_status &= ~0x00000001; // Display off
    
    COMPONENT_LOG_INFO("MIPI-DSI display stopped");
    return {};
}

Result<void> MIPIDSIDisplay::send_dcs_command(u8 command, const std::vector<u8>& parameters) {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "MIPI-DSI display not initialized"));
    }
    
    DSIPacket packet;
    packet.virtual_channel = virtual_channel_;
    packet.timestamp = std::chrono::steady_clock::now();
    
    if (parameters.empty()) {
        packet.data_type = static_cast<u8>(MIPIDataType::DCS_SHORT_WRITE_0);
        packet.word_count = 1;
        packet.payload = {command};
    } else if (parameters.size() == 1) {
        packet.data_type = static_cast<u8>(MIPIDataType::DCS_SHORT_WRITE_1);
        packet.word_count = 2;
        packet.payload = {command, parameters[0]};
    } else {
        packet.data_type = static_cast<u8>(MIPIDataType::DCS_LONG_WRITE);
        packet.word_count = static_cast<u16>(1 + parameters.size());
        packet.payload = {command};
        packet.payload.insert(packet.payload.end(), parameters.begin(), parameters.end());
    }
    
    encode_packet(packet);
    
    if (packet_processor_.tx_fifo.size() >= FIFO_SIZE) {
        return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
            "DSI TX FIFO full"));
    }
    
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
    
    return {};
}

void MIPIDSIDisplay::update() {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);
    last_update_ = now;
    
    if (display_enabled_) {
        process_display_pipeline();
    }
    
    update_statistics();
}

void MIPIDSIDisplay::process_display_pipeline() {
    if (dsi_mode_ == DSIMode::VIDEO_MODE) {
        generate_video_stream();
    } else {
        process_command_queue();
    }
    
    handle_blanking_periods();
}

void MIPIDSIDisplay::generate_video_stream() {
    auto now = std::chrono::steady_clock::now();
    
    // Check if it's time for next line
    auto line_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_line_start_);
    if (line_elapsed >= video_timing_.line_time) {
        // Start new line
        current_line_++;
        current_pixel_ = 0;
        last_line_start_ = now;
        
        if (current_line_ >= video_timing_.total_height) {
            // Start new frame
            current_line_ = 0;
            last_frame_start_ = now;
            statistics_.frames_transmitted++;
            trigger_vsync_interrupt();
        }
        
        // Send sync packets
        if (current_line_ == 0) {
            send_vsync_start();
        } else if (current_line_ == timing_config_.v_sync_width) {
            send_vsync_end();
        }
        
        send_hsync_start();
        
        // Check if in active region
        in_active_region_ = (current_line_ >= timing_config_.v_sync_width + timing_config_.v_back_porch) &&
                           (current_line_ < timing_config_.v_sync_width + timing_config_.v_back_porch + timing_config_.v_active);
        
        if (in_active_region_ && framebuffer_) {
            // Send pixel data for this line
            std::vector<u8> pixel_data = extract_pixel_data_from_framebuffer();
            if (!pixel_data.empty()) {
                send_pixel_data(pixel_data);
            }
        }
        
        send_hsync_end();
    }
}

void MIPIDSIDisplay::send_vsync_start() {
    DSIPacket packet;
    packet.data_type = static_cast<u8>(MIPIDataType::VSYNC_START);
    packet.virtual_channel = virtual_channel_;
    packet.word_count = 0;
    packet.timestamp = std::chrono::steady_clock::now();
    
    encode_packet(packet);
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
}

void MIPIDSIDisplay::send_vsync_end() {
    DSIPacket packet;
    packet.data_type = static_cast<u8>(MIPIDataType::VSYNC_END);
    packet.virtual_channel = virtual_channel_;
    packet.word_count = 0;
    packet.timestamp = std::chrono::steady_clock::now();
    
    encode_packet(packet);
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
}

void MIPIDSIDisplay::send_hsync_start() {
    DSIPacket packet;
    packet.data_type = static_cast<u8>(MIPIDataType::HSYNC_START);
    packet.virtual_channel = virtual_channel_;
    packet.word_count = 0;
    packet.timestamp = std::chrono::steady_clock::now();
    
    encode_packet(packet);
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
}

void MIPIDSIDisplay::send_hsync_end() {
    DSIPacket packet;
    packet.data_type = static_cast<u8>(MIPIDataType::HSYNC_END);
    packet.virtual_channel = virtual_channel_;
    packet.word_count = 0;
    packet.timestamp = std::chrono::steady_clock::now();
    
    encode_packet(packet);
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
}

void MIPIDSIDisplay::send_pixel_data(const std::vector<u8>& pixel_data) {
    DSIPacket packet;
    packet.virtual_channel = virtual_channel_;
    packet.payload = pixel_data;
    packet.word_count = static_cast<u16>(pixel_data.size());
    packet.timestamp = std::chrono::steady_clock::now();
    
    // Select packet type based on color format
    switch (color_format_) {
        case ColorFormat::RGB565:
            packet.data_type = static_cast<u8>(MIPIDataType::PACKED_PIXEL_STREAM_16);
            break;
        case ColorFormat::RGB666_PACKED:
        case ColorFormat::RGB666_LOOSELY_PACKED:
            packet.data_type = static_cast<u8>(MIPIDataType::PACKED_PIXEL_STREAM_18);
            break;
        case ColorFormat::RGB888:
            packet.data_type = static_cast<u8>(MIPIDataType::PACKED_PIXEL_STREAM_24);
            break;
    }
    
    encode_packet(packet);
    packet_processor_.tx_fifo.push(packet);
    statistics_.packets_transmitted++;
    statistics_.bytes_transmitted += pixel_data.size();
}

std::vector<u8> MIPIDSIDisplay::extract_pixel_data_from_framebuffer() {
    if (!framebuffer_ || !in_active_region_) {
        return {};
    }
    
    // Extract one line of pixel data from framebuffer
    u32 line_offset = current_line_ - (timing_config_.v_sync_width + timing_config_.v_back_porch);
    if (line_offset >= timing_config_.v_active) {
        return {};
    }
    
    std::vector<u8> pixel_data;
    
    // Simple RGB888 extraction (would need actual framebuffer interface)
    size_t bytes_per_pixel = 3; // RGB888
    size_t line_size = timing_config_.h_active * bytes_per_pixel;
    
    pixel_data.reserve(line_size);
    
    // Generate sample pixel data (would read from actual framebuffer)
    for (u32 x = 0; x < timing_config_.h_active; ++x) {
        // Simple test pattern
        u8 red = static_cast<u8>(x % 256);
        u8 green = static_cast<u8>(line_offset % 256);
        u8 blue = static_cast<u8>((x + line_offset) % 256);
        
        pixel_data.push_back(red);
        pixel_data.push_back(green);
        pixel_data.push_back(blue);
    }
    
    return pixel_data;
}

void MIPIDSIDisplay::update_video_timing() {
    video_timing_.total_width = timing_config_.h_active + timing_config_.h_front_porch + 
                                timing_config_.h_back_porch + timing_config_.h_sync_width;
    video_timing_.total_height = timing_config_.v_active + timing_config_.v_front_porch +
                                 timing_config_.v_back_porch + timing_config_.v_sync_width;
    video_timing_.active_width = timing_config_.h_active;
    video_timing_.active_height = timing_config_.v_active;
    
    // Calculate timing based on pixel clock
    u64 pixel_period_ns = 1000000000ULL / (timing_config_.pixel_clock_khz * 1000ULL);
    video_timing_.pixel_time = std::chrono::nanoseconds(pixel_period_ns);
    video_timing_.line_time = std::chrono::nanoseconds(pixel_period_ns * video_timing_.total_width);
    video_timing_.frame_time = std::chrono::nanoseconds(pixel_period_ns * video_timing_.total_width * video_timing_.total_height);
}

void MIPIDSIDisplay::encode_packet(DSIPacket& packet) {
    // Calculate ECC for packet header
    u8 header_byte1 = packet.data_type | (packet.virtual_channel << 6);
    u8 header_byte2 = packet.word_count & 0xFF;
    u8 header_byte3 = (packet.word_count >> 8) & 0xFF;
    
    packet.ecc = calculate_ecc(header_byte1, header_byte2, header_byte3);
    
    // Calculate CRC16 for long packets
    if (packet.word_count > 0) {
        packet.checksum = calculate_crc16(packet.payload);
    }
}

u8 MIPIDSIDisplay::calculate_ecc(u8 byte1, u8 byte2, u8 byte3) {
    // Simplified ECC calculation (would use proper Hamming code in real implementation)
    return byte1 ^ byte2 ^ byte3;
}

u16 MIPIDSIDisplay::calculate_crc16(const std::vector<u8>& data) {
    // Simplified CRC16 calculation
    u16 crc = 0xFFFF;
    for (u8 byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void MIPIDSIDisplay::process_command_queue() {
    // Process pending command mode packets
    while (!packet_processor_.tx_fifo.empty()) {
        DSIPacket packet = packet_processor_.tx_fifo.front();
        packet_processor_.tx_fifo.pop();
        
        // Process command (simplified)
        if (packet.data_type == static_cast<u8>(MIPIDataType::DCS_SHORT_WRITE_0) ||
            packet.data_type == static_cast<u8>(MIPIDataType::DCS_SHORT_WRITE_1) ||
            packet.data_type == static_cast<u8>(MIPIDataType::DCS_LONG_WRITE)) {
            
            if (!packet.payload.empty()) {
                process_dcs_command(packet.payload[0], 
                    std::vector<u8>(packet.payload.begin() + 1, packet.payload.end()));
            }
        }
    }
}

void MIPIDSIDisplay::process_dcs_command(u8 command, const std::vector<u8>& parameters) {
    // Process common DCS commands
    switch (command) {
        case 0x10: // Sleep In
            in_sleep_mode_ = true;
            display_enabled_ = false;
            break;
        case 0x11: // Sleep Out
            in_sleep_mode_ = false;
            break;
        case 0x28: // Display Off
            display_enabled_ = false;
            break;
        case 0x29: // Display On
            display_enabled_ = true;
            in_sleep_mode_ = false;
            break;
        case 0x51: // Write Display Brightness
            if (!parameters.empty()) {
                // Set brightness (simplified)
                COMPONENT_LOG_DEBUG("MIPI-DSI brightness set to {}", parameters[0]);
            }
            break;
        default:
            COMPONENT_LOG_DEBUG("MIPI-DSI unknown DCS command: 0x{:02X}", command);
            break;
    }
}

void MIPIDSIDisplay::handle_blanking_periods() {
    statistics_.in_blanking_period = !in_active_region_;
    statistics_.current_line = current_line_;
}

void MIPIDSIDisplay::trigger_vsync_interrupt() {
    if (interrupt_controller_) {
        interrupt_controller_->trigger_interrupt(InterruptType::DISPLAY_VSYNC);
    }
    statistics_.vsync_interrupts++;
}

void MIPIDSIDisplay::trigger_interrupt(DSIInterruptType interrupt_type) {
    if (interrupt_controller_) {
        interrupt_controller_->trigger_interrupt(InterruptType::DISPLAY_ERROR);
    }
    controller_regs_.dsi_interrupt_status |= static_cast<u32>(interrupt_type);
}

void MIPIDSIDisplay::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    auto frame_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_start_);
    
    if (frame_elapsed.count() > 0) {
        statistics_.average_fps = 1000.0 / frame_elapsed.count();
    }
    
    // Calculate pixel throughput
    if (statistics_.bytes_transmitted > 0) {
        auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - std::chrono::steady_clock::time_point{});
        if (total_elapsed.count() > 0) {
            statistics_.pixel_throughput_mbps = 
                (statistics_.bytes_transmitted * 8.0) / (total_elapsed.count() * 1000000.0);
        }
    }
}

void MIPIDSIDisplay::clear_statistics() {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    statistics_ = {};
}

void MIPIDSIDisplay::dump_status() const {
    std::lock_guard<std::mutex> lock(dsi_mutex_);
    
    COMPONENT_LOG_INFO("=== MIPI-DSI Display Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Display enabled: {}", display_enabled_);
        COMPONENT_LOG_INFO("Sleep mode: {}", in_sleep_mode_);
        COMPONENT_LOG_INFO("DSI mode: {}", static_cast<u8>(dsi_mode_));
        COMPONENT_LOG_INFO("Video mode: {}", static_cast<u8>(video_mode_));
        COMPONENT_LOG_INFO("Color format: {}", static_cast<u8>(color_format_));
        COMPONENT_LOG_INFO("Data lanes: {}", num_data_lanes_);
        COMPONENT_LOG_INFO("Virtual channel: {}", virtual_channel_);
        
        COMPONENT_LOG_INFO("Timing: {}x{}@{} Hz", 
                          timing_config_.h_active, timing_config_.v_active,
                          1000000000ULL / video_timing_.frame_time.count());
        
        COMPONENT_LOG_INFO("Current position: line {}, pixel {}", current_line_, current_pixel_);
        COMPONENT_LOG_INFO("In active region: {}", in_active_region_);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Frames transmitted: {}", statistics_.frames_transmitted);
        COMPONENT_LOG_INFO("  Packets transmitted: {}", statistics_.packets_transmitted);
        COMPONENT_LOG_INFO("  Bytes transmitted: {}", statistics_.bytes_transmitted);
        COMPONENT_LOG_INFO("  Average FPS: {:.1f}", statistics_.average_fps);
        COMPONENT_LOG_INFO("  Pixel throughput: {:.1f} Mbps", statistics_.pixel_throughput_mbps);
        COMPONENT_LOG_INFO("  VSYNC interrupts: {}", statistics_.vsync_interrupts);
        COMPONENT_LOG_INFO("  Errors: sync={}, ecc={}, checksum={}", 
                          statistics_.sync_errors, statistics_.ecc_errors, statistics_.checksum_errors);
    }
}

}  // namespace m5tab5::emulator