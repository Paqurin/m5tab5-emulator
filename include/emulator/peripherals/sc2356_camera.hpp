#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/peripherals/gpio_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <functional>
#include <array>

namespace m5tab5::emulator {

enum class CameraInterface : u8 {
    MIPI_CSI = 0,
    DVP = 1
};

enum class CameraResolution : u8 {
    QVGA_320x240 = 0,    // 320x240
    VGA_640x480 = 1,     // 640x480
    SVGA_800x600 = 2,    // 800x600
    XGA_1024x768 = 3,    // 1024x768
    HD_1280x720 = 4,     // 1280x720 (720p)
    SXGA_1280x1024 = 5,  // 1280x1024
    UXGA_1600x1200 = 6,  // 1600x1200
    FHD_1920x1080 = 7,   // 1920x1080 (1080p)
    QXGA_2048x1536 = 8,  // 2048x1536
    QSXGA_2592x1944 = 9  // 2592x1944 (5MP)
};

enum class CameraPixelFormat : u8 {
    RGB565 = 0,
    RGB888 = 1,
    YUV422 = 2,
    YUV420 = 3,
    GRAYSCALE = 4,
    BAYER_RGGB = 5,
    BAYER_GRBG = 6,
    BAYER_GBRG = 7,
    BAYER_BGGR = 8,
    JPEG = 9
};

enum class CameraFrameRate : u8 {
    FPS_5 = 0,
    FPS_10 = 1,
    FPS_15 = 2,
    FPS_20 = 3,
    FPS_25 = 4,
    FPS_30 = 5,
    FPS_60 = 6,
    FPS_120 = 7
};

enum class CameraExposureMode : u8 {
    AUTO = 0,
    MANUAL = 1,
    APERTURE_PRIORITY = 2,
    SHUTTER_PRIORITY = 3
};

enum class CameraWhiteBalance : u8 {
    AUTO = 0,
    SUNNY = 1,
    CLOUDY = 2,
    OFFICE = 3,
    HOME = 4,
    NIGHT = 5,
    CUSTOM = 6
};

enum class CameraEffect : u8 {
    NONE = 0,
    NEGATIVE = 1,
    GRAYSCALE = 2,
    SEPIA = 3,
    VINTAGE = 4,
    EMBOSS = 5,
    SKETCH = 6,
    BLUE_TINT = 7,
    GREEN_TINT = 8,
    RED_TINT = 9
};

enum class CameraInterruptType : u8 {
    FRAME_DONE = 0x01,
    FRAME_ERROR = 0x02,
    FIFO_OVERFLOW = 0x04,
    EXPOSURE_DONE = 0x08,
    FOCUS_DONE = 0x10,
    MOTION_DETECT = 0x20,
    LINE_COUNT = 0x40,
    VSYNC = 0x80
};

struct CameraFrame {
    std::vector<u8> data;
    CameraResolution resolution;
    CameraPixelFormat format;
    u32 width;
    u32 height;
    u32 bytes_per_pixel;
    std::chrono::steady_clock::time_point timestamp;
    u32 frame_number;
    bool is_valid;
};

struct CameraConfig {
    CameraInterface interface = CameraInterface::MIPI_CSI;
    CameraResolution resolution = CameraResolution::VGA_640x480;
    CameraPixelFormat format = CameraPixelFormat::RGB565;
    CameraFrameRate frame_rate = CameraFrameRate::FPS_30;
    CameraExposureMode exposure_mode = CameraExposureMode::AUTO;
    CameraWhiteBalance white_balance = CameraWhiteBalance::AUTO;
    CameraEffect effect = CameraEffect::NONE;
    
    // Image quality settings
    u8 brightness = 128;        // 0-255
    u8 contrast = 128;          // 0-255
    u8 saturation = 128;        // 0-255
    i8 sharpness = 0;          // -100 to +100
    u8 quality = 85;            // JPEG quality 1-100
    
    // Exposure settings
    u16 exposure_time_ms = 33;  // Manual exposure time
    u16 iso_sensitivity = 100;  // ISO 100-3200
    bool auto_exposure = true;
    bool auto_white_balance = true;
    
    // Focus settings
    bool auto_focus = true;
    u16 focus_position = 50;    // Manual focus 0-100
    
    // Advanced settings
    bool horizontal_mirror = false;
    bool vertical_flip = false;
    bool noise_reduction = true;
    bool motion_detection = false;
    u8 motion_threshold = 50;   // Motion sensitivity 0-100
    
    // Buffer settings
    u8 frame_buffer_count = 3;
    bool continuous_capture = false;
};

struct CameraStatistics {
    u64 frames_captured = 0;
    u64 frames_dropped = 0;
    u64 bytes_transferred = 0;
    u64 buffer_overflows = 0;
    u64 exposure_adjustments = 0;
    u64 focus_adjustments = 0;
    u64 white_balance_adjustments = 0;
    u64 motion_events_detected = 0;
    double average_frame_rate = 0.0;
    double current_exposure_time_ms = 0.0;
    u16 current_iso = 0;
    u8 current_focus_position = 0;
    float image_brightness_level = 0.0f;
    float focus_quality_score = 0.0f;
};

using CameraFrameCallback = std::function<void(const CameraFrame& frame)>;
using CameraErrorCallback = std::function<void(const std::string& error_message)>;

class SC2356Camera {
public:
    static constexpr u8 I2C_ADDRESS = 0x30;
    static constexpr u16 CHIP_ID = 0x2356;
    static constexpr size_t MAX_FRAME_BUFFER_COUNT = 8;
    static constexpr size_t MIN_FRAME_BUFFER_COUNT = 2;
    static constexpr size_t FIFO_SIZE = 4096; // Frame metadata FIFO
    
    SC2356Camera();
    ~SC2356Camera();

    Result<void> initialize(const Configuration& config,
                           InterruptController* interrupt_controller,
                           I2CController* i2c_controller,
                           GPIOController* gpio_controller);
    Result<void> shutdown();

    // Configuration
    Result<void> configure_camera(const CameraConfig& config);
    Result<CameraConfig> get_camera_config() const;
    Result<void> set_resolution(CameraResolution resolution);
    Result<void> set_pixel_format(CameraPixelFormat format);
    Result<void> set_frame_rate(CameraFrameRate frame_rate);
    
    // Image quality controls
    Result<void> set_brightness(u8 brightness);
    Result<void> set_contrast(u8 contrast);
    Result<void> set_saturation(u8 saturation);
    Result<void> set_sharpness(i8 sharpness);
    Result<void> apply_effect(CameraEffect effect);
    
    // Exposure and focus
    Result<void> set_exposure_mode(CameraExposureMode mode);
    Result<void> set_manual_exposure(u16 exposure_time_ms, u16 iso);
    Result<void> enable_auto_exposure(bool enable);
    Result<void> set_white_balance(CameraWhiteBalance wb);
    Result<void> enable_auto_focus(bool enable);
    Result<void> set_manual_focus(u16 position);
    Result<void> trigger_auto_focus();
    
    // Capture control
    Result<void> start_preview();
    Result<void> stop_preview();
    Result<void> capture_frame();
    Result<void> start_continuous_capture();
    Result<void> stop_continuous_capture();
    
    // Frame access
    Result<CameraFrame> get_latest_frame();
    Result<std::vector<CameraFrame>> get_frame_buffer();
    Result<void> release_frame(u32 frame_number);
    
    // Callback interface
    Result<void> set_frame_callback(CameraFrameCallback callback);
    Result<void> set_error_callback(CameraErrorCallback callback);
    
    // Motion detection
    Result<void> enable_motion_detection(bool enable, u8 threshold = 50);
    Result<bool> is_motion_detected() const;
    Result<void> clear_motion_detection();
    
    // Image processing
    Result<void> enable_noise_reduction(bool enable);
    Result<void> set_mirror_flip(bool horizontal_mirror, bool vertical_flip);
    
    // Advanced features
    Result<void> enable_histogram_equalization(bool enable);
    Result<void> set_roi(u16 x, u16 y, u16 width, u16 height); // Region of Interest
    Result<void> enable_image_stabilization(bool enable);
    
    // Interrupts
    Result<void> enable_interrupt(CameraInterruptType interrupt_type, bool enable);
    Result<u32> get_interrupt_status();
    Result<void> clear_interrupt(CameraInterruptType interrupt_type);
    
    // Register interface
    Result<void> write_register(u16 reg_addr, u8 value);
    Result<u8> read_register(u16 reg_addr);
    Result<void> write_registers(u16 reg_addr, const std::vector<u8>& data);
    Result<std::vector<u8>> read_registers(u16 reg_addr, size_t count);
    
    // Self-test and diagnostics
    Result<bool> perform_self_test();
    Result<void> reset_sensor();
    Result<u16> get_chip_id();
    Result<void> calibrate_lens();
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_capturing() const { return capturing_active_; }
    const CameraStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct SC2356Registers {
        u16 chip_id = CHIP_ID;
        u8 mode_ctrl = 0x00;        // Standby/streaming mode
        u8 image_orientation = 0x00; // Mirror/flip settings
        u8 software_reset = 0x00;
        u8 exposure_h = 0x00;       // Exposure time high byte
        u8 exposure_l = 0x20;       // Exposure time low byte
        u8 analog_gain = 0x10;      // Analog gain control
        u8 digital_gain_h = 0x01;   // Digital gain high
        u8 digital_gain_l = 0x00;   // Digital gain low
        u8 test_pattern = 0x00;     // Test pattern control
        u8 awb_ctrl = 0x00;         // Auto white balance control
        u8 awb_r_gain_h = 0x04;     // R gain for AWB
        u8 awb_r_gain_l = 0x00;
        u8 awb_g_gain_h = 0x04;     // G gain for AWB
        u8 awb_g_gain_l = 0x00;
        u8 awb_b_gain_h = 0x04;     // B gain for AWB
        u8 awb_b_gain_l = 0x00;
        u8 blc_ctrl = 0x00;         // Black level calibration
        u8 frame_rate_ctrl = 0x00;  // Frame rate control
        u8 interrupt_enable = 0x00;
        u8 interrupt_status = 0x00;
        u8 fifo_ctrl = 0x00;
        u8 fifo_status = 0x00;
    };

    struct ImageProcessor {
        bool noise_reduction_enabled = true;
        bool histogram_eq_enabled = false;
        bool image_stabilization_enabled = false;
        CameraEffect current_effect = CameraEffect::NONE;
        
        // ROI settings
        u16 roi_x = 0, roi_y = 0;
        u16 roi_width = 640, roi_height = 480;
        bool roi_enabled = false;
        
        // Auto-exposure state
        float current_brightness = 0.5f;
        float target_brightness = 0.5f;
        u16 current_exposure_ms = 33;
        u16 current_iso = 100;
        
        // Auto-focus state
        u16 current_focus_pos = 50;
        float focus_quality = 0.8f;
        bool focus_searching = false;
        
        // White balance state
        float r_gain = 1.0f, g_gain = 1.0f, b_gain = 1.0f;
        
        // Motion detection
        std::vector<u8> previous_frame;
        float motion_level = 0.0f;
        bool motion_detected = false;
    };

    void simulate_camera_capture();
    void process_frame(CameraFrame& frame);
    void apply_image_effects(CameraFrame& frame);
    void update_auto_exposure(const CameraFrame& frame);
    void update_auto_focus();
    void update_auto_white_balance(const CameraFrame& frame);
    void detect_motion(const CameraFrame& frame);
    void trigger_interrupt(CameraInterruptType interrupt_type);
    
    // Image processing algorithms
    void apply_brightness_contrast(std::vector<u8>& data, u8 brightness, u8 contrast);
    void apply_saturation(std::vector<u8>& data, u8 saturation);
    void apply_sharpness(std::vector<u8>& data, u32 width, u32 height, i8 sharpness);
    void apply_noise_reduction(std::vector<u8>& data, u32 width, u32 height);
    void apply_histogram_equalization(std::vector<u8>& data);
    void apply_color_effect(std::vector<u8>& data, CameraEffect effect);
    
    // Format conversion utilities
    std::vector<u8> convert_to_rgb565(const std::vector<u8>& rgb888_data);
    std::vector<u8> convert_to_yuv422(const std::vector<u8>& rgb888_data);
    std::vector<u8> convert_to_grayscale(const std::vector<u8>& rgb_data);
    std::vector<u8> simulate_jpeg_compression(const std::vector<u8>& rgb_data, u8 quality);
    
    // Auto algorithms
    float calculate_image_brightness(const std::vector<u8>& data);
    float calculate_focus_quality(const std::vector<u8>& data, u32 width, u32 height);
    void calculate_white_balance_gains(const std::vector<u8>& data, 
                                     float& r_gain, float& g_gain, float& b_gain);
    
    // Utility functions
    void get_resolution_dimensions(CameraResolution resolution, u32& width, u32& height);
    u32 get_bytes_per_pixel(CameraPixelFormat format);
    u32 calculate_frame_rate_delay_ms(CameraFrameRate frame_rate);
    std::vector<u8> generate_test_pattern(u32 width, u32 height, CameraPixelFormat format);
    
    // Communication helpers
    Result<void> write_reg_i2c(u16 reg_addr, u8 value);
    Result<u8> read_reg_i2c(u16 reg_addr);
    
    bool initialized_;
    CameraConfig config_;
    bool capturing_active_;
    bool preview_active_;
    bool continuous_capture_;
    
    SC2356Registers registers_;
    InterruptController* interrupt_controller_;
    I2CController* i2c_controller_;
    GPIOController* gpio_controller_;
    
    // Frame management
    std::queue<CameraFrame> frame_buffer_;
    u32 next_frame_number_;
    std::chrono::steady_clock::time_point last_capture_time_;
    
    // Callback interface
    CameraFrameCallback frame_callback_;
    CameraErrorCallback error_callback_;
    
    // Image processing state
    ImageProcessor processor_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    u32 interrupt_status_;
    
    CameraStatistics statistics_;
    
    mutable std::mutex camera_mutex_;
};

}  // namespace m5tab5::emulator