#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/graphics/touch_input.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <atomic>
#include <array>

namespace m5tab5::emulator {

enum class TouchResolution : u16 {
    RES_1280x720 = 0,    // M5Stack Tab5 native resolution
    RES_1024x600 = 1,    // Alternative resolution
    RES_800x480 = 2,     // Lower resolution mode
    RES_CUSTOM = 255
};

enum class TouchSensitivity : u8 {
    VERY_LOW = 1,
    LOW = 2,
    NORMAL = 3,
    HIGH = 4,
    VERY_HIGH = 5
};

enum class TouchGestureType : u8 {
    NONE = 0,
    SINGLE_TAP = 1,
    DOUBLE_TAP = 2,
    LONG_PRESS = 3,
    SWIPE_UP = 4,
    SWIPE_DOWN = 5,
    SWIPE_LEFT = 6,
    SWIPE_RIGHT = 7,
    PINCH_IN = 8,
    PINCH_OUT = 9,
    ROTATE_CW = 10,
    ROTATE_CCW = 11,
    TWO_FINGER_TAP = 12,
    THREE_FINGER_TAP = 13,
    PALM_REJECTION = 14
};

enum class GT911InterruptType : u8 {
    TOUCH_EVENT = 0x01,
    GESTURE_EVENT = 0x02,
    PROXIMITY_EVENT = 0x04,
    LARGE_TOUCH_EVENT = 0x08,
    CONFIG_UPDATE = 0x10,
    ERROR_EVENT = 0x20,
    CALIBRATION_COMPLETE = 0x40,
    FIRMWARE_UPDATE = 0x80
};

struct TouchConfiguration {
    TouchResolution resolution = TouchResolution::RES_1280x720;
    TouchSensitivity sensitivity = TouchSensitivity::NORMAL;
    u8 max_touch_points = 5;                    // GT911 supports up to 5 points
    u16 touch_threshold = 30;                   // Touch detection threshold
    u16 leave_threshold = 20;                   // Touch release threshold
    u8 noise_reduction = 3;                     // Noise filtering level (0-7)
    bool gesture_recognition = true;            // Enable gesture detection
    bool palm_rejection = true;                 // Enable palm rejection
    bool proximity_detection = false;           // Enable proximity sensing
    u16 debounce_time_ms = 10;                 // Touch debounce time
    u16 gesture_timeout_ms = 500;              // Gesture recognition timeout
};

struct TouchCalibration {
    u16 x_min = 0, x_max = 1279;              // X-axis calibration
    u16 y_min = 0, y_max = 719;               // Y-axis calibration
    i16 x_offset = 0, y_offset = 0;           // Coordinate offsets
    float x_scale = 1.0f, y_scale = 1.0f;    // Scaling factors
    u8 rotation = 0;                          // Screen rotation (0, 90, 180, 270)
    bool flip_x = false, flip_y = false;      // Axis flipping
    u8 pressure_sensitivity = 128;           // Pressure sensitivity (0-255)
};

struct TouchGesture {
    TouchGestureType type = TouchGestureType::NONE;
    u16 start_x = 0, start_y = 0;             // Gesture start coordinates
    u16 end_x = 0, end_y = 0;                 // Gesture end coordinates
    u16 duration_ms = 0;                      // Gesture duration
    float velocity = 0.0f;                    // Gesture velocity (pixels/ms)
    float angle = 0.0f;                       // Gesture angle (degrees)
    u8 finger_count = 1;                      // Number of fingers
    std::chrono::steady_clock::time_point timestamp;
    bool completed = false;
};

struct TouchPointData {
    u8 point_id = 0;                          // Touch point ID (0-4)
    u16 x = 0, y = 0;                         // Touch coordinates
    u16 pressure = 0;                         // Pressure level (0-1023)
    u8 size = 0;                              // Touch area size (0-255)
    bool active = false;                      // Touch point active status
    std::chrono::steady_clock::time_point timestamp;
    std::chrono::steady_clock::time_point first_contact;
    u16 raw_x = 0, raw_y = 0;                // Raw uncalibrated coordinates
};

struct TouchStatistics {
    u64 total_touches = 0;
    u64 gestures_detected = 0;
    u64 palm_rejections = 0;
    u64 false_positives = 0;
    u64 calibration_events = 0;
    u64 proximity_events = 0;
    double average_touch_duration_ms = 0.0;
    double average_touch_pressure = 0.0;
    float touch_accuracy_percentage = 100.0f;
    u32 max_simultaneous_touches = 0;
};

class GT911_Touch {
public:
    static constexpr u8 I2C_ADDRESS_PRIMARY = 0x5D;   // Primary I2C address
    static constexpr u8 I2C_ADDRESS_SECONDARY = 0x14; // Secondary I2C address
    static constexpr u16 PRODUCT_ID = 0x0911;         // GT911 product ID
    static constexpr u8 MAX_TOUCH_POINTS = 5;         // Maximum simultaneous touches
    static constexpr u16 TOUCH_BUFFER_SIZE = 64;      // Touch event buffer size
    static constexpr u16 DEFAULT_WIDTH = 1280;        // Default screen width
    static constexpr u16 DEFAULT_HEIGHT = 720;        // Default screen height
    
    GT911_Touch();
    ~GT911_Touch();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller,
                           I2CController* i2c_controller);
    Result<void> shutdown();

    // Configuration
    Result<void> configure_touch(const TouchConfiguration& touch_config);
    Result<void> set_resolution(TouchResolution resolution, u16 custom_width = 0, u16 custom_height = 0);
    Result<void> set_sensitivity(TouchSensitivity sensitivity);
    Result<void> set_noise_reduction(u8 level);
    Result<void> enable_gesture_recognition(bool enable);
    Result<void> enable_palm_rejection(bool enable);
    Result<void> enable_proximity_detection(bool enable);
    
    // Calibration
    Result<void> start_calibration();
    Result<bool> is_calibration_complete() const;
    Result<void> apply_calibration(const TouchCalibration& calibration);
    Result<TouchCalibration> get_calibration() const;
    Result<void> save_calibration_to_flash();
    Result<void> load_calibration_from_flash();
    Result<void> reset_calibration();
    
    // Touch input processing
    Result<std::vector<TouchPointData>> read_touch_points();
    Result<TouchGesture> read_latest_gesture();
    Result<std::vector<TouchGesture>> get_gesture_history();
    Result<void> clear_gesture_history();
    
    // Touch simulation (for testing)
    Result<void> simulate_touch(u16 x, u16 y, bool pressed, u8 point_id = 0);
    Result<void> simulate_multi_touch(const std::vector<TouchPoint>& points);
    Result<void> simulate_gesture(TouchGestureType gesture_type, 
                                  u16 start_x, u16 start_y, 
                                  u16 end_x = 0, u16 end_y = 0);
    
    // Advanced features
    Result<void> set_touch_threshold(u16 touch_threshold, u16 leave_threshold);
    Result<void> configure_debounce(u16 debounce_time_ms);
    Result<bool> is_proximity_detected() const;
    Result<float> get_proximity_distance() const;
    Result<void> set_rotation(u8 rotation_degrees);
    Result<void> set_coordinate_flip(bool flip_x, bool flip_y);
    
    // Firmware and configuration
    Result<u16> get_product_id() const;
    Result<u16> get_firmware_version() const;
    Result<void> update_firmware(const std::vector<u8>& firmware_data);
    Result<void> reset_controller();
    Result<bool> perform_self_test();
    
    // I2C register interface
    Result<void> write_register(u16 reg_addr, u8 value);
    Result<u8> read_register(u16 reg_addr) const;
    Result<void> write_registers(u16 reg_addr, const std::vector<u8>& data);
    Result<std::vector<u8>> read_registers(u16 reg_addr, size_t count);
    
    // Interrupt handling
    Result<void> enable_interrupt(GT911InterruptType interrupt_type);
    Result<void> disable_interrupt(GT911InterruptType interrupt_type);
    Result<u8> get_interrupt_status();
    Result<void> clear_interrupt(GT911InterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool has_touch_data() const { return !touch_buffer_.empty(); }
    u8 get_active_touch_count() const { return active_touch_count_; }
    const TouchStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct GT911Registers {
        // Configuration registers (0x8047-0x80FF)
        u8 config_version = 0x60;         // 0x8047 - Configuration version
        u16 x_resolution = DEFAULT_WIDTH; // 0x8048-0x8049 - X resolution
        u16 y_resolution = DEFAULT_HEIGHT;// 0x804A-0x804B - Y resolution  
        u8 max_touch_num = 5;             // 0x804C - Maximum touch points
        u8 module_switch1 = 0x00;         // 0x804D - Module switch 1
        u8 module_switch2 = 0x00;         // 0x804E - Module switch 2
        u8 shake_count = 0x05;            // 0x804F - Shake count for debounce
        u8 filter = 0x03;                 // 0x8053 - Touch filter parameter
        u8 large_touch = 0x28;            // 0x8054 - Large touch threshold
        u8 noise_reduction = 0x00;        // 0x8055 - Noise reduction level
        u8 screen_touch_level = 0x1E;     // 0x8056 - Screen touch threshold
        u8 screen_leave_level = 0x14;     // 0x8057 - Screen leave threshold
        u8 low_power_control = 0x00;      // 0x8058 - Low power control
        u8 refresh_rate = 0x0A;           // 0x8059 - Refresh rate
        u8 x_threshold = 0x00;            // 0x805A - X coordinate threshold
        u8 y_threshold = 0x00;            // 0x805B - Y coordinate threshold  
        u8 x_speed_limit = 0x00;          // 0x805C - X speed limit
        u8 y_speed_limit = 0x00;          // 0x805D - Y speed limit
        u16 config_checksum = 0x0000;     // 0x80FF-0x8100 - Configuration checksum
        
        // Touch data registers (0x814E-0x8155)
        u8 touch_status = 0x00;           // 0x814E - Touch status
        u8 touch_point1[8] = {0};         // 0x814F-0x8156 - Touch point 1 data
        u8 touch_point2[8] = {0};         // 0x8157-0x815E - Touch point 2 data  
        u8 touch_point3[8] = {0};         // 0x815F-0x8166 - Touch point 3 data
        u8 touch_point4[8] = {0};         // 0x8167-0x816E - Touch point 4 data
        u8 touch_point5[8] = {0};         // 0x816F-0x8176 - Touch point 5 data
        
        // Gesture registers (0x8177-0x817F)
        u8 gesture_flag = 0x00;           // 0x8177 - Gesture flag
        u8 gesture_count = 0x00;          // 0x8178 - Gesture count
        u8 gesture_data[7] = {0};         // 0x8179-0x817F - Gesture data
        
        // System information (read-only)
        u16 product_id = PRODUCT_ID;      // 0x8140-0x8141 - Product ID
        u16 firmware_version = 0x1060;    // 0x8142-0x8143 - Firmware version
        u16 x_coordinate_range = DEFAULT_WIDTH;  // 0x8144-0x8145 - X coordinate range
        u16 y_coordinate_range = DEFAULT_HEIGHT; // 0x8146-0x8147 - Y coordinate range
        u8 vendor_id = 0x00;              // 0x8148 - Vendor ID
    };

    struct GestureDetector {
        std::array<TouchPointData, 10> history; // Touch point history
        size_t history_index = 0;
        TouchGestureType current_gesture = TouchGestureType::NONE;
        std::chrono::steady_clock::time_point gesture_start_time;
        std::chrono::steady_clock::time_point last_touch_time;
        u16 gesture_start_x = 0, gesture_start_y = 0;
        u16 gesture_current_x = 0, gesture_current_y = 0;
        float initial_distance = 0.0f;    // For pinch gestures
        float initial_angle = 0.0f;       // For rotation gestures
        u8 finger_count = 0;
        bool gesture_in_progress = false;
    };

    struct ProximityDetector {
        bool enabled = false;
        float distance_cm = 10.0f;        // Simulated proximity distance
        u8 threshold = 50;                // Proximity threshold
        std::chrono::steady_clock::time_point last_proximity_time;
        bool proximity_state = false;
    };

    // Touch processing methods
    void process_touch_input();
    void detect_gestures();
    void apply_noise_filtering(std::vector<TouchPointData>& points);
    void apply_palm_rejection(std::vector<TouchPointData>& points);
    void calibrate_touch_points(std::vector<TouchPointData>& points);
    
    // Gesture recognition algorithms
    TouchGestureType classify_single_touch_gesture(const TouchPointData& point);
    TouchGestureType classify_multi_touch_gesture(const std::vector<TouchPointData>& points);
    bool detect_swipe_gesture(const TouchPointData& start, const TouchPointData& end, TouchGestureType& gesture_type);
    bool detect_pinch_gesture(const std::vector<TouchPointData>& points, TouchGestureType& gesture_type);
    bool detect_rotation_gesture(const std::vector<TouchPointData>& points, TouchGestureType& gesture_type);
    
    // Calibration algorithms
    void auto_calibrate_sensitivity();
    void auto_calibrate_noise_threshold();
    void calculate_calibration_matrix();
    TouchPointData apply_calibration_transform(const TouchPointData& raw_point);
    
    // Utility functions
    float calculate_distance(const TouchPointData& point1, const TouchPointData& point2);
    float calculate_angle(const TouchPointData& point1, const TouchPointData& point2);
    float calculate_velocity(const TouchPointData& start, const TouchPointData& end, u32 time_diff_ms);
    bool is_palm_touch(const TouchPointData& point);
    u16 calculate_checksum(const u8* data, size_t length);
    
    // Simulation helpers
    void simulate_touch_noise(TouchPointData& point);
    void simulate_environmental_interference();
    void update_proximity_simulation();
    
    void trigger_interrupt(GT911InterruptType interrupt_type);
    void update_touch_registers();
    void update_gesture_registers();
    
    bool initialized_;
    bool calibration_in_progress_;
    u8 device_address_;
    u8 active_touch_count_;
    
    GT911Registers registers_;
    InterruptController* interrupt_controller_;
    I2CController* i2c_controller_;
    
    TouchConfiguration touch_config_;
    TouchCalibration calibration_;
    GestureDetector gesture_detector_;
    ProximityDetector proximity_detector_;
    
    // Data buffers
    std::queue<TouchPointData> touch_buffer_;
    std::queue<TouchGesture> gesture_history_;
    std::array<TouchPointData, MAX_TOUCH_POINTS> current_touches_;
    
    // Calibration state
    std::chrono::steady_clock::time_point calibration_start_;
    std::vector<TouchPointData> calibration_samples_;
    
    // Timing and synchronization
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_touch_time_;
    u32 update_interval_ms_;
    
    TouchStatistics statistics_;
    
    mutable std::mutex touch_mutex_;
};

}  // namespace m5tab5::emulator