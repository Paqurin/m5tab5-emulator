#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/peripherals/spi_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <array>

namespace m5tab5::emulator {

enum class IMUInterface : u8 {
    I2C = 0,
    SPI = 1
};

enum class IMUAccelRange : u8 {
    RANGE_2G = 0,   // ±2g
    RANGE_4G = 1,   // ±4g  
    RANGE_8G = 2,   // ±8g
    RANGE_16G = 3   // ±16g
};

enum class IMUGyroRange : u8 {
    RANGE_125DPS = 0,  // ±125°/s
    RANGE_250DPS = 1,  // ±250°/s
    RANGE_500DPS = 2,  // ±500°/s
    RANGE_1000DPS = 3, // ±1000°/s
    RANGE_2000DPS = 4  // ±2000°/s
};

enum class IMUOutputDataRate : u8 {
    ODR_25HZ = 0,
    ODR_50HZ = 1,
    ODR_100HZ = 2,
    ODR_200HZ = 3,
    ODR_400HZ = 4,
    ODR_800HZ = 5,
    ODR_1600HZ = 6
};

enum class IMUPowerMode : u8 {
    SUSPEND = 0,
    NORMAL = 1,
    LOW_POWER = 2,
    FAST_STARTUP = 3
};

enum class IMUInterruptType : u8 {
    DATA_READY = 0x01,
    MOTION_DETECT = 0x02,
    NO_MOTION = 0x04,
    TAP_DETECT = 0x08,
    DOUBLE_TAP = 0x10,
    FREE_FALL = 0x20,
    ORIENTATION = 0x40,
    STEP_DETECT = 0x80,
    FIFO_WATERMARK = 0x100,
    FIFO_FULL = 0x200
};

enum class IMUGesture : u8 {
    NONE = 0,
    TAP = 1,
    DOUBLE_TAP = 2,
    SHAKE = 3,
    TILT_LEFT = 4,
    TILT_RIGHT = 5,
    TILT_UP = 6,
    TILT_DOWN = 7,
    ROTATION_CW = 8,
    ROTATION_CCW = 9,
    FREE_FALL = 10,
    STEP = 11
};

enum class IMUOrientation : u8 {
    UNKNOWN = 0,
    PORTRAIT_UP = 1,
    PORTRAIT_DOWN = 2,
    LANDSCAPE_LEFT = 3,
    LANDSCAPE_RIGHT = 4,
    FACE_UP = 5,
    FACE_DOWN = 6
};

struct IMUData {
    // Raw sensor data
    i16 accel_x, accel_y, accel_z;    // Raw accelerometer (LSB)
    i16 gyro_x, gyro_y, gyro_z;       // Raw gyroscope (LSB)
    i16 temperature;                   // Temperature (LSB)
    
    // Converted sensor data  
    float accel_x_g, accel_y_g, accel_z_g;           // Acceleration (g)
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;        // Angular velocity (°/s)
    float temperature_c;                               // Temperature (°C)
    
    // Sensor fusion results
    float pitch_deg, roll_deg, yaw_deg;               // Euler angles (degrees)
    float quaternion[4];                               // Quaternion (w, x, y, z)
    
    std::chrono::steady_clock::time_point timestamp;
    bool data_valid;
};

struct IMUCalibration {
    // Accelerometer calibration
    float accel_offset[3] = {0.0f, 0.0f, 0.0f};      // Offset compensation (g)
    float accel_scale[3] = {1.0f, 1.0f, 1.0f};       // Scale factor
    float accel_cross_axis[3][3] = {{1.0f, 0.0f, 0.0f}, 
                                   {0.0f, 1.0f, 0.0f}, 
                                   {0.0f, 0.0f, 1.0f}}; // Cross-axis compensation
    
    // Gyroscope calibration  
    float gyro_offset[3] = {0.0f, 0.0f, 0.0f};       // Offset compensation (°/s)
    float gyro_scale[3] = {1.0f, 1.0f, 1.0f};        // Scale factor
    
    // Temperature compensation
    float temp_coeff_accel[3] = {0.0f, 0.0f, 0.0f};  // Temperature coefficients
    float temp_coeff_gyro[3] = {0.0f, 0.0f, 0.0f};
};

struct IMUStatistics {
    u64 samples_read = 0;
    u64 interrupts_generated = 0;
    u64 gestures_detected = 0;
    u64 steps_counted = 0;
    u64 calibration_cycles = 0;
    u64 fifo_overflows = 0;
    double average_data_rate_hz = 0.0;
    double current_motion_intensity = 0.0;
    float current_orientation_stability = 0.0f;
};

class BMI270_IMU {
public:
    static constexpr u8 I2C_ADDRESS_PRIMARY = 0x68;
    static constexpr u8 I2C_ADDRESS_SECONDARY = 0x69;
    static constexpr u8 CHIP_ID = 0x24;
    static constexpr size_t FIFO_SIZE = 1024; // 1KB FIFO buffer
    static constexpr size_t GESTURE_HISTORY_SIZE = 32;
    
    BMI270_IMU();
    ~BMI270_IMU();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller,
                           I2CController* i2c_controller = nullptr,
                           SPIController* spi_controller = nullptr);
    Result<void> shutdown();

    // Configuration
    Result<void> configure_interface(IMUInterface interface, u8 address = I2C_ADDRESS_PRIMARY);
    Result<void> set_accelerometer_config(IMUAccelRange range, IMUOutputDataRate odr);
    Result<void> set_gyroscope_config(IMUGyroRange range, IMUOutputDataRate odr);
    Result<void> set_power_mode(IMUPowerMode mode);
    
    // Data acquisition
    Result<IMUData> read_sensor_data();
    Result<std::vector<IMUData>> read_fifo_data();
    Result<void> reset_fifo();
    
    // Calibration
    Result<void> start_calibration();
    Result<bool> is_calibration_complete() const;
    Result<void> apply_calibration(const IMUCalibration& calibration);
    Result<IMUCalibration> get_calibration() const;
    Result<void> save_calibration_to_flash();
    Result<void> load_calibration_from_flash();
    
    // Motion detection
    Result<void> enable_motion_detection(bool enable, float threshold_g = 0.5f);
    Result<void> enable_tap_detection(bool enable, float threshold_g = 2.0f);
    Result<void> enable_step_counter(bool enable);
    Result<void> enable_free_fall_detection(bool enable, float threshold_g = 0.3f);
    
    // Gesture recognition
    Result<IMUGesture> get_latest_gesture();
    Result<std::vector<IMUGesture>> get_gesture_history();
    Result<void> clear_gesture_history();
    Result<void> configure_gesture_sensitivity(float sensitivity);
    
    // Orientation
    Result<IMUOrientation> get_orientation();
    Result<float> get_orientation_confidence();
    
    // Advanced features
    Result<void> enable_sensor_fusion(bool enable);
    Result<void> set_fusion_filter_params(float alpha = 0.98f, float beta = 0.02f);
    Result<u32> get_step_count() const;
    Result<void> reset_step_count();
    
    // Interrupts
    Result<void> enable_interrupt(IMUInterruptType interrupt_type, bool enable);
    Result<u32> get_interrupt_status();
    Result<void> clear_interrupt(IMUInterruptType interrupt_type);
    
    // Register interface
    Result<void> write_register(u8 reg_addr, u8 value);
    Result<u8> read_register(u8 reg_addr);
    Result<void> write_registers(u8 reg_addr, const std::vector<u8>& data);
    Result<std::vector<u8>> read_registers(u8 reg_addr, size_t count);
    
    // Self-test and diagnostics
    Result<bool> perform_self_test();
    Result<void> reset_sensor();
    Result<u8> get_chip_id();
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    IMUPowerMode get_power_mode() const { return power_mode_; }
    const IMUStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct BMI270Registers {
        u8 chip_id = CHIP_ID;
        u8 err_reg = 0x00;
        u8 status = 0x00;
        u8 acc_x_lsb = 0x00, acc_x_msb = 0x00;
        u8 acc_y_lsb = 0x00, acc_y_msb = 0x00;
        u8 acc_z_lsb = 0x00, acc_z_msb = 0x00;
        u8 gyr_x_lsb = 0x00, gyr_x_msb = 0x00;
        u8 gyr_y_lsb = 0x00, gyr_y_msb = 0x00;
        u8 gyr_z_lsb = 0x00, gyr_z_msb = 0x00;
        u8 temperature_lsb = 0x00, temperature_msb = 0x00;
        u8 acc_conf = 0xA8;    // Default: 100Hz, normal mode
        u8 acc_range = 0x00;   // Default: ±2g
        u8 gyr_conf = 0xA9;    // Default: 200Hz, normal mode  
        u8 gyr_range = 0x01;   // Default: ±500°/s
        u8 pwr_conf = 0x00;    // Advanced power save disabled
        u8 pwr_ctrl = 0x0E;    // Accel + Gyro + Temp enabled
        u8 int1_io_ctrl = 0x08; // INT1 active high, push-pull
        u8 int2_io_ctrl = 0x08; // INT2 active high, push-pull
        u8 int_map_data = 0x04; // Data ready on INT1
        u8 fifo_config_0 = 0x00;
        u8 fifo_config_1 = 0x00;
        u8 fifo_length_lsb = 0x00, fifo_length_msb = 0x00;
        u8 fifo_data = 0x00;
    };

    struct MotionState {
        float motion_intensity = 0.0f;
        float last_accel_magnitude = 0.0f;
        float orientation_stability = 1.0f;
        IMUOrientation current_orientation = IMUOrientation::UNKNOWN;
        std::array<float, 10> accel_magnitude_history = {0.0f};
        size_t history_index = 0;
        u32 step_count = 0;
        std::chrono::steady_clock::time_point last_step_time;
        bool in_free_fall = false;
    };

    struct SensorFusion {
        bool enabled = false;
        float alpha = 0.98f;   // Complementary filter coefficient
        float beta = 0.02f;    // Madgwick filter gain
        float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        float euler_angles[3] = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw
        std::chrono::steady_clock::time_point last_update;
    };

    void simulate_sensor_data();
    void apply_calibration_to_data(IMUData& data);
    void update_sensor_fusion(const IMUData& data);
    void detect_gestures(const IMUData& data);
    void detect_orientation(const IMUData& data);
    void detect_motion_events(const IMUData& data);
    void update_fifo();
    void trigger_interrupt(IMUInterruptType interrupt_type);
    
    // Gesture detection algorithms
    bool detect_tap(const IMUData& data);
    bool detect_double_tap(const IMUData& data);
    bool detect_shake(const IMUData& data);
    bool detect_tilt(const IMUData& data, IMUGesture& gesture_type);
    bool detect_rotation(const IMUData& data, IMUGesture& gesture_type);
    bool detect_free_fall(const IMUData& data);
    bool detect_step(const IMUData& data);
    
    // Sensor fusion algorithms
    void complementary_filter(const IMUData& data);
    void madgwick_filter(const IMUData& data);
    void quaternion_to_euler(const float q[4], float euler[3]);
    
    // Utility functions
    float calculate_magnitude(float x, float y, float z);
    float low_pass_filter(float current, float previous, float alpha);
    float high_pass_filter(float current, float previous, float alpha);
    i16 convert_to_raw_accel(float value_g);
    i16 convert_to_raw_gyro(float value_dps);
    float convert_from_raw_accel(i16 raw_value);
    float convert_from_raw_gyro(i16 raw_value);
    
    // Communication helpers
    Result<void> write_reg_i2c(u8 reg_addr, u8 value);
    Result<u8> read_reg_i2c(u8 reg_addr);
    Result<void> write_reg_spi(u8 reg_addr, u8 value);
    Result<u8> read_reg_spi(u8 reg_addr);
    
    bool initialized_;
    IMUInterface interface_;
    u8 device_address_;
    IMUAccelRange accel_range_;
    IMUGyroRange gyro_range_;
    IMUOutputDataRate accel_odr_;
    IMUOutputDataRate gyro_odr_;
    IMUPowerMode power_mode_;
    
    BMI270Registers registers_;
    InterruptController* interrupt_controller_;
    I2CController* i2c_controller_;
    SPIController* spi_controller_;
    
    // Sensor data and state
    std::queue<IMUData> fifo_buffer_;
    std::queue<IMUGesture> gesture_history_;
    MotionState motion_state_;
    SensorFusion sensor_fusion_;
    IMUCalibration calibration_;
    
    // Motion detection parameters
    bool motion_detection_enabled_;
    bool tap_detection_enabled_;
    bool step_counter_enabled_;
    bool free_fall_detection_enabled_;
    float motion_threshold_;
    float tap_threshold_;
    float free_fall_threshold_;
    float gesture_sensitivity_;
    
    // Calibration state
    bool calibration_in_progress_;
    std::chrono::steady_clock::time_point calibration_start_;
    std::vector<IMUData> calibration_samples_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_data_ready_;
    u32 interrupt_status_;
    
    IMUStatistics statistics_;
    
    mutable std::mutex imu_mutex_;
};

}  // namespace m5tab5::emulator