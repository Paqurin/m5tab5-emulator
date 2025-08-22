#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/bmi270_imu.hpp"
#include "emulator/peripherals/sc2356_camera.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <array>
#include <functional>

namespace m5tab5::emulator {

enum class FusionAlgorithm : u8 {
    COMPLEMENTARY = 0,
    KALMAN = 1,
    MADGWICK = 2,
    MAHONY = 3,
    PARTICLE_FILTER = 4
};

enum class FilterType : u8 {
    LOW_PASS = 0,
    HIGH_PASS = 1,
    BAND_PASS = 2,
    BAND_STOP = 3,
    MEDIAN = 4,
    MOVING_AVERAGE = 5,
    EXPONENTIAL_SMOOTHING = 6,
    BUTTERWORTH = 7,
    CHEBYSHEV = 8,
    ELLIPTIC = 9
};

enum class SensorDataType : u8 {
    ACCELEROMETER = 0,
    GYROSCOPE = 1,
    MAGNETOMETER = 2,
    CAMERA_MOTION = 3,
    FUSION_ORIENTATION = 4,
    FUSION_POSITION = 5,
    FUSION_VELOCITY = 6
};

struct Vector3D {
    float x, y, z;
    
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3D operator*(float scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
    
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    Vector3D normalized() const {
        float mag = magnitude();
        return mag > 0.0f ? Vector3D(x / mag, y / mag, z / mag) : Vector3D();
    }
    
    float dot(const Vector3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    Vector3D cross(const Vector3D& other) const {
        return Vector3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
};

struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }
    
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    float magnitude() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }
    
    Quaternion normalized() const {
        float mag = magnitude();
        return mag > 0.0f ? Quaternion(w / mag, x / mag, y / mag, z / mag) : Quaternion();
    }
    
    Vector3D rotate(const Vector3D& v) const {
        Quaternion q_v(0, v.x, v.y, v.z);
        Quaternion result = (*this) * q_v * conjugate();
        return Vector3D(result.x, result.y, result.z);
    }
    
    Vector3D toEuler() const {
        Vector3D euler;
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler.y = std::copysign(M_PI / 2, sinp);
        else
            euler.y = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
    
    static Quaternion fromEuler(const Vector3D& euler) {
        float cr = std::cos(euler.x * 0.5f);
        float sr = std::sin(euler.x * 0.5f);
        float cp = std::cos(euler.y * 0.5f);
        float sp = std::sin(euler.y * 0.5f);
        float cy = std::cos(euler.z * 0.5f);
        float sy = std::sin(euler.z * 0.5f);
        
        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
};

struct SensorReading {
    SensorDataType type;
    Vector3D data;
    std::chrono::steady_clock::time_point timestamp;
    float confidence;
    bool is_valid;
};

struct FusedSensorData {
    Vector3D orientation_euler;     // Roll, pitch, yaw in radians
    Quaternion orientation_quat;    // Quaternion representation
    Vector3D angular_velocity;      // rad/s
    Vector3D linear_acceleration;   // m/s²
    Vector3D position;              // m (integrated)
    Vector3D velocity;              // m/s (integrated)
    Vector3D gravity_vector;        // Estimated gravity direction
    
    float orientation_confidence;
    float motion_confidence;
    float calibration_quality;
    
    std::chrono::steady_clock::time_point timestamp;
    bool is_valid;
};

struct FilterConfiguration {
    FilterType type = FilterType::LOW_PASS;
    float cutoff_frequency = 10.0f;    // Hz
    float sample_rate = 100.0f;        // Hz
    float damping_ratio = 0.707f;      // For second-order filters
    u32 order = 2;                     // Filter order
    u32 window_size = 5;               // For moving average/median
    float alpha = 0.1f;                // For exponential smoothing
    bool zero_phase = true;            // Apply filter forward and backward
};

struct FusionConfiguration {
    FusionAlgorithm algorithm = FusionAlgorithm::MADGWICK;
    float update_rate = 100.0f;        // Hz
    float gyro_noise_std = 0.1f;       // rad/s
    float accel_noise_std = 0.2f;      // m/s²
    float mag_noise_std = 0.1f;        // µT
    float beta = 0.041f;               // Madgwick filter gain
    float zeta = 0.0f;                 // Mahony filter gain
    float process_noise = 0.01f;       // Kalman filter process noise
    float measurement_noise = 0.1f;    // Kalman filter measurement noise
    bool use_magnetometer = false;     // Include magnetometer in fusion
    bool use_camera_motion = true;     // Include visual motion estimation
    bool adaptive_filtering = true;    // Adjust filter parameters based on motion
};

struct FusionStatistics {
    u64 samples_processed = 0;
    u64 filter_updates = 0;
    u64 convergence_events = 0;
    u64 outlier_rejections = 0;
    double average_update_rate = 0.0;
    double current_latency_ms = 0.0;
    float orientation_drift_deg_per_min = 0.0f;
    float position_error_estimate_m = 0.0f;
    float filter_stability_score = 0.0f;
    
    // Algorithm-specific statistics
    float madgwick_beta_adaptive = 0.0f;
    float kalman_innovation_magnitude = 0.0f;
    u32 particle_filter_effective_size = 0;
};

using FusionCallback = std::function<void(const FusedSensorData& data)>;
using FilterCallback = std::function<void(const SensorReading& reading)>;

class SensorFusion {
public:
    static constexpr size_t MAX_HISTORY_SIZE = 1000;
    static constexpr size_t MAX_FILTER_ORDER = 10;
    static constexpr float GRAVITY_MAGNITUDE = 9.80665f; // m/s²
    
    SensorFusion();
    ~SensorFusion();

    Result<void> initialize(const Configuration& config,
                           BMI270_IMU* imu_sensor,
                           SC2356Camera* camera_sensor = nullptr);
    Result<void> shutdown();

    // Configuration
    Result<void> configure_fusion(const FusionConfiguration& config);
    Result<FusionConfiguration> get_fusion_config() const;
    Result<void> set_fusion_algorithm(FusionAlgorithm algorithm);
    Result<void> configure_filter(SensorDataType sensor_type, const FilterConfiguration& config);
    
    // Data input
    Result<void> add_sensor_reading(const SensorReading& reading);
    Result<void> add_imu_data(const IMUData& imu_data);
    Result<void> add_camera_motion(const Vector3D& motion_vector, float confidence);
    
    // Data output
    Result<FusedSensorData> get_fused_data() const;
    Result<std::vector<FusedSensorData>> get_history(size_t count = 100) const;
    Result<SensorReading> get_filtered_reading(SensorDataType sensor_type) const;
    
    // Calibration and bias estimation
    Result<void> start_calibration();
    Result<bool> is_calibration_complete() const;
    Result<void> apply_bias_correction(SensorDataType sensor_type, const Vector3D& bias);
    Result<Vector3D> get_estimated_bias(SensorDataType sensor_type) const;
    Result<void> reset_integration(); // Reset position/velocity integration
    
    // Advanced features
    Result<void> enable_outlier_detection(bool enable, float threshold_sigma = 3.0f);
    Result<void> enable_adaptive_filtering(bool enable);
    Result<void> set_reference_frame(const Quaternion& reference);
    Result<void> enable_motion_compensation(bool enable); // For camera-IMU fusion
    
    // Callback interface
    Result<void> set_fusion_callback(FusionCallback callback);
    Result<void> set_filter_callback(SensorDataType sensor_type, FilterCallback callback);
    
    // Real-time processing
    void update();
    Result<void> force_update(); // Immediate update regardless of timing
    
    bool is_initialized() const { return initialized_; }
    const FusionStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct FilterState {
        FilterConfiguration config;
        std::vector<float> input_history;
        std::vector<float> output_history;
        std::vector<float> coefficients_b; // Numerator coefficients
        std::vector<float> coefficients_a; // Denominator coefficients
        bool initialized = false;
    };

    struct KalmanFilterState {
        std::array<std::array<float, 4>, 4> P; // Error covariance matrix
        std::array<float, 4> x;                // State vector [q0, q1, q2, q3]
        std::array<std::array<float, 4>, 4> Q; // Process noise covariance
        std::array<std::array<float, 3>, 3> R; // Measurement noise covariance
        bool initialized = false;
    };

    struct MadgwickFilterState {
        Quaternion q;
        float beta;
        float sample_period;
        bool initialized = false;
    };

    struct BiasEstimator {
        Vector3D current_bias;
        Vector3D bias_accumulator;
        u32 sample_count = 0;
        bool converged = false;
        std::chrono::steady_clock::time_point start_time;
    };

    // Core fusion algorithms
    void update_complementary_filter();
    void update_kalman_filter();
    void update_madgwick_filter();
    void update_mahony_filter();
    void update_particle_filter();
    
    // Digital signal processing
    float apply_digital_filter(FilterState& filter, float input);
    void design_butterworth_filter(FilterState& filter);
    void design_chebyshev_filter(FilterState& filter);
    void design_elliptic_filter(FilterState& filter);
    float apply_median_filter(std::vector<float>& history, float input, u32 window_size);
    float apply_moving_average(std::vector<float>& history, float input, u32 window_size);
    float apply_exponential_smoothing(float previous, float input, float alpha);
    
    // Sensor data processing
    Vector3D remove_gravity(const Vector3D& accel_raw, const Quaternion& orientation);
    Vector3D estimate_gravity_direction(const Vector3D& accel_readings, u32 sample_count);
    bool detect_outlier(const SensorReading& reading, float threshold_sigma);
    void update_bias_estimation(SensorDataType sensor_type, const Vector3D& reading);
    
    // Motion integration
    void integrate_motion();
    void apply_motion_constraints();
    Vector3D integrate_acceleration(const Vector3D& accel, float dt);
    Vector3D integrate_velocity(const Vector3D& velocity, float dt);
    
    // Camera-IMU fusion
    void fuse_camera_motion(const Vector3D& camera_motion, float confidence);
    Vector3D estimate_camera_motion_from_imu(const Vector3D& angular_vel, const Vector3D& linear_accel);
    
    // Utility functions
    float calculate_confidence(const SensorReading& reading);
    void adaptive_filter_tuning();
    bool check_convergence();
    void update_statistics();
    
    // Math utilities
    std::array<std::array<float, 4>, 4> multiply_4x4(
        const std::array<std::array<float, 4>, 4>& A,
        const std::array<std::array<float, 4>, 4>& B);
    std::array<float, 4> multiply_4x4_4x1(
        const std::array<std::array<float, 4>, 4>& A,
        const std::array<float, 4>& x);
    std::array<std::array<float, 4>, 4> transpose_4x4(
        const std::array<std::array<float, 4>, 4>& A);
    std::array<std::array<float, 4>, 4> inverse_4x4(
        const std::array<std::array<float, 4>, 4>& A);
    
    bool initialized_;
    FusionConfiguration fusion_config_;
    
    BMI270_IMU* imu_sensor_;
    SC2356Camera* camera_sensor_;
    
    // Filter states for each sensor type
    std::array<FilterState, 7> filter_states_; // One for each SensorDataType
    
    // Fusion algorithm states
    KalmanFilterState kalman_state_;
    MadgwickFilterState madgwick_state_;
    
    // Sensor data buffers
    std::queue<SensorReading> sensor_buffer_;
    std::vector<FusedSensorData> fusion_history_;
    
    // Bias estimation
    std::array<BiasEstimator, 3> bias_estimators_; // Accel, Gyro, Mag
    
    // Current sensor state
    Vector3D last_accel_reading_;
    Vector3D last_gyro_reading_;
    Vector3D last_mag_reading_;
    Vector3D estimated_gravity_;
    
    // Fusion state
    FusedSensorData current_fused_data_;
    Quaternion reference_frame_;
    bool calibration_active_;
    std::chrono::steady_clock::time_point calibration_start_;
    
    // Integration state
    Vector3D integrated_velocity_;
    Vector3D integrated_position_;
    std::chrono::steady_clock::time_point last_integration_time_;
    
    // Advanced features
    bool outlier_detection_enabled_;
    float outlier_threshold_sigma_;
    bool adaptive_filtering_enabled_;
    bool motion_compensation_enabled_;
    
    // Callback interface
    FusionCallback fusion_callback_;
    std::array<FilterCallback, 7> filter_callbacks_;
    
    // Timing and statistics
    std::chrono::steady_clock::time_point last_update_;
    FusionStatistics statistics_;
    
    mutable std::mutex fusion_mutex_;
};

}  // namespace m5tab5::emulator