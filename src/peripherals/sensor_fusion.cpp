#include "emulator/peripherals/sensor_fusion.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>

namespace m5tab5::emulator {

SensorFusion::SensorFusion() 
    : initialized_(false)
    , imu_sensor_(nullptr)
    , camera_sensor_(nullptr)
    , calibration_active_(false)
    , outlier_detection_enabled_(true)
    , outlier_threshold_sigma_(3.0f)
    , adaptive_filtering_enabled_(true)
    , motion_compensation_enabled_(true) {
    
    // Initialize default fusion configuration
    fusion_config_.algorithm = FusionAlgorithm::MADGWICK;
    fusion_config_.update_rate = 100.0f;
    fusion_config_.beta = 0.041f;
    
    // Initialize fusion state
    current_fused_data_.orientation_quat = Quaternion();
    current_fused_data_.is_valid = false;
    reference_frame_ = Quaternion();
    
    // Initialize bias estimators
    for (auto& estimator : bias_estimators_) {
        estimator.current_bias = Vector3D();
        estimator.converged = false;
    }
    
    // Initialize filter states
    for (size_t i = 0; i < filter_states_.size(); ++i) {
        filter_states_[i].config.type = FilterType::LOW_PASS;
        filter_states_[i].config.cutoff_frequency = 10.0f;
        filter_states_[i].config.sample_rate = 100.0f;
        filter_states_[i].initialized = false;
    }
    
    LOG_DEBUG("SensorFusion", "Sensor fusion system initialized");
}

SensorFusion::~SensorFusion() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> SensorFusion::initialize(const Configuration& config,
                                     BMI270_IMU* imu_sensor,
                                     SC2356Camera* camera_sensor) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "Sensor fusion already initialized");
    }
    
    if (!imu_sensor) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "IMU sensor cannot be null");
    }
    
    imu_sensor_ = imu_sensor;
    camera_sensor_ = camera_sensor;
    
    // Initialize Madgwick filter state
    madgwick_state_.q = Quaternion();
    madgwick_state_.beta = fusion_config_.beta;
    madgwick_state_.sample_period = 1.0f / fusion_config_.update_rate;
    madgwick_state_.initialized = true;
    
    // Initialize Kalman filter state
    kalman_state_.x = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
    
    // Initialize P matrix (error covariance)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            kalman_state_.P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Initialize Q matrix (process noise)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            kalman_state_.Q[i][j] = (i == j) ? fusion_config_.process_noise : 0.0f;
        }
    }
    
    // Initialize R matrix (measurement noise)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            kalman_state_.R[i][j] = (i == j) ? fusion_config_.measurement_noise : 0.0f;
        }
    }
    
    kalman_state_.initialized = true;
    
    // Initialize integration state
    integrated_velocity_ = Vector3D();
    integrated_position_ = Vector3D();
    last_integration_time_ = std::chrono::steady_clock::now();
    
    last_update_ = std::chrono::steady_clock::now();
    
    initialized_ = true;
    
    LOG_INFO("SensorFusion", "Sensor fusion system initialized successfully");
    return {};
}

Result<void> SensorFusion::shutdown() {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Clear data buffers
    while (!sensor_buffer_.empty()) {
        sensor_buffer_.pop();
    }
    fusion_history_.clear();
    
    // Reset filter states
    for (auto& filter : filter_states_) {
        filter.input_history.clear();
        filter.output_history.clear();
        filter.coefficients_a.clear();
        filter.coefficients_b.clear();
        filter.initialized = false;
    }
    
    imu_sensor_ = nullptr;
    camera_sensor_ = nullptr;
    
    initialized_ = false;
    
    LOG_INFO("SensorFusion", "Sensor fusion system shutdown complete");
    return {};
}

Result<void> SensorFusion::configure_fusion(const FusionConfiguration& config) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    fusion_config_ = config;
    
    // Update algorithm-specific parameters
    madgwick_state_.beta = config.beta;
    madgwick_state_.sample_period = 1.0f / config.update_rate;
    
    // Update Kalman filter noise matrices
    for (int i = 0; i < 4; ++i) {
        kalman_state_.Q[i][i] = config.process_noise;
    }
    
    for (int i = 0; i < 3; ++i) {
        kalman_state_.R[i][i] = config.measurement_noise;
    }
    
    LOG_INFO("SensorFusion", "Fusion configuration updated");
    return {};
}

Result<void> SensorFusion::configure_filter(SensorDataType sensor_type, const FilterConfiguration& config) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    size_t index = static_cast<size_t>(sensor_type);
    if (index >= filter_states_.size()) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid sensor type");
    }
    
    filter_states_[index].config = config;
    
    // Design filter coefficients based on type
    switch (config.type) {
        case FilterType::BUTTERWORTH:
            design_butterworth_filter(filter_states_[index]);
            break;
        case FilterType::CHEBYSHEV:
            design_chebyshev_filter(filter_states_[index]);
            break;
        case FilterType::ELLIPTIC:
            design_elliptic_filter(filter_states_[index]);
            break;
        default:
            // Simple filters don't need coefficient design
            break;
    }
    
    filter_states_[index].initialized = true;
    
    LOG_DEBUG("SensorFusion", "Filter configured for sensor type {}", static_cast<int>(sensor_type));
    return {};
}

Result<void> SensorFusion::add_imu_data(const IMUData& imu_data) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    // Convert IMU data to sensor readings
    SensorReading accel_reading;
    accel_reading.type = SensorDataType::ACCELEROMETER;
    accel_reading.data = Vector3D(imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g);
    accel_reading.timestamp = imu_data.timestamp;
    accel_reading.confidence = imu_data.data_valid ? 1.0f : 0.0f;
    accel_reading.is_valid = imu_data.data_valid;
    
    SensorReading gyro_reading;
    gyro_reading.type = SensorDataType::GYROSCOPE;
    gyro_reading.data = Vector3D(
        imu_data.gyro_x_dps * M_PI / 180.0f,  // Convert to rad/s
        imu_data.gyro_y_dps * M_PI / 180.0f,
        imu_data.gyro_z_dps * M_PI / 180.0f
    );
    gyro_reading.timestamp = imu_data.timestamp;
    gyro_reading.confidence = imu_data.data_valid ? 1.0f : 0.0f;
    gyro_reading.is_valid = imu_data.data_valid;
    
    // Add readings to processing queue
    auto accel_result = add_sensor_reading(accel_reading);
    auto gyro_result = add_sensor_reading(gyro_reading);
    
    if (!accel_result || !gyro_result) {
        return make_error(ErrorCode::PROCESSING_ERROR, "Failed to add IMU readings");
    }
    
    return {};
}

Result<void> SensorFusion::add_sensor_reading(const SensorReading& reading) {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    // Outlier detection
    if (outlier_detection_enabled_ && detect_outlier(reading, outlier_threshold_sigma_)) {
        statistics_.outlier_rejections++;
        LOG_DEBUG("SensorFusion", "Outlier rejected for sensor type {}", static_cast<int>(reading.type));
        return {};
    }
    
    // Apply digital filtering
    SensorReading filtered_reading = reading;
    size_t filter_index = static_cast<size_t>(reading.type);
    if (filter_index < filter_states_.size() && filter_states_[filter_index].initialized) {
        FilterState& filter = filter_states_[filter_index];
        filtered_reading.data.x = apply_digital_filter(filter, reading.data.x);
        filtered_reading.data.y = apply_digital_filter(filter, reading.data.y);
        filtered_reading.data.z = apply_digital_filter(filter, reading.data.z);
        
        statistics_.filter_updates++;
    }
    
    // Update bias estimation
    if (calibration_active_) {
        update_bias_estimation(reading.type, reading.data);
    }
    
    // Store filtered reading
    sensor_buffer_.push(filtered_reading);
    
    // Maintain buffer size
    while (sensor_buffer_.size() > MAX_HISTORY_SIZE) {
        sensor_buffer_.pop();
    }
    
    // Update last readings for fusion algorithms
    switch (reading.type) {
        case SensorDataType::ACCELEROMETER:
            last_accel_reading_ = filtered_reading.data;
            break;
        case SensorDataType::GYROSCOPE:
            last_gyro_reading_ = filtered_reading.data;
            break;
        case SensorDataType::MAGNETOMETER:
            last_mag_reading_ = filtered_reading.data;
            break;
        default:
            break;
    }
    
    // Trigger filter callback if set
    if (filter_callbacks_[filter_index]) {
        filter_callbacks_[filter_index](filtered_reading);
    }
    
    statistics_.samples_processed++;
    
    return {};
}

Result<FusedSensorData> SensorFusion::get_fused_data() const {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    return current_fused_data_;
}

Result<void> SensorFusion::start_calibration() {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    calibration_active_ = true;
    calibration_start_ = std::chrono::steady_clock::now();
    
    // Reset bias estimators
    for (auto& estimator : bias_estimators_) {
        estimator.current_bias = Vector3D();
        estimator.bias_accumulator = Vector3D();
        estimator.sample_count = 0;
        estimator.converged = false;
        estimator.start_time = calibration_start_;
    }
    
    LOG_INFO("SensorFusion", "Calibration started");
    return {};
}

Result<bool> SensorFusion::is_calibration_complete() const {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Sensor fusion not initialized");
    }
    
    if (!calibration_active_) {
        return true;
    }
    
    // Check if all bias estimators have converged
    bool all_converged = true;
    for (const auto& estimator : bias_estimators_) {
        if (!estimator.converged) {
            all_converged = false;
            break;
        }
    }
    
    return all_converged;
}

void SensorFusion::update() {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < (1000.0f / fusion_config_.update_rate)) {
        return; // Not time for update yet
    }
    
    // Run fusion algorithm
    switch (fusion_config_.algorithm) {
        case FusionAlgorithm::COMPLEMENTARY:
            update_complementary_filter();
            break;
        case FusionAlgorithm::KALMAN:
            update_kalman_filter();
            break;
        case FusionAlgorithm::MADGWICK:
            update_madgwick_filter();
            break;
        case FusionAlgorithm::MAHONY:
            update_mahony_filter();
            break;
        case FusionAlgorithm::PARTICLE_FILTER:
            update_particle_filter();
            break;
    }
    
    // Integrate motion
    integrate_motion();
    
    // Update statistics
    update_statistics();
    
    // Adaptive filtering
    if (adaptive_filtering_enabled_) {
        adaptive_filter_tuning();
    }
    
    // Store in history
    if (fusion_history_.size() >= MAX_HISTORY_SIZE) {
        fusion_history_.erase(fusion_history_.begin());
    }
    fusion_history_.push_back(current_fused_data_);
    
    // Trigger callback if set
    if (fusion_callback_) {
        fusion_callback_(current_fused_data_);
    }
    
    last_update_ = now;
}

void SensorFusion::update_madgwick_filter() {
    if (!madgwick_state_.initialized) {
        return;
    }
    
    // Madgwick AHRS algorithm implementation
    Quaternion& q = madgwick_state_.q;
    float beta = madgwick_state_.beta;
    float dt = madgwick_state_.sample_period;
    
    Vector3D gyro = last_gyro_reading_;
    Vector3D accel = last_accel_reading_.normalized();
    
    // Rate of change of quaternion from gyroscope
    Quaternion qDot = q * Quaternion(0, gyro.x, gyro.y, gyro.z) * 0.5f;
    
    // Gradient descent algorithm corrective step
    Vector3D f(
        2 * (q.x * q.z - q.w * q.y) - accel.x,
        2 * (q.w * q.x + q.y * q.z) - accel.y,
        2 * (0.5f - q.x * q.x - q.y * q.y) - accel.z
    );
    
    // Jacobian matrix
    std::array<std::array<float, 4>, 3> J = {{
        {-2 * q.y, 2 * q.z, -2 * q.w, 2 * q.x},
        {2 * q.x, 2 * q.w, 2 * q.z, 2 * q.y},
        {0, -4 * q.x, -4 * q.y, 0}
    }};
    
    // Gradient
    Quaternion gradient(
        J[0][0] * f.x + J[1][0] * f.y + J[2][0] * f.z,
        J[0][1] * f.x + J[1][1] * f.y + J[2][1] * f.z,
        J[0][2] * f.x + J[1][2] * f.y + J[2][2] * f.z,
        J[0][3] * f.x + J[1][3] * f.y + J[2][3] * f.z
    );
    
    gradient = gradient.normalized();
    
    // Apply feedback step
    qDot = Quaternion(qDot.w - beta * gradient.w,
                      qDot.x - beta * gradient.x,
                      qDot.y - beta * gradient.y,
                      qDot.z - beta * gradient.z);
    
    // Integrate rate of change of quaternion
    q = Quaternion(q.w + qDot.w * dt,
                   q.x + qDot.x * dt,
                   q.y + qDot.y * dt,
                   q.z + qDot.z * dt);
    
    q = q.normalized();
    
    // Update fused data
    current_fused_data_.orientation_quat = q;
    current_fused_data_.orientation_euler = q.toEuler();
    current_fused_data_.angular_velocity = last_gyro_reading_;
    
    // Estimate gravity-free acceleration
    Vector3D gravity_world(0, 0, -GRAVITY_MAGNITUDE);
    Vector3D gravity_body = q.conjugate().rotate(gravity_world);
    current_fused_data_.gravity_vector = gravity_body;
    current_fused_data_.linear_acceleration = last_accel_reading_ * GRAVITY_MAGNITUDE - gravity_body;
    
    current_fused_data_.timestamp = std::chrono::steady_clock::now();
    current_fused_data_.is_valid = true;
    
    // Calculate confidence based on accelerometer magnitude consistency
    float accel_magnitude = last_accel_reading_.magnitude();
    float gravity_consistency = 1.0f - std::abs(accel_magnitude - 1.0f); // Should be ~1g when stationary
    current_fused_data_.orientation_confidence = std::clamp(gravity_consistency, 0.0f, 1.0f);
    
    statistics_.madgwick_beta_adaptive = beta;
}

void SensorFusion::update_complementary_filter() {
    // Simple complementary filter implementation
    float alpha = 0.98f; // Trust gyroscope more for short term
    float dt = 1.0f / fusion_config_.update_rate;
    
    // Integrate gyroscope
    Vector3D gyro_angles = current_fused_data_.orientation_euler + last_gyro_reading_ * dt;
    
    // Calculate angles from accelerometer
    Vector3D accel = last_accel_reading_.normalized();
    Vector3D accel_angles(
        std::atan2(accel.y, accel.z),
        std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z)),
        0 // Yaw cannot be determined from accelerometer alone
    );
    
    // Complementary filter
    current_fused_data_.orientation_euler = Vector3D(
        alpha * gyro_angles.x + (1 - alpha) * accel_angles.x,
        alpha * gyro_angles.y + (1 - alpha) * accel_angles.y,
        gyro_angles.z // Keep gyro-based yaw
    );
    
    current_fused_data_.orientation_quat = Quaternion::fromEuler(current_fused_data_.orientation_euler);
    current_fused_data_.angular_velocity = last_gyro_reading_;
    current_fused_data_.linear_acceleration = remove_gravity(last_accel_reading_ * GRAVITY_MAGNITUDE, 
                                                            current_fused_data_.orientation_quat);
    
    current_fused_data_.timestamp = std::chrono::steady_clock::now();
    current_fused_data_.is_valid = true;
    current_fused_data_.orientation_confidence = 0.8f; // Fixed confidence for complementary filter
}

void SensorFusion::update_kalman_filter() {
    if (!kalman_state_.initialized) {
        return;
    }
    
    // Extended Kalman Filter for quaternion estimation
    // This is a simplified implementation
    
    float dt = 1.0f / fusion_config_.update_rate;
    Vector3D gyro = last_gyro_reading_;
    Vector3D accel = last_accel_reading_.normalized();
    
    // Prediction step
    // State transition matrix F (identity for quaternion)
    auto F = kalman_state_.P; // Copy for multiplication
    
    // Predict state
    Quaternion q_prev(kalman_state_.x[0], kalman_state_.x[1], kalman_state_.x[2], kalman_state_.x[3]);
    Quaternion q_gyro = Quaternion(0, gyro.x, gyro.y, gyro.z);
    Quaternion q_dot = q_prev * q_gyro * 0.5f;
    
    Quaternion q_pred(
        kalman_state_.x[0] + q_dot.w * dt,
        kalman_state_.x[1] + q_dot.x * dt,
        kalman_state_.x[2] + q_dot.y * dt,
        kalman_state_.x[3] + q_dot.z * dt
    );
    q_pred = q_pred.normalized();
    
    kalman_state_.x[0] = q_pred.w;
    kalman_state_.x[1] = q_pred.x;
    kalman_state_.x[2] = q_pred.y;
    kalman_state_.x[3] = q_pred.z;
    
    // Predict covariance: P = F*P*F' + Q
    kalman_state_.P = multiply_4x4(multiply_4x4(F, kalman_state_.P), transpose_4x4(F));
    for (int i = 0; i < 4; ++i) {
        kalman_state_.P[i][i] += kalman_state_.Q[i][i];
    }
    
    // Update step with accelerometer measurement
    // Measurement model: h(x) = q^-1 * [0,0,0,1] * q (gravity in body frame)
    Vector3D gravity_world(0, 0, 1);
    Vector3D gravity_body_expected = q_pred.conjugate().rotate(gravity_world);
    
    Vector3D innovation = accel - gravity_body_expected;
    
    // Update state (simplified)
    float innovation_magnitude = innovation.magnitude();
    if (innovation_magnitude < 0.5f) { // Only update if reasonable
        // Apply small correction to quaternion
        Vector3D correction = innovation * 0.1f;
        Quaternion q_correction = Quaternion::fromEuler(correction);
        q_pred = q_pred * q_correction;
        q_pred = q_pred.normalized();
        
        kalman_state_.x[0] = q_pred.w;
        kalman_state_.x[1] = q_pred.x;
        kalman_state_.x[2] = q_pred.y;
        kalman_state_.x[3] = q_pred.z;
    }
    
    // Update fused data
    current_fused_data_.orientation_quat = q_pred;
    current_fused_data_.orientation_euler = q_pred.toEuler();
    current_fused_data_.angular_velocity = last_gyro_reading_;
    current_fused_data_.linear_acceleration = remove_gravity(last_accel_reading_ * GRAVITY_MAGNITUDE, q_pred);
    
    current_fused_data_.timestamp = std::chrono::steady_clock::now();
    current_fused_data_.is_valid = true;
    current_fused_data_.orientation_confidence = 1.0f - innovation_magnitude;
    
    statistics_.kalman_innovation_magnitude = innovation_magnitude;
}

void SensorFusion::update_mahony_filter() {
    // Mahony AHRS algorithm (simplified)
    Vector3D gyro = last_gyro_reading_;
    Vector3D accel = last_accel_reading_.normalized();
    
    static Vector3D integral_error(0, 0, 0);
    float kp = 2.0f; // Proportional gain
    float ki = 0.005f; // Integral gain
    float dt = 1.0f / fusion_config_.update_rate;
    
    // Estimated direction of gravity
    Quaternion& q = current_fused_data_.orientation_quat;
    Vector3D v(2 * (q.x * q.z - q.w * q.y),
               2 * (q.w * q.x + q.y * q.z),
               q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    
    // Error is cross product between estimated and measured gravity
    Vector3D error = accel.cross(v);
    
    // Apply integral feedback
    integral_error = integral_error + error * dt;
    
    // Apply proportional and integral feedback
    Vector3D adjusted_gyro = gyro + error * kp + integral_error * ki;
    
    // Integrate quaternion
    Quaternion q_dot = q * Quaternion(0, adjusted_gyro.x, adjusted_gyro.y, adjusted_gyro.z) * 0.5f;
    q = Quaternion(q.w + q_dot.w * dt,
                   q.x + q_dot.x * dt,
                   q.y + q_dot.y * dt,
                   q.z + q_dot.z * dt);
    q = q.normalized();
    
    current_fused_data_.orientation_quat = q;
    current_fused_data_.orientation_euler = q.toEuler();
    current_fused_data_.angular_velocity = last_gyro_reading_;
    current_fused_data_.linear_acceleration = remove_gravity(last_accel_reading_ * GRAVITY_MAGNITUDE, q);
    
    current_fused_data_.timestamp = std::chrono::steady_clock::now();
    current_fused_data_.is_valid = true;
    current_fused_data_.orientation_confidence = 1.0f - error.magnitude();
}

void SensorFusion::update_particle_filter() {
    // Simplified particle filter implementation
    // This would typically use hundreds of particles
    static constexpr size_t NUM_PARTICLES = 100;
    static std::vector<Quaternion> particles(NUM_PARTICLES, Quaternion());
    static std::vector<float> weights(NUM_PARTICLES, 1.0f / NUM_PARTICLES);
    static bool particles_initialized = false;
    
    if (!particles_initialized) {
        // Initialize particles around identity quaternion
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> noise(0.0f, 0.1f);
        
        for (auto& particle : particles) {
            Vector3D noise_euler(noise(gen), noise(gen), noise(gen));
            particle = Quaternion::fromEuler(noise_euler);
        }
        particles_initialized = true;
    }
    
    float dt = 1.0f / fusion_config_.update_rate;
    Vector3D gyro = last_gyro_reading_;
    Vector3D accel = last_accel_reading_.normalized();
    
    // Prediction step
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> process_noise(0.0f, fusion_config_.gyro_noise_std);
    
    for (auto& particle : particles) {
        Vector3D noisy_gyro(
            gyro.x + process_noise(gen),
            gyro.y + process_noise(gen),
            gyro.z + process_noise(gen)
        );
        
        Quaternion q_dot = particle * Quaternion(0, noisy_gyro.x, noisy_gyro.y, noisy_gyro.z) * 0.5f;
        particle = Quaternion(
            particle.w + q_dot.w * dt,
            particle.x + q_dot.x * dt,
            particle.y + q_dot.y * dt,
            particle.z + q_dot.z * dt
        ).normalized();
    }
    
    // Update step
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        Vector3D gravity_expected = particles[i].conjugate().rotate(Vector3D(0, 0, 1));
        Vector3D error = accel - gravity_expected;
        float likelihood = std::exp(-error.magnitude() / (2 * fusion_config_.accel_noise_std));
        weights[i] = likelihood;
    }
    
    // Normalize weights
    float weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0f);
    if (weight_sum > 0.0f) {
        for (auto& weight : weights) {
            weight /= weight_sum;
        }
    }
    
    // Estimate state as weighted average
    float w_sum = 0, x_sum = 0, y_sum = 0, z_sum = 0;
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        w_sum += weights[i] * particles[i].w;
        x_sum += weights[i] * particles[i].x;
        y_sum += weights[i] * particles[i].y;
        z_sum += weights[i] * particles[i].z;
    }
    
    Quaternion estimated_q(w_sum, x_sum, y_sum, z_sum);
    estimated_q = estimated_q.normalized();
    
    // Resample if needed (when effective sample size is low)
    float effective_size = 1.0f / std::inner_product(weights.begin(), weights.end(), weights.begin(), 0.0f);
    if (effective_size < NUM_PARTICLES / 3) {
        // Simple resampling
        std::discrete_distribution<size_t> sampler(weights.begin(), weights.end());
        std::vector<Quaternion> new_particles(NUM_PARTICLES);
        for (size_t i = 0; i < NUM_PARTICLES; ++i) {
            new_particles[i] = particles[sampler(gen)];
        }
        particles = new_particles;
        std::fill(weights.begin(), weights.end(), 1.0f / NUM_PARTICLES);
    }
    
    current_fused_data_.orientation_quat = estimated_q;
    current_fused_data_.orientation_euler = estimated_q.toEuler();
    current_fused_data_.angular_velocity = last_gyro_reading_;
    current_fused_data_.linear_acceleration = remove_gravity(last_accel_reading_ * GRAVITY_MAGNITUDE, estimated_q);
    
    current_fused_data_.timestamp = std::chrono::steady_clock::now();
    current_fused_data_.is_valid = true;
    current_fused_data_.orientation_confidence = effective_size / NUM_PARTICLES;
    
    statistics_.particle_filter_effective_size = static_cast<u32>(effective_size);
}

void SensorFusion::integrate_motion() {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_integration_time_).count();
    
    if (dt > 0.1f) { // Limit maximum dt to prevent instability
        dt = 0.1f;
    }
    
    // Integration with bias compensation
    Vector3D corrected_accel = current_fused_data_.linear_acceleration - bias_estimators_[0].current_bias;
    
    // Velocity integration
    integrated_velocity_ = integrated_velocity_ + corrected_accel * dt;
    
    // Position integration  
    integrated_position_ = integrated_position_ + integrated_velocity_ * dt;
    
    // Apply motion constraints (e.g., zero velocity updates during stationary periods)
    apply_motion_constraints();
    
    current_fused_data_.velocity = integrated_velocity_;
    current_fused_data_.position = integrated_position_;
    
    last_integration_time_ = now;
}

void SensorFusion::apply_motion_constraints() {
    // Zero velocity update when device is stationary
    float accel_magnitude = current_fused_data_.linear_acceleration.magnitude();
    float gyro_magnitude = current_fused_data_.angular_velocity.magnitude();
    
    if (accel_magnitude < 0.5f && gyro_magnitude < 0.1f) { // Stationary thresholds
        integrated_velocity_ = integrated_velocity_ * 0.95f; // Decay velocity
    }
}

Vector3D SensorFusion::remove_gravity(const Vector3D& accel_raw, const Quaternion& orientation) {
    Vector3D gravity_world(0, 0, -GRAVITY_MAGNITUDE);
    Vector3D gravity_body = orientation.conjugate().rotate(gravity_world);
    return accel_raw - gravity_body;
}

bool SensorFusion::detect_outlier(const SensorReading& reading, float threshold_sigma) {
    // Simple outlier detection based on magnitude
    float magnitude = reading.data.magnitude();
    
    // Define expected ranges for different sensor types
    float expected_min = 0.0f, expected_max = 0.0f;
    switch (reading.type) {
        case SensorDataType::ACCELEROMETER:
            expected_min = 0.5f * GRAVITY_MAGNITUDE;
            expected_max = 3.0f * GRAVITY_MAGNITUDE;
            break;
        case SensorDataType::GYROSCOPE:
            expected_min = 0.0f;
            expected_max = 10.0f; // rad/s
            break;
        case SensorDataType::MAGNETOMETER:
            expected_min = 10.0f; // µT
            expected_max = 100.0f;
            break;
        default:
            return false; // No outlier detection for other types
    }
    
    return magnitude < expected_min || magnitude > expected_max;
}

void SensorFusion::update_bias_estimation(SensorDataType sensor_type, const Vector3D& reading) {
    size_t estimator_index = 0;
    switch (sensor_type) {
        case SensorDataType::ACCELEROMETER: estimator_index = 0; break;
        case SensorDataType::GYROSCOPE: estimator_index = 1; break;
        case SensorDataType::MAGNETOMETER: estimator_index = 2; break;
        default: return;
    }
    
    BiasEstimator& estimator = bias_estimators_[estimator_index];
    
    if (estimator.sample_count < 1000) { // Collect samples for bias estimation
        estimator.bias_accumulator = estimator.bias_accumulator + reading;
        estimator.sample_count++;
        
        if (estimator.sample_count >= 100) {
            estimator.current_bias = estimator.bias_accumulator * (1.0f / estimator.sample_count);
            
            // For accelerometer, subtract gravity
            if (sensor_type == SensorDataType::ACCELEROMETER) {
                estimator.current_bias = estimator.current_bias - Vector3D(0, 0, 1); // Assume Z-up
            }
            
            estimator.converged = true;
        }
    }
}

float SensorFusion::apply_digital_filter(FilterState& filter, float input) {
    switch (filter.config.type) {
        case FilterType::LOW_PASS: {
            // Simple first-order low-pass filter
            float alpha = filter.config.cutoff_frequency / (filter.config.cutoff_frequency + filter.config.sample_rate);
            if (filter.output_history.empty()) {
                filter.output_history.push_back(input);
                return input;
            }
            float output = alpha * input + (1 - alpha) * filter.output_history.back();
            filter.output_history.push_back(output);
            if (filter.output_history.size() > 10) {
                filter.output_history.erase(filter.output_history.begin());
            }
            return output;
        }
        
        case FilterType::MOVING_AVERAGE:
            return apply_moving_average(filter.input_history, input, filter.config.window_size);
            
        case FilterType::MEDIAN:
            return apply_median_filter(filter.input_history, input, filter.config.window_size);
            
        case FilterType::EXPONENTIAL_SMOOTHING:
            if (filter.output_history.empty()) {
                filter.output_history.push_back(input);
                return input;
            }
            return apply_exponential_smoothing(filter.output_history.back(), input, filter.config.alpha);
            
        default:
            return input; // Pass through for unimplemented filters
    }
}

float SensorFusion::apply_moving_average(std::vector<float>& history, float input, u32 window_size) {
    history.push_back(input);
    if (history.size() > window_size) {
        history.erase(history.begin());
    }
    
    float sum = std::accumulate(history.begin(), history.end(), 0.0f);
    return sum / history.size();
}

float SensorFusion::apply_median_filter(std::vector<float>& history, float input, u32 window_size) {
    history.push_back(input);
    if (history.size() > window_size) {
        history.erase(history.begin());
    }
    
    std::vector<float> sorted_history = history;
    std::sort(sorted_history.begin(), sorted_history.end());
    
    size_t n = sorted_history.size();
    if (n % 2 == 0) {
        return (sorted_history[n/2 - 1] + sorted_history[n/2]) / 2.0f;
    } else {
        return sorted_history[n/2];
    }
}

float SensorFusion::apply_exponential_smoothing(float previous, float input, float alpha) {
    return alpha * input + (1 - alpha) * previous;
}

void SensorFusion::adaptive_filter_tuning() {
    // Adjust filter parameters based on motion characteristics
    float motion_intensity = current_fused_data_.linear_acceleration.magnitude() + 
                           current_fused_data_.angular_velocity.magnitude();
    
    // Increase filter aggressiveness during high motion
    if (motion_intensity > 5.0f) {
        madgwick_state_.beta = std::min(fusion_config_.beta * 2.0f, 0.1f);
    } else {
        madgwick_state_.beta = fusion_config_.beta;
    }
}

void SensorFusion::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_update_).count();
    
    if (dt > 0.0f) {
        statistics_.average_update_rate = 0.9 * statistics_.average_update_rate + 0.1 * (1.0f / dt);
    }
    
    statistics_.current_latency_ms = std::chrono::duration<double, std::milli>(
        now - current_fused_data_.timestamp).count();
    
    // Calculate filter stability (simplified)
    if (fusion_history_.size() > 10) {
        float orientation_variance = 0.0f;
        for (size_t i = fusion_history_.size() - 10; i < fusion_history_.size() - 1; ++i) {
            Vector3D diff = fusion_history_[i+1].orientation_euler - fusion_history_[i].orientation_euler;
            orientation_variance += diff.magnitude();
        }
        statistics_.filter_stability_score = 1.0f - std::clamp(orientation_variance / 10.0f, 0.0f, 1.0f);
    }
}

// Matrix utility implementations
std::array<std::array<float, 4>, 4> SensorFusion::multiply_4x4(
    const std::array<std::array<float, 4>, 4>& A,
    const std::array<std::array<float, 4>, 4>& B) {
    
    std::array<std::array<float, 4>, 4> result = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

std::array<std::array<float, 4>, 4> SensorFusion::transpose_4x4(
    const std::array<std::array<float, 4>, 4>& A) {
    
    std::array<std::array<float, 4>, 4> result = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result[i][j] = A[j][i];
        }
    }
    return result;
}

void SensorFusion::design_butterworth_filter(FilterState& filter) {
    // Simplified Butterworth filter design
    // This is a placeholder - real implementation would use proper filter design algorithms
    float omega_c = 2.0f * M_PI * filter.config.cutoff_frequency / filter.config.sample_rate;
    float alpha = std::sin(omega_c) / (std::cos(omega_c) + 1.0f);
    
    filter.coefficients_b = {alpha, alpha};
    filter.coefficients_a = {1.0f, alpha - 1.0f};
}

void SensorFusion::design_chebyshev_filter(FilterState& filter) {
    // Placeholder for Chebyshev filter design
    design_butterworth_filter(filter); // Use Butterworth as fallback
}

void SensorFusion::design_elliptic_filter(FilterState& filter) {
    // Placeholder for Elliptic filter design  
    design_butterworth_filter(filter); // Use Butterworth as fallback
}

void SensorFusion::clear_statistics() {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    statistics_ = FusionStatistics{};
}

void SensorFusion::dump_status() const {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    
    LOG_INFO("SensorFusion", "=== Sensor Fusion Status ===");
    LOG_INFO("SensorFusion", "Initialized: {}", initialized_);
    LOG_INFO("SensorFusion", "Algorithm: {}", static_cast<int>(fusion_config_.algorithm));
    LOG_INFO("SensorFusion", "Update Rate: {:.1f} Hz", statistics_.average_update_rate);
    LOG_INFO("SensorFusion", "Samples Processed: {}", statistics_.samples_processed);
    LOG_INFO("SensorFusion", "Orientation (Euler): [{:.2f}, {:.2f}, {:.2f}] deg", 
             current_fused_data_.orientation_euler.x * 180.0f / M_PI,
             current_fused_data_.orientation_euler.y * 180.0f / M_PI,
             current_fused_data_.orientation_euler.z * 180.0f / M_PI);
    LOG_INFO("SensorFusion", "Orientation Confidence: {:.2f}", current_fused_data_.orientation_confidence);
    LOG_INFO("SensorFusion", "Linear Acceleration: [{:.2f}, {:.2f}, {:.2f}] m/s²",
             current_fused_data_.linear_acceleration.x,
             current_fused_data_.linear_acceleration.y,
             current_fused_data_.linear_acceleration.z);
    LOG_INFO("SensorFusion", "Position: [{:.2f}, {:.2f}, {:.2f}] m",
             current_fused_data_.position.x,
             current_fused_data_.position.y,
             current_fused_data_.position.z);
    LOG_INFO("SensorFusion", "Filter Stability: {:.2f}", statistics_.filter_stability_score);
    LOG_INFO("SensorFusion", "Calibration Active: {}", calibration_active_);
    
    for (size_t i = 0; i < bias_estimators_.size(); ++i) {
        const auto& estimator = bias_estimators_[i];
        LOG_INFO("SensorFusion", "Bias[{}]: [{:.3f}, {:.3f}, {:.3f}] (converged: {})",
                 i, estimator.current_bias.x, estimator.current_bias.y, estimator.current_bias.z,
                 estimator.converged);
    }
}

}  // namespace m5tab5::emulator