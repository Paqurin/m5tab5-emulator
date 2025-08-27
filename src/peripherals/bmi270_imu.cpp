#include "emulator/peripherals/bmi270_imu.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <numeric>

namespace m5tab5::emulator {

DECLARE_LOGGER("BMI270_IMU");

BMI270_IMU::BMI270_IMU()
    : initialized_(false),
      interface_(IMUInterface::I2C),
      device_address_(I2C_ADDRESS_PRIMARY),
      accel_range_(IMUAccelRange::RANGE_2G),
      gyro_range_(IMUGyroRange::RANGE_500DPS),
      accel_odr_(IMUOutputDataRate::ODR_100HZ),
      gyro_odr_(IMUOutputDataRate::ODR_200HZ),
      power_mode_(IMUPowerMode::SUSPEND),
      interrupt_controller_(nullptr),
      i2c_controller_(nullptr),
      spi_controller_(nullptr),
      motion_detection_enabled_(false),
      tap_detection_enabled_(false),
      step_counter_enabled_(false),
      free_fall_detection_enabled_(false),
      motion_threshold_(0.5f),
      tap_threshold_(2.0f),
      free_fall_threshold_(0.3f),
      gesture_sensitivity_(1.0f),
      calibration_in_progress_(false),
      interrupt_status_(0) {
    
    // Initialize sensor fusion
    sensor_fusion_.quaternion[0] = 1.0f; // w
    sensor_fusion_.quaternion[1] = 0.0f; // x
    sensor_fusion_.quaternion[2] = 0.0f; // y
    sensor_fusion_.quaternion[3] = 0.0f; // z
    
    COMPONENT_LOG_DEBUG("BMI270 IMU created");
}

BMI270_IMU::~BMI270_IMU() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("BMI270 IMU destroyed");
}

Result<void> BMI270_IMU::initialize(const Configuration& config, 
                                   InterruptController* interrupt_controller,
                                   I2CController* i2c_controller,
                                   SPIController* spi_controller) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "BMI270 IMU already initialized"));
    }
    
    if (!interrupt_controller) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Interrupt controller required"));
    }
    
    COMPONENT_LOG_INFO("Initializing BMI270 IMU sensor");
    
    interrupt_controller_ = interrupt_controller;
    i2c_controller_ = i2c_controller;
    spi_controller_ = spi_controller;
    
    // Default to I2C interface if available
    if (i2c_controller_) {
        interface_ = IMUInterface::I2C;
        device_address_ = I2C_ADDRESS_PRIMARY;
    } else if (spi_controller_) {
        interface_ = IMUInterface::SPI;
    } else {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Either I2C or SPI controller required"));
    }
    
    // Reset all registers to default values
    registers_ = {};
    
    // Clear FIFO and gesture history
    while (!fifo_buffer_.empty()) fifo_buffer_.pop();
    while (!gesture_history_.empty()) gesture_history_.pop();
    
    // Initialize motion state
    motion_state_ = {};
    motion_state_.current_orientation = IMUOrientation::UNKNOWN;
    motion_state_.last_step_time = std::chrono::steady_clock::now();
    
    // Initialize calibration
    calibration_ = {};
    calibration_in_progress_ = false;
    
    // Set power mode to normal
    power_mode_ = IMUPowerMode::NORMAL;
    registers_.pwr_ctrl = 0x0E; // Enable accel, gyro, temp
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_data_ready_ = last_update_;
    sensor_fusion_.last_update = last_update_;
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("BMI270 IMU initialized successfully");
    
    return {};
}

Result<void> BMI270_IMU::shutdown() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down BMI270 IMU");
    
    // Set to suspend mode
    power_mode_ = IMUPowerMode::SUSPEND;
    registers_.pwr_ctrl = 0x00;
    
    // Clear buffers
    while (!fifo_buffer_.empty()) fifo_buffer_.pop();
    while (!gesture_history_.empty()) gesture_history_.pop();
    calibration_samples_.clear();
    
    interrupt_controller_ = nullptr;
    i2c_controller_ = nullptr;
    spi_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("BMI270 IMU shutdown completed");
    return {};
}

Result<void> BMI270_IMU::configure_interface(IMUInterface interface, u8 address) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (interface == IMUInterface::I2C && !i2c_controller_) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "I2C controller not available"));
    }
    
    if (interface == IMUInterface::SPI && !spi_controller_) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "SPI controller not available"));
    }
    
    interface_ = interface;
    if (interface == IMUInterface::I2C) {
        device_address_ = address;
    }
    
    COMPONENT_LOG_DEBUG("BMI270 interface configured: {}, address=0x{:02X}",
                       static_cast<u8>(interface), address);
    
    return {};
}

Result<void> BMI270_IMU::set_accelerometer_config(IMUAccelRange range, IMUOutputDataRate odr) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    accel_range_ = range;
    accel_odr_ = odr;
    
    // Update registers
    registers_.acc_range = static_cast<u8>(range);
    registers_.acc_conf = (static_cast<u8>(odr) << 0) | 0xA0; // Normal mode, no filter
    
    COMPONENT_LOG_DEBUG("BMI270 accelerometer configured: range={}, ODR={}",
                       static_cast<u8>(range), static_cast<u8>(odr));
    
    return {};
}

Result<void> BMI270_IMU::set_gyroscope_config(IMUGyroRange range, IMUOutputDataRate odr) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    gyro_range_ = range;
    gyro_odr_ = odr;
    
    // Update registers
    registers_.gyr_range = static_cast<u8>(range);
    registers_.gyr_conf = (static_cast<u8>(odr) << 0) | 0xA0; // Normal mode, no filter
    
    COMPONENT_LOG_DEBUG("BMI270 gyroscope configured: range={}, ODR={}",
                       static_cast<u8>(range), static_cast<u8>(odr));
    
    return {};
}

Result<void> BMI270_IMU::set_power_mode(IMUPowerMode mode) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    power_mode_ = mode;
    
    switch (mode) {
        case IMUPowerMode::SUSPEND:
            registers_.pwr_ctrl = 0x00;
            break;
        case IMUPowerMode::NORMAL:
            registers_.pwr_ctrl = 0x0E; // Accel + Gyro + Temp
            break;
        case IMUPowerMode::LOW_POWER:
            registers_.pwr_ctrl = 0x04; // Accel only
            break;
        case IMUPowerMode::FAST_STARTUP:
            registers_.pwr_ctrl = 0x0E;
            registers_.pwr_conf = 0x02; // Fast startup mode
            break;
    }
    
    COMPONENT_LOG_DEBUG("BMI270 power mode set to {}", static_cast<u8>(mode));
    return {};
}

Result<IMUData> BMI270_IMU::read_sensor_data() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (power_mode_ == IMUPowerMode::SUSPEND) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Sensor in suspend mode"));
    }
    
    // Simulate sensor data
    simulate_sensor_data();
    
    IMUData data = {};
    
    // Read raw values from registers
    data.accel_x = static_cast<i16>((registers_.acc_x_msb << 8) | registers_.acc_x_lsb);
    data.accel_y = static_cast<i16>((registers_.acc_y_msb << 8) | registers_.acc_y_lsb);
    data.accel_z = static_cast<i16>((registers_.acc_z_msb << 8) | registers_.acc_z_lsb);
    
    data.gyro_x = static_cast<i16>((registers_.gyr_x_msb << 8) | registers_.gyr_x_lsb);
    data.gyro_y = static_cast<i16>((registers_.gyr_y_msb << 8) | registers_.gyr_y_lsb);
    data.gyro_z = static_cast<i16>((registers_.gyr_z_msb << 8) | registers_.gyr_z_lsb);
    
    data.temperature = static_cast<i16>((registers_.temperature_msb << 8) | registers_.temperature_lsb);
    
    // Convert to engineering units
    data.accel_x_g = convert_from_raw_accel(data.accel_x);
    data.accel_y_g = convert_from_raw_accel(data.accel_y);
    data.accel_z_g = convert_from_raw_accel(data.accel_z);
    
    data.gyro_x_dps = convert_from_raw_gyro(data.gyro_x);
    data.gyro_y_dps = convert_from_raw_gyro(data.gyro_y);
    data.gyro_z_dps = convert_from_raw_gyro(data.gyro_z);
    
    data.temperature_c = (data.temperature / 512.0f) + 23.0f; // BMI270 formula
    
    data.timestamp = std::chrono::steady_clock::now();
    data.data_valid = true;
    
    // Apply calibration
    apply_calibration_to_data(data);
    
    // Update sensor fusion
    if (sensor_fusion_.enabled) {
        update_sensor_fusion(data);
        
        // Copy fusion results
        data.pitch_deg = sensor_fusion_.euler_angles[0];
        data.roll_deg = sensor_fusion_.euler_angles[1];  
        data.yaw_deg = sensor_fusion_.euler_angles[2];
        
        std::copy(sensor_fusion_.quaternion, sensor_fusion_.quaternion + 4, data.quaternion);
    }
    
    // Detect gestures and motion events
    detect_gestures(data);
    detect_orientation(data);
    detect_motion_events(data);
    
    // Update statistics
    statistics_.samples_read++;
    last_data_ready_ = data.timestamp;
    
    return data;
}

Result<std::vector<IMUData>> BMI270_IMU::read_fifo_data() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    std::vector<IMUData> result;
    
    while (!fifo_buffer_.empty()) {
        result.push_back(fifo_buffer_.front());
        fifo_buffer_.pop();
    }
    
    // Update FIFO length registers
    registers_.fifo_length_lsb = 0;
    registers_.fifo_length_msb = 0;
    
    return result;
}

Result<void> BMI270_IMU::reset_fifo() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    while (!fifo_buffer_.empty()) {
        fifo_buffer_.pop();
    }
    
    registers_.fifo_length_lsb = 0;
    registers_.fifo_length_msb = 0;
    
    COMPONENT_LOG_DEBUG("BMI270 FIFO reset");
    return {};
}

Result<void> BMI270_IMU::start_calibration() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (calibration_in_progress_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Calibration already in progress"));
    }
    
    calibration_in_progress_ = true;
    calibration_start_ = std::chrono::steady_clock::now();
    calibration_samples_.clear();
    calibration_samples_.reserve(1000); // Collect 1000 samples
    
    COMPONENT_LOG_INFO("BMI270 calibration started");
    return {};
}

Result<bool> BMI270_IMU::is_calibration_complete() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (!calibration_in_progress_) {
        return true;
    }
    
    // Calibration complete after 10 seconds or 1000 samples
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - calibration_start_);
    
    return elapsed.count() >= 10 || calibration_samples_.size() >= 1000;
}

Result<void> BMI270_IMU::apply_calibration(const IMUCalibration& calibration) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    calibration_ = calibration;
    
    COMPONENT_LOG_INFO("BMI270 calibration applied");
    return {};
}

Result<IMUCalibration> BMI270_IMU::get_calibration() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return calibration_;
}

Result<void> BMI270_IMU::save_calibration_to_flash() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    // In a real implementation, this would save to flash memory
    COMPONENT_LOG_INFO("BMI270 calibration saved to flash (simulated)");
    return {};
}

Result<void> BMI270_IMU::load_calibration_from_flash() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    // In a real implementation, this would load from flash memory
    // For simulation, use default calibration
    calibration_ = {};
    
    COMPONENT_LOG_INFO("BMI270 calibration loaded from flash (simulated)");
    return {};
}

Result<void> BMI270_IMU::enable_motion_detection(bool enable, float threshold_g) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (threshold_g < 0.0f || threshold_g > 8.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Threshold out of range (0-8g)"));
    }
    
    motion_detection_enabled_ = enable;
    motion_threshold_ = threshold_g;
    
    COMPONENT_LOG_DEBUG("BMI270 motion detection {}: threshold={:.2f}g",
                       enable ? "enabled" : "disabled", threshold_g);
    
    return {};
}

Result<void> BMI270_IMU::enable_tap_detection(bool enable, float threshold_g) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (threshold_g < 0.5f || threshold_g > 8.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Threshold out of range (0.5-8g)"));
    }
    
    tap_detection_enabled_ = enable;
    tap_threshold_ = threshold_g;
    
    COMPONENT_LOG_DEBUG("BMI270 tap detection {}: threshold={:.2f}g",
                       enable ? "enabled" : "disabled", threshold_g);
    
    return {};
}

Result<void> BMI270_IMU::enable_step_counter(bool enable) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    step_counter_enabled_ = enable;
    
    COMPONENT_LOG_DEBUG("BMI270 step counter {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> BMI270_IMU::enable_free_fall_detection(bool enable, float threshold_g) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (threshold_g < 0.1f || threshold_g > 1.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Threshold out of range (0.1-1.0g)"));
    }
    
    free_fall_detection_enabled_ = enable;
    free_fall_threshold_ = threshold_g;
    
    COMPONENT_LOG_DEBUG("BMI270 free fall detection {}: threshold={:.2f}g",
                       enable ? "enabled" : "disabled", threshold_g);
    
    return {};
}

Result<IMUGesture> BMI270_IMU::get_latest_gesture() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (gesture_history_.empty()) {
        return IMUGesture::NONE;
    }
    
    IMUGesture latest = gesture_history_.back();
    return latest;
}

Result<std::vector<IMUGesture>> BMI270_IMU::get_gesture_history() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    std::vector<IMUGesture> result;
    std::queue<IMUGesture> temp = gesture_history_;
    
    while (!temp.empty()) {
        result.push_back(temp.front());
        temp.pop();
    }
    
    return result;
}

Result<void> BMI270_IMU::clear_gesture_history() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    while (!gesture_history_.empty()) {
        gesture_history_.pop();
    }
    
    return {};
}

Result<void> BMI270_IMU::configure_gesture_sensitivity(float sensitivity) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (sensitivity < 0.1f || sensitivity > 5.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Sensitivity out of range (0.1-5.0)"));
    }
    
    gesture_sensitivity_ = sensitivity;
    
    COMPONENT_LOG_DEBUG("BMI270 gesture sensitivity set to {:.2f}", sensitivity);
    return {};
}

Result<IMUOrientation> BMI270_IMU::get_orientation() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return motion_state_.current_orientation;
}

Result<float> BMI270_IMU::get_orientation_confidence() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return motion_state_.orientation_stability;
}

Result<void> BMI270_IMU::enable_sensor_fusion(bool enable) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    sensor_fusion_.enabled = enable;
    
    if (enable) {
        // Reset fusion state
        sensor_fusion_.quaternion[0] = 1.0f;
        sensor_fusion_.quaternion[1] = 0.0f;
        sensor_fusion_.quaternion[2] = 0.0f;
        sensor_fusion_.quaternion[3] = 0.0f;
        sensor_fusion_.last_update = std::chrono::steady_clock::now();
    }
    
    COMPONENT_LOG_DEBUG("BMI270 sensor fusion {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> BMI270_IMU::set_fusion_filter_params(float alpha, float beta) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (alpha < 0.0f || alpha > 1.0f || beta < 0.0f || beta > 1.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Filter parameters out of range (0.0-1.0)"));
    }
    
    sensor_fusion_.alpha = alpha;
    sensor_fusion_.beta = beta;
    
    COMPONENT_LOG_DEBUG("BMI270 fusion filter params: alpha={:.3f}, beta={:.3f}", alpha, beta);
    return {};
}

Result<u32> BMI270_IMU::get_step_count() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return motion_state_.step_count;
}

Result<void> BMI270_IMU::reset_step_count() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    motion_state_.step_count = 0;
    statistics_.steps_counted = 0;
    
    COMPONENT_LOG_DEBUG("BMI270 step count reset");
    return {};
}

Result<void> BMI270_IMU::enable_interrupt(IMUInterruptType interrupt_type, bool enable) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (enable) {
        registers_.int_map_data |= static_cast<u32>(interrupt_type);
    } else {
        registers_.int_map_data &= ~static_cast<u32>(interrupt_type);
    }
    
    COMPONENT_LOG_DEBUG("BMI270 interrupt {} {}", 
                       static_cast<u32>(interrupt_type), enable ? "enabled" : "disabled");
    
    return {};
}

Result<u32> BMI270_IMU::get_interrupt_status() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return interrupt_status_;
}

Result<void> BMI270_IMU::clear_interrupt(IMUInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    interrupt_status_ &= ~static_cast<u32>(interrupt_type);
    
    return {};
}

Result<void> BMI270_IMU::write_register(u8 reg_addr, u8 value) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (interface_ == IMUInterface::I2C) {
        return write_reg_i2c(reg_addr, value);
    } else {
        return write_reg_spi(reg_addr, value);
    }
}

Result<u8> BMI270_IMU::read_register(u8 reg_addr) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    if (interface_ == IMUInterface::I2C) {
        return read_reg_i2c(reg_addr);
    } else {
        return read_reg_spi(reg_addr);
    }
}

Result<void> BMI270_IMU::write_registers(u8 reg_addr, const std::vector<u8>& data) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    for (size_t i = 0; i < data.size(); ++i) {
        auto result = write_register(reg_addr + static_cast<u8>(i), data[i]);
        if (!result) {
            return result;
        }
    }
    
    return {};
}

Result<std::vector<u8>> BMI270_IMU::read_registers(u8 reg_addr, size_t count) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    std::vector<u8> result;
    result.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        auto reg_result = read_register(reg_addr + static_cast<u8>(i));
        if (!reg_result) {
            return unexpected(reg_result.error());
        }
        result.push_back(reg_result.value());
    }
    
    return result;
}

Result<bool> BMI270_IMU::perform_self_test() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    COMPONENT_LOG_INFO("BMI270 performing self-test");
    
    // Simulate self-test (in real hardware, this would run built-in tests)
    bool test_passed = true;
    
    // Check if sensor responses are within expected ranges
    auto data_result = read_sensor_data();
    if (data_result) {
        const auto& data = data_result.value();
        
        // Accelerometer should detect gravity (approximately 1g)
        float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
        if (accel_magnitude < 0.8f || accel_magnitude > 1.2f) {
            test_passed = false;
        }
        
        // Gyroscope should be close to zero when stationary
        float gyro_magnitude = calculate_magnitude(data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps);
        if (gyro_magnitude > 10.0f) {
            test_passed = false;
        }
    } else {
        test_passed = false;
    }
    
    COMPONENT_LOG_INFO("BMI270 self-test {}", test_passed ? "PASSED" : "FAILED");
    return test_passed;
}

Result<void> BMI270_IMU::reset_sensor() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    COMPONENT_LOG_INFO("BMI270 sensor reset");
    
    // Reset all registers to default values
    registers_ = {};
    registers_.chip_id = CHIP_ID;
    
    // Reset internal state
    motion_state_ = {};
    sensor_fusion_.quaternion[0] = 1.0f;
    sensor_fusion_.quaternion[1] = 0.0f;
    sensor_fusion_.quaternion[2] = 0.0f;
    sensor_fusion_.quaternion[3] = 0.0f;
    
    // Clear buffers
    while (!fifo_buffer_.empty()) fifo_buffer_.pop();
    while (!gesture_history_.empty()) gesture_history_.pop();
    
    return {};
}

Result<u8> BMI270_IMU::get_chip_id() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "BMI270 IMU not initialized"));
    }
    
    return registers_.chip_id;
}

void BMI270_IMU::update() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    if (!initialized_ || power_mode_ == IMUPowerMode::SUSPEND) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);
    last_update_ = now;
    
    // Update data rate based on ODR settings
    u32 accel_period_ms = 1000 / (25 << static_cast<u8>(accel_odr_));
    u32 gyro_period_ms = 1000 / (25 << static_cast<u8>(gyro_odr_));
    
    if (elapsed.count() >= std::min(accel_period_ms, gyro_period_ms)) {
        // Generate new sensor data
        simulate_sensor_data();
        
        // Update FIFO
        update_fifo();
        
        // Trigger data ready interrupt
        trigger_interrupt(IMUInterruptType::DATA_READY);
        
        // Update statistics
        statistics_.average_data_rate_hz = 1000.0 / elapsed.count();
    }
    
    // Check calibration progress
    if (calibration_in_progress_) {
        auto calibration_complete = is_calibration_complete();
        if (calibration_complete.has_value() && calibration_complete.value()) {
            // Finish calibration
            if (!calibration_samples_.empty()) {
                // Calculate calibration parameters
                float accel_sum[3] = {0.0f, 0.0f, 0.0f};
                float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
                
                for (const auto& sample : calibration_samples_) {
                    accel_sum[0] += sample.accel_x_g;
                    accel_sum[1] += sample.accel_y_g;
                    accel_sum[2] += sample.accel_z_g;
                    
                    gyro_sum[0] += sample.gyro_x_dps;
                    gyro_sum[1] += sample.gyro_y_dps;
                    gyro_sum[2] += sample.gyro_z_dps;
                }
                
                size_t count = calibration_samples_.size();
                
                // Calculate offsets
                calibration_.accel_offset[0] = accel_sum[0] / count;
                calibration_.accel_offset[1] = accel_sum[1] / count;
                calibration_.accel_offset[2] = (accel_sum[2] / count) - 1.0f; // Subtract gravity
                
                calibration_.gyro_offset[0] = gyro_sum[0] / count;
                calibration_.gyro_offset[1] = gyro_sum[1] / count;
                calibration_.gyro_offset[2] = gyro_sum[2] / count;
            }
            
            calibration_in_progress_ = false;
            statistics_.calibration_cycles++;
            
            COMPONENT_LOG_INFO("BMI270 calibration completed: {} samples", calibration_samples_.size());
        }
    }
}

void BMI270_IMU::simulate_sensor_data() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> noise_dist(0.0f, 0.01f);
    static std::uniform_real_distribution<float> motion_dist(-0.1f, 0.1f);
    
    // Simulate device motion (simple physics simulation)
    static float device_orientation[3] = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw
    static float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
    static float linear_acceleration[3] = {0.0f, 0.0f, 1.0f}; // Start with gravity
    
    // Add some random motion
    for (int i = 0; i < 3; ++i) {
        angular_velocity[i] += motion_dist(gen) * gesture_sensitivity_;
        angular_velocity[i] *= 0.95f; // Damping
        device_orientation[i] += angular_velocity[i] * 0.01f; // Integration
        
        linear_acceleration[i] += motion_dist(gen) * gesture_sensitivity_;
        linear_acceleration[i] *= 0.9f; // Damping
    }
    
    // Ensure Z-axis has gravity component
    linear_acceleration[2] = 1.0f + motion_dist(gen) * 0.1f;
    
    // Convert to sensor values with noise
    float accel_x = linear_acceleration[0] + noise_dist(gen);
    float accel_y = linear_acceleration[1] + noise_dist(gen);
    float accel_z = linear_acceleration[2] + noise_dist(gen);
    
    float gyro_x = angular_velocity[0] * 57.2958f + noise_dist(gen) * 10.0f; // Convert to deg/s
    float gyro_y = angular_velocity[1] * 57.2958f + noise_dist(gen) * 10.0f;
    float gyro_z = angular_velocity[2] * 57.2958f + noise_dist(gen) * 10.0f;
    
    // Simulate temperature
    float temperature = 25.0f + noise_dist(gen) * 2.0f;
    
    // Convert to raw values and update registers
    i16 raw_accel_x = convert_to_raw_accel(accel_x);
    i16 raw_accel_y = convert_to_raw_accel(accel_y);
    i16 raw_accel_z = convert_to_raw_accel(accel_z);
    
    i16 raw_gyro_x = convert_to_raw_gyro(gyro_x);
    i16 raw_gyro_y = convert_to_raw_gyro(gyro_y);
    i16 raw_gyro_z = convert_to_raw_gyro(gyro_z);
    
    i16 raw_temperature = static_cast<i16>((temperature - 23.0f) * 512);
    
    // Update registers
    registers_.acc_x_lsb = raw_accel_x & 0xFF;
    registers_.acc_x_msb = (raw_accel_x >> 8) & 0xFF;
    registers_.acc_y_lsb = raw_accel_y & 0xFF;
    registers_.acc_y_msb = (raw_accel_y >> 8) & 0xFF;
    registers_.acc_z_lsb = raw_accel_z & 0xFF;
    registers_.acc_z_msb = (raw_accel_z >> 8) & 0xFF;
    
    registers_.gyr_x_lsb = raw_gyro_x & 0xFF;
    registers_.gyr_x_msb = (raw_gyro_x >> 8) & 0xFF;
    registers_.gyr_y_lsb = raw_gyro_y & 0xFF;
    registers_.gyr_y_msb = (raw_gyro_y >> 8) & 0xFF;
    registers_.gyr_z_lsb = raw_gyro_z & 0xFF;
    registers_.gyr_z_msb = (raw_gyro_z >> 8) & 0xFF;
    
    registers_.temperature_lsb = raw_temperature & 0xFF;
    registers_.temperature_msb = (raw_temperature >> 8) & 0xFF;
    
    // Update status register
    registers_.status = 0x80; // Data ready
}

void BMI270_IMU::apply_calibration_to_data(IMUData& data) {
    // Apply accelerometer calibration
    data.accel_x_g -= calibration_.accel_offset[0];
    data.accel_y_g -= calibration_.accel_offset[1];
    data.accel_z_g -= calibration_.accel_offset[2];
    
    data.accel_x_g *= calibration_.accel_scale[0];
    data.accel_y_g *= calibration_.accel_scale[1];
    data.accel_z_g *= calibration_.accel_scale[2];
    
    // Apply gyroscope calibration
    data.gyro_x_dps -= calibration_.gyro_offset[0];
    data.gyro_y_dps -= calibration_.gyro_offset[1];
    data.gyro_z_dps -= calibration_.gyro_offset[2];
    
    data.gyro_x_dps *= calibration_.gyro_scale[0];
    data.gyro_y_dps *= calibration_.gyro_scale[1];
    data.gyro_z_dps *= calibration_.gyro_scale[2];
    
    // Add to calibration samples if calibrating
    if (calibration_in_progress_ && calibration_samples_.size() < 1000) {
        calibration_samples_.push_back(data);
    }
}

void BMI270_IMU::update_sensor_fusion(const IMUData& data) {
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - sensor_fusion_.last_update);
    float dt_seconds = dt.count() / 1000000.0f;
    
    if (dt_seconds > 0.001f) { // Minimum 1ms update
        madgwick_filter(data);
        quaternion_to_euler(sensor_fusion_.quaternion, sensor_fusion_.euler_angles);
        sensor_fusion_.last_update = now;
    }
}

void BMI270_IMU::detect_gestures(const IMUData& data) {
    // Simple gesture detection based on acceleration patterns
    
    if (tap_detection_enabled_ && detect_tap(data)) {
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(IMUGesture::TAP);
        statistics_.gestures_detected++;
        trigger_interrupt(IMUInterruptType::TAP_DETECT);
    }
    
    if (detect_double_tap(data)) {
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(IMUGesture::DOUBLE_TAP);
        statistics_.gestures_detected++;
        trigger_interrupt(IMUInterruptType::DOUBLE_TAP);
    }
    
    if (detect_shake(data)) {
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(IMUGesture::SHAKE);
        statistics_.gestures_detected++;
    }
    
    IMUGesture tilt_gesture;
    if (detect_tilt(data, tilt_gesture)) {
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(tilt_gesture);
        statistics_.gestures_detected++;
    }
    
    if (free_fall_detection_enabled_ && detect_free_fall(data)) {
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(IMUGesture::FREE_FALL);
        statistics_.gestures_detected++;
        trigger_interrupt(IMUInterruptType::FREE_FALL);
    }
    
    if (step_counter_enabled_ && detect_step(data)) {
        motion_state_.step_count++;
        statistics_.steps_counted++;
        if (gesture_history_.size() >= GESTURE_HISTORY_SIZE) {
            gesture_history_.pop();
        }
        gesture_history_.push(IMUGesture::STEP);
        trigger_interrupt(IMUInterruptType::STEP_DETECT);
    }
}

void BMI270_IMU::detect_orientation(const IMUData& data) {
    // Calculate orientation based on accelerometer data
    float ax = data.accel_x_g;
    float ay = data.accel_y_g;
    float az = data.accel_z_g;
    
    IMUOrientation new_orientation = IMUOrientation::UNKNOWN;
    
    // Determine dominant axis
    float abs_ax = std::abs(ax);
    float abs_ay = std::abs(ay);
    float abs_az = std::abs(az);
    
    if (abs_az > 0.8f) {
        new_orientation = (az > 0) ? IMUOrientation::FACE_UP : IMUOrientation::FACE_DOWN;
    } else if (abs_ay > 0.8f) {
        new_orientation = (ay > 0) ? IMUOrientation::PORTRAIT_UP : IMUOrientation::PORTRAIT_DOWN;
    } else if (abs_ax > 0.8f) {
        new_orientation = (ax > 0) ? IMUOrientation::LANDSCAPE_RIGHT : IMUOrientation::LANDSCAPE_LEFT;
    }
    
    // Update orientation stability
    if (new_orientation == motion_state_.current_orientation) {
        motion_state_.orientation_stability = std::min(motion_state_.orientation_stability + 0.1f, 1.0f);
    } else {
        motion_state_.orientation_stability = std::max(motion_state_.orientation_stability - 0.2f, 0.0f);
        
        if (motion_state_.orientation_stability < 0.3f) {
            motion_state_.current_orientation = new_orientation;
            trigger_interrupt(IMUInterruptType::ORIENTATION);
        }
    }
}

void BMI270_IMU::detect_motion_events(const IMUData& data) {
    float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
    
    // Update motion intensity
    motion_state_.motion_intensity = low_pass_filter(accel_magnitude, motion_state_.motion_intensity, 0.1f);
    statistics_.current_motion_intensity = motion_state_.motion_intensity;
    
    // Motion detection
    if (motion_detection_enabled_) {
        float motion_delta = std::abs(accel_magnitude - motion_state_.last_accel_magnitude);
        if (motion_delta > motion_threshold_) {
            trigger_interrupt(IMUInterruptType::MOTION_DETECT);
        }
    }
    
    // No motion detection (inverse)
    if (motion_state_.motion_intensity < 0.1f) {
        trigger_interrupt(IMUInterruptType::NO_MOTION);
    }
    
    motion_state_.last_accel_magnitude = accel_magnitude;
    
    // Update acceleration history for gesture detection
    motion_state_.accel_magnitude_history[motion_state_.history_index] = accel_magnitude;
    motion_state_.history_index = (motion_state_.history_index + 1) % 10;
}

void BMI270_IMU::update_fifo() {
    if (fifo_buffer_.size() >= FIFO_SIZE) {
        // FIFO overflow
        statistics_.fifo_overflows++;
        trigger_interrupt(IMUInterruptType::FIFO_FULL);
        return;
    }
    
    // Add current data to FIFO
    auto data_result = read_sensor_data();
    if (data_result) {
        fifo_buffer_.push(data_result.value());
        
        // Update FIFO length registers
        u16 fifo_length = static_cast<u16>(fifo_buffer_.size() * 12); // 12 bytes per sample
        registers_.fifo_length_lsb = fifo_length & 0xFF;
        registers_.fifo_length_msb = (fifo_length >> 8) & 0xFF;
        
        // Check watermark
        if (fifo_buffer_.size() >= FIFO_SIZE / 2) {
            trigger_interrupt(IMUInterruptType::FIFO_WATERMARK);
        }
    }
}

void BMI270_IMU::trigger_interrupt(IMUInterruptType interrupt_type) {
    interrupt_status_ |= static_cast<u32>(interrupt_type);
    
    if (registers_.int_map_data & static_cast<u32>(interrupt_type)) {
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(InterruptType::GPIO_BANK0); // IMU interrupt line
        }
        statistics_.interrupts_generated++;
    }
}

// Gesture detection implementations
bool BMI270_IMU::detect_tap(const IMUData& data) {
    float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
    
    static float last_magnitude = 1.0f;
    static auto last_tap_time = std::chrono::steady_clock::now();
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tap_time);
    
    // Detect sudden acceleration change
    float magnitude_change = accel_magnitude - last_magnitude;
    
    if (magnitude_change > tap_threshold_ && elapsed.count() > 200) { // Minimum 200ms between taps
        last_tap_time = now;
        last_magnitude = accel_magnitude;
        return true;
    }
    
    last_magnitude = accel_magnitude;
    return false;
}

bool BMI270_IMU::detect_double_tap(const IMUData& data) {
    static auto last_tap_time = std::chrono::steady_clock::now();
    static int tap_count = 0;
    
    if (detect_tap(data)) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tap_time);
        
        if (elapsed.count() < 500) { // Double tap within 500ms
            tap_count++;
            if (tap_count >= 2) {
                tap_count = 0;
                return true;
            }
        } else {
            tap_count = 1;
        }
        
        last_tap_time = now;
    }
    
    return false;
}

bool BMI270_IMU::detect_shake(const IMUData& data) {
    // Detect shaking by looking for rapid changes in acceleration
    static float shake_energy = 0.0f;
    static auto last_shake_time = std::chrono::steady_clock::now();
    
    float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
    float energy = std::abs(accel_magnitude - 1.0f); // Deviation from gravity
    
    shake_energy = low_pass_filter(energy, shake_energy, 0.3f);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_shake_time);
    
    if (shake_energy > (0.5f * gesture_sensitivity_) && elapsed.count() > 1000) {
        last_shake_time = now;
        return true;
    }
    
    return false;
}

bool BMI270_IMU::detect_tilt(const IMUData& data, IMUGesture& gesture_type) {
    static float baseline_x = 0.0f;
    static float baseline_y = 0.0f;
    static bool baseline_set = false;
    
    if (!baseline_set) {
        baseline_x = data.accel_x_g;
        baseline_y = data.accel_y_g;
        baseline_set = true;
        return false;
    }
    
    float tilt_x = data.accel_x_g - baseline_x;
    float tilt_y = data.accel_y_g - baseline_y;
    
    float tilt_threshold = 0.3f * gesture_sensitivity_;
    
    if (std::abs(tilt_x) > tilt_threshold) {
        gesture_type = (tilt_x > 0) ? IMUGesture::TILT_RIGHT : IMUGesture::TILT_LEFT;
        return true;
    }
    
    if (std::abs(tilt_y) > tilt_threshold) {
        gesture_type = (tilt_y > 0) ? IMUGesture::TILT_UP : IMUGesture::TILT_DOWN;
        return true;
    }
    
    return false;
}

bool BMI270_IMU::detect_rotation(const IMUData& data, IMUGesture& gesture_type) {
    static float rotation_accumulator = 0.0f;
    
    // Integrate gyroscope Z-axis for rotation detection
    rotation_accumulator += data.gyro_z_dps * 0.01f; // Assume 10ms update rate
    
    float rotation_threshold = 90.0f * gesture_sensitivity_; // 90 degrees
    
    if (std::abs(rotation_accumulator) > rotation_threshold) {
        gesture_type = (rotation_accumulator > 0) ? IMUGesture::ROTATION_CW : IMUGesture::ROTATION_CCW;
        rotation_accumulator = 0.0f; // Reset
        return true;
    }
    
    return false;
}

bool BMI270_IMU::detect_free_fall(const IMUData& data) {
    float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
    
    static int free_fall_counter = 0;
    
    if (accel_magnitude < free_fall_threshold_) {
        free_fall_counter++;
        if (free_fall_counter > 10) { // Sustained free fall for 100ms
            motion_state_.in_free_fall = true;
            free_fall_counter = 0;
            return true;
        }
    } else {
        free_fall_counter = 0;
        motion_state_.in_free_fall = false;
    }
    
    return false;
}

bool BMI270_IMU::detect_step(const IMUData& data) {
    static float step_energy = 0.0f;
    static auto last_step_time = std::chrono::steady_clock::now();
    
    [[maybe_unused]] float accel_magnitude = calculate_magnitude(data.accel_x_g, data.accel_y_g, data.accel_z_g);
    float vertical_component = std::abs(data.accel_z_g - 1.0f); // Deviation from gravity
    
    step_energy = high_pass_filter(vertical_component, step_energy, 0.1f);
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_step_time);
    
    // Step detection based on vertical acceleration pattern
    if (step_energy > (0.2f * gesture_sensitivity_) && elapsed.count() > 300) { // Minimum 300ms between steps
        last_step_time = now;
        motion_state_.last_step_time = now;
        return true;
    }
    
    return false;
}

// Sensor fusion implementations
void BMI270_IMU::complementary_filter(const IMUData& data) {
    // Simple complementary filter for pitch and roll
    float accel_pitch = std::atan2(data.accel_y_g, data.accel_z_g) * 180.0f / M_PI;
    float accel_roll = std::atan2(-data.accel_x_g, data.accel_z_g) * 180.0f / M_PI;
    
    // Integrate gyroscope
    sensor_fusion_.euler_angles[0] += data.gyro_x_dps * 0.01f; // pitch
    sensor_fusion_.euler_angles[1] += data.gyro_y_dps * 0.01f; // roll
    sensor_fusion_.euler_angles[2] += data.gyro_z_dps * 0.01f; // yaw
    
    // Apply complementary filter
    sensor_fusion_.euler_angles[0] = sensor_fusion_.alpha * sensor_fusion_.euler_angles[0] + 
                                    (1.0f - sensor_fusion_.alpha) * accel_pitch;
    sensor_fusion_.euler_angles[1] = sensor_fusion_.alpha * sensor_fusion_.euler_angles[1] + 
                                    (1.0f - sensor_fusion_.alpha) * accel_roll;
}

void BMI270_IMU::madgwick_filter(const IMUData& data) {
    // Simplified Madgwick AHRS algorithm
    float q1 = sensor_fusion_.quaternion[0], q2 = sensor_fusion_.quaternion[1];
    float q3 = sensor_fusion_.quaternion[2], q4 = sensor_fusion_.quaternion[3];
    
    // Convert gyroscope to rad/s
    float gx = data.gyro_x_dps * M_PI / 180.0f;
    float gy = data.gyro_y_dps * M_PI / 180.0f;
    float gz = data.gyro_z_dps * M_PI / 180.0f;
    
    // Normalize accelerometer measurement
    float ax = data.accel_x_g, ay = data.accel_y_g, az = data.accel_z_g;
    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }
    
    // Auxiliary variables to avoid repeated arithmetic
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _4q3 = 4.0f * q3;
    float _8q2 = 8.0f * q2;
    float _8q3 = 8.0f * q3;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q4q4 = q4 * q4;
    
    // Gradient descent algorithm corrective step
    float s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    float s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
    float s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
    
    norm = std::sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    if (norm > 0.0f) {
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
        s4 /= norm;
    }
    
    // Apply feedback step
    float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - sensor_fusion_.beta * s1;
    float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - sensor_fusion_.beta * s2;
    float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - sensor_fusion_.beta * s3;
    float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - sensor_fusion_.beta * s4;
    
    // Integrate rate of change to yield quaternion
    q1 += qDot1 * 0.01f; // Assume 10ms sample period
    q2 += qDot2 * 0.01f;
    q3 += qDot3 * 0.01f;
    q4 += qDot4 * 0.01f;
    
    // Normalize quaternion
    norm = std::sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    if (norm > 0.0f) {
        sensor_fusion_.quaternion[0] = q1 / norm;
        sensor_fusion_.quaternion[1] = q2 / norm;
        sensor_fusion_.quaternion[2] = q3 / norm;
        sensor_fusion_.quaternion[3] = q4 / norm;
    }
}

void BMI270_IMU::quaternion_to_euler(const float q[4], float euler[3]) {
    // Convert quaternion to Euler angles (pitch, roll, yaw)
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    
    // Pitch (x-axis rotation)
    float sinr_cosp = 2 * (q1 * q2 + q3 * q4);
    float cosr_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Roll (y-axis rotation)
    float sinp = 2 * (q1 * q3 - q4 * q2);
    if (std::abs(sinp) >= 1) {
        euler[1] = std::copysign(M_PI / 2, sinp) * 180.0f / M_PI;
    } else {
        euler[1] = std::asin(sinp) * 180.0f / M_PI;
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q1 * q4 + q2 * q3);
    float cosy_cosp = 1 - 2 * (q3 * q3 + q4 * q4);
    euler[2] = std::atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// Utility function implementations
float BMI270_IMU::calculate_magnitude(float x, float y, float z) {
    return std::sqrt(x*x + y*y + z*z);
}

float BMI270_IMU::low_pass_filter(float current, float previous, float alpha) {
    return alpha * previous + (1.0f - alpha) * current;
}

float BMI270_IMU::high_pass_filter(float current, float previous, float alpha) {
    return alpha * (previous + current - previous);
}

i16 BMI270_IMU::convert_to_raw_accel(float value_g) {
    float scale_factor;
    switch (accel_range_) {
        case IMUAccelRange::RANGE_2G:  scale_factor = 16384.0f; break;
        case IMUAccelRange::RANGE_4G:  scale_factor = 8192.0f; break;
        case IMUAccelRange::RANGE_8G:  scale_factor = 4096.0f; break;
        case IMUAccelRange::RANGE_16G: scale_factor = 2048.0f; break;
        default: scale_factor = 16384.0f; break;
    }
    
    return static_cast<i16>(std::clamp(value_g * scale_factor, -32768.0f, 32767.0f));
}

i16 BMI270_IMU::convert_to_raw_gyro(float value_dps) {
    float scale_factor;
    switch (gyro_range_) {
        case IMUGyroRange::RANGE_125DPS:  scale_factor = 262.14f; break;
        case IMUGyroRange::RANGE_250DPS:  scale_factor = 131.07f; break;
        case IMUGyroRange::RANGE_500DPS:  scale_factor = 65.54f; break;
        case IMUGyroRange::RANGE_1000DPS: scale_factor = 32.77f; break;
        case IMUGyroRange::RANGE_2000DPS: scale_factor = 16.38f; break;
        default: scale_factor = 65.54f; break;
    }
    
    return static_cast<i16>(std::clamp(value_dps * scale_factor, -32768.0f, 32767.0f));
}

float BMI270_IMU::convert_from_raw_accel(i16 raw_value) {
    float scale_factor;
    switch (accel_range_) {
        case IMUAccelRange::RANGE_2G:  scale_factor = 1.0f / 16384.0f; break;
        case IMUAccelRange::RANGE_4G:  scale_factor = 1.0f / 8192.0f; break;
        case IMUAccelRange::RANGE_8G:  scale_factor = 1.0f / 4096.0f; break;
        case IMUAccelRange::RANGE_16G: scale_factor = 1.0f / 2048.0f; break;
        default: scale_factor = 1.0f / 16384.0f; break;
    }
    
    return raw_value * scale_factor;
}

float BMI270_IMU::convert_from_raw_gyro(i16 raw_value) {
    float scale_factor;
    switch (gyro_range_) {
        case IMUGyroRange::RANGE_125DPS:  scale_factor = 1.0f / 262.14f; break;
        case IMUGyroRange::RANGE_250DPS:  scale_factor = 1.0f / 131.07f; break;
        case IMUGyroRange::RANGE_500DPS:  scale_factor = 1.0f / 65.54f; break;
        case IMUGyroRange::RANGE_1000DPS: scale_factor = 1.0f / 32.77f; break;
        case IMUGyroRange::RANGE_2000DPS: scale_factor = 1.0f / 16.38f; break;
        default: scale_factor = 1.0f / 65.54f; break;
    }
    
    return raw_value * scale_factor;
}

// Communication helper implementations
Result<void> BMI270_IMU::write_reg_i2c(u8 reg_addr, u8 value) {
    if (!i2c_controller_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "I2C controller not available"));
    }
    
    // Simple register write simulation
    // In a real implementation, this would use the I2C controller
    switch (reg_addr) {
        case 0x40: registers_.acc_conf = value; break;
        case 0x41: registers_.acc_range = value; break;
        case 0x42: registers_.gyr_conf = value; break;
        case 0x43: registers_.gyr_range = value; break;
        case 0x7C: registers_.pwr_conf = value; break;
        case 0x7D: registers_.pwr_ctrl = value; break;
        default: break; // Ignore unknown registers
    }
    
    return {};
}

Result<u8> BMI270_IMU::read_reg_i2c(u8 reg_addr) {
    if (!i2c_controller_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "I2C controller not available"));
    }
    
    // Simple register read simulation
    switch (reg_addr) {
        case 0x00: return registers_.chip_id;
        case 0x02: return registers_.err_reg;
        case 0x03: return registers_.status;
        case 0x0C: return registers_.acc_x_lsb;
        case 0x0D: return registers_.acc_x_msb;
        case 0x0E: return registers_.acc_y_lsb;
        case 0x0F: return registers_.acc_y_msb;
        case 0x10: return registers_.acc_z_lsb;
        case 0x11: return registers_.acc_z_msb;
        case 0x12: return registers_.gyr_x_lsb;
        case 0x13: return registers_.gyr_x_msb;
        case 0x14: return registers_.gyr_y_lsb;
        case 0x15: return registers_.gyr_y_msb;
        case 0x16: return registers_.gyr_z_lsb;
        case 0x17: return registers_.gyr_z_msb;
        case 0x22: return registers_.temperature_lsb;
        case 0x23: return registers_.temperature_msb;
        case 0x40: return registers_.acc_conf;
        case 0x41: return registers_.acc_range;
        case 0x42: return registers_.gyr_conf;
        case 0x43: return registers_.gyr_range;
        case 0x7C: return registers_.pwr_conf;
        case 0x7D: return registers_.pwr_ctrl;
        default: return 0x00;
    }
}

Result<void> BMI270_IMU::write_reg_spi(u8 reg_addr, u8 value) {
    if (!spi_controller_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "SPI controller not available"));
    }
    
    // SPI register write simulation (similar to I2C)
    return write_reg_i2c(reg_addr, value);
}

Result<u8> BMI270_IMU::read_reg_spi(u8 reg_addr) {
    if (!spi_controller_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "SPI controller not available"));
    }
    
    // SPI register read simulation (similar to I2C)
    return read_reg_i2c(reg_addr);
}

void BMI270_IMU::clear_statistics() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    statistics_ = {};
}

void BMI270_IMU::dump_status() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    COMPONENT_LOG_INFO("=== BMI270 IMU Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Interface: {}, Address: 0x{:02X}", 
                          static_cast<u8>(interface_), device_address_);
        COMPONENT_LOG_INFO("Power mode: {}", static_cast<u8>(power_mode_));
        COMPONENT_LOG_INFO("Accelerometer: range={}, ODR={}", 
                          static_cast<u8>(accel_range_), static_cast<u8>(accel_odr_));
        COMPONENT_LOG_INFO("Gyroscope: range={}, ODR={}", 
                          static_cast<u8>(gyro_range_), static_cast<u8>(gyro_odr_));
        
        COMPONENT_LOG_INFO("Motion detection: enabled={}, threshold={:.2f}g",
                          motion_detection_enabled_, motion_threshold_);
        COMPONENT_LOG_INFO("Tap detection: enabled={}, threshold={:.2f}g",
                          tap_detection_enabled_, tap_threshold_);
        COMPONENT_LOG_INFO("Step counter: enabled={}, count={}",
                          step_counter_enabled_, motion_state_.step_count);
        COMPONENT_LOG_INFO("Free fall detection: enabled={}, threshold={:.2f}g",
                          free_fall_detection_enabled_, free_fall_threshold_);
        
        COMPONENT_LOG_INFO("Sensor fusion: enabled={}", sensor_fusion_.enabled);
        if (sensor_fusion_.enabled) {
            COMPONENT_LOG_INFO("  Euler angles: pitch={:.1f}°, roll={:.1f}°, yaw={:.1f}°",
                              sensor_fusion_.euler_angles[0], sensor_fusion_.euler_angles[1], 
                              sensor_fusion_.euler_angles[2]);
        }
        
        COMPONENT_LOG_INFO("Current orientation: {} (stability={:.2f})",
                          static_cast<u8>(motion_state_.current_orientation),
                          motion_state_.orientation_stability);
        
        COMPONENT_LOG_INFO("FIFO: {}/{} samples", fifo_buffer_.size(), FIFO_SIZE);
        COMPONENT_LOG_INFO("Gesture history: {}/{}", gesture_history_.size(), GESTURE_HISTORY_SIZE);
        
        COMPONENT_LOG_INFO("Calibration: in_progress={}", calibration_in_progress_);
        if (calibration_in_progress_) {
            COMPONENT_LOG_INFO("  Samples collected: {}/1000", calibration_samples_.size());
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Samples read: {}", statistics_.samples_read);
        COMPONENT_LOG_INFO("  Interrupts generated: {}", statistics_.interrupts_generated);
        COMPONENT_LOG_INFO("  Gestures detected: {}", statistics_.gestures_detected);
        COMPONENT_LOG_INFO("  Steps counted: {}", statistics_.steps_counted);
        COMPONENT_LOG_INFO("  Calibration cycles: {}", statistics_.calibration_cycles);
        COMPONENT_LOG_INFO("  FIFO overflows: {}", statistics_.fifo_overflows);
        COMPONENT_LOG_INFO("  Average data rate: {:.1f} Hz", statistics_.average_data_rate_hz);
        COMPONENT_LOG_INFO("  Motion intensity: {:.3f}", statistics_.current_motion_intensity);
        COMPONENT_LOG_INFO("  Orientation stability: {:.2f}", motion_state_.orientation_stability);
    }
}

}  // namespace m5tab5::emulator