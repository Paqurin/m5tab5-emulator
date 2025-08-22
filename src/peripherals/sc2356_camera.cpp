#include "emulator/peripherals/sc2356_camera.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <random>
#include <cmath>
#include <cstring>

namespace m5tab5::emulator {

SC2356Camera::SC2356Camera() 
    : initialized_(false)
    , capturing_active_(false)
    , preview_active_(false)
    , continuous_capture_(false)
    , interrupt_controller_(nullptr)
    , i2c_controller_(nullptr)
    , gpio_controller_(nullptr)
    , next_frame_number_(1)
    , interrupt_status_(0) {
    
    // Initialize default configuration
    config_.interface = CameraInterface::MIPI_CSI;
    config_.resolution = CameraResolution::VGA_640x480;
    config_.format = CameraPixelFormat::RGB565;
    config_.frame_rate = CameraFrameRate::FPS_30;
    
    // Initialize processor state
    processor_.current_brightness = 0.5f;
    processor_.target_brightness = 0.5f;
    processor_.current_exposure_ms = 33;
    processor_.current_iso = 100;
    processor_.current_focus_pos = 50;
    processor_.focus_quality = 0.8f;
    
    get_resolution_dimensions(config_.resolution, processor_.roi_width, processor_.roi_height);
    
    LOG_DEBUG("SC2356Camera", "Camera sensor initialized");
}

SC2356Camera::~SC2356Camera() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> SC2356Camera::initialize(const Configuration& config,
                                     InterruptController* interrupt_controller,
                                     I2CController* i2c_controller,
                                     GPIOController* gpio_controller) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (initialized_) {
        return make_error(ErrorCode::ALREADY_INITIALIZED, "SC2356 camera already initialized");
    }
    
    if (!interrupt_controller || !i2c_controller || !gpio_controller) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Required controllers cannot be null");
    }
    
    interrupt_controller_ = interrupt_controller;
    i2c_controller_ = i2c_controller;
    gpio_controller_ = gpio_controller;
    
    // Initialize camera registers to default values
    registers_ = SC2356Registers{};
    
    // Configure I2C device
    auto result = i2c_controller_->register_device(I2C_ADDRESS, "SC2356_Camera");
    if (!result) {
        return make_error(ErrorCode::DEVICE_ERROR, "Failed to register I2C device: " + result.error().message);
    }
    
    // Reset camera sensor
    auto reset_result = reset_sensor();
    if (!reset_result) {
        return reset_result;
    }
    
    last_update_ = std::chrono::steady_clock::now();
    last_capture_time_ = last_update_;
    
    initialized_ = true;
    
    LOG_INFO("SC2356Camera", "Camera sensor initialized successfully");
    return {};
}

Result<void> SC2356Camera::shutdown() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    // Stop any active capture
    if (capturing_active_ || preview_active_) {
        stop_continuous_capture();
        stop_preview();
    }
    
    // Clear frame buffer
    while (!frame_buffer_.empty()) {
        frame_buffer_.pop();
    }
    
    // Unregister I2C device
    if (i2c_controller_) {
        i2c_controller_->unregister_device(I2C_ADDRESS);
    }
    
    interrupt_controller_ = nullptr;
    i2c_controller_ = nullptr;
    gpio_controller_ = nullptr;
    
    initialized_ = false;
    
    LOG_INFO("SC2356Camera", "Camera sensor shutdown complete");
    return {};
}

Result<void> SC2356Camera::configure_camera(const CameraConfig& config) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    // Validate configuration
    if (config.frame_buffer_count < MIN_FRAME_BUFFER_COUNT || 
        config.frame_buffer_count > MAX_FRAME_BUFFER_COUNT) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "Invalid frame buffer count");
    }
    
    config_ = config;
    
    // Update registers based on configuration
    registers_.mode_ctrl = capturing_active_ ? 0x01 : 0x00;
    registers_.image_orientation = (config_.horizontal_mirror ? 0x01 : 0x00) | 
                                  (config_.vertical_flip ? 0x02 : 0x00);
    
    // Configure exposure
    if (config_.exposure_mode == CameraExposureMode::MANUAL) {
        u16 exposure_val = config_.exposure_time_ms * 10; // Convert to register units
        registers_.exposure_h = (exposure_val >> 8) & 0xFF;
        registers_.exposure_l = exposure_val & 0xFF;
        processor_.current_exposure_ms = config_.exposure_time_ms;
        processor_.current_iso = config_.iso_sensitivity;
    }
    
    // Configure frame rate
    registers_.frame_rate_ctrl = static_cast<u8>(config_.frame_rate);
    
    // Update ROI settings
    get_resolution_dimensions(config_.resolution, processor_.roi_width, processor_.roi_height);
    
    // Update auto-algorithm states
    processor_.current_effect = config_.effect;
    processor_.noise_reduction_enabled = config_.noise_reduction;
    
    LOG_INFO("SC2356Camera", "Camera configuration updated");
    return {};
}

Result<CameraConfig> SC2356Camera::get_camera_config() const {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    return config_;
}

Result<void> SC2356Camera::set_resolution(CameraResolution resolution) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    config_.resolution = resolution;
    get_resolution_dimensions(resolution, processor_.roi_width, processor_.roi_height);
    
    LOG_DEBUG("SC2356Camera", "Resolution set to {}x{}", processor_.roi_width, processor_.roi_height);
    return {};
}

Result<void> SC2356Camera::set_pixel_format(CameraPixelFormat format) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    config_.format = format;
    
    LOG_DEBUG("SC2356Camera", "Pixel format set to {}", static_cast<int>(format));
    return {};
}

Result<void> SC2356Camera::start_preview() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    preview_active_ = true;
    registers_.mode_ctrl |= 0x01; // Set streaming mode
    
    LOG_INFO("SC2356Camera", "Preview started");
    return {};
}

Result<void> SC2356Camera::stop_preview() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    preview_active_ = false;
    if (!capturing_active_) {
        registers_.mode_ctrl &= ~0x01; // Clear streaming mode if not capturing
    }
    
    LOG_INFO("SC2356Camera", "Preview stopped");
    return {};
}

Result<void> SC2356Camera::capture_frame() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    simulate_camera_capture();
    
    statistics_.frames_captured++;
    
    LOG_DEBUG("SC2356Camera", "Single frame captured");
    return {};
}

Result<void> SC2356Camera::start_continuous_capture() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    capturing_active_ = true;
    continuous_capture_ = true;
    registers_.mode_ctrl |= 0x01; // Set streaming mode
    
    LOG_INFO("SC2356Camera", "Continuous capture started");
    return {};
}

Result<void> SC2356Camera::stop_continuous_capture() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    capturing_active_ = false;
    continuous_capture_ = false;
    if (!preview_active_) {
        registers_.mode_ctrl &= ~0x01; // Clear streaming mode if not previewing
    }
    
    LOG_INFO("SC2356Camera", "Continuous capture stopped");
    return {};
}

Result<CameraFrame> SC2356Camera::get_latest_frame() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    if (frame_buffer_.empty()) {
        return make_error(ErrorCode::NO_DATA_AVAILABLE, "No frames available");
    }
    
    CameraFrame frame = frame_buffer_.back();
    return frame;
}

Result<void> SC2356Camera::enable_motion_detection(bool enable, u8 threshold) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    config_.motion_detection = enable;
    config_.motion_threshold = threshold;
    processor_.motion_detected = false;
    processor_.motion_level = 0.0f;
    
    if (enable) {
        registers_.interrupt_enable |= static_cast<u8>(CameraInterruptType::MOTION_DETECT);
    } else {
        registers_.interrupt_enable &= ~static_cast<u8>(CameraInterruptType::MOTION_DETECT);
    }
    
    LOG_DEBUG("SC2356Camera", "Motion detection {}, threshold: {}", 
              enable ? "enabled" : "disabled", threshold);
    return {};
}

Result<void> SC2356Camera::set_exposure_mode(CameraExposureMode mode) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    config_.exposure_mode = mode;
    
    switch (mode) {
        case CameraExposureMode::AUTO:
            registers_.awb_ctrl |= 0x01; // Enable AE
            break;
        case CameraExposureMode::MANUAL:
            registers_.awb_ctrl &= ~0x01; // Disable AE
            break;
        default:
            break;
    }
    
    LOG_DEBUG("SC2356Camera", "Exposure mode set to {}", static_cast<int>(mode));
    return {};
}

Result<void> SC2356Camera::enable_auto_focus(bool enable) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    config_.auto_focus = enable;
    processor_.focus_searching = false;
    
    LOG_DEBUG("SC2356Camera", "Auto focus {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> SC2356Camera::write_register(u16 reg_addr, u8 value) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    return write_reg_i2c(reg_addr, value);
}

Result<u8> SC2356Camera::read_register(u16 reg_addr) {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    return read_reg_i2c(reg_addr);
}

Result<bool> SC2356Camera::perform_self_test() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    // Simulate self-test procedure
    registers_.test_pattern = 0x01; // Enable test pattern
    
    // Generate test frame
    auto test_frame = generate_test_pattern(640, 480, CameraPixelFormat::RGB565);
    if (test_frame.empty()) {
        return false;
    }
    
    registers_.test_pattern = 0x00; // Disable test pattern
    
    LOG_INFO("SC2356Camera", "Self-test completed successfully");
    return true;
}

Result<void> SC2356Camera::reset_sensor() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    // Reset all registers to default values
    registers_ = SC2356Registers{};
    
    // Reset processor state
    processor_.current_brightness = 0.5f;
    processor_.target_brightness = 0.5f;
    processor_.current_exposure_ms = 33;
    processor_.current_iso = 100;
    processor_.current_focus_pos = 50;
    processor_.focus_quality = 0.8f;
    processor_.r_gain = 1.0f;
    processor_.g_gain = 1.0f;
    processor_.b_gain = 1.0f;
    processor_.motion_detected = false;
    processor_.motion_level = 0.0f;
    
    // Clear frame buffer
    while (!frame_buffer_.empty()) {
        frame_buffer_.pop();
    }
    
    next_frame_number_ = 1;
    interrupt_status_ = 0;
    
    LOG_INFO("SC2356Camera", "Sensor reset completed");
    return {};
}

Result<u16> SC2356Camera::get_chip_id() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return make_error(ErrorCode::NOT_INITIALIZED, "Camera not initialized");
    }
    
    return registers_.chip_id;
}

void SC2356Camera::update() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    if (dt < 10) { // Update at 100Hz maximum
        return;
    }
    
    // Update auto-algorithms
    if (config_.auto_exposure) {
        // Simulate auto-exposure convergence
        float brightness_error = processor_.target_brightness - processor_.current_brightness;
        if (std::abs(brightness_error) > 0.05f) {
            processor_.current_exposure_ms += static_cast<u16>(brightness_error * 10.0f);
            processor_.current_exposure_ms = std::clamp(processor_.current_exposure_ms, u16(1), u16(1000));
            statistics_.exposure_adjustments++;
        }
    }
    
    if (config_.auto_focus && processor_.focus_searching) {
        // Simulate auto-focus search
        static int focus_direction = 1;
        processor_.current_focus_pos += focus_direction * 2;
        
        if (processor_.current_focus_pos >= 100 || processor_.current_focus_pos <= 0) {
            focus_direction = -focus_direction;
            processor_.focus_searching = false;
            statistics_.focus_adjustments++;
        }
    }
    
    // Continuous capture
    if (continuous_capture_ && capturing_active_) {
        u32 frame_delay_ms = calculate_frame_rate_delay_ms(config_.frame_rate);
        auto time_since_capture = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_capture_time_).count();
        
        if (time_since_capture >= frame_delay_ms) {
            simulate_camera_capture();
            last_capture_time_ = now;
            statistics_.frames_captured++;
        }
    }
    
    // Update statistics
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count() >= 1000) {
        statistics_.average_frame_rate = statistics_.frames_captured > 0 ? 
            1000.0 / calculate_frame_rate_delay_ms(config_.frame_rate) : 0.0;
        statistics_.current_exposure_time_ms = processor_.current_exposure_ms;
        statistics_.current_iso = processor_.current_iso;
        statistics_.current_focus_position = processor_.current_focus_pos;
        statistics_.image_brightness_level = processor_.current_brightness;
        statistics_.focus_quality_score = processor_.focus_quality;
    }
    
    last_update_ = now;
}

void SC2356Camera::simulate_camera_capture() {
    u32 width, height;
    get_resolution_dimensions(config_.resolution, width, height);
    
    CameraFrame frame;
    frame.resolution = config_.resolution;
    frame.format = config_.format;
    frame.width = width;
    frame.height = height;
    frame.bytes_per_pixel = get_bytes_per_pixel(config_.format);
    frame.timestamp = std::chrono::steady_clock::now();
    frame.frame_number = next_frame_number_++;
    frame.is_valid = true;
    
    // Generate simulated image data
    size_t data_size = width * height * frame.bytes_per_pixel;
    frame.data.resize(data_size);
    
    if (registers_.test_pattern & 0x01) {
        // Generate test pattern
        frame.data = generate_test_pattern(width, height, config_.format);
    } else {
        // Generate realistic camera data (noise + gradients)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> noise_dist(0, 50);
        
        for (size_t i = 0; i < data_size; ++i) {
            u8 base_value = static_cast<u8>((i % width) * 255 / width); // Gradient
            u8 noise = static_cast<u8>(noise_dist(gen));
            frame.data[i] = std::clamp(base_value + noise, 0, 255);
        }
    }
    
    // Apply image processing
    process_frame(frame);
    
    // Add to frame buffer
    if (frame_buffer_.size() >= config_.frame_buffer_count) {
        frame_buffer_.pop(); // Remove oldest frame
        statistics_.frames_dropped++;
    }
    frame_buffer_.push(frame);
    
    // Trigger callback if set
    if (frame_callback_) {
        frame_callback_(frame);
    }
    
    // Trigger frame done interrupt
    trigger_interrupt(CameraInterruptType::FRAME_DONE);
}

void SC2356Camera::process_frame(CameraFrame& frame) {
    // Apply image effects and processing
    apply_image_effects(frame);
    
    // Update auto-algorithms
    if (config_.auto_exposure) {
        update_auto_exposure(frame);
    }
    
    if (config_.auto_white_balance) {
        update_auto_white_balance(frame);
    }
    
    if (config_.motion_detection) {
        detect_motion(frame);
    }
    
    // Calculate focus quality for auto-focus
    processor_.focus_quality = calculate_focus_quality(frame.data, frame.width, frame.height);
}

void SC2356Camera::apply_image_effects(CameraFrame& frame) {
    // Apply brightness and contrast
    apply_brightness_contrast(frame.data, config_.brightness, config_.contrast);
    
    // Apply saturation
    if (config_.format == CameraPixelFormat::RGB565 || config_.format == CameraPixelFormat::RGB888) {
        apply_saturation(frame.data, config_.saturation);
    }
    
    // Apply sharpness
    if (config_.sharpness != 0) {
        apply_sharpness(frame.data, frame.width, frame.height, config_.sharpness);
    }
    
    // Apply noise reduction
    if (processor_.noise_reduction_enabled) {
        apply_noise_reduction(frame.data, frame.width, frame.height);
    }
    
    // Apply color effects
    if (processor_.current_effect != CameraEffect::NONE) {
        apply_color_effect(frame.data, processor_.current_effect);
    }
    
    // Apply histogram equalization
    if (processor_.histogram_eq_enabled) {
        apply_histogram_equalization(frame.data);
    }
}

void SC2356Camera::update_auto_exposure(const CameraFrame& frame) {
    processor_.current_brightness = calculate_image_brightness(frame.data);
    
    // Simple auto-exposure algorithm
    float brightness_error = processor_.target_brightness - processor_.current_brightness;
    if (std::abs(brightness_error) > 0.02f) {
        if (brightness_error > 0) {
            // Image too dark, increase exposure
            processor_.current_exposure_ms = std::min(processor_.current_exposure_ms + 1, u16(1000));
        } else {
            // Image too bright, decrease exposure
            processor_.current_exposure_ms = std::max(processor_.current_exposure_ms - 1, u16(1));
        }
        
        // Update registers
        u16 exposure_val = processor_.current_exposure_ms * 10;
        registers_.exposure_h = (exposure_val >> 8) & 0xFF;
        registers_.exposure_l = exposure_val & 0xFF;
    }
}

void SC2356Camera::update_auto_white_balance(const CameraFrame& frame) {
    calculate_white_balance_gains(frame.data, 
                                 processor_.r_gain, 
                                 processor_.g_gain, 
                                 processor_.b_gain);
    
    // Update registers
    u16 r_gain_reg = static_cast<u16>(processor_.r_gain * 1024);
    u16 g_gain_reg = static_cast<u16>(processor_.g_gain * 1024);
    u16 b_gain_reg = static_cast<u16>(processor_.b_gain * 1024);
    
    registers_.awb_r_gain_h = (r_gain_reg >> 8) & 0xFF;
    registers_.awb_r_gain_l = r_gain_reg & 0xFF;
    registers_.awb_g_gain_h = (g_gain_reg >> 8) & 0xFF;
    registers_.awb_g_gain_l = g_gain_reg & 0xFF;
    registers_.awb_b_gain_h = (b_gain_reg >> 8) & 0xFF;
    registers_.awb_b_gain_l = b_gain_reg & 0xFF;
    
    statistics_.white_balance_adjustments++;
}

void SC2356Camera::detect_motion(const CameraFrame& frame) {
    if (processor_.previous_frame.empty()) {
        processor_.previous_frame = frame.data;
        return;
    }
    
    // Simple motion detection using frame difference
    size_t diff_sum = 0;
    size_t pixel_count = std::min(frame.data.size(), processor_.previous_frame.size());
    
    for (size_t i = 0; i < pixel_count; ++i) {
        int diff = std::abs(static_cast<int>(frame.data[i]) - 
                           static_cast<int>(processor_.previous_frame[i]));
        diff_sum += diff;
    }
    
    processor_.motion_level = static_cast<float>(diff_sum) / pixel_count / 255.0f;
    
    bool motion_detected = processor_.motion_level > (config_.motion_threshold / 100.0f);
    if (motion_detected && !processor_.motion_detected) {
        processor_.motion_detected = true;
        trigger_interrupt(CameraInterruptType::MOTION_DETECT);
        statistics_.motion_events_detected++;
    } else if (!motion_detected) {
        processor_.motion_detected = false;
    }
    
    processor_.previous_frame = frame.data;
}

void SC2356Camera::trigger_interrupt(CameraInterruptType interrupt_type) {
    u8 interrupt_mask = static_cast<u8>(interrupt_type);
    if (registers_.interrupt_enable & interrupt_mask) {
        interrupt_status_ |= interrupt_mask;
        registers_.interrupt_status = interrupt_status_;
        
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(42); // Camera interrupt line
        }
    }
}

// Image processing algorithm implementations

void SC2356Camera::apply_brightness_contrast(std::vector<u8>& data, u8 brightness, u8 contrast) {
    float bright_factor = (brightness - 128) / 128.0f;
    float contrast_factor = contrast / 128.0f;
    
    for (auto& pixel : data) {
        float value = pixel / 255.0f;
        value = (value - 0.5f) * contrast_factor + 0.5f + bright_factor;
        pixel = static_cast<u8>(std::clamp(value * 255.0f, 0.0f, 255.0f));
    }
}

void SC2356Camera::apply_saturation(std::vector<u8>& data, u8 saturation) {
    float sat_factor = saturation / 128.0f;
    
    // Simple saturation adjustment for RGB data
    for (size_t i = 0; i < data.size(); i += 3) {
        if (i + 2 >= data.size()) break;
        
        float r = data[i] / 255.0f;
        float g = data[i + 1] / 255.0f;
        float b = data[i + 2] / 255.0f;
        
        float gray = 0.299f * r + 0.587f * g + 0.114f * b;
        
        r = gray + sat_factor * (r - gray);
        g = gray + sat_factor * (g - gray);
        b = gray + sat_factor * (b - gray);
        
        data[i] = static_cast<u8>(std::clamp(r * 255.0f, 0.0f, 255.0f));
        data[i + 1] = static_cast<u8>(std::clamp(g * 255.0f, 0.0f, 255.0f));
        data[i + 2] = static_cast<u8>(std::clamp(b * 255.0f, 0.0f, 255.0f));
    }
}

void SC2356Camera::apply_sharpness(std::vector<u8>& data, u32 width, u32 height, i8 sharpness) {
    if (sharpness == 0 || data.empty()) return;
    
    float strength = sharpness / 100.0f;
    std::vector<u8> temp = data;
    
    // Simple sharpening kernel
    for (u32 y = 1; y < height - 1; ++y) {
        for (u32 x = 1; x < width - 1; ++x) {
            size_t idx = y * width + x;
            if (idx >= data.size()) continue;
            
            float center = temp[idx];
            float neighbors = (temp[idx - 1] + temp[idx + 1] + 
                             temp[idx - width] + temp[idx + width]) / 4.0f;
            
            float sharpened = center + strength * (center - neighbors);
            data[idx] = static_cast<u8>(std::clamp(sharpened, 0.0f, 255.0f));
        }
    }
}

void SC2356Camera::apply_noise_reduction(std::vector<u8>& data, u32 width, u32 height) {
    std::vector<u8> temp = data;
    
    // Simple 3x3 averaging filter
    for (u32 y = 1; y < height - 1; ++y) {
        for (u32 x = 1; x < width - 1; ++x) {
            size_t idx = y * width + x;
            if (idx >= data.size()) continue;
            
            u32 sum = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    size_t neighbor_idx = (y + dy) * width + (x + dx);
                    if (neighbor_idx < temp.size()) {
                        sum += temp[neighbor_idx];
                    }
                }
            }
            
            data[idx] = static_cast<u8>(sum / 9);
        }
    }
}

void SC2356Camera::apply_color_effect(std::vector<u8>& data, CameraEffect effect) {
    switch (effect) {
        case CameraEffect::GRAYSCALE:
            for (size_t i = 0; i < data.size(); i += 3) {
                if (i + 2 >= data.size()) break;
                u8 gray = static_cast<u8>(0.299f * data[i] + 0.587f * data[i + 1] + 0.114f * data[i + 2]);
                data[i] = data[i + 1] = data[i + 2] = gray;
            }
            break;
            
        case CameraEffect::SEPIA:
            for (size_t i = 0; i < data.size(); i += 3) {
                if (i + 2 >= data.size()) break;
                float r = data[i] / 255.0f;
                float g = data[i + 1] / 255.0f;
                float b = data[i + 2] / 255.0f;
                
                data[i] = static_cast<u8>(std::clamp((r * 0.393f + g * 0.769f + b * 0.189f) * 255.0f, 0.0f, 255.0f));
                data[i + 1] = static_cast<u8>(std::clamp((r * 0.349f + g * 0.686f + b * 0.168f) * 255.0f, 0.0f, 255.0f));
                data[i + 2] = static_cast<u8>(std::clamp((r * 0.272f + g * 0.534f + b * 0.131f) * 255.0f, 0.0f, 255.0f));
            }
            break;
            
        case CameraEffect::NEGATIVE:
            for (auto& pixel : data) {
                pixel = 255 - pixel;
            }
            break;
            
        default:
            break;
    }
}

float SC2356Camera::calculate_image_brightness(const std::vector<u8>& data) {
    if (data.empty()) return 0.0f;
    
    u64 sum = 0;
    for (u8 pixel : data) {
        sum += pixel;
    }
    
    return static_cast<float>(sum) / (data.size() * 255.0f);
}

float SC2356Camera::calculate_focus_quality(const std::vector<u8>& data, u32 width, u32 height) {
    if (data.empty() || width == 0 || height == 0) return 0.0f;
    
    // Calculate image gradient magnitude (simple focus measure)
    float gradient_sum = 0.0f;
    u32 count = 0;
    
    for (u32 y = 1; y < height - 1; ++y) {
        for (u32 x = 1; x < width - 1; ++x) {
            size_t idx = y * width + x;
            if (idx >= data.size()) continue;
            
            float dx = static_cast<float>(data[idx + 1]) - data[idx - 1];
            float dy = static_cast<float>(data[idx + width]) - data[idx - width];
            gradient_sum += std::sqrt(dx * dx + dy * dy);
            count++;
        }
    }
    
    return count > 0 ? gradient_sum / count / 255.0f : 0.0f;
}

void SC2356Camera::calculate_white_balance_gains(const std::vector<u8>& data, 
                                                float& r_gain, float& g_gain, float& b_gain) {
    if (data.size() < 3) return;
    
    u64 r_sum = 0, g_sum = 0, b_sum = 0;
    u32 pixel_count = 0;
    
    // Simple gray world white balance
    for (size_t i = 0; i < data.size(); i += 3) {
        if (i + 2 >= data.size()) break;
        r_sum += data[i];
        g_sum += data[i + 1];
        b_sum += data[i + 2];
        pixel_count++;
    }
    
    if (pixel_count > 0) {
        float r_avg = static_cast<float>(r_sum) / pixel_count;
        float g_avg = static_cast<float>(g_sum) / pixel_count;
        float b_avg = static_cast<float>(b_sum) / pixel_count;
        
        float gray_avg = (r_avg + g_avg + b_avg) / 3.0f;
        
        r_gain = gray_avg / r_avg;
        g_gain = gray_avg / g_avg;
        b_gain = gray_avg / b_avg;
        
        // Clamp gains to reasonable values
        r_gain = std::clamp(r_gain, 0.5f, 2.0f);
        g_gain = std::clamp(g_gain, 0.5f, 2.0f);
        b_gain = std::clamp(b_gain, 0.5f, 2.0f);
    }
}

void SC2356Camera::get_resolution_dimensions(CameraResolution resolution, u32& width, u32& height) {
    switch (resolution) {
        case CameraResolution::QVGA_320x240:   width = 320;  height = 240;  break;
        case CameraResolution::VGA_640x480:   width = 640;  height = 480;  break;
        case CameraResolution::SVGA_800x600:  width = 800;  height = 600;  break;
        case CameraResolution::XGA_1024x768:  width = 1024; height = 768;  break;
        case CameraResolution::HD_1280x720:   width = 1280; height = 720;  break;
        case CameraResolution::SXGA_1280x1024: width = 1280; height = 1024; break;
        case CameraResolution::UXGA_1600x1200: width = 1600; height = 1200; break;
        case CameraResolution::FHD_1920x1080: width = 1920; height = 1080; break;
        case CameraResolution::QXGA_2048x1536: width = 2048; height = 1536; break;
        case CameraResolution::QSXGA_2592x1944: width = 2592; height = 1944; break;
        default: width = 640; height = 480; break;
    }
}

u32 SC2356Camera::get_bytes_per_pixel(CameraPixelFormat format) {
    switch (format) {
        case CameraPixelFormat::RGB565:    return 2;
        case CameraPixelFormat::RGB888:    return 3;
        case CameraPixelFormat::YUV422:    return 2;
        case CameraPixelFormat::YUV420:    return 1; // Average
        case CameraPixelFormat::GRAYSCALE: return 1;
        case CameraPixelFormat::BAYER_RGGB:
        case CameraPixelFormat::BAYER_GRBG:
        case CameraPixelFormat::BAYER_GBRG:
        case CameraPixelFormat::BAYER_BGGR: return 1;
        case CameraPixelFormat::JPEG:      return 1; // Variable
        default: return 2;
    }
}

u32 SC2356Camera::calculate_frame_rate_delay_ms(CameraFrameRate frame_rate) {
    switch (frame_rate) {
        case CameraFrameRate::FPS_5:   return 200;
        case CameraFrameRate::FPS_10:  return 100;
        case CameraFrameRate::FPS_15:  return 67;
        case CameraFrameRate::FPS_20:  return 50;
        case CameraFrameRate::FPS_25:  return 40;
        case CameraFrameRate::FPS_30:  return 33;
        case CameraFrameRate::FPS_60:  return 17;
        case CameraFrameRate::FPS_120: return 8;
        default: return 33;
    }
}

std::vector<u8> SC2356Camera::generate_test_pattern(u32 width, u32 height, CameraPixelFormat format) {
    size_t bytes_per_pixel = get_bytes_per_pixel(format);
    size_t data_size = width * height * bytes_per_pixel;
    std::vector<u8> data(data_size);
    
    // Generate color bars test pattern
    u32 bar_width = width / 8;
    std::array<std::array<u8, 3>, 8> colors = {{
        {255, 255, 255}, // White
        {255, 255, 0},   // Yellow
        {0, 255, 255},   // Cyan
        {0, 255, 0},     // Green
        {255, 0, 255},   // Magenta
        {255, 0, 0},     // Red
        {0, 0, 255},     // Blue
        {0, 0, 0}        // Black
    }};
    
    for (u32 y = 0; y < height; ++y) {
        for (u32 x = 0; x < width; ++x) {
            u32 bar_index = std::min(x / bar_width, 7u);
            size_t pixel_idx = (y * width + x) * bytes_per_pixel;
            
            if (pixel_idx + bytes_per_pixel <= data.size()) {
                switch (format) {
                    case CameraPixelFormat::RGB888:
                        data[pixel_idx] = colors[bar_index][0];
                        data[pixel_idx + 1] = colors[bar_index][1];
                        data[pixel_idx + 2] = colors[bar_index][2];
                        break;
                        
                    case CameraPixelFormat::RGB565: {
                        u16 rgb565 = ((colors[bar_index][0] >> 3) << 11) |
                                    ((colors[bar_index][1] >> 2) << 5) |
                                    (colors[bar_index][2] >> 3);
                        data[pixel_idx] = rgb565 & 0xFF;
                        data[pixel_idx + 1] = (rgb565 >> 8) & 0xFF;
                        break;
                    }
                    
                    case CameraPixelFormat::GRAYSCALE:
                        data[pixel_idx] = static_cast<u8>(0.299f * colors[bar_index][0] + 
                                                         0.587f * colors[bar_index][1] + 
                                                         0.114f * colors[bar_index][2]);
                        break;
                        
                    default:
                        data[pixel_idx] = colors[bar_index][0];
                        if (bytes_per_pixel > 1) data[pixel_idx + 1] = colors[bar_index][1];
                        if (bytes_per_pixel > 2) data[pixel_idx + 2] = colors[bar_index][2];
                        break;
                }
            }
        }
    }
    
    return data;
}

Result<void> SC2356Camera::write_reg_i2c(u16 reg_addr, u8 value) {
    if (!i2c_controller_) {
        return make_error(ErrorCode::DEVICE_ERROR, "I2C controller not available");
    }
    
    std::vector<u8> data = {static_cast<u8>(reg_addr >> 8), static_cast<u8>(reg_addr & 0xFF), value};
    auto result = i2c_controller_->write_data(I2C_ADDRESS, data);
    if (!result) {
        return result;
    }
    
    // Update internal register state based on address
    switch (reg_addr) {
        case 0x0100: registers_.mode_ctrl = value; break;
        case 0x0101: registers_.image_orientation = value; break;
        case 0x0103: registers_.software_reset = value; break;
        case 0x3500: registers_.exposure_h = value; break;
        case 0x3501: registers_.exposure_l = value; break;
        case 0x3508: registers_.analog_gain = value; break;
        case 0x350A: registers_.digital_gain_h = value; break;
        case 0x350B: registers_.digital_gain_l = value; break;
        default: break;
    }
    
    return {};
}

Result<u8> SC2356Camera::read_reg_i2c(u16 reg_addr) {
    if (!i2c_controller_) {
        return make_error(ErrorCode::DEVICE_ERROR, "I2C controller not available");
    }
    
    // Return register value based on address
    switch (reg_addr) {
        case 0x300A: return (registers_.chip_id >> 8) & 0xFF;
        case 0x300B: return registers_.chip_id & 0xFF;
        case 0x0100: return registers_.mode_ctrl;
        case 0x0101: return registers_.image_orientation;
        case 0x3500: return registers_.exposure_h;
        case 0x3501: return registers_.exposure_l;
        case 0x3508: return registers_.analog_gain;
        case 0x350A: return registers_.digital_gain_h;
        case 0x350B: return registers_.digital_gain_l;
        case 0x4F00: return registers_.interrupt_status;
        case 0x4F01: return registers_.interrupt_enable;
        default: return 0x00;
    }
}

void SC2356Camera::clear_statistics() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    statistics_ = CameraStatistics{};
}

void SC2356Camera::dump_status() const {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    LOG_INFO("SC2356Camera", "=== SC2356 Camera Status ===");
    LOG_INFO("SC2356Camera", "Initialized: {}", initialized_);
    LOG_INFO("SC2356Camera", "Capturing: {}", capturing_active_);
    LOG_INFO("SC2356Camera", "Preview: {}", preview_active_);
    LOG_INFO("SC2356Camera", "Resolution: {}x{}", processor_.roi_width, processor_.roi_height);
    LOG_INFO("SC2356Camera", "Frame Rate: {} fps", statistics_.average_frame_rate);
    LOG_INFO("SC2356Camera", "Frames Captured: {}", statistics_.frames_captured);
    LOG_INFO("SC2356Camera", "Frames Dropped: {}", statistics_.frames_dropped);
    LOG_INFO("SC2356Camera", "Buffer Count: {}", frame_buffer_.size());
    LOG_INFO("SC2356Camera", "Exposure: {} ms", processor_.current_exposure_ms);
    LOG_INFO("SC2356Camera", "ISO: {}", processor_.current_iso);
    LOG_INFO("SC2356Camera", "Focus Position: {}", processor_.current_focus_pos);
    LOG_INFO("SC2356Camera", "Brightness: {:.2f}", processor_.current_brightness);
    LOG_INFO("SC2356Camera", "Motion Detected: {}", processor_.motion_detected);
    LOG_INFO("SC2356Camera", "Motion Level: {:.2f}", processor_.motion_level);
}

}  // namespace m5tab5::emulator