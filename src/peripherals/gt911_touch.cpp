#include "emulator/peripherals/gt911_touch.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace m5tab5::emulator {

DECLARE_LOGGER("GT911_Touch");

GT911_Touch::GT911_Touch()
    : initialized_(false),
      calibration_in_progress_(false),
      device_address_(I2C_ADDRESS_PRIMARY),
      active_touch_count_(0),
      interrupt_controller_(nullptr),
      i2c_controller_(nullptr),
      update_interval_ms_(10) {
    
    // Initialize touch configuration
    touch_config_.resolution = TouchResolution::RES_1280x720;
    touch_config_.sensitivity = TouchSensitivity::NORMAL;
    touch_config_.max_touch_points = MAX_TOUCH_POINTS;
    touch_config_.touch_threshold = 30;
    touch_config_.leave_threshold = 20;
    touch_config_.gesture_recognition = true;
    touch_config_.palm_rejection = true;
    
    // Initialize calibration
    calibration_.x_min = 0;
    calibration_.x_max = DEFAULT_WIDTH - 1;
    calibration_.y_min = 0;
    calibration_.y_max = DEFAULT_HEIGHT - 1;
    calibration_.x_scale = 1.0f;
    calibration_.y_scale = 1.0f;
    
    // Initialize gesture detector
    gesture_detector_.history.fill({});
    gesture_detector_.current_gesture = TouchGestureType::NONE;
    
    // Clear current touches
    current_touches_.fill({});
    
    COMPONENT_LOG_DEBUG("GT911 touch controller created");
}

GT911_Touch::~GT911_Touch() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("GT911 touch controller destroyed");
}

Result<void> GT911_Touch::initialize(const Configuration& config,
                                    InterruptController* interrupt_controller,
                                    I2CController* i2c_controller) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "GT911 touch controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing GT911 touch controller");
    
    interrupt_controller_ = interrupt_controller;
    i2c_controller_ = i2c_controller;
    
    // Initialize registers with default values
    registers_ = {};
    registers_.product_id = PRODUCT_ID;
    registers_.firmware_version = 0x1060;
    registers_.x_resolution = DEFAULT_WIDTH;
    registers_.y_resolution = DEFAULT_HEIGHT;
    registers_.max_touch_num = MAX_TOUCH_POINTS;
    registers_.config_version = 0x60;
    
    // Configure touch parameters
    registers_.screen_touch_level = touch_config_.touch_threshold;
    registers_.screen_leave_level = touch_config_.leave_threshold;
    registers_.filter = touch_config_.noise_reduction;
    registers_.refresh_rate = 1000 / update_interval_ms_;
    
    // Clear touch buffers
    while (!touch_buffer_.empty()) touch_buffer_.pop();
    while (!gesture_history_.empty()) gesture_history_.pop();
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_touch_time_ = last_update_;
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("GT911 touch controller initialized successfully");
    
    return {};
}

Result<void> GT911_Touch::shutdown() {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down GT911 touch controller");
    
    // Clear all touch data
    while (!touch_buffer_.empty()) touch_buffer_.pop();
    while (!gesture_history_.empty()) gesture_history_.pop();
    current_touches_.fill({});
    active_touch_count_ = 0;
    
    interrupt_controller_ = nullptr;
    i2c_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("GT911 touch controller shutdown completed");
    return {};
}

Result<void> GT911_Touch::configure_touch(const TouchConfiguration& touch_config) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    touch_config_ = touch_config;
    
    // Update registers based on configuration
    registers_.max_touch_num = touch_config.max_touch_points;
    registers_.screen_touch_level = touch_config.touch_threshold;
    registers_.screen_leave_level = touch_config.leave_threshold;
    registers_.filter = touch_config.noise_reduction;
    
    // Update module switches for features
    registers_.module_switch1 = 0x00;
    if (touch_config.gesture_recognition) {
        registers_.module_switch1 |= 0x01;
    }
    if (touch_config.palm_rejection) {
        registers_.module_switch1 |= 0x02;
    }
    if (touch_config.proximity_detection) {
        registers_.module_switch1 |= 0x04;
        proximity_detector_.enabled = true;
    }
    
    // Update timing parameters
    update_interval_ms_ = touch_config.debounce_time_ms;
    registers_.refresh_rate = 1000 / update_interval_ms_;
    
    COMPONENT_LOG_INFO("GT911 touch configuration updated");
    return {};
}

Result<void> GT911_Touch::set_resolution(TouchResolution resolution, u16 custom_width, u16 custom_height) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    switch (resolution) {
        case TouchResolution::RES_1280x720:
            registers_.x_resolution = 1280;
            registers_.y_resolution = 720;
            break;
        case TouchResolution::RES_1024x600:
            registers_.x_resolution = 1024;
            registers_.y_resolution = 600;
            break;
        case TouchResolution::RES_800x480:
            registers_.x_resolution = 800;
            registers_.y_resolution = 480;
            break;
        case TouchResolution::RES_CUSTOM:
            if (custom_width > 0 && custom_height > 0) {
                registers_.x_resolution = custom_width;
                registers_.y_resolution = custom_height;
            } else {
                return unexpected(MAKE_ERROR(INVALID_PARAMETER,
                    "Custom resolution requires valid width and height"));
            }
            break;
    }
    
    // Update coordinate ranges
    registers_.x_coordinate_range = registers_.x_resolution;
    registers_.y_coordinate_range = registers_.y_resolution;
    
    // Update calibration bounds
    calibration_.x_max = registers_.x_resolution - 1;
    calibration_.y_max = registers_.y_resolution - 1;
    
    COMPONENT_LOG_INFO("GT911 resolution set to {}x{}", 
                      registers_.x_resolution, registers_.y_resolution);
    
    return {};
}

Result<std::vector<TouchPointData>> GT911_Touch::read_touch_points() {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    std::vector<TouchPointData> touch_points;
    
    // Get all active touches
    for (u8 i = 0; i < MAX_TOUCH_POINTS; ++i) {
        if (current_touches_[i].active) {
            touch_points.push_back(current_touches_[i]);
        }
    }
    
    return touch_points;
}

Result<void> GT911_Touch::simulate_touch(u16 x, u16 y, bool pressed, u8 point_id) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    if (point_id >= MAX_TOUCH_POINTS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Touch point ID out of range"));
    }
    
    auto now = std::chrono::steady_clock::now();
    
    if (pressed) {
        // Create new touch point
        TouchPointData& touch = current_touches_[point_id];
        touch.point_id = point_id;
        touch.x = std::clamp(x, static_cast<u16>(0), static_cast<u16>(registers_.x_resolution - 1));
        touch.y = std::clamp(y, static_cast<u16>(0), static_cast<u16>(registers_.y_resolution - 1));
        touch.raw_x = touch.x;
        touch.raw_y = touch.y;
        touch.pressure = 512; // Default pressure
        touch.size = 15;      // Default size
        touch.active = true;
        touch.timestamp = now;
        
        if (!touch.active) {
            touch.first_contact = now;
        }
        
        // Apply calibration
        touch = apply_calibration_transform(touch);
        
        // Update active count
        u8 new_active_count = 0;
        for (const auto& t : current_touches_) {
            if (t.active) new_active_count++;
        }
        active_touch_count_ = new_active_count;
        
        statistics_.total_touches++;
        statistics_.max_simultaneous_touches = std::max(statistics_.max_simultaneous_touches, 
                                                       static_cast<u32>(active_touch_count_));
        
        // Trigger touch interrupt
        trigger_interrupt(GT911InterruptType::TOUCH_EVENT);
        
        COMPONENT_LOG_DEBUG("GT911 touch down: point={}, pos=({}, {})", point_id, x, y);
    } else {
        // Release touch point
        if (current_touches_[point_id].active) {
            current_touches_[point_id].active = false;
            
            // Calculate touch duration for statistics
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - current_touches_[point_id].first_contact);
            
            statistics_.average_touch_duration_ms = 
                (statistics_.average_touch_duration_ms + duration.count()) / 2.0;
            
            // Update active count
            active_touch_count_--;
            
            COMPONENT_LOG_DEBUG("GT911 touch up: point={}", point_id);
        }
    }
    
    last_touch_time_ = now;
    return {};
}

Result<void> GT911_Touch::simulate_multi_touch(const std::vector<TouchPoint>& points) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    // Clear all current touches first
    for (auto& touch : current_touches_) {
        touch.active = false;
    }
    active_touch_count_ = 0;
    
    // Apply new touches
    for (size_t i = 0; i < points.size() && i < MAX_TOUCH_POINTS; ++i) {
        const auto& point = points[i];
        if (point.active) {
            auto result = simulate_touch(point.x, point.y, true, static_cast<u8>(i));
            if (!result) {
                return result;
            }
        }
    }
    
    return {};
}

Result<TouchGesture> GT911_Touch::read_latest_gesture() {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    if (gesture_history_.empty()) {
        TouchGesture empty_gesture;
        empty_gesture.type = TouchGestureType::NONE;
        return empty_gesture;
    }
    
    TouchGesture latest_gesture = gesture_history_.back();
    return latest_gesture;
}

Result<void> GT911_Touch::write_register(u16 reg_addr, u8 value) {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    // Update appropriate register (simplified implementation)
    switch (reg_addr) {
        case 0x8047: registers_.config_version = value; break;
        case 0x8048: registers_.x_resolution = (registers_.x_resolution & 0xFF00) | value; break;
        case 0x8049: registers_.x_resolution = (registers_.x_resolution & 0x00FF) | (value << 8); break;
        case 0x804A: registers_.y_resolution = (registers_.y_resolution & 0xFF00) | value; break;
        case 0x804B: registers_.y_resolution = (registers_.y_resolution & 0x00FF) | (value << 8); break;
        case 0x804C: registers_.max_touch_num = std::min(value, static_cast<u8>(MAX_TOUCH_POINTS)); break;
        case 0x804D: registers_.module_switch1 = value; break;
        case 0x804E: registers_.module_switch2 = value; break;
        case 0x8056: registers_.screen_touch_level = value; break;
        case 0x8057: registers_.screen_leave_level = value; break;
        case 0x8053: registers_.filter = value; break;
        case 0x814E: registers_.touch_status = value; break; // Touch status (usually read-only)
        default:
            COMPONENT_LOG_WARN("GT911 write to unimplemented register 0x{:04X} = 0x{:02X}", 
                              reg_addr, value);
            break;
    }
    
    return {};
}

Result<u8> GT911_Touch::read_register(u16 reg_addr) const {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GT911 touch controller not initialized"));
    }
    
    switch (reg_addr) {
        case 0x8140: return registers_.product_id & 0xFF;
        case 0x8141: return (registers_.product_id >> 8) & 0xFF;
        case 0x8142: return registers_.firmware_version & 0xFF;
        case 0x8143: return (registers_.firmware_version >> 8) & 0xFF;
        case 0x8144: return registers_.x_coordinate_range & 0xFF;
        case 0x8145: return (registers_.x_coordinate_range >> 8) & 0xFF;
        case 0x8146: return registers_.y_coordinate_range & 0xFF;
        case 0x8147: return (registers_.y_coordinate_range >> 8) & 0xFF;
        case 0x8148: return registers_.vendor_id;
        case 0x8047: return registers_.config_version;
        case 0x8048: return registers_.x_resolution & 0xFF;
        case 0x8049: return (registers_.x_resolution >> 8) & 0xFF;
        case 0x804A: return registers_.y_resolution & 0xFF;
        case 0x804B: return (registers_.y_resolution >> 8) & 0xFF;
        case 0x804C: return registers_.max_touch_num;
        case 0x804D: return registers_.module_switch1;
        case 0x804E: return registers_.module_switch2;
        case 0x8056: return registers_.screen_touch_level;
        case 0x8057: return registers_.screen_leave_level;
        case 0x8053: return registers_.filter;
        case 0x814E: return registers_.touch_status;
        case 0x8177: return registers_.gesture_flag;
        case 0x8178: return registers_.gesture_count;
        default:
            COMPONENT_LOG_WARN("GT911 read from unimplemented register 0x{:04X}", reg_addr);
            return 0x00;
    }
}

void GT911_Touch::update() {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);
    
    if (elapsed.count() >= update_interval_ms_) {
        process_touch_input();
        detect_gestures();
        update_proximity_simulation();
        update_touch_registers();
        
        last_update_ = now;
    }
}

void GT911_Touch::process_touch_input() {
    // Apply noise filtering to active touches
    std::vector<TouchPointData> active_points;
    for (auto& touch : current_touches_) {
        if (touch.active) {
            active_points.push_back(touch);
        }
    }
    
    if (!active_points.empty()) {
        apply_noise_filtering(active_points);
        apply_palm_rejection(active_points);
        
        // Update current touches with filtered data
        size_t point_index = 0;
        for (auto& touch : current_touches_) {
            if (touch.active && point_index < active_points.size()) {
                touch = active_points[point_index++];
            }
        }
    }
}

void GT911_Touch::detect_gestures() {
    if (!touch_config_.gesture_recognition) {
        return;
    }
    
    // Simple gesture detection based on active touches
    std::vector<TouchPointData> active_points;
    for (const auto& touch : current_touches_) {
        if (touch.active) {
            active_points.push_back(touch);
        }
    }
    
    if (active_points.empty()) {
        gesture_detector_.current_gesture = TouchGestureType::NONE;
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    if (active_points.size() == 1) {
        // Single touch gestures
        TouchGestureType gesture = classify_single_touch_gesture(active_points[0]);
        if (gesture != TouchGestureType::NONE) {
            TouchGesture detected_gesture;
            detected_gesture.type = gesture;
            detected_gesture.start_x = active_points[0].x;
            detected_gesture.start_y = active_points[0].y;
            detected_gesture.timestamp = now;
            detected_gesture.finger_count = 1;
            detected_gesture.completed = true;
            
            if (gesture_history_.size() >= TOUCH_BUFFER_SIZE) {
                gesture_history_.pop();
            }
            gesture_history_.push(detected_gesture);
            
            statistics_.gestures_detected++;
            trigger_interrupt(GT911InterruptType::GESTURE_EVENT);
        }
    } else if (active_points.size() > 1) {
        // Multi-touch gestures
        TouchGestureType gesture = classify_multi_touch_gesture(active_points);
        if (gesture != TouchGestureType::NONE) {
            TouchGesture detected_gesture;
            detected_gesture.type = gesture;
            detected_gesture.start_x = active_points[0].x;
            detected_gesture.start_y = active_points[0].y;
            detected_gesture.timestamp = now;
            detected_gesture.finger_count = static_cast<u8>(active_points.size());
            detected_gesture.completed = true;
            
            if (gesture_history_.size() >= TOUCH_BUFFER_SIZE) {
                gesture_history_.pop();
            }
            gesture_history_.push(detected_gesture);
            
            statistics_.gestures_detected++;
            trigger_interrupt(GT911InterruptType::GESTURE_EVENT);
        }
    }
}

TouchGestureType GT911_Touch::classify_single_touch_gesture(const TouchPointData& point) {
    auto now = std::chrono::steady_clock::now();
    auto touch_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - point.first_contact);
    
    // Long press detection
    if (touch_duration.count() > 1000) {
        return TouchGestureType::LONG_PRESS;
    }
    
    // Simple tap detection (would need more sophisticated implementation for real gestures)
    if (touch_duration.count() > 50 && touch_duration.count() < 500) {
        return TouchGestureType::SINGLE_TAP;
    }
    
    return TouchGestureType::NONE;
}

TouchGestureType GT911_Touch::classify_multi_touch_gesture(const std::vector<TouchPointData>& points) {
    if (points.size() == 2) {
        return TouchGestureType::TWO_FINGER_TAP;
    } else if (points.size() == 3) {
        return TouchGestureType::THREE_FINGER_TAP;
    }
    
    return TouchGestureType::NONE;
}

void GT911_Touch::apply_noise_filtering(std::vector<TouchPointData>& points) {
    // Simple noise filtering - add some jitter to simulate real touch noise
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> jitter_dist(-2, 2);
    
    for (auto& point : points) {
        // Add small amount of noise simulation
        point.x = std::clamp(static_cast<int>(point.x) + jitter_dist(gen), 
                           0, static_cast<int>(registers_.x_resolution - 1));
        point.y = std::clamp(static_cast<int>(point.y) + jitter_dist(gen),
                           0, static_cast<int>(registers_.y_resolution - 1));
    }
}

void GT911_Touch::apply_palm_rejection(std::vector<TouchPointData>& points) {
    if (!touch_config_.palm_rejection) {
        return;
    }
    
    // Simple palm rejection based on touch size
    auto it = std::remove_if(points.begin(), points.end(), 
        [this](const TouchPointData& point) {
            return is_palm_touch(point);
        });
    
    size_t rejected_count = std::distance(it, points.end());
    points.erase(it, points.end());
    
    statistics_.palm_rejections += rejected_count;
}

TouchPointData GT911_Touch::apply_calibration_transform(const TouchPointData& raw_point) {
    TouchPointData calibrated_point = raw_point;
    
    // Apply scaling and offset
    float cal_x = (raw_point.raw_x - calibration_.x_offset) * calibration_.x_scale;
    float cal_y = (raw_point.raw_y - calibration_.y_offset) * calibration_.y_scale;
    
    // Apply coordinate flipping
    if (calibration_.flip_x) {
        cal_x = registers_.x_resolution - 1 - cal_x;
    }
    if (calibration_.flip_y) {
        cal_y = registers_.y_resolution - 1 - cal_y;
    }
    
    // Apply rotation (simplified - would need full matrix transform for arbitrary angles)
    if (calibration_.rotation == 90) {
        std::swap(cal_x, cal_y);
        cal_x = registers_.x_resolution - 1 - cal_x;
    } else if (calibration_.rotation == 180) {
        cal_x = registers_.x_resolution - 1 - cal_x;
        cal_y = registers_.y_resolution - 1 - cal_y;
    } else if (calibration_.rotation == 270) {
        std::swap(cal_x, cal_y);
        cal_y = registers_.y_resolution - 1 - cal_y;
    }
    
    // Clamp to screen bounds
    calibrated_point.x = std::clamp(static_cast<u16>(cal_x), 
                                  static_cast<u16>(0), 
                                  static_cast<u16>(registers_.x_resolution - 1));
    calibrated_point.y = std::clamp(static_cast<u16>(cal_y),
                                  static_cast<u16>(0),
                                  static_cast<u16>(registers_.y_resolution - 1));
    
    return calibrated_point;
}

bool GT911_Touch::is_palm_touch(const TouchPointData& point) {
    // Simple palm detection based on touch size
    return point.size > 100; // Large touches are likely palms
}

void GT911_Touch::update_proximity_simulation() {
    if (!proximity_detector_.enabled) {
        return;
    }
    
    // Simulate proximity detection state changes occasionally
    static auto last_change = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_change);
    
    if (elapsed.count() >= 10) { // Change every 10 seconds for simulation
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        if (gen() % 100 < 10) { // 10% chance to change state
            proximity_detector_.proximity_state = !proximity_detector_.proximity_state;
            proximity_detector_.last_proximity_time = now;
            statistics_.proximity_events++;
            trigger_interrupt(GT911InterruptType::PROXIMITY_EVENT);
        }
        
        last_change = now;
    }
}

void GT911_Touch::update_touch_registers() {
    // Update touch status register
    registers_.touch_status = active_touch_count_;
    if (active_touch_count_ > 0) {
        registers_.touch_status |= 0x80; // Set buffer status bit
    }
    
    // Update gesture registers
    if (!gesture_history_.empty()) {
        registers_.gesture_flag = 0x01; // Gesture available
        registers_.gesture_count = static_cast<u8>(std::min(gesture_history_.size(), size_t(255)));
    } else {
        registers_.gesture_flag = 0x00;
        registers_.gesture_count = 0;
    }
    
    // Update touch point data registers (simplified)
    for (u8 i = 0; i < MAX_TOUCH_POINTS && i < current_touches_.size(); ++i) {
        const auto& touch = current_touches_[i];
        u8* point_data = nullptr;
        
        switch (i) {
            case 0: point_data = registers_.touch_point1; break;
            case 1: point_data = registers_.touch_point2; break;
            case 2: point_data = registers_.touch_point3; break;
            case 3: point_data = registers_.touch_point4; break;
            case 4: point_data = registers_.touch_point5; break;
        }
        
        if (point_data && touch.active) {
            point_data[0] = touch.point_id;
            point_data[1] = touch.x & 0xFF;
            point_data[2] = (touch.x >> 8) & 0xFF;
            point_data[3] = touch.y & 0xFF;
            point_data[4] = (touch.y >> 8) & 0xFF;
            point_data[5] = touch.size;
            point_data[6] = touch.pressure & 0xFF;
            point_data[7] = (touch.pressure >> 8) & 0xFF;
        } else if (point_data) {
            // Clear inactive touch point
            std::fill_n(point_data, 8, 0);
        }
    }
}

void GT911_Touch::trigger_interrupt(GT911InterruptType interrupt_type) {
    if (interrupt_controller_) {
        // Map GT911 interrupts to system interrupts
        interrupt_controller_->trigger_interrupt(InterruptType::TOUCH_IRQ); // GT911 touch interrupt
    }
}

void GT911_Touch::clear_statistics() {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    statistics_ = {};
}

void GT911_Touch::dump_status() const {
    std::lock_guard<std::mutex> lock(touch_mutex_);
    
    COMPONENT_LOG_INFO("=== GT911 Touch Controller Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Resolution: {}x{}", registers_.x_resolution, registers_.y_resolution);
        COMPONENT_LOG_INFO("Active touches: {}/{}", active_touch_count_, registers_.max_touch_num);
        COMPONENT_LOG_INFO("Touch threshold: {}, Leave threshold: {}", 
                          registers_.screen_touch_level, registers_.screen_leave_level);
        COMPONENT_LOG_INFO("Gesture recognition: {}", touch_config_.gesture_recognition);
        COMPONENT_LOG_INFO("Palm rejection: {}", touch_config_.palm_rejection);
        COMPONENT_LOG_INFO("Proximity detection: {}", proximity_detector_.enabled);
        
        COMPONENT_LOG_INFO("Touch points:");
        for (u8 i = 0; i < MAX_TOUCH_POINTS; ++i) {
            const auto& touch = current_touches_[i];
            if (touch.active) {
                COMPONENT_LOG_INFO("  Point {}: ({}, {}) pressure={} size={}",
                                  i, touch.x, touch.y, touch.pressure, touch.size);
            }
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Total touches: {}", statistics_.total_touches);
        COMPONENT_LOG_INFO("  Gestures detected: {}", statistics_.gestures_detected);
        COMPONENT_LOG_INFO("  Palm rejections: {}", statistics_.palm_rejections);
        COMPONENT_LOG_INFO("  Average touch duration: {:.1f} ms", statistics_.average_touch_duration_ms);
        COMPONENT_LOG_INFO("  Max simultaneous touches: {}", statistics_.max_simultaneous_touches);
        COMPONENT_LOG_INFO("  Touch accuracy: {:.1f}%", statistics_.touch_accuracy_percentage);
    }
}

}  // namespace m5tab5::emulator