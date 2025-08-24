#include "emulator/graphics/touch_input.hpp"
#include "emulator/utils/logging.hpp"

#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("TouchInput");

TouchInput::TouchInput()
    : cal_min_x_(0), cal_max_x_(DISPLAY_WIDTH - 1),
      cal_min_y_(0), cal_max_y_(DISPLAY_HEIGHT - 1) {
    // Initialize all touch points as inactive
    for (auto& point : touch_points_) {
        point.active = false;
        point.id = 0;
    }
    
    COMPONENT_LOG_DEBUG("TouchInput created with calibration bounds: {}x{}", 
                       cal_max_x_ + 1, cal_max_y_ + 1);
}

TouchInput::~TouchInput() {
    COMPONENT_LOG_DEBUG("TouchInput destroyed");
}

void TouchInput::addTouchPoint(const TouchPoint& point) {
    // Find an available slot or replace the oldest one
    for (uint8_t i = 0; i < MAX_TOUCH_POINTS; ++i) {
        if (!touch_points_[i].active) {
            touch_points_[i] = point;
            touch_points_[i].id = next_id_++;
            touch_points_[i].active = true;
            
            COMPONENT_LOG_DEBUG("Added touch point {} at ({}, {}) with pressure {}", 
                               touch_points_[i].id, point.x, point.y, point.pressure);
            return;
        }
    }
    
    // No free slots - replace the first one (oldest)
    touch_points_[0] = point;
    touch_points_[0].id = next_id_++;
    touch_points_[0].active = true;
    
    COMPONENT_LOG_DEBUG("Replaced touch point {} at ({}, {}) with pressure {}", 
                       touch_points_[0].id, point.x, point.y, point.pressure);
}

void TouchInput::updateTouchPoint(uint8_t id, const TouchPoint& point) {
    for (auto& touch_point : touch_points_) {
        if (touch_point.active && touch_point.id == id) {
            touch_point.x = point.x;
            touch_point.y = point.y;
            touch_point.pressure = point.pressure;
            touch_point.size = point.size;
            
            COMPONENT_LOG_DEBUG("Updated touch point {} to ({}, {}) with pressure {}", 
                               id, point.x, point.y, point.pressure);
            return;
        }
    }
    
    // Touch point with this ID not found - add as new
    addTouchPoint(point);
}

void TouchInput::removeTouchPoint(uint8_t id) {
    for (auto& touch_point : touch_points_) {
        if (touch_point.active && touch_point.id == id) {
            touch_point.active = false;
            touch_point.pressure = 0;
            
            COMPONENT_LOG_DEBUG("Removed touch point {}", id);
            return;
        }
    }
}

void TouchInput::clearAllTouchPoints() {
    for (auto& touch_point : touch_points_) {
        touch_point.active = false;
        touch_point.pressure = 0;
    }
    
    COMPONENT_LOG_DEBUG("Cleared all touch points");
}

std::vector<TouchPoint> TouchInput::getActiveTouchPoints() const {
    std::vector<TouchPoint> active_points;
    active_points.reserve(MAX_TOUCH_POINTS);
    
    for (const auto& touch_point : touch_points_) {
        if (touch_point.active) {
            active_points.push_back(touch_point);
        }
    }
    
    return active_points;
}

uint8_t TouchInput::getActiveTouchCount() const {
    uint8_t count = 0;
    
    for (const auto& touch_point : touch_points_) {
        if (touch_point.active) {
            count++;
        }
    }
    
    return count;
}

bool TouchInput::hasTouchInput() const {
    return getActiveTouchCount() > 0;
}

void TouchInput::setCalibration(uint16_t min_x, uint16_t max_x, uint16_t min_y, uint16_t max_y) {
    cal_min_x_ = min_x;
    cal_max_x_ = max_x;
    cal_min_y_ = min_y;
    cal_max_y_ = max_y;
    
    COMPONENT_LOG_INFO("Touch calibration set: X({}-{}), Y({}-{})", 
                      min_x, max_x, min_y, max_y);
}

TouchPoint TouchInput::calibratePoint(const TouchPoint& raw_point) const {
    TouchPoint calibrated_point = raw_point;
    
    // Apply calibration transformation
    if (cal_max_x_ > cal_min_x_) {
        float x_ratio = static_cast<float>(raw_point.x - cal_min_x_) / (cal_max_x_ - cal_min_x_);
        calibrated_point.x = static_cast<uint16_t>(std::clamp(x_ratio * DISPLAY_WIDTH, 0.0f, static_cast<float>(DISPLAY_WIDTH - 1)));
    }
    
    if (cal_max_y_ > cal_min_y_) {
        float y_ratio = static_cast<float>(raw_point.y - cal_min_y_) / (cal_max_y_ - cal_min_y_);
        calibrated_point.y = static_cast<uint16_t>(std::clamp(y_ratio * DISPLAY_HEIGHT, 0.0f, static_cast<float>(DISPLAY_HEIGHT - 1)));
    }
    
    return calibrated_point;
}

} // namespace m5tab5::emulator