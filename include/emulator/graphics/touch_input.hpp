#pragma once

#include "emulator/core/types.hpp"
#include <vector>
#include <chrono>

namespace m5tab5::emulator {

/**
 * @brief Touch input handling for GT911 touch controller emulation
 */
struct TouchPoint {
    uint16_t x, y;          // Coordinates
    uint16_t pressure;      // Pressure level (0-1023)
    uint8_t size;           // Touch size
    uint8_t id;             // Touch ID for multi-touch
    bool active;            // Whether this touch point is active
    
    TouchPoint() : x(0), y(0), pressure(0), size(0), id(0), active(false) {}
    TouchPoint(uint16_t x_, uint16_t y_, uint16_t pressure_ = 512, uint8_t size_ = 10)
        : x(x_), y(y_), pressure(pressure_), size(size_), id(0), active(true) {}
};

class TouchInput {
public:
    static constexpr uint8_t MAX_TOUCH_POINTS = 5;
    
    explicit TouchInput();
    ~TouchInput();
    
    // Touch point management
    void addTouchPoint(const TouchPoint& point);
    void updateTouchPoint(uint8_t id, const TouchPoint& point);
    void removeTouchPoint(uint8_t id);
    void clearAllTouchPoints();
    
    // Current state
    std::vector<TouchPoint> getActiveTouchPoints() const;
    uint8_t getActiveTouchCount() const;
    bool hasTouchInput() const;
    
    // Calibration
    void setCalibration(uint16_t min_x, uint16_t max_x, uint16_t min_y, uint16_t max_y);
    TouchPoint calibratePoint(const TouchPoint& raw_point) const;

private:
    std::array<TouchPoint, MAX_TOUCH_POINTS> touch_points_;
    uint16_t cal_min_x_, cal_max_x_, cal_min_y_, cal_max_y_;
    uint8_t next_id_ = 0;
};

} // namespace m5tab5::emulator