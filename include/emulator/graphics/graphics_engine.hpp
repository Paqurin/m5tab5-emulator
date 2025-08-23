#pragma once

#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/config/configuration.hpp"

#include <memory>
#include <atomic>

namespace m5tab5::emulator {

/**
 * @brief Graphics rendering engine for M5Stack Tab5 display
 * 
 * Manages display output, framebuffer operations, and rendering pipeline
 * for the emulated M5Stack Tab5 device.
 */
class GraphicsEngine {
public:
    GraphicsEngine();
    ~GraphicsEngine();

    // Lifecycle management
    Result<void> initialize(const Configuration& config);
    Result<void> shutdown();
    Result<void> reset();

    // Rendering operations
    Result<void> render_frame();
    Result<void> clear_screen(u32 color);
    Result<void> present();

    // Display properties
    u32 get_width() const { return width_; }
    u32 get_height() const { return height_; }
    u32 get_bpp() const { return bpp_; }

    // Framebuffer access
    Result<void> set_pixel(u32 x, u32 y, u32 color);
    Result<u32> get_pixel(u32 x, u32 y) const;
    Result<void> draw_rectangle(u32 x, u32 y, u32 width, u32 height, u32 color);

    // Performance metrics
    double get_fps() const { return current_fps_; }
    u64 get_frames_rendered() const { return frames_rendered_; }

private:
    // Display configuration
    u32 width_;
    u32 height_;
    u32 bpp_;
    
    // Framebuffer
    std::unique_ptr<u8[]> framebuffer_;
    size_t framebuffer_size_;
    
    // Rendering state
    std::atomic<bool> initialized_{false};
    std::atomic<bool> vsync_enabled_{true};
    
    // Performance tracking
    mutable double current_fps_{0.0};
    mutable u64 frames_rendered_{0};
    mutable std::chrono::steady_clock::time_point last_frame_time_;
    
    // Internal methods
    void update_fps_counter();
    Result<void> validate_coordinates(u32 x, u32 y) const;
    size_t calculate_pixel_offset(u32 x, u32 y) const;
};

} // namespace m5tab5::emulator