#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/utils/logging.hpp"
#include <chrono>
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("GraphicsEngine");

GraphicsEngine::GraphicsEngine()
    : width_(0),
      height_(0),
      bpp_(32),
      framebuffer_(nullptr),
      framebuffer_size_(0) {
    COMPONENT_LOG_DEBUG("GraphicsEngine created");
}

GraphicsEngine::~GraphicsEngine() {
    if (initialized_.load()) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("GraphicsEngine destroyed");
}

Result<void> GraphicsEngine::initialize(const Configuration& config) {
    if (initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Graphics engine already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing graphics engine");
    
    // Extract display configuration (stub implementation)
    width_ = 1280;  // M5Tab5 default width
    height_ = 720;  // M5Tab5 default height
    bpp_ = 32;      // 32-bit RGBA
    framebuffer_size_ = width_ * height_ * (bpp_ / 8);
    framebuffer_ = std::make_unique<u8[]>(framebuffer_size_);
    
    // Clear framebuffer
    std::memset(framebuffer_.get(), 0, framebuffer_size_);
    
    // Initialize timing
    last_frame_time_ = std::chrono::steady_clock::now();
    
    initialized_.store(true);
    COMPONENT_LOG_INFO("Graphics engine initialized successfully (stub implementation)");
    
    return {};
}

Result<void> GraphicsEngine::shutdown() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    COMPONENT_LOG_INFO("Shutting down graphics engine");
    
    framebuffer_.reset();
    framebuffer_size_ = 0;
    width_ = 0;
    height_ = 0;
    
    initialized_.store(false);
    COMPONENT_LOG_INFO("Graphics engine shutdown complete");
    
    return {};
}

Result<void> GraphicsEngine::reset() {
    COMPONENT_LOG_INFO("Resetting graphics engine");
    
    if (framebuffer_) {
        std::memset(framebuffer_.get(), 0, framebuffer_size_);
    }
    
    frames_rendered_ = 0;
    current_fps_ = 0.0;
    last_frame_time_ = std::chrono::steady_clock::now();
    
    return {};
}

Result<void> GraphicsEngine::render_frame() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    // Stub implementation - just update FPS counter
    update_fps_counter();
    frames_rendered_++;
    
    return {};
}

Result<void> GraphicsEngine::clear_screen(u32 color) {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (!framebuffer_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No framebuffer available"));
    }
    
    // Fill framebuffer with color (RGBA)
    u32* pixels = reinterpret_cast<u32*>(framebuffer_.get());
    std::fill(pixels, pixels + (width_ * height_), color);
    
    return {};
}

Result<void> GraphicsEngine::present() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    // Stub implementation - would present to actual display
    return {};
}

Result<void> GraphicsEngine::set_pixel(u32 x, u32 y, u32 color) {
    RETURN_IF_ERROR(validate_coordinates(x, y));
    
    size_t offset = calculate_pixel_offset(x, y);
    u32* pixel = reinterpret_cast<u32*>(framebuffer_.get() + offset);
    *pixel = color;
    
    return {};
}

Result<u32> GraphicsEngine::get_pixel(u32 x, u32 y) const {
    RETURN_IF_ERROR(validate_coordinates(x, y));
    
    size_t offset = calculate_pixel_offset(x, y);
    const u32* pixel = reinterpret_cast<const u32*>(framebuffer_.get() + offset);
    
    return *pixel;
}

Result<void> GraphicsEngine::draw_rectangle(u32 x, u32 y, u32 width, u32 height, u32 color) {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    // Draw rectangle by setting individual pixels
    for (u32 dy = 0; dy < height && (y + dy) < height_; ++dy) {
        for (u32 dx = 0; dx < width && (x + dx) < width_; ++dx) {
            RETURN_IF_ERROR(set_pixel(x + dx, y + dy, color));
        }
    }
    
    return {};
}

void GraphicsEngine::update_fps_counter() {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_frame_time_);
    
    if (elapsed.count() > 0) {
        current_fps_ = 1000000.0 / elapsed.count();
    }
    
    last_frame_time_ = current_time;
}

Result<void> GraphicsEngine::validate_coordinates(u32 x, u32 y) const {
    if (x >= width_ || y >= height_) {
        return unexpected(MAKE_ERROR(MEMORY_OUT_OF_BOUNDS,
            "Pixel coordinates out of bounds"));
    }
    return {};
}

size_t GraphicsEngine::calculate_pixel_offset(u32 x, u32 y) const {
    return (y * width_ + x) * (bpp_ / 8);
}

}  // namespace m5tab5::emulator