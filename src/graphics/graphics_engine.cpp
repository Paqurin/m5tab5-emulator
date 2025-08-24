#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/utils/logging.hpp"
#include <chrono>
#include <algorithm>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("GraphicsEngine");

GraphicsEngine::GraphicsEngine()
    : width_(0),
      height_(0),
      bpp_(32),
      framebuffer_(nullptr),
      display_bridge_(nullptr),
      display_adapter_(nullptr),
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
    
    COMPONENT_LOG_INFO("Initializing graphics engine with FramebufferBridge");
    
    // Extract display configuration
    width_ = DISPLAY_WIDTH;   // 1280 from constants
    height_ = DISPLAY_HEIGHT; // 720 from constants
    bpp_ = 32;               // 32-bit RGBA
    framebuffer_size_ = width_ * height_ * (bpp_ / 8);
    
    try {
        // Create main framebuffer
        framebuffer_ = std::make_unique<Framebuffer>(
            width_, height_, Framebuffer::PixelFormat::ARGB8888
        );
        
        // Initialize display bridge
        FramebufferBridge::BridgeConfig bridge_config;
        bridge_config.display_width = width_;
        bridge_config.display_height = height_;
        bridge_config.target_fps = 60;
        bridge_config.enable_vsync = true;
        bridge_config.enable_touch = true;
        bridge_config.window_title = "M5Stack Tab5 Emulator";
        
        display_bridge_ = std::make_unique<FramebufferBridge>(bridge_config);
        RETURN_IF_ERROR(display_bridge_->initialize());
        
        // Initialize display adapter for M5Stack Tab5 optimizations
        DisplayAdapter::AdapterConfig adapter_config;
        adapter_config.display_mode = DisplayAdapter::DisplayMode::RGB888;
        adapter_config.refresh_mode = DisplayAdapter::RefreshMode::VSYNC;
        adapter_config.hardware_acceleration = true;
        adapter_config.color_correction = true;
        
        display_adapter_ = std::make_unique<DisplayAdapter>(adapter_config);
        
        // Clear framebuffer
        framebuffer_->clear();
        
        // Initialize timing
        last_frame_time_ = std::chrono::steady_clock::now();
        
        initialized_.store(true);
        COMPONENT_LOG_INFO("Graphics engine initialized successfully with real-time display");
        
        return {};
        
    } catch (const std::exception& e) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to initialize graphics engine: " + std::string(e.what())));
    }
}

Result<void> GraphicsEngine::shutdown() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    COMPONENT_LOG_INFO("Shutting down graphics engine");
    
    // Shutdown display bridge first
    if (display_bridge_) {
        display_bridge_->shutdown();
        display_bridge_.reset();
    }
    
    // Clean up display adapter
    display_adapter_.reset();
    
    // Clean up framebuffer
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
        framebuffer_->clear();
    }
    
    if (display_bridge_) {
        display_bridge_->reset();
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
    
    if (!framebuffer_ || !display_bridge_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics components not properly initialized"));
    }
    
    // Apply display adapter optimizations if available
    if (display_adapter_) {
        if (auto result = display_adapter_->optimize_for_content_type(*framebuffer_);
            !result.has_value()) {
            COMPONENT_LOG_WARN("Display optimization failed: {}", 
                               result.error().to_string());
        }
        
        // Apply color corrections and hardware acceleration simulation
        if (auto result = display_adapter_->apply_color_correction(*framebuffer_);
            !result.has_value()) {
            COMPONENT_LOG_WARN("Color correction failed: {}", 
                               result.error().to_string());
        }
        
        if (auto result = display_adapter_->simulate_gpu_acceleration(*framebuffer_);
            !result.has_value()) {
            COMPONENT_LOG_WARN("GPU acceleration simulation failed: {}", 
                               result.error().to_string());
        }
        
        // Apply MIPI-DSI timing simulation
        if (auto result = display_adapter_->apply_mipi_dsi_timing();
            !result.has_value()) {
            COMPONENT_LOG_WARN("MIPI-DSI timing simulation failed: {}", 
                               result.error().to_string());
        }
    }
    
    // Stream framebuffer to display bridge for real-time rendering
    RETURN_IF_ERROR(display_bridge_->update_framebuffer(*framebuffer_));
    
    // Update performance metrics
    update_fps_counter();
    frames_rendered_++;
    
    return {};
}

Result<void> GraphicsEngine::present() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (!display_bridge_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display bridge not initialized"));
    }
    
    // Present the frame through the display bridge
    RETURN_IF_ERROR(display_bridge_->present_frame());
    
    return {};
}

Result<void> GraphicsEngine::update_display() {
    // Render current framebuffer content and present to display
    RETURN_IF_ERROR(render_frame());
    RETURN_IF_ERROR(present());
    
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
    
    // Convert color to Framebuffer::Color format
    Framebuffer::Color clear_color(
        static_cast<u8>((color >> 16) & 0xFF),  // Red
        static_cast<u8>((color >> 8) & 0xFF),   // Green  
        static_cast<u8>(color & 0xFF),          // Blue
        static_cast<u8>((color >> 24) & 0xFF)   // Alpha
    );
    
    auto clear_result = framebuffer_->clear(clear_color);
    if (clear_result != EmulatorError::Success) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to clear framebuffer"));
    }
    
    // Also clear the display bridge
    if (display_bridge_) {
        if (auto result = display_bridge_->clear_display(color); !result.has_value()) {
            COMPONENT_LOG_WARN("Failed to clear display bridge: {}", 
                               result.error().to_string());
        }
    }
    
    return {};
}


Result<void> GraphicsEngine::set_pixel(u32 x, u32 y, u32 color) {
    RETURN_IF_ERROR(validate_coordinates(x, y));
    
    if (!framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    Framebuffer::Color pixel_color(
        static_cast<u8>((color >> 16) & 0xFF),  // Red
        static_cast<u8>((color >> 8) & 0xFF),   // Green
        static_cast<u8>(color & 0xFF),          // Blue
        static_cast<u8>((color >> 24) & 0xFF)   // Alpha
    );
    
    auto result = framebuffer_->setPixel(x, y, pixel_color);
    if (result != EmulatorError::Success) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to set pixel"));
    }
    return {};
}

Result<u32> GraphicsEngine::get_pixel(u32 x, u32 y) const {
    RETURN_IF_ERROR(validate_coordinates(x, y));
    
    if (!framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    auto pixel_color = framebuffer_->getPixel(x, y);
    
    // Convert Framebuffer::Color back to u32 ARGB format
    u32 color = (static_cast<u32>(pixel_color.a) << 24) |
                (static_cast<u32>(pixel_color.r) << 16) |
                (static_cast<u32>(pixel_color.g) << 8) |
                 static_cast<u32>(pixel_color.b);
    
    return color;
}

Result<void> GraphicsEngine::draw_rectangle(u32 x, u32 y, u32 width, u32 height, u32 color) {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (!framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    // Convert color and use framebuffer's optimized fill operation
    Framebuffer::Color fill_color(
        static_cast<u8>((color >> 16) & 0xFF),  // Red
        static_cast<u8>((color >> 8) & 0xFF),   // Green
        static_cast<u8>(color & 0xFF),          // Blue
        static_cast<u8>((color >> 24) & 0xFF)   // Alpha
    );
    
    Framebuffer::Rectangle rect(x, y, width, height);
    auto result = framebuffer_->fillRect(rect, fill_color);
    if (result != EmulatorError::Success) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to fill rectangle"));
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


Result<std::vector<TouchPoint>> GraphicsEngine::get_touch_events() const {
    if (!display_bridge_) {
        return std::vector<TouchPoint>{};
    }
    
    return display_bridge_->get_current_touches();
}

Result<void> GraphicsEngine::inject_touch_event(const TouchPoint& touch) {
    if (!display_bridge_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display bridge not initialized"));
    }
    
    return display_bridge_->inject_touch_event(touch);
}

bool GraphicsEngine::is_display_active() const {
    return display_bridge_ ? display_bridge_->is_display_active() : false;
}

bool GraphicsEngine::should_close_display() const {
    return display_bridge_ ? display_bridge_->should_close() : false;
}

}  // namespace m5tab5::emulator