#include "emulator/graphics/framebuffer_bridge.hpp"
#include "emulator/utils/logging.hpp"

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

#include <algorithm>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("FramebufferBridge");

FramebufferBridge::FramebufferBridge()
    : FramebufferBridge(BridgeConfig{}) {
    // Default constructor delegates to parameterized constructor
}

FramebufferBridge::FramebufferBridge(const BridgeConfig& config)
    : config_(config),
      last_frame_time_(std::chrono::steady_clock::now()),
      last_fps_update_(std::chrono::steady_clock::now()),
      last_touch_update_(std::chrono::steady_clock::now()) {
    COMPONENT_LOG_DEBUG("FramebufferBridge created with {}x{} @ {}fps", 
                       config_.display_width, config_.display_height, config_.target_fps);
}

FramebufferBridge::~FramebufferBridge() {
    if (initialized_.load()) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("FramebufferBridge destroyed");
}

Result<void> FramebufferBridge::initialize() {
    if (initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING, "Bridge already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing FramebufferBridge");
    
    try {
        // Initialize SDL renderer
        sdl_renderer_ = std::make_unique<SdlRenderer>();
        RETURN_IF_ERROR(sdl_renderer_->initialize(
            config_.display_width, 
            config_.display_height, 
            config_.enable_vsync,
            config_.window_title
        ));
        
        // Create framebuffers
        display_framebuffer_ = std::make_unique<Framebuffer>(
            config_.display_width, 
            config_.display_height, 
            Framebuffer::PixelFormat::RGB888
        );
        
        if (config_.enable_double_buffer) {
            back_framebuffer_ = std::make_unique<Framebuffer>(
                config_.display_width, 
                config_.display_height, 
                Framebuffer::PixelFormat::RGB888
            );
        }
        
        // Initialize touch input system
        if (config_.enable_touch) {
            touch_input_ = std::make_unique<TouchInput>();
            // Set calibration for 1280x720 display
            touch_input_->setCalibration(0, config_.display_width - 1, 
                                       0, config_.display_height - 1);
        }
        
        // Start worker threads
        should_stop_.store(false);
        
        render_thread_ = std::thread(&FramebufferBridge::render_loop, this);
        
        if (config_.enable_touch) {
            touch_thread_ = std::thread(&FramebufferBridge::touch_loop, this);
        }
        
        // Initialize statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            statistics_ = {};
        }
        
        initialized_.store(true);
        display_active_.store(true);
        
        COMPONENT_LOG_INFO("FramebufferBridge initialized successfully");
        return {};
        
    } catch (const std::exception& e) {
        cleanup_resources();
        return unexpected(MAKE_ERROR(OPERATION_FAILED, 
                                   "Failed to initialize bridge: " + std::string(e.what())));
    }
}

Result<void> FramebufferBridge::shutdown() {
    if (!initialized_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Bridge not initialized"));
    }
    
    COMPONENT_LOG_INFO("Shutting down FramebufferBridge");
    
    // Signal threads to stop
    should_stop_.store(true);
    display_active_.store(false);
    
    // Wake up any waiting threads
    frame_cv_.notify_all();
    
    // Cleanup threads and resources
    cleanup_threads();
    cleanup_resources();
    
    initialized_.store(false);
    
    COMPONENT_LOG_INFO("FramebufferBridge shutdown complete");
    return {};
}

Result<void> FramebufferBridge::reset() {
    COMPONENT_LOG_INFO("Resetting FramebufferBridge");
    
    // Clear framebuffers
    if (display_framebuffer_) {
        display_framebuffer_->clear();
    }
    if (back_framebuffer_) {
        back_framebuffer_->clear();
    }
    
    // Reset statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_ = {};
    }
    
    // Reset timing
    last_frame_time_ = std::chrono::steady_clock::now();
    last_fps_update_ = std::chrono::steady_clock::now();
    frames_rendered_.store(0);
    frames_dropped_.store(0);
    
    return {};
}

Result<void> FramebufferBridge::update_framebuffer(const Framebuffer& source_framebuffer) {
    if (!initialized_.load() || !display_active_.load()) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Bridge not active"));
    }
    
    // Check if we should drop this frame due to performance
    if (should_drop_frame()) {
        frames_dropped_.fetch_add(1);
        
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.frames_dropped++;
        return {};
    }
    
    // Wait for previous frame to complete processing
    {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        if (frame_processing_.load()) {
            // Skip this frame if previous is still processing
            frames_dropped_.fetch_add(1);
            
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            statistics_.frames_dropped++;
            return {};
        }
        
        frame_processing_.store(true);
    }
    
    // Copy framebuffer data efficiently
    RETURN_IF_ERROR(convert_and_copy_framebuffer(source_framebuffer));
    
    // Signal new frame is available
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        new_frame_available_.store(true);
        frame_processing_.store(false);
    }
    frame_cv_.notify_one();
    
    // Update statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.frames_processed++;
        statistics_.memory_transfers++;
    }
    
    return {};
}

Result<void> FramebufferBridge::present_frame() {
    if (!sdl_renderer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Renderer not initialized"));
    }
    
    // Present the current framebuffer
    RETURN_IF_ERROR(sdl_renderer_->render(*display_framebuffer_));
    RETURN_IF_ERROR(sdl_renderer_->present());
    
    frames_rendered_.fetch_add(1);
    update_fps_counter();
    
    return {};
}

Result<void> FramebufferBridge::clear_display(u32 color) {
    if (!display_framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Framebuffer not initialized"));
    }
    
    Framebuffer::Color clear_color(
        static_cast<u8>((color >> 16) & 0xFF),  // Red
        static_cast<u8>((color >> 8) & 0xFF),   // Green
        static_cast<u8>(color & 0xFF),          // Blue
        255                                      // Alpha
    );
    
    auto clear_result = display_framebuffer_->clear(clear_color);
    if (clear_result != EmulatorError::Success) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to clear framebuffer"));
    }
    return {};
}

Result<void> FramebufferBridge::process_touch_events() {
    if (!config_.enable_touch || !touch_input_) {
        return {};
    }
    
    process_mouse_to_touch();
    RETURN_IF_ERROR(update_touch_state());
    
    return {};
}

Result<std::vector<TouchPoint>> FramebufferBridge::get_current_touches() const {
    if (!touch_input_) {
        return std::vector<TouchPoint>{};
    }
    
    std::lock_guard<std::mutex> lock(touch_mutex_);
    return current_touches_;
}

Result<void> FramebufferBridge::inject_touch_event(const TouchPoint& touch) {
    if (!touch_input_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Touch input not enabled"));
    }
    
    touch_input_->addTouchPoint(touch);
    
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.touch_events_processed++;
    }
    
    return {};
}

bool FramebufferBridge::should_close() const {
    return sdl_renderer_ ? sdl_renderer_->should_close() : false;
}

FramebufferBridge::BridgeStatistics FramebufferBridge::get_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    // Update current FPS in statistics
    auto stats_copy = statistics_;
    stats_copy.current_fps = static_cast<double>(current_fps_.load());
    return stats_copy;
}

void FramebufferBridge::reset_statistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    statistics_ = {};
    frames_rendered_.store(0);
    frames_dropped_.store(0);
}

void FramebufferBridge::render_loop() {
    COMPONENT_LOG_DEBUG("Render loop started");
    
    const auto target_frame_time = std::chrono::microseconds(1000000 / config_.target_fps);
    
    while (!should_stop_.load()) {
        // Wait for new frame or timeout
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait_for(lock, target_frame_time, [this] {
                return new_frame_available_.load() || should_stop_.load();
            });
            
            if (should_stop_.load()) break;
            
            if (new_frame_available_.load()) {
                new_frame_available_.store(false);
                
                // Process and present the frame
                if (auto result = process_frame(); !result.has_value()) {
                    COMPONENT_LOG_ERROR("Frame processing failed: {}", 
                                       static_cast<int>(result.error()));
                }
            }
        }
        
        // Handle frame timing for consistent FPS
        if (auto result = handle_frame_timing(); !result.has_value()) {
            COMPONENT_LOG_WARN("Frame timing adjustment failed");
        }
        
        // Adaptive FPS adjustment
        if (config_.adaptive_fps) {
            adjust_adaptive_fps();
        }
    }
    
    COMPONENT_LOG_DEBUG("Render loop stopped");
}

void FramebufferBridge::touch_loop() {
    COMPONENT_LOG_DEBUG("Touch loop started");
    
    const auto touch_sample_interval = std::chrono::microseconds(1000000 / config_.touch_sample_rate);
    
    while (!should_stop_.load()) {
        auto touch_start = std::chrono::steady_clock::now();
        
        // Process touch events
        if (auto result = process_touch_events(); !result.has_value()) {
            COMPONENT_LOG_WARN("Touch processing failed: {}", 
                               result.error().to_string());
        }
        
        // Handle SDL events (including mouse for touch simulation)
        if (sdl_renderer_) {
            sdl_renderer_->handle_events();
        }
        
        // Sleep for the remainder of the sample interval
        auto elapsed = std::chrono::steady_clock::now() - touch_start;
        if (elapsed < touch_sample_interval) {
            std::this_thread::sleep_for(touch_sample_interval - elapsed);
        }
    }
    
    COMPONENT_LOG_DEBUG("Touch loop stopped");
}

Result<void> FramebufferBridge::process_frame() {
    if (!sdl_renderer_ || !display_framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Components not initialized"));
    }
    
    // Present current framebuffer to screen
    RETURN_IF_ERROR(present_frame());
    
    return {};
}

Result<void> FramebufferBridge::convert_and_copy_framebuffer(const Framebuffer& source) {
    if (!display_framebuffer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "Display framebuffer not initialized"));
    }
    
    // For now, assume source and destination are same format (RGB888)
    // In production, this would handle format conversion
    const size_t copy_size = std::min(source.getTotalSize(), display_framebuffer_->getTotalSize());
    
    std::memcpy(display_framebuffer_->getBuffer(), source.getBuffer(), copy_size);
    
    // Mark the entire display as dirty for update
    display_framebuffer_->markDirty({0, 0, config_.display_width, config_.display_height});
    
    return {};
}

Result<void> FramebufferBridge::handle_frame_timing() {
    const auto now = std::chrono::steady_clock::now();
    const auto target_frame_time = std::chrono::microseconds(1000000 / config_.target_fps);
    const auto elapsed = now - last_frame_time_;
    
    if (elapsed < target_frame_time) {
        // Sleep for remaining time to maintain target FPS
        std::this_thread::sleep_for(target_frame_time - elapsed);
    }
    
    last_frame_time_ = std::chrono::steady_clock::now();
    return {};
}

bool FramebufferBridge::should_drop_frame() const {
    // Drop frames if we're significantly behind target FPS
    const auto now = std::chrono::steady_clock::now();
    const auto target_frame_time = std::chrono::microseconds(1000000 / config_.target_fps);
    const auto elapsed = now - last_frame_time_;
    
    // If we're more than max_frame_skip frames behind, start dropping
    const auto max_delay = target_frame_time * (config_.max_frame_skip + 1);
    
    return elapsed > max_delay;
}

void FramebufferBridge::process_mouse_to_touch() {
#ifndef NO_GRAPHICS
    if (!sdl_renderer_ || !touch_input_) return;
    
    i32 mouse_x, mouse_y;
    u32 mouse_state = SDL_GetMouseState(&mouse_x, &mouse_y);
    
    bool mouse_pressed = (mouse_state & SDL_BUTTON(SDL_BUTTON_LEFT)) != 0;
    
    if (mouse_pressed || !current_touches_.empty()) {
        TouchPoint touch_point = convert_mouse_to_touch(mouse_x, mouse_y, mouse_pressed);
        
        std::lock_guard<std::mutex> lock(touch_mutex_);
        
        if (mouse_pressed) {
            current_touches_.clear();
            current_touches_.push_back(touch_point);
            touch_input_->addTouchPoint(touch_point);
        } else {
            current_touches_.clear();
            touch_input_->clearAllTouchPoints();
        }
        
        last_touch_update_ = std::chrono::steady_clock::now();
    }
#endif
}

Result<void> FramebufferBridge::update_touch_state() {
    if (!touch_input_) {
        return {};
    }
    
    // Update touch input state and get active points
    auto active_touches = touch_input_->getActiveTouchPoints();
    
    {
        std::lock_guard<std::mutex> lock(touch_mutex_);
        current_touches_ = std::move(active_touches);
    }
    
    return {};
}

TouchPoint FramebufferBridge::convert_mouse_to_touch(i32 mouse_x, i32 mouse_y, bool pressed) const {
    TouchPoint touch;
    
    // Clamp coordinates to display bounds
    touch.x = static_cast<u16>(std::clamp(mouse_x, 0, static_cast<i32>(config_.display_width - 1)));
    touch.y = static_cast<u16>(std::clamp(mouse_y, 0, static_cast<i32>(config_.display_height - 1)));
    touch.pressure = pressed ? 512 : 0;  // Medium pressure when pressed
    touch.size = 10;                     // Default touch size
    touch.id = 0;                        // Primary touch ID
    touch.active = pressed;
    
    return touch;
}

void FramebufferBridge::update_fps_counter() {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - last_fps_update_;
    
    if (elapsed >= std::chrono::seconds(1)) {
        // Calculate FPS over the last second
        const auto frames = frames_rendered_.load();
        const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        
        if (duration_ms > 0) {
            const double fps = (frames * 1000.0) / duration_ms;
            current_fps_.store(static_cast<u32>(fps));
            
            std::lock_guard<std::mutex> lock(stats_mutex_);
            statistics_.current_fps = fps;
            statistics_.average_frame_time_ms = fps > 0 ? (1000.0 / fps) : 0.0;
        }
        
        last_fps_update_ = now;
        frames_rendered_.store(0); // Reset counter for next measurement
    }
}

void FramebufferBridge::adjust_adaptive_fps() {
    // Simple adaptive FPS: reduce target FPS if dropping too many frames
    const auto total_frames = frames_rendered_.load() + frames_dropped_.load();
    
    if (total_frames > 0) {
        const double drop_rate = static_cast<double>(frames_dropped_.load()) / total_frames;
        
        if (drop_rate > 0.1) { // If dropping more than 10% of frames
            // Reduce target FPS slightly
            if (config_.target_fps > 30) {
                config_.target_fps = std::max(30u, config_.target_fps - 5);
                COMPONENT_LOG_DEBUG("Adaptive FPS: reduced to {}", config_.target_fps);
            }
        } else if (drop_rate < 0.05 && config_.target_fps < 60) {
            // Increase target FPS if performance improves
            config_.target_fps = std::min(60u, config_.target_fps + 5);
            COMPONENT_LOG_DEBUG("Adaptive FPS: increased to {}", config_.target_fps);
        }
    }
}

Result<void> FramebufferBridge::optimize_rendering_pipeline() {
    // Implement rendering pipeline optimizations
    // This could include:
    // - Dirty rectangle tracking
    // - Partial frame updates
    // - Memory bandwidth optimization
    
    return {};
}

void FramebufferBridge::cleanup_threads() {
    // Join render thread
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
    
    // Join touch thread
    if (touch_thread_.joinable()) {
        touch_thread_.join();
    }
}

void FramebufferBridge::cleanup_resources() {
    // Shutdown SDL renderer
    if (sdl_renderer_) {
        sdl_renderer_->shutdown();
        sdl_renderer_.reset();
    }
    
    // Release framebuffers
    display_framebuffer_.reset();
    back_framebuffer_.reset();
    
    // Release touch input
    touch_input_.reset();
}

} // namespace m5tab5::emulator