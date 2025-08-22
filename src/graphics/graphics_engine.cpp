#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("GraphicsEngine");

GraphicsEngine::GraphicsEngine()
    : initialized_(false),
      memory_controller_(nullptr),
      display_controller_(nullptr),
      sdl_renderer_(nullptr),
      touch_input_(nullptr) {
    COMPONENT_LOG_DEBUG("GraphicsEngine created");
}

GraphicsEngine::~GraphicsEngine() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("GraphicsEngine destroyed");
}

Result<void> GraphicsEngine::initialize(const Configuration& config) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Graphics engine already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing graphics engine");
    
    // Store configuration
    config_ = config;
    
    // Initialize SDL renderer
    sdl_renderer_ = std::make_unique<SdlRenderer>();
    RETURN_IF_ERROR(sdl_renderer_->initialize(
        config.get_display_width(),
        config.get_display_height(),
        config.is_vsync_enabled(),
        "M5Stack Tab5 Emulator"
    ));
    
    // Initialize display controller
    display_controller_ = std::make_unique<DisplayController>();
    // Note: Display controller needs memory and interrupt controllers which are passed later
    
    // Initialize touch input
    touch_input_ = std::make_unique<TouchInput>();
    RETURN_IF_ERROR(touch_input_->initialize(config));
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Graphics engine initialized successfully");
    
    return {};
}

Result<void> GraphicsEngine::initialize_with_controllers(MemoryController& memory_controller,
                                                         InterruptController* interrupt_controller) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    memory_controller_ = &memory_controller;
    
    // Now initialize display controller with the controllers
    RETURN_IF_ERROR(display_controller_->initialize(config_, memory_controller, interrupt_controller));
    
    COMPONENT_LOG_INFO("Graphics engine controllers initialized");
    return {};
}

Result<void> GraphicsEngine::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down graphics engine");
    
    // Shutdown components
    if (touch_input_) {
        touch_input_->shutdown();
        touch_input_.reset();
    }
    
    if (display_controller_) {
        display_controller_->shutdown();
        display_controller_.reset();
    }
    
    if (sdl_renderer_) {
        sdl_renderer_->shutdown();
        sdl_renderer_.reset();
    }
    
    memory_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Graphics engine shutdown completed");
    return {};
}

Result<void> GraphicsEngine::render_frame() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    auto frame_start = std::chrono::steady_clock::now();
    
    // Update display controller
    if (display_controller_) {
        RETURN_IF_ERROR(display_controller_->update());
    }
    
    // Get framebuffer from display controller
    if (display_controller_ && display_controller_->get_framebuffer()) {
        auto* framebuffer = display_controller_->get_framebuffer();
        const u8* fb_data = framebuffer->get_front_buffer();
        
        if (fb_data) {
            // Copy framebuffer to SDL renderer
            RETURN_IF_ERROR(sdl_renderer_->blit_framebuffer(
                fb_data,
                framebuffer->get_width(),
                framebuffer->get_height(),
                0, 0
            ));
        }
    }
    
    // Process input events
    RETURN_IF_ERROR(process_input_events());
    
    // Present the frame
    RETURN_IF_ERROR(sdl_renderer_->present());
    
    // Update statistics
    auto frame_end = std::chrono::steady_clock::now();
    auto frame_time = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
    
    statistics_.frames_rendered++;
    statistics_.total_render_time += frame_time.count();
    
    // Calculate average FPS
    static auto last_fps_update = frame_start;
    static u32 frame_count = 0;
    frame_count++;
    
    auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frame_start - last_fps_update);
    if (fps_elapsed.count() >= 1000) {
        statistics_.average_fps = static_cast<double>(frame_count) * 1000.0 / fps_elapsed.count();
        frame_count = 0;
        last_fps_update = frame_start;
    }
    
    return {};
}

Result<void> GraphicsEngine::set_framebuffer_address(Address address) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (display_controller_) {
        return display_controller_->set_framebuffer_address(address);
    }
    
    return {};
}

Result<void> GraphicsEngine::set_display_mode(u32 width, u32 height, u32 refresh_rate) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    COMPONENT_LOG_INFO("Setting graphics engine display mode: {}x{} @{}Hz", width, height, refresh_rate);
    
    // Update display controller
    if (display_controller_) {
        RETURN_IF_ERROR(display_controller_->set_display_mode(width, height, refresh_rate));
    }
    
    // Resize SDL renderer
    if (sdl_renderer_) {
        RETURN_IF_ERROR(sdl_renderer_->resize(width, height));
    }
    
    // Update touch input scaling
    if (touch_input_) {
        RETURN_IF_ERROR(touch_input_->set_screen_resolution(width, height));
    }
    
    return {};
}

Result<void> GraphicsEngine::enable_display() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (display_controller_) {
        return display_controller_->enable();
    }
    
    return {};
}

Result<void> GraphicsEngine::disable_display() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (display_controller_) {
        return display_controller_->disable();
    }
    
    return {};
}

Result<std::vector<TouchEvent>> GraphicsEngine::get_touch_events() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (touch_input_) {
        return touch_input_->get_events();
    }
    
    return std::vector<TouchEvent>{};
}

Result<void> GraphicsEngine::inject_touch_event(const TouchEvent& event) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    if (touch_input_) {
        return touch_input_->inject_event(event);
    }
    
    return {};
}

const GraphicsStatistics& GraphicsEngine::get_statistics() const {
    return statistics_;
}

void GraphicsEngine::clear_statistics() {
    statistics_ = {};
    
    if (sdl_renderer_) {
        sdl_renderer_->clear_statistics();
    }
    
    if (display_controller_) {
        display_controller_->clear_statistics();
    }
    
    if (touch_input_) {
        touch_input_->clear_statistics();
    }
}

DisplayController* GraphicsEngine::get_display_controller() const {
    return display_controller_.get();
}

SdlRenderer* GraphicsEngine::get_sdl_renderer() const {
    return sdl_renderer_.get();
}

TouchInput* GraphicsEngine::get_touch_input() const {
    return touch_input_.get();
}

Result<void> GraphicsEngine::handle_mmio_write(Address address, u32 value) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    // Route MMIO writes to appropriate controller
    if (address >= DISPLAY_CONTROLLER_BASE_ADDR && 
        address < DISPLAY_CONTROLLER_BASE_ADDR + DISPLAY_CONTROLLER_SIZE) {
        if (display_controller_) {
            return display_controller_->handle_mmio_write(address, value);
        }
    } else if (address >= TOUCH_CONTROLLER_BASE_ADDR && 
               address < TOUCH_CONTROLLER_BASE_ADDR + TOUCH_CONTROLLER_SIZE) {
        if (touch_input_) {
            return touch_input_->handle_mmio_write(address, value);
        }
    }
    
    return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
        "Invalid graphics MMIO address: 0x" + std::to_string(address)));
}

Result<u32> GraphicsEngine::handle_mmio_read(Address address) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Graphics engine not initialized"));
    }
    
    // Route MMIO reads to appropriate controller
    if (address >= DISPLAY_CONTROLLER_BASE_ADDR && 
        address < DISPLAY_CONTROLLER_BASE_ADDR + DISPLAY_CONTROLLER_SIZE) {
        if (display_controller_) {
            return display_controller_->handle_mmio_read(address);
        }
    } else if (address >= TOUCH_CONTROLLER_BASE_ADDR && 
               address < TOUCH_CONTROLLER_BASE_ADDR + TOUCH_CONTROLLER_SIZE) {
        if (touch_input_) {
            return touch_input_->handle_mmio_read(address);
        }
    }
    
    return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
        "Invalid graphics MMIO address: 0x" + std::to_string(address)));
}

Result<void> GraphicsEngine::process_input_events() {
    if (!sdl_renderer_ || !touch_input_) {
        return {};
    }
    
    // Get SDL input events
    auto sdl_events_result = sdl_renderer_->poll_events();
    if (!sdl_events_result) {
        return std::unexpected(sdl_events_result.error());
    }
    
    auto sdl_events = sdl_events_result.value();
    
    // Convert SDL events to touch events
    for (const auto& sdl_event : sdl_events) {
        switch (sdl_event.type) {
            case InputEventType::MOUSE_DOWN:
            case InputEventType::MOUSE_UP:
            case InputEventType::MOUSE_MOTION: {
                TouchEvent touch_event = convert_mouse_to_touch(sdl_event);
                RETURN_IF_ERROR(touch_input_->inject_event(touch_event));
                break;
            }
            
            case InputEventType::QUIT:
                // Handle quit event (could trigger emulator shutdown)
                statistics_.quit_requested = true;
                break;
                
            case InputEventType::WINDOW_RESIZED:
                // Handle window resize
                COMPONENT_LOG_DEBUG("Window resized to {}x{}", 
                                   sdl_event.window.width, sdl_event.window.height);
                break;
                
            default:
                // Ignore other events for now
                break;
        }
    }
    
    return {};
}

TouchEvent GraphicsEngine::convert_mouse_to_touch(const InputEvent& mouse_event) {
    TouchEvent touch_event{};
    
    switch (mouse_event.type) {
        case InputEventType::MOUSE_DOWN:
            touch_event.type = TouchEventType::PRESS;
            touch_event.x = mouse_event.mouse.x;
            touch_event.y = mouse_event.mouse.y;
            touch_event.pressure = 255; // Maximum pressure
            touch_event.touch_id = 0;   // Single touch
            break;
            
        case InputEventType::MOUSE_UP:
            touch_event.type = TouchEventType::RELEASE;
            touch_event.x = mouse_event.mouse.x;
            touch_event.y = mouse_event.mouse.y;
            touch_event.pressure = 0;
            touch_event.touch_id = 0;
            break;
            
        case InputEventType::MOUSE_MOTION:
            // Only generate move events if mouse is pressed
            // This would need to track mouse button state
            touch_event.type = TouchEventType::MOVE;
            touch_event.x = mouse_event.mouse.x;
            touch_event.y = mouse_event.mouse.y;
            touch_event.pressure = 128; // Medium pressure
            touch_event.touch_id = 0;
            break;
            
        default:
            touch_event.type = TouchEventType::INVALID;
            break;
    }
    
    touch_event.timestamp = std::chrono::steady_clock::now();
    return touch_event;
}

void GraphicsEngine::dump_status() const {
    COMPONENT_LOG_INFO("=== Graphics Engine Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Frames rendered: {}", statistics_.frames_rendered);
        COMPONENT_LOG_INFO("  Average FPS: {:.2f}", statistics_.average_fps);
        COMPONENT_LOG_INFO("  Total render time: {} μs", statistics_.total_render_time);
        COMPONENT_LOG_INFO("  Quit requested: {}", statistics_.quit_requested);
        
        if (statistics_.frames_rendered > 0) {
            double avg_frame_time = static_cast<double>(statistics_.total_render_time) / 
                                   statistics_.frames_rendered;
            COMPONENT_LOG_INFO("  Average frame time: {:.2f} μs", avg_frame_time);
        }
        
        // Display controller status
        if (display_controller_) {
            const auto& display_config = display_controller_->get_display_config();
            COMPONENT_LOG_INFO("Display: {}x{} @{}Hz", 
                              display_config.width, display_config.height, display_config.refresh_rate);
            
            const auto& display_stats = display_controller_->get_statistics();
            COMPONENT_LOG_INFO("  Frames processed: {}", display_stats.frames_processed);
            COMPONENT_LOG_INFO("  VSync interrupts: {}", display_stats.vsync_interrupts);
            COMPONENT_LOG_INFO("  Commands processed: {}", display_stats.commands_processed);
        }
        
        // SDL renderer status
        if (sdl_renderer_) {
            const auto& render_stats = sdl_renderer_->get_statistics();
            COMPONENT_LOG_INFO("SDL Renderer:");
            COMPONENT_LOG_INFO("  Frames rendered: {}", render_stats.frames_rendered);
            COMPONENT_LOG_INFO("  Current FPS: {:.2f}", render_stats.current_fps);
            COMPONENT_LOG_INFO("  Total render time: {} μs", render_stats.total_render_time);
        }
        
        // Touch input status
        if (touch_input_) {
            const auto& touch_stats = touch_input_->get_statistics();
            COMPONENT_LOG_INFO("Touch Input:");
            COMPONENT_LOG_INFO("  Events processed: {}", touch_stats.events_processed);
            COMPONENT_LOG_INFO("  Touch points active: {}", touch_stats.active_touch_points);
        }
    }
}

}  // namespace m5tab5::emulator