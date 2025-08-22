#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/utils/logging.hpp"
#include <SDL2/SDL.h>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("SdlRenderer");

SdlRenderer::SdlRenderer()
    : initialized_(false),
      window_(nullptr),
      renderer_(nullptr),
      texture_(nullptr),
      width_(0),
      height_(0),
      framebuffer_(nullptr),
      vsync_enabled_(true) {
    COMPONENT_LOG_DEBUG("SdlRenderer created");
}

SdlRenderer::~SdlRenderer() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("SdlRenderer destroyed");
}

Result<void> SdlRenderer::initialize(u32 width, u32 height, bool vsync, const std::string& title) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "SDL renderer already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing SDL renderer: {}x{}", width, height);
    
    width_ = width;
    height_ = height;
    vsync_enabled_ = vsync;
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to initialize SDL: " + std::string(SDL_GetError())));
    }
    
    // Create window
    window_ = SDL_CreateWindow(
        title.c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        static_cast<int>(width_),
        static_cast<int>(height_),
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    
    if (!window_) {
        SDL_Quit();
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL window: " + std::string(SDL_GetError())));
    }
    
    // Create renderer
    u32 renderer_flags = SDL_RENDERER_ACCELERATED;
    if (vsync_enabled_) {
        renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
    }
    
    renderer_ = SDL_CreateRenderer(window_, -1, renderer_flags);
    if (!renderer_) {
        SDL_DestroyWindow(window_);
        SDL_Quit();
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL renderer: " + std::string(SDL_GetError())));
    }
    
    // Set logical size to maintain aspect ratio
    SDL_RenderSetLogicalSize(renderer_, static_cast<int>(width_), static_cast<int>(height_));
    
    // Create texture for framebuffer
    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_RGB888,
        SDL_TEXTUREACCESS_STREAMING,
        static_cast<int>(width_),
        static_cast<int>(height_)
    );
    
    if (!texture_) {
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL texture: " + std::string(SDL_GetError())));
    }
    
    // Allocate framebuffer
    size_t framebuffer_size = width_ * height_ * BYTES_PER_PIXEL;
    framebuffer_ = std::make_unique<u8[]>(framebuffer_size);
    std::memset(framebuffer_.get(), 0, framebuffer_size);
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("SDL renderer initialized successfully");
    
    return {};
}

Result<void> SdlRenderer::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down SDL renderer");
    
    // Clean up SDL resources
    if (texture_) {
        SDL_DestroyTexture(texture_);
        texture_ = nullptr;
    }
    
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    
    SDL_Quit();
    
    // Free framebuffer
    framebuffer_.reset();
    
    initialized_ = false;
    COMPONENT_LOG_INFO("SDL renderer shutdown completed");
    
    return {};
}

Result<void> SdlRenderer::clear(u32 color) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    // Clear framebuffer with color
    u8 r = (color >> 16) & 0xFF;
    u8 g = (color >> 8) & 0xFF;
    u8 b = color & 0xFF;
    
    size_t pixel_count = width_ * height_;
    u8* pixel = framebuffer_.get();
    
    for (size_t i = 0; i < pixel_count; ++i) {
        *pixel++ = r;
        *pixel++ = g;
        *pixel++ = b;
    }
    
    return {};
}

Result<void> SdlRenderer::set_pixel(u32 x, u32 y, u32 color) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    if (x >= width_ || y >= height_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Pixel coordinates out of bounds"));
    }
    
    size_t offset = (y * width_ + x) * BYTES_PER_PIXEL;
    u8* pixel = framebuffer_.get() + offset;
    
    pixel[0] = (color >> 16) & 0xFF; // R
    pixel[1] = (color >> 8) & 0xFF;  // G
    pixel[2] = color & 0xFF;         // B
    
    return {};
}

Result<u32> SdlRenderer::get_pixel(u32 x, u32 y) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    if (x >= width_ || y >= height_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Pixel coordinates out of bounds"));
    }
    
    size_t offset = (y * width_ + x) * BYTES_PER_PIXEL;
    const u8* pixel = framebuffer_.get() + offset;
    
    u32 color = (static_cast<u32>(pixel[0]) << 16) |  // R
                (static_cast<u32>(pixel[1]) << 8) |   // G
                static_cast<u32>(pixel[2]);           // B
    
    return color;
}

Result<void> SdlRenderer::draw_rectangle(u32 x, u32 y, u32 width, u32 height, u32 color, bool filled) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    if (filled) {
        // Draw filled rectangle
        for (u32 dy = 0; dy < height; ++dy) {
            for (u32 dx = 0; dx < width; ++dx) {
                if (x + dx < width_ && y + dy < height_) {
                    RETURN_IF_ERROR(set_pixel(x + dx, y + dy, color));
                }
            }
        }
    } else {
        // Draw rectangle outline
        // Top and bottom edges
        for (u32 dx = 0; dx < width; ++dx) {
            if (x + dx < width_) {
                if (y < height_) {
                    RETURN_IF_ERROR(set_pixel(x + dx, y, color));
                }
                if (y + height - 1 < height_) {
                    RETURN_IF_ERROR(set_pixel(x + dx, y + height - 1, color));
                }
            }
        }
        
        // Left and right edges
        for (u32 dy = 0; dy < height; ++dy) {
            if (y + dy < height_) {
                if (x < width_) {
                    RETURN_IF_ERROR(set_pixel(x, y + dy, color));
                }
                if (x + width - 1 < width_) {
                    RETURN_IF_ERROR(set_pixel(x + width - 1, y + dy, color));
                }
            }
        }
    }
    
    return {};
}

Result<void> SdlRenderer::draw_line(u32 x1, u32 y1, u32 x2, u32 y2, u32 color) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    // Bresenham's line algorithm
    i32 dx = abs(static_cast<i32>(x2) - static_cast<i32>(x1));
    i32 dy = abs(static_cast<i32>(y2) - static_cast<i32>(y1));
    i32 sx = (x1 < x2) ? 1 : -1;
    i32 sy = (y1 < y2) ? 1 : -1;
    i32 err = dx - dy;
    
    i32 x = static_cast<i32>(x1);
    i32 y = static_cast<i32>(y1);
    
    while (true) {
        if (x >= 0 && x < static_cast<i32>(width_) && 
            y >= 0 && y < static_cast<i32>(height_)) {
            RETURN_IF_ERROR(set_pixel(static_cast<u32>(x), static_cast<u32>(y), color));
        }
        
        if (x == static_cast<i32>(x2) && y == static_cast<i32>(y2)) {
            break;
        }
        
        i32 e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return {};
}

Result<void> SdlRenderer::blit_framebuffer(const u8* source_buffer, u32 source_width, u32 source_height,
                                           u32 dest_x, u32 dest_y) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    if (!source_buffer) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Source buffer is null"));
    }
    
    // Copy pixels from source to destination
    for (u32 y = 0; y < source_height; ++y) {
        for (u32 x = 0; x < source_width; ++x) {
            u32 dest_pixel_x = dest_x + x;
            u32 dest_pixel_y = dest_y + y;
            
            if (dest_pixel_x < width_ && dest_pixel_y < height_) {
                size_t source_offset = (y * source_width + x) * BYTES_PER_PIXEL;
                const u8* source_pixel = source_buffer + source_offset;
                
                u32 color = (static_cast<u32>(source_pixel[0]) << 16) |  // R
                           (static_cast<u32>(source_pixel[1]) << 8) |   // G
                           static_cast<u32>(source_pixel[2]);           // B
                
                RETURN_IF_ERROR(set_pixel(dest_pixel_x, dest_pixel_y, color));
            }
        }
    }
    
    return {};
}

Result<void> SdlRenderer::present() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    auto frame_start = std::chrono::steady_clock::now();
    
    // Update texture with framebuffer data
    void* texture_pixels;
    int pitch;
    
    if (SDL_LockTexture(texture_, nullptr, &texture_pixels, &pitch) < 0) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to lock SDL texture: " + std::string(SDL_GetError())));
    }
    
    // Copy framebuffer to texture
    size_t row_size = width_ * BYTES_PER_PIXEL;
    const u8* src = framebuffer_.get();
    u8* dst = static_cast<u8*>(texture_pixels);
    
    for (u32 y = 0; y < height_; ++y) {
        std::memcpy(dst, src, row_size);
        src += row_size;
        dst += pitch;
    }
    
    SDL_UnlockTexture(texture_);
    
    // Clear renderer and draw texture
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
    SDL_RenderPresent(renderer_);
    
    // Update statistics
    auto frame_end = std::chrono::steady_clock::now();
    auto frame_time = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
    
    statistics_.frames_rendered++;
    statistics_.total_render_time += frame_time.count();
    
    // Calculate FPS
    static auto last_fps_update = frame_start;
    static u32 frame_count = 0;
    frame_count++;
    
    auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frame_start - last_fps_update);
    if (fps_elapsed.count() >= 1000) {
        statistics_.current_fps = static_cast<double>(frame_count) * 1000.0 / fps_elapsed.count();
        frame_count = 0;
        last_fps_update = frame_start;
    }
    
    return {};
}

Result<std::vector<InputEvent>> SdlRenderer::poll_events() {
    std::vector<InputEvent> events;
    
    if (!initialized_) {
        return events;
    }
    
    SDL_Event sdl_event;
    while (SDL_PollEvent(&sdl_event)) {
        InputEvent event{};
        
        switch (sdl_event.type) {
            case SDL_QUIT:
                event.type = InputEventType::QUIT;
                events.push_back(event);
                break;
                
            case SDL_KEYDOWN:
                event.type = InputEventType::KEY_DOWN;
                event.key.scancode = sdl_event.key.keysym.scancode;
                event.key.keycode = sdl_event.key.keysym.sym;
                event.key.modifiers = sdl_event.key.keysym.mod;
                events.push_back(event);
                break;
                
            case SDL_KEYUP:
                event.type = InputEventType::KEY_UP;
                event.key.scancode = sdl_event.key.keysym.scancode;
                event.key.keycode = sdl_event.key.keysym.sym;
                event.key.modifiers = sdl_event.key.keysym.mod;
                events.push_back(event);
                break;
                
            case SDL_MOUSEBUTTONDOWN:
                event.type = InputEventType::MOUSE_DOWN;
                event.mouse.button = sdl_event.button.button;
                event.mouse.x = sdl_event.button.x;
                event.mouse.y = sdl_event.button.y;
                events.push_back(event);
                break;
                
            case SDL_MOUSEBUTTONUP:
                event.type = InputEventType::MOUSE_UP;
                event.mouse.button = sdl_event.button.button;
                event.mouse.x = sdl_event.button.x;
                event.mouse.y = sdl_event.button.y;
                events.push_back(event);
                break;
                
            case SDL_MOUSEMOTION:
                event.type = InputEventType::MOUSE_MOTION;
                event.mouse.x = sdl_event.motion.x;
                event.mouse.y = sdl_event.motion.y;
                event.mouse.relative_x = sdl_event.motion.xrel;
                event.mouse.relative_y = sdl_event.motion.yrel;
                events.push_back(event);
                break;
                
            case SDL_WINDOWEVENT:
                if (sdl_event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    event.type = InputEventType::WINDOW_RESIZED;
                    event.window.width = sdl_event.window.data1;
                    event.window.height = sdl_event.window.data2;
                    events.push_back(event);
                }
                break;
        }
    }
    
    return events;
}

u32 SdlRenderer::get_width() const {
    return width_;
}

u32 SdlRenderer::get_height() const {
    return height_;
}

bool SdlRenderer::is_vsync_enabled() const {
    return vsync_enabled_;
}

const RenderStatistics& SdlRenderer::get_statistics() const {
    return statistics_;
}

void SdlRenderer::clear_statistics() {
    statistics_ = RenderStatistics{};
}

u8* SdlRenderer::get_framebuffer() const {
    return framebuffer_.get();
}

size_t SdlRenderer::get_framebuffer_size() const {
    return width_ * height_ * BYTES_PER_PIXEL;
}

Result<void> SdlRenderer::resize(u32 new_width, u32 new_height) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    COMPONENT_LOG_INFO("Resizing framebuffer from {}x{} to {}x{}", 
                      width_, height_, new_width, new_height);
    
    // Destroy old texture
    if (texture_) {
        SDL_DestroyTexture(texture_);
    }
    
    // Create new texture
    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_RGB888,
        SDL_TEXTUREACCESS_STREAMING,
        static_cast<int>(new_width),
        static_cast<int>(new_height)
    );
    
    if (!texture_) {
        return std::unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create resized SDL texture: " + std::string(SDL_GetError())));
    }
    
    // Reallocate framebuffer
    width_ = new_width;
    height_ = new_height;
    
    size_t framebuffer_size = width_ * height_ * BYTES_PER_PIXEL;
    framebuffer_ = std::make_unique<u8[]>(framebuffer_size);
    std::memset(framebuffer_.get(), 0, framebuffer_size);
    
    // Update logical size
    SDL_RenderSetLogicalSize(renderer_, static_cast<int>(width_), static_cast<int>(height_));
    
    COMPONENT_LOG_INFO("Framebuffer resized successfully");
    return {};
}

}  // namespace m5tab5::emulator