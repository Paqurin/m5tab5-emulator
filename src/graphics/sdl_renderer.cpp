#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include "emulator/utils/logging.hpp"

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

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
      should_close_(false),
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
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "SDL renderer already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing SDL renderer: {}x{}", width, height);
    
    width_ = width;
    height_ = height;
    vsync_enabled_ = vsync;
    window_title_ = title;
    
#ifndef NO_GRAPHICS
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to initialize SDL: " + std::string(SDL_GetError())));
    }
    
    RETURN_IF_ERROR(create_window(title));
    RETURN_IF_ERROR(create_renderer());
    RETURN_IF_ERROR(create_texture());
    
#else
    // No graphics - stub implementation
    COMPONENT_LOG_INFO("Graphics disabled (NO_GRAPHICS defined)");
#endif
    
    // Allocate framebuffer
    size_t buffer_size = width_ * height_ * 4; // 4 bytes per pixel (RGBA)
    framebuffer_ = new u8[buffer_size];
    std::memset(framebuffer_, 0, buffer_size);
    
    initialized_ = true;
    COMPONENT_LOG_INFO("SDL renderer initialized successfully");
    
    return {};
}

Result<void> SdlRenderer::shutdown() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    COMPONENT_LOG_INFO("Shutting down SDL renderer");
    
    cleanup();
    
    if (framebuffer_) {
        delete[] framebuffer_;
        framebuffer_ = nullptr;
    }
    
#ifndef NO_GRAPHICS
    SDL_Quit();
#endif
    
    initialized_ = false;
    COMPONENT_LOG_INFO("SDL renderer shutdown complete");
    
    return {};
}

Result<void> SdlRenderer::render(const Framebuffer& framebuffer) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
#ifndef NO_GRAPHICS
    // Copy framebuffer data to texture and render
    if (texture_) {
        const void* buffer = framebuffer.getBuffer();
        SDL_UpdateTexture(texture_, nullptr, buffer, static_cast<int>(framebuffer.getStride()));
        
        SDL_RenderClear(renderer_);
        SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
    }
#else
    // Stub implementation for no graphics
    COMPONENT_LOG_DEBUG("Render called (NO_GRAPHICS - stub)");
#endif
    
    return {};
}

Result<void> SdlRenderer::present() {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
#ifndef NO_GRAPHICS
    if (renderer_) {
        SDL_RenderPresent(renderer_);
    }
#else
    // Stub implementation
#endif
    
    return {};
}

Result<void> SdlRenderer::clear(u32 color) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
#ifndef NO_GRAPHICS
    if (renderer_) {
        u8 r = static_cast<u8>((color >> 16) & 0xFF);
        u8 g = static_cast<u8>((color >> 8) & 0xFF);
        u8 b = static_cast<u8>(color & 0xFF);
        
        SDL_SetRenderDrawColor(renderer_, r, g, b, 255);
        SDL_RenderClear(renderer_);
    }
#else
    // Clear framebuffer manually
    if (framebuffer_) {
        size_t buffer_size = width_ * height_ * 4;
        u32* pixels = reinterpret_cast<u32*>(framebuffer_);
        for (size_t i = 0; i < width_ * height_; ++i) {
            pixels[i] = color;
        }
    }
#endif
    
    return {};
}

void SdlRenderer::set_vsync(bool enabled) {
    vsync_enabled_ = enabled;
#ifndef NO_GRAPHICS
    // Would need to recreate renderer to change vsync
    COMPONENT_LOG_DEBUG("VSync setting changed to: {}", enabled);
#endif
}

void SdlRenderer::handle_events() {
#ifndef NO_GRAPHICS
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            should_close_ = true;
        }
        // Handle other events as needed
    }
#endif
}

bool SdlRenderer::should_close() const {
    return should_close_;
}

Result<void> SdlRenderer::create_window(const std::string& title) {
#ifndef NO_GRAPHICS
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
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL window: " + std::string(SDL_GetError())));
    }
#endif
    return {};
}

Result<void> SdlRenderer::create_renderer() {
#ifndef NO_GRAPHICS
    u32 renderer_flags = SDL_RENDERER_ACCELERATED;
    if (vsync_enabled_) {
        renderer_flags |= SDL_RENDERER_PRESENTVSYNC;
    }
    
    renderer_ = SDL_CreateRenderer(window_, -1, renderer_flags);
    if (!renderer_) {
        SDL_DestroyWindow(window_);
        SDL_Quit();
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL renderer: " + std::string(SDL_GetError())));
    }
#endif
    return {};
}

Result<void> SdlRenderer::create_texture() {
#ifndef NO_GRAPHICS
    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_RGBA8888,
        SDL_TEXTUREACCESS_STREAMING,
        static_cast<int>(width_),
        static_cast<int>(height_)
    );
    
    if (!texture_) {
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
        return unexpected(MAKE_ERROR(OPERATION_FAILED,
            "Failed to create SDL texture: " + std::string(SDL_GetError())));
    }
#endif
    return {};
}

void SdlRenderer::cleanup() {
#ifndef NO_GRAPHICS
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
#endif
}

// Additional drawing methods for GUI support
Result<void> SdlRenderer::draw_rect(i32 x, i32 y, u32 width, u32 height, u32 color) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
#ifndef NO_GRAPHICS
    if (renderer_) {
        u8 r = static_cast<u8>((color >> 16) & 0xFF);
        u8 g = static_cast<u8>((color >> 8) & 0xFF);
        u8 b = static_cast<u8>(color & 0xFF);
        
        SDL_SetRenderDrawColor(renderer_, r, g, b, 255);
        
        SDL_Rect rect;
        rect.x = x;
        rect.y = y;
        rect.w = static_cast<int>(width);
        rect.h = static_cast<int>(height);
        SDL_RenderFillRect(renderer_, &rect);
    }
#else
    // Manual framebuffer drawing
    if (framebuffer_) {
        u32* pixels = reinterpret_cast<u32*>(framebuffer_);
        for (u32 py = y; py < y + height && py < height_; ++py) {
            for (u32 px = x; px < x + width && px < width_; ++px) {
                pixels[py * width_ + px] = color;
            }
        }
    }
#endif
    
    return {};
}

Result<void> SdlRenderer::draw_text(i32 x, i32 y, const std::string& text, u32 color) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not initialized"));
    }
    
    // Simple text rendering stub - in a full implementation would use TTF fonts
    COMPONENT_LOG_DEBUG("Drawing text at ({}, {}): '{}'", x, y, text);
    
    // Draw a simple rectangle as placeholder for text
    u32 text_width = static_cast<u32>(text.length() * 8); // 8 pixels per char
    u32 text_height = 12; // 12 pixels tall
    
    return draw_rect(x, y, text_width, text_height, color);
}

}  // namespace m5tab5::emulator