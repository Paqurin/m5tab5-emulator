#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include "emulator/utils/logging.hpp"

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
// SDL_ttf not available - using bitmap font fallback
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

// Simple 8x8 bitmap font data (first 128 ASCII characters)
static const u8 bitmap_font_8x8[128][8] = {
    // ASCII 0-31 (control characters) - empty
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    // ASCII 32-47 (space and symbols)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // "
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // $
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x06,0x00}, // ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // /
    // ASCII 48-57 (digits 0-9)
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 9
    // ASCII 58-64 (symbols)
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x06,0x00}, // ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // <
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // @
    // ASCII 65-90 (uppercase A-Z)
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // Z
    // ASCII 91-96 (symbols)
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // [
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // backslash
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // ]
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // _
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, // `
    // ASCII 97-122 (lowercase a-z)
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, // a
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, // b
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, // c
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00}, // d
    {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00}, // e
    {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00}, // f
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, // g
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, // h
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, // i
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, // j
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, // k
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // l
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, // m
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, // n
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, // o
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, // p
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, // q
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, // r
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, // s
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, // t
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, // u
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, // v
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // w
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, // x
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, // y
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, // z
    // ASCII 123-127 (symbols)
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // {
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // |
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // }
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, // ~
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // DEL
};

void SdlRenderer::draw_bitmap_char(i32 x, i32 y, char c, u32 color) {
#ifndef NO_GRAPHICS
    if (!renderer_) return;
    
    u8 char_index = static_cast<u8>(c);
    if (char_index >= 128) char_index = 63; // fallback to '?' character (ASCII 63)
    
    u8 r = static_cast<u8>((color >> 16) & 0xFF);
    u8 g = static_cast<u8>((color >> 8) & 0xFF);
    u8 b = static_cast<u8>(color & 0xFF);
    SDL_SetRenderDrawColor(renderer_, r, g, b, 255);
    
    const u8* font_data = bitmap_font_8x8[char_index];
    
    // Render character with bitmap font data
    
    // Render each row of the character
    int pixels_drawn = 0;
    for (int row = 0; row < 8; row++) {
        u8 row_data = font_data[row];
        for (int col = 0; col < 8; col++) {
            // Check bit from right to left (LSB first) - fix backwards letters
            if (row_data & (0x01 << col)) {
                SDL_Rect point_rect = {x + col, y + row, 1, 1};
                SDL_RenderFillRect(renderer_, &point_rect);
                pixels_drawn++;
            }
        }
    }
    
    // Space character and some control characters naturally have no pixels
    if (pixels_drawn == 0 && c != ' ' && char_index >= 32) {
        COMPONENT_LOG_DEBUG("Character '{}' rendered with no pixels", c);
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
    
    if (text.empty()) {
        return {}; // Nothing to draw
    }
    
    // Render text using bitmap font
    COMPONENT_LOG_DEBUG("Rendering text: '{}' at ({},{}) with color 0x{:06X}", text, x, y, color);
    
#ifndef NO_GRAPHICS
    if (!renderer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "SDL renderer not available"));
    }
    
    // Render each character using bitmap font
    i32 char_x = x;
    for (size_t i = 0; i < text.length(); ++i) {
        draw_bitmap_char(char_x, y, text[i], color);
        char_x += 8; // Each character is 8 pixels wide
    }
    
    COMPONENT_LOG_DEBUG("Text rendered: '{}' ({}x8 pixels)", text, static_cast<int>(text.length() * 8));
    
#else
    // Stub implementation for NO_GRAPHICS
    COMPONENT_LOG_DEBUG("Drawing text at ({}, {}): '{}' (NO_GRAPHICS)", x, y, text);
#endif
    
    return {};
}

Result<void> SdlRenderer::render_framebuffer(i32 x, i32 y, u32 dest_width, u32 dest_height, const Framebuffer* framebuffer) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "SDL renderer not initialized"));
    }
    
    if (!framebuffer) {
        return unexpected(MAKE_ERROR(INVALID_ARGUMENT, "Framebuffer is null"));
    }
    
    u32 src_width = framebuffer->getWidth();
    u32 src_height = framebuffer->getHeight();
    
    if (src_width == 0 || src_height == 0) {
        return unexpected(MAKE_ERROR(INVALID_ARGUMENT, "Framebuffer has zero dimensions"));
    }
    
    COMPONENT_LOG_DEBUG("Rendering framebuffer ({}x{}) to destination ({}x{}) at ({}, {})",
                       src_width, src_height, dest_width, dest_height, x, y);

#ifndef NO_GRAPHICS
    if (!renderer_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED, "SDL renderer not available"));
    }
    
    // Get framebuffer data
    const u8* pixel_data = static_cast<const u8*>(framebuffer->getBuffer());
    if (!pixel_data) {
        return unexpected(MAKE_ERROR(INVALID_ARGUMENT, "Framebuffer data is null"));
    }
    
    // Simple pixel-by-pixel scaling for now
    // TODO: Optimize with SDL texture scaling for better performance
    float scale_x = static_cast<float>(src_width) / dest_width;
    float scale_y = static_cast<float>(src_height) / dest_height;
    
    for (u32 dy = 0; dy < dest_height; ++dy) {
        for (u32 dx = 0; dx < dest_width; ++dx) {
            // Sample source pixel (nearest neighbor)
            u32 sx = static_cast<u32>(dx * scale_x);
            u32 sy = static_cast<u32>(dy * scale_y);
            
            if (sx < src_width && sy < src_height) {
                // Assume RGBA format for framebuffer
                u32 pixel_index = (sy * src_width + sx) * 4; // 4 bytes per pixel (RGBA)
                u8 r = pixel_data[pixel_index];
                u8 g = pixel_data[pixel_index + 1]; 
                u8 b = pixel_data[pixel_index + 2];
                u8 a = pixel_data[pixel_index + 3];
                
                // Set pixel color and draw 1x1 rect
                SDL_SetRenderDrawColor(renderer_, r, g, b, a);
                SDL_Rect pixel_rect = {x + static_cast<int>(dx), y + static_cast<int>(dy), 1, 1};
                SDL_RenderFillRect(renderer_, &pixel_rect);
            }
        }
    }
    
#else
    // Stub implementation for NO_GRAPHICS
    COMPONENT_LOG_DEBUG("Would render framebuffer at ({}, {}) (NO_GRAPHICS)", x, y);
#endif
    
    return {};
}

}  // namespace m5tab5::emulator