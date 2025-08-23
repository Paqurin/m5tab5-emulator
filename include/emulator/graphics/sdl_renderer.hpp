#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <string>
#include <memory>

#ifndef NO_GRAPHICS
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;
#endif

namespace m5tab5::emulator {

class Framebuffer;

class SdlRenderer {
public:
    SdlRenderer();
    ~SdlRenderer();

    Result<void> initialize(u32 width, u32 height, bool vsync = true, const std::string& title = "M5Tab5 Emulator");
    Result<void> shutdown();

    Result<void> render(const Framebuffer& framebuffer);
    Result<void> present();
    Result<void> clear(u32 color = 0x000000);
    
    void set_vsync(bool enabled);
    bool is_vsync_enabled() const { return vsync_enabled_; }
    
    u32 get_width() const { return width_; }
    u32 get_height() const { return height_; }
    bool is_initialized() const { return initialized_; }

    void handle_events();
    bool should_close() const;

private:
    bool initialized_;
    u32 width_;
    u32 height_;
    bool vsync_enabled_;
    std::string window_title_;
    
#ifndef NO_GRAPHICS
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;
#else
    void* window_;
    void* renderer_;
    void* texture_;
#endif

    u8* framebuffer_;
    bool should_close_;
    
    Result<void> create_window(const std::string& title);
    Result<void> create_renderer();
    Result<void> create_texture();
    void cleanup();
};

}  // namespace m5tab5::emulator