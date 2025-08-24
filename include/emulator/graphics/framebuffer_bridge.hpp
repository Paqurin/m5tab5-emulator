#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/graphics/touch_input.hpp"

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace m5tab5::emulator {

/**
 * @brief Bridge between emulator output and display renderer
 * 
 * Connects the EmulatorCore display output to SDL2 renderer providing:
 * - Real-time framebuffer streaming (30-60 FPS)
 * - Efficient memory management for 1280×720 display
 * - Double buffering with vsync synchronization
 * - Touch input conversion (mouse → GT911 touch events)
 * - Display refresh synchronization with emulator timing
 */
class FramebufferBridge {
public:
    struct BridgeConfig {
        u32 display_width = DISPLAY_WIDTH;      // 1280
        u32 display_height = DISPLAY_HEIGHT;    // 720
        u32 target_fps = 60;                    // Target framerate
        bool enable_vsync = true;               // VSync enabled
        bool enable_touch = true;               // Touch input enabled
        bool enable_double_buffer = true;       // Double buffering
        std::string window_title = "M5Stack Tab5 Emulator";
        
        // Performance tuning
        u32 max_frame_skip = 2;                // Skip frames under load
        bool adaptive_fps = true;              // Adapt to system performance
        u32 touch_sample_rate = 120;          // Hz for touch sampling
    };

    explicit FramebufferBridge(const BridgeConfig& config);
    FramebufferBridge(); // Default constructor with default config
    ~FramebufferBridge();

    // Lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    Result<void> reset();

    // Framebuffer operations
    Result<void> update_framebuffer(const Framebuffer& source_framebuffer);
    Result<void> present_frame();
    Result<void> clear_display(u32 color = 0x000000);

    // Touch input integration
    Result<void> process_touch_events();
    Result<std::vector<TouchPoint>> get_current_touches() const;
    Result<void> inject_touch_event(const TouchPoint& touch);

    // Display properties
    u32 get_display_width() const { return config_.display_width; }
    u32 get_display_height() const { return config_.display_height; }
    u32 get_current_fps() const { return current_fps_.load(); }
    bool is_vsync_enabled() const { return config_.enable_vsync; }

    // Renderer access
    SdlRenderer& get_renderer() { return *sdl_renderer_; }
    const SdlRenderer& get_renderer() const { return *sdl_renderer_; }

    // Display state
    bool is_display_active() const { return display_active_.load(); }
    bool should_close() const;

    // Performance monitoring
    struct BridgeStatistics {
        u64 frames_processed = 0;
        u64 frames_dropped = 0;
        u64 touch_events_processed = 0;
        double average_frame_time_ms = 0.0;
        double current_fps = 0.0;
        u64 memory_transfers = 0;
    };
    
    BridgeStatistics get_statistics() const;
    void reset_statistics();

private:
    // Core components
    BridgeConfig config_;
    std::unique_ptr<SdlRenderer> sdl_renderer_;
    std::unique_ptr<Framebuffer> display_framebuffer_;
    std::unique_ptr<Framebuffer> back_framebuffer_;
    std::unique_ptr<TouchInput> touch_input_;

    // Threading and synchronization
    std::thread render_thread_;
    std::thread touch_thread_;
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> display_active_{false};

    // Frame synchronization
    mutable std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::atomic<bool> new_frame_available_{false};
    std::atomic<bool> frame_processing_{false};

    // Performance tracking
    std::atomic<u32> current_fps_{0};
    std::atomic<u64> frames_rendered_{0};
    std::atomic<u64> frames_dropped_{0};
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point last_fps_update_;

    // Touch tracking
    mutable std::mutex touch_mutex_;
    std::vector<TouchPoint> current_touches_;
    std::chrono::steady_clock::time_point last_touch_update_;

    // Statistics
    mutable std::mutex stats_mutex_;
    BridgeStatistics statistics_;

    // Thread functions
    void render_loop();
    void touch_loop();

    // Frame processing
    Result<void> process_frame();
    Result<void> convert_and_copy_framebuffer(const Framebuffer& source);
    Result<void> handle_frame_timing();
    bool should_drop_frame() const;

    // Touch processing
    void process_mouse_to_touch();
    Result<void> update_touch_state();
    TouchPoint convert_mouse_to_touch(i32 mouse_x, i32 mouse_y, bool pressed) const;

    // Performance optimization
    void update_fps_counter();
    void adjust_adaptive_fps();
    Result<void> optimize_rendering_pipeline();

    // Cleanup
    void cleanup_threads();
    void cleanup_resources();
};

/**
 * @brief Display adapter for handling different display modes and optimizations
 * 
 * Manages display-specific optimizations for M5Stack Tab5:
 * - ESP32-P4 graphics acceleration features simulation
 * - MIPI-DSI interface emulation
 * - Color space conversion optimizations
 * - Memory bandwidth optimization
 */
class DisplayAdapter {
public:
    enum class DisplayMode {
        RGB565,         // 16-bit color mode (memory optimized)
        RGB888,         // 24-bit color mode (standard)
        ARGB8888,       // 32-bit with alpha (full featured)
        MONO            // Monochrome mode (ultra low power)
    };

    enum class RefreshMode {
        VSYNC,          // Standard VSync timing
        ADAPTIVE,       // Adaptive refresh based on content
        IMMEDIATE,      // No sync (maximum performance)
        POWER_SAVE      // Reduced refresh for battery life
    };

    struct AdapterConfig {
        DisplayMode display_mode = DisplayMode::RGB888;
        RefreshMode refresh_mode = RefreshMode::VSYNC;
        bool hardware_acceleration = true;
        bool color_correction = true;
        float gamma = 2.2f;
        u32 brightness = 255;       // 0-255
        u32 contrast = 128;         // 0-255
        bool anti_aliasing = false; // For scaled content
    };

    explicit DisplayAdapter(const AdapterConfig& config);
    DisplayAdapter(); // Default constructor with default config
    ~DisplayAdapter();

    // Configuration
    Result<void> set_display_mode(DisplayMode mode);
    Result<void> set_refresh_mode(RefreshMode mode);
    Result<void> set_brightness(u32 brightness);
    Result<void> set_contrast(u32 contrast);
    Result<void> set_gamma(float gamma);

    // Color processing
    Result<void> apply_color_correction(Framebuffer& framebuffer);
    Result<void> convert_color_space(const Framebuffer& source, Framebuffer& dest);
    
    // Performance optimizations
    Result<void> optimize_for_content_type(const Framebuffer& framebuffer);
    bool should_skip_frame(const Framebuffer& current, const Framebuffer& previous) const;
    
    // Hardware acceleration simulation
    Result<void> simulate_gpu_acceleration(Framebuffer& framebuffer);
    Result<void> apply_mipi_dsi_timing();

    // Diagnostics
    struct AdapterStatistics {
        u64 color_conversions = 0;
        u64 frames_optimized = 0;
        u64 frames_skipped = 0;
        double processing_time_ms = 0.0;
        u32 current_brightness = 255;
        DisplayMode current_mode = DisplayMode::RGB888;
    };
    
    AdapterStatistics get_statistics() const { return stats_; }
    void reset_statistics() { stats_ = {}; }

private:
    AdapterConfig config_;
    mutable AdapterStatistics stats_;
    
    // Color correction tables
    std::array<u8, 256> gamma_table_;
    std::array<u8, 256> brightness_table_;
    std::array<u8, 256> contrast_table_;
    
    // Content analysis
    struct ContentAnalysis {
        bool is_static_content = false;
        bool has_transparency = false;
        bool is_high_contrast = false;
        u32 dominant_colors[8] = {0};
        float motion_level = 0.0f;
    };
    
    ContentAnalysis last_analysis_;
    
    // Internal methods
    void build_correction_tables();
    ContentAnalysis analyze_content(const Framebuffer& framebuffer) const;
    Result<void> apply_gamma_correction(Framebuffer& framebuffer);
    Result<void> apply_brightness_adjustment(Framebuffer& framebuffer);
    Result<void> apply_contrast_adjustment(Framebuffer& framebuffer);
    
    // Hardware simulation
    void simulate_mipi_dsi_transfer();
    void simulate_lcd_controller_timing();
};

} // namespace m5tab5::emulator