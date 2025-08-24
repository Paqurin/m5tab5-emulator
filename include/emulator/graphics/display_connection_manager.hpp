#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include "emulator/graphics/framebuffer_bridge.hpp"
#include "emulator/graphics/touch_input.hpp"

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <functional>

namespace m5tab5::emulator {

class GraphicsEngine;

/**
 * @brief High-performance display connection manager for M5Stack Tab5 emulator
 * 
 * Manages the complete pipeline from EmulatorCore display output to SDL2 renderer:
 * - Real-time framebuffer streaming at 30-60 FPS
 * - M5Stack Tab5 authentic 1280×720 display simulation
 * - GT911 touch controller simulation with sub-millisecond response
 * - Efficient memory management for high-resolution display
 * - Display refresh synchronization with emulator timing
 * - ESP32-P4 graphics acceleration features simulation
 * - MIPI-DSI interface emulation with proper timing
 */
class DisplayConnectionManager {
public:
    /**
     * @brief Performance optimization levels for different use cases
     */
    enum class PerformanceMode {
        POWER_SAVE,     // Lower FPS, reduced CPU usage
        BALANCED,       // Standard 60 FPS operation
        HIGH_PERFORMANCE, // Maximum FPS, prioritize responsiveness
        DEVELOPMENT     // Extra diagnostics and logging
    };

    /**
     * @brief Connection configuration for display pipeline
     */
    struct ConnectionConfig {
        // Display properties
        u32 display_width = DISPLAY_WIDTH;      // 1280
        u32 display_height = DISPLAY_HEIGHT;    // 720
        u32 target_fps = 60;                    // Target frame rate
        PerformanceMode performance_mode = PerformanceMode::BALANCED;
        
        // Touch input configuration
        bool enable_touch = true;               // Enable GT911 touch simulation
        u32 touch_sample_rate = 120;           // Hz for touch polling
        float touch_sensitivity = 1.0f;        // Touch sensitivity multiplier
        
        // Memory and threading
        bool enable_triple_buffering = false;  // Triple buffering for smoothness
        u32 render_thread_priority = 0;        // Thread priority (0 = default)
        u32 memory_pool_size = 64 * 1024 * 1024; // 64MB memory pool
        
        // Quality settings
        bool enable_anti_aliasing = false;     // Anti-aliasing for scaled content
        bool enable_color_correction = true;   // Color correction pipeline
        float gamma = 2.2f;                     // Gamma correction
        u32 brightness = 255;                   // Brightness (0-255)
        u32 contrast = 128;                     // Contrast (0-255)
        
        // Emulator integration
        bool sync_with_emulator_timing = true; // Sync with emulator clock
        bool enable_frame_skip = true;         // Allow frame skipping under load
        u32 max_frame_skip = 2;                // Maximum consecutive frame skips
        
        // Window properties
        std::string window_title = "M5Stack Tab5 Emulator - Real-time Display";
        bool window_resizable = true;          // Allow window resizing
        bool fullscreen = false;               // Start in fullscreen mode
    };

    /**
     * @brief Real-time performance statistics
     */
    struct PerformanceStats {
        // Frame timing
        double current_fps = 0.0;
        double average_fps = 0.0;
        double frame_time_ms = 0.0;
        double min_frame_time_ms = 0.0;
        double max_frame_time_ms = 0.0;
        
        // Frame processing
        u64 frames_rendered = 0;
        u64 frames_dropped = 0;
        u64 frames_skipped = 0;
        double frame_drop_rate = 0.0;
        
        // Memory usage
        u64 memory_used_bytes = 0;
        u64 peak_memory_bytes = 0;
        double memory_bandwidth_mbps = 0.0;
        
        // Touch input
        u64 touch_events_processed = 0;
        double touch_latency_ms = 0.0;
        u32 active_touch_points = 0;
        
        // System integration
        double cpu_usage_percent = 0.0;
        double gpu_usage_percent = 0.0;
        bool vsync_active = false;
        
        // Quality metrics
        double color_accuracy = 1.0;
        u32 artifacts_detected = 0;
        bool display_tearing = false;
    };

    /**
     * @brief Callback for emulator frame updates
     */
    using FrameUpdateCallback = std::function<void(const Framebuffer& framebuffer)>;
    
    /**
     * @brief Callback for touch events
     */
    using TouchEventCallback = std::function<void(const std::vector<TouchPoint>& touches)>;

    explicit DisplayConnectionManager(const ConnectionConfig& config);
    DisplayConnectionManager(); // Default constructor with default config
    ~DisplayConnectionManager();

    // Lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    Result<void> reset();

    // Connection interface
    Result<void> connect_to_graphics_engine(GraphicsEngine* graphics_engine);
    Result<void> disconnect_from_graphics_engine();
    bool is_connected() const { return graphics_engine_ != nullptr; }

    // Display operations
    Result<void> update_display(const Framebuffer& source_framebuffer);
    Result<void> force_refresh();
    Result<void> clear_display(u32 color = 0x000000);

    // Touch input management
    Result<std::vector<TouchPoint>> get_current_touches() const;
    Result<void> inject_touch_event(const TouchPoint& touch);
    void set_touch_event_callback(TouchEventCallback callback);

    // Frame update notifications
    void set_frame_update_callback(FrameUpdateCallback callback);

    // Performance management
    Result<void> set_performance_mode(PerformanceMode mode);
    Result<void> set_target_fps(u32 fps);
    Result<void> enable_vsync(bool enabled);
    Result<void> enable_frame_skip(bool enabled);

    // Quality control
    Result<void> set_gamma(float gamma);
    Result<void> set_brightness(u32 brightness);
    Result<void> set_contrast(u32 contrast);
    Result<void> enable_anti_aliasing(bool enabled);

    // Statistics and monitoring
    PerformanceStats get_performance_stats() const;
    void reset_statistics();
    bool is_display_healthy() const;

    // Display state
    bool is_display_active() const;
    bool should_close_display() const;
    u32 get_current_fps() const;

    // Advanced features
    Result<void> take_screenshot(const std::string& filename);
    Result<void> start_frame_capture(const std::string& output_dir);
    Result<void> stop_frame_capture();

private:
    // Core components
    ConnectionConfig config_;
    std::unique_ptr<FramebufferBridge> display_bridge_;
    std::unique_ptr<DisplayAdapter> display_adapter_;
    GraphicsEngine* graphics_engine_;

    // Performance optimization
    PerformanceMode current_performance_mode_;
    std::atomic<u32> target_fps_;
    std::atomic<bool> vsync_enabled_;
    std::atomic<bool> frame_skip_enabled_;

    // Threading and synchronization
    std::thread connection_thread_;
    std::thread performance_monitor_thread_;
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> initialized_{false};

    // Frame management
    mutable std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::unique_ptr<Framebuffer> current_framebuffer_;
    std::unique_ptr<Framebuffer> previous_framebuffer_;
    std::atomic<bool> new_frame_available_{false};

    // Performance tracking
    mutable std::mutex stats_mutex_;
    PerformanceStats statistics_;
    std::chrono::steady_clock::time_point last_stats_update_;
    std::chrono::steady_clock::time_point connection_start_time_;

    // Callbacks
    FrameUpdateCallback frame_update_callback_;
    TouchEventCallback touch_event_callback_;
    mutable std::mutex callback_mutex_;

    // Frame capture
    std::atomic<bool> capturing_frames_{false};
    std::string capture_directory_;
    u64 capture_frame_count_;

    // Thread functions
    void connection_loop();
    void performance_monitor_loop();

    // Frame processing
    Result<void> process_incoming_frame(const Framebuffer& source);
    Result<void> apply_quality_enhancements(Framebuffer& framebuffer);
    Result<void> optimize_for_performance_mode(Framebuffer& framebuffer);
    bool should_skip_frame() const;

    // Performance optimization
    void update_performance_statistics();
    void adjust_performance_settings();
    Result<void> optimize_memory_usage();
    Result<void> tune_threading_parameters();

    // Touch processing
    void process_touch_input();
    Result<void> handle_touch_events();

    // Quality processing
    Result<void> apply_gamma_correction(Framebuffer& framebuffer);
    Result<void> apply_brightness_contrast(Framebuffer& framebuffer);
    Result<void> apply_anti_aliasing(Framebuffer& framebuffer);

    // Frame capture
    Result<void> capture_frame_to_file(const Framebuffer& framebuffer, const std::string& filename);

    // Cleanup
    void cleanup_threads();
    void cleanup_resources();

    // Performance mode configurations
    ConnectionConfig get_performance_mode_config(PerformanceMode mode) const;
    void apply_performance_mode_settings(PerformanceMode mode);

    // Health monitoring
    bool check_display_health() const;
    void log_performance_warnings() const;
};

/**
 * @brief Factory for creating optimized display connection managers
 */
class DisplayConnectionFactory {
public:
    /**
     * @brief Create connection manager optimized for development
     */
    static std::unique_ptr<DisplayConnectionManager> create_development_connection();

    /**
     * @brief Create connection manager optimized for production
     */
    static std::unique_ptr<DisplayConnectionManager> create_production_connection();

    /**
     * @brief Create connection manager optimized for performance testing
     */
    static std::unique_ptr<DisplayConnectionManager> create_performance_test_connection();

    /**
     * @brief Create connection manager optimized for power efficiency
     */
    static std::unique_ptr<DisplayConnectionManager> create_power_efficient_connection();

    /**
     * @brief Create connection manager with custom configuration
     */
    static std::unique_ptr<DisplayConnectionManager> create_custom_connection(
        const DisplayConnectionManager::ConnectionConfig& config);
};

} // namespace m5tab5::emulator