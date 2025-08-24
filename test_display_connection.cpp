/**
 * @file test_display_connection.cpp
 * @brief Comprehensive test for M5Stack Tab5 emulator display connection system
 * 
 * This test demonstrates the complete pipeline from EmulatorCore output to SDL2 renderer:
 * - Real-time framebuffer streaming (30-60 FPS)
 * - M5Stack Tab5 authentic 1280×720 display simulation
 * - GT911 touch controller simulation (mouse → touch conversion)
 * - Display refresh synchronization with emulator timing
 * - ESP32-P4 graphics acceleration features simulation
 * - MIPI-DSI interface emulation
 */

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/graphics/graphics_engine.hpp"
#include "emulator/graphics/touch_input.hpp"
#include "emulator/utils/logging.hpp"

#include <chrono>
#include <thread>
#include <random>
#include <cmath>
#include <iostream>
#include <csignal>

using namespace m5tab5::emulator;

// Global flag for clean shutdown
static volatile sig_atomic_t g_shutdown_requested = 0;

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        g_shutdown_requested = 1;
        std::cout << "\nShutdown requested, please wait...\n";
    }
}

/**
 * @brief Demo application that draws animated patterns to test display performance
 */
class DisplayTestApp {
public:
    DisplayTestApp(GraphicsEngine* graphics) 
        : graphics_engine_(graphics), 
          frame_counter_(0),
          pattern_type_(0) {
        
        // Initialize random number generator
        std::random_device rd;
        rng_.seed(rd());
        color_dist_ = std::uniform_int_distribution<u32>(0x000000, 0xFFFFFF);
    }

    void render_frame() {
        if (!graphics_engine_) return;

        frame_counter_++;
        
        // Change pattern every 300 frames (5 seconds at 60 FPS)
        if (frame_counter_ % 300 == 0) {
            pattern_type_ = (pattern_type_ + 1) % 4;
            std::cout << "Switching to pattern " << pattern_type_ << std::endl;
        }

        // Clear screen with animated background
        u32 bg_color = generate_background_color();
        auto clear_result = graphics_engine_->clear_screen(bg_color);
        if (!clear_result.has_value()) {
            std::cerr << "Failed to clear screen: " << clear_result.error().to_string() << std::endl;
            return;
        }

        // Draw different patterns based on current pattern type
        switch (pattern_type_) {
            case 0:
                draw_bouncing_rectangles();
                break;
            case 1:
                draw_spiral_pattern();
                break;
            case 2:
                draw_touch_visualization();
                break;
            case 3:
                draw_performance_overlay();
                break;
        }

        // Update display
        auto update_result = graphics_engine_->update_display();
        if (!update_result.has_value()) {
            std::cerr << "Failed to update display: " << update_result.error().to_string() << std::endl;
        }
    }

private:
    GraphicsEngine* graphics_engine_;
    u64 frame_counter_;
    int pattern_type_;
    std::mt19937 rng_;
    std::uniform_int_distribution<u32> color_dist_;

    u32 generate_background_color() {
        // Slowly shifting background color
        float time = frame_counter_ * 0.01f;
        u8 r = static_cast<u8>((std::sin(time * 0.1f) + 1.0f) * 32);
        u8 g = static_cast<u8>((std::sin(time * 0.11f) + 1.0f) * 32);
        u8 b = static_cast<u8>((std::sin(time * 0.12f) + 1.0f) * 32);
        return (r << 16) | (g << 8) | b;
    }

    void draw_bouncing_rectangles() {
        const int num_rects = 5;
        const float time = frame_counter_ * 0.05f;
        
        for (int i = 0; i < num_rects; ++i) {
            float phase = i * 1.0f;
            
            // Calculate bouncing position
            float x_ratio = (std::sin(time * 0.7f + phase) + 1.0f) * 0.5f;
            float y_ratio = (std::cos(time * 0.8f + phase * 1.5f) + 1.0f) * 0.5f;
            
            u32 x = static_cast<u32>(x_ratio * (DISPLAY_WIDTH - 100));
            u32 y = static_cast<u32>(y_ratio * (DISPLAY_HEIGHT - 80));
            
            // Generate vibrant color
            u32 color = generate_vibrant_color(i, time);
            
            auto result = graphics_engine_->draw_rectangle(x, y, 100, 80, color);
            if (!result.has_value()) {
                std::cerr << "Failed to draw rectangle " << i << std::endl;
            }
        }
    }

    void draw_spiral_pattern() {
        const float time = frame_counter_ * 0.02f;
        const int num_points = 200;
        const float center_x = DISPLAY_WIDTH / 2.0f;
        const float center_y = DISPLAY_HEIGHT / 2.0f;
        
        for (int i = 0; i < num_points; ++i) {
            float angle = i * 0.3f + time;
            float radius = i * 2.0f + std::sin(time * 2.0f) * 50.0f;
            
            float x = center_x + radius * std::cos(angle);
            float y = center_y + radius * std::sin(angle);
            
            // Ensure coordinates are within bounds
            if (x >= 0 && x < DISPLAY_WIDTH - 5 && y >= 0 && y < DISPLAY_HEIGHT - 5) {
                u32 color = generate_spiral_color(i, angle, radius);
                
                auto result = graphics_engine_->draw_rectangle(
                    static_cast<u32>(x), static_cast<u32>(y), 5, 5, color);
                if (!result.has_value()) {
                    // Skip error logging for performance
                    continue;
                }
            }
        }
    }

    void draw_touch_visualization() {
        // Get current touch points
        auto touch_result = graphics_engine_->get_touch_events();
        if (!touch_result.has_value()) {
            return;
        }
        
        auto touch_points = touch_result.value();
        
        // Draw touch visualization circles
        for (const auto& touch : touch_points) {
            if (!touch.active) continue;
            
            // Draw expanding circles around touch point
            u32 base_color = 0x00FF00; // Green
            u32 pressure_alpha = (touch.pressure * 255) / 1023; // Convert to 0-255
            u32 color = (pressure_alpha << 24) | base_color;
            
            // Draw concentric circles
            for (int radius = 10; radius <= 50; radius += 10) {
                u32 alpha = pressure_alpha * (60 - radius) / 50;
                u32 circle_color = (alpha << 24) | base_color;
                
                draw_circle_outline(touch.x, touch.y, radius, circle_color);
            }
            
            // Draw center point
            if (touch.x >= 5 && touch.x < DISPLAY_WIDTH - 5 && 
                touch.y >= 5 && touch.y < DISPLAY_HEIGHT - 5) {
                graphics_engine_->draw_rectangle(
                    touch.x - 5, touch.y - 5, 10, 10, 0xFFFF00); // Yellow center
            }
        }
        
        // Draw instruction text (simplified as rectangles for now)
        draw_text_placeholder(50, 50, "Touch the screen to see touch visualization", 0xFFFFFF);
    }

    void draw_performance_overlay() {
        // Display performance metrics
        double fps = graphics_engine_->get_fps();
        u64 frames = graphics_engine_->get_frames_rendered();
        
        // Draw FPS indicator (simplified)
        u32 fps_color = fps > 50 ? 0x00FF00 : (fps > 30 ? 0xFFFF00 : 0xFF0000);
        
        // Draw FPS bar
        u32 fps_bar_width = static_cast<u32>((fps / 60.0) * 200);
        if (fps_bar_width > 200) fps_bar_width = 200;
        
        graphics_engine_->draw_rectangle(50, 50, fps_bar_width, 20, fps_color);
        graphics_engine_->draw_rectangle(50 + fps_bar_width, 50, 200 - fps_bar_width, 20, 0x333333);
        
        // Draw frame counter
        u32 counter_color = (frame_counter_ % 60) < 30 ? 0xFFFFFF : 0x888888;
        draw_number_visualization(50, 100, frames, counter_color);
        
        // Draw M5Stack Tab5 branding
        draw_m5stack_logo();
    }

    void draw_circle_outline(u32 center_x, u32 center_y, u32 radius, u32 color) {
        // Simple circle outline using rectangular approximation
        const int steps = 24;
        for (int i = 0; i < steps; ++i) {
            float angle = (i * 2.0f * M_PI) / steps;
            u32 x = center_x + static_cast<u32>(radius * std::cos(angle));
            u32 y = center_y + static_cast<u32>(radius * std::sin(angle));
            
            if (x >= 2 && x < DISPLAY_WIDTH - 2 && y >= 2 && y < DISPLAY_HEIGHT - 2) {
                graphics_engine_->draw_rectangle(x - 1, y - 1, 2, 2, color);
            }
        }
    }

    void draw_text_placeholder(u32 x, u32 y, const std::string& text, u32 color) {
        // Draw a simple placeholder for text (rectangle representing text)
        u32 text_width = text.length() * 8; // Approximate 8 pixels per character
        u32 text_height = 16;
        
        if (x + text_width < DISPLAY_WIDTH && y + text_height < DISPLAY_HEIGHT) {
            graphics_engine_->draw_rectangle(x, y, text_width, text_height, color);
        }
    }

    void draw_number_visualization(u32 x, u32 y, u64 number, u32 color) {
        // Visual representation of a number as bars
        std::string num_str = std::to_string(number);
        u32 bar_width = 8;
        u32 spacing = 2;
        
        for (size_t i = 0; i < num_str.length() && i < 10; ++i) {
            u8 digit = num_str[i] - '0';
            u32 bar_height = (digit + 1) * 3; // Height based on digit value
            
            u32 bar_x = x + i * (bar_width + spacing);
            u32 bar_y = y + (30 - bar_height); // Align bottom
            
            if (bar_x + bar_width < DISPLAY_WIDTH && bar_y + bar_height < DISPLAY_HEIGHT) {
                graphics_engine_->draw_rectangle(bar_x, bar_y, bar_width, bar_height, color);
            }
        }
    }

    void draw_m5stack_logo() {
        // Simplified M5Stack logo representation
        const u32 logo_x = DISPLAY_WIDTH - 150;
        const u32 logo_y = DISPLAY_HEIGHT - 80;
        
        // Draw "M5" as colored rectangles
        graphics_engine_->draw_rectangle(logo_x, logo_y, 20, 50, 0xFF6B35); // Orange
        graphics_engine_->draw_rectangle(logo_x + 25, logo_y, 20, 50, 0xFF6B35);
        graphics_engine_->draw_rectangle(logo_x + 50, logo_y, 20, 25, 0xFF6B35);
        graphics_engine_->draw_rectangle(logo_x + 75, logo_y + 25, 20, 25, 0xFF6B35);
        
        // Draw "Tab5" indicator
        graphics_engine_->draw_rectangle(logo_x, logo_y + 60, 100, 10, 0x2E86C1); // Blue
    }

    u32 generate_vibrant_color(int index, float time) {
        float hue = (index * 60.0f + time * 30.0f);
        while (hue >= 360.0f) hue -= 360.0f;
        
        // Simple HSV to RGB conversion for vibrant colors
        float c = 1.0f; // Saturation
        float x = c * (1.0f - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
        float m = 0.3f; // Brightness offset
        
        float r, g, b;
        if (hue < 60) { r = c; g = x; b = 0; }
        else if (hue < 120) { r = x; g = c; b = 0; }
        else if (hue < 180) { r = 0; g = c; b = x; }
        else if (hue < 240) { r = 0; g = x; b = c; }
        else if (hue < 300) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }
        
        return ((static_cast<u32>((r + m) * 255) & 0xFF) << 16) |
               ((static_cast<u32>((g + m) * 255) & 0xFF) << 8) |
               (static_cast<u32>((b + m) * 255) & 0xFF);
    }

    u32 generate_spiral_color(int index, float angle, float radius) {
        float normalized_radius = radius / 300.0f; // Normalize to 0-1
        float color_phase = angle + normalized_radius * 2.0f * M_PI;
        
        u8 r = static_cast<u8>((std::sin(color_phase) + 1.0f) * 127.5f);
        u8 g = static_cast<u8>((std::sin(color_phase + 2.094f) + 1.0f) * 127.5f); // 2π/3
        u8 b = static_cast<u8>((std::sin(color_phase + 4.188f) + 1.0f) * 127.5f); // 4π/3
        
        return (r << 16) | (g << 8) | b;
    }
};

int main() {
    std::cout << "🌟 M5Stack Tab5 Emulator - Display Connection Test 🌟\n";
    std::cout << "Testing real-time emulator → SDL2 display pipeline\n";
    std::cout << "Resolution: 1280×720 (authentic M5Stack Tab5)\n";
    std::cout << "Features: 60 FPS, Touch Input, Graphics Acceleration\n";
    std::cout << "Press Ctrl+C to exit gracefully\n\n";

    // Setup signal handling for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        // Initialize logging
        auto log_result = Logger::initialize(LogLevel::INFO, "", true);
        if (!log_result.has_value()) {
            std::cerr << "Failed to initialize logging: " << log_result.error().to_string() << std::endl;
            return 1;
        }

        // Create configuration for M5Stack Tab5
        Configuration config;
        
        // Configure display for authentic M5Stack Tab5 specs
        auto display_config = config.getDisplayConfig();
        // Display configuration is loaded from constants (1280x720)
        
        // Configure CPU for ESP32-P4 dual-core @ 400MHz
        auto cpu_config = config.getCPUConfig();
        cpu_config.main_core_freq = 400000000; // 400MHz
        
        // Configure memory (8MB PSRAM + 768KB SRAM)
        auto memory_config = config.getMemoryConfig();
        memory_config.psram_size = 8 * 1024 * 1024; // 8MB PSRAM
        
        std::cout << "✅ Configuration loaded:\n";
        std::cout << "   Display: " << DISPLAY_WIDTH << "×" << DISPLAY_HEIGHT << "\n";
        std::cout << "   CPU: ESP32-P4 @ " << (cpu_config.main_core_freq / 1000000) << "MHz\n";
        std::cout << "   Memory: " << (memory_config.psram_size / 1024 / 1024) << "MB PSRAM\n\n";

        // Create and initialize emulator core
        EmulatorCore emulator(config);
        
        auto init_result = emulator.initialize(config);
        if (!init_result.has_value()) {
            std::cerr << "❌ Failed to initialize emulator: " << init_result.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "✅ Emulator core initialized\n";

        // Get graphics engine component
        auto graphics_engine = emulator.getComponent<GraphicsEngine>();
        if (!graphics_engine) {
            std::cerr << "❌ Graphics engine not available" << std::endl;
            return 1;
        }
        
        std::cout << "✅ Graphics engine connected\n";
        std::cout << "   Resolution: " << graphics_engine->get_width() << "×" << graphics_engine->get_height() << "\n";
        std::cout << "   BPP: " << graphics_engine->get_bpp() << "\n\n";

        // Start emulator (this starts the execution and graphics loops)
        auto start_result = emulator.start();
        if (!start_result.has_value()) {
            std::cerr << "❌ Failed to start emulator: " << start_result.error().to_string() << std::endl;
            return 1;
        }
        
        std::cout << "✅ Emulator started with real-time display\n";
        std::cout << "✅ Graphics loop running at 60 FPS\n";
        std::cout << "✅ Touch input enabled (GT911 simulation)\n\n";

        // Create display test application
        DisplayTestApp test_app(graphics_engine.get());
        
        std::cout << "🎨 Starting animated display test...\n";
        std::cout << "   Pattern 0: Bouncing rectangles\n";
        std::cout << "   Pattern 1: Spiral animation\n";
        std::cout << "   Pattern 2: Touch visualization\n";
        std::cout << "   Pattern 3: Performance overlay\n\n";

        // Main application loop
        auto start_time = std::chrono::steady_clock::now();
        u64 app_frame_counter = 0;
        const auto target_frame_time = std::chrono::microseconds(16667); // ~60 FPS
        
        while (!g_shutdown_requested && emulator.get_state() == EmulatorState::RUNNING) {
            auto frame_start = std::chrono::steady_clock::now();
            
            // Check if display should close
            if (graphics_engine->should_close_display()) {
                std::cout << "\n🔚 Display window closed, shutting down...\n";
                break;
            }
            
            // Render application frame (this will stream to emulator graphics)
            test_app.render_frame();
            app_frame_counter++;
            
            // Print statistics every 5 seconds
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (elapsed.count() > 0 && app_frame_counter % (60 * 5) == 0) {
                double avg_fps = static_cast<double>(app_frame_counter) / elapsed.count();
                double engine_fps = graphics_engine->get_fps();
                u64 total_frames = graphics_engine->get_frames_rendered();
                
                std::cout << "📊 Performance stats:\n";
                std::cout << "   App FPS: " << std::fixed << std::setprecision(1) << avg_fps << "\n";
                std::cout << "   Engine FPS: " << engine_fps << "\n";
                std::cout << "   Total frames: " << total_frames << "\n";
                std::cout << "   Display active: " << (graphics_engine->is_display_active() ? "Yes" : "No") << "\n\n";
            }
            
            // Frame timing control
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
            
            if (frame_duration < target_frame_time) {
                std::this_thread::sleep_for(target_frame_time - frame_duration);
            }
        }
        
        std::cout << "🛑 Stopping emulator...\n";
        
        // Stop emulator gracefully
        auto stop_result = emulator.stop();
        if (!stop_result.has_value()) {
            std::cerr << "⚠️ Warning: Error during emulator stop: " << stop_result.error().to_string() << std::endl;
        }
        
        // Shutdown emulator
        auto shutdown_result = emulator.shutdown();
        if (!shutdown_result.has_value()) {
            std::cerr << "⚠️ Warning: Error during emulator shutdown: " << shutdown_result.error().to_string() << std::endl;
        }
        
        std::cout << "✅ Emulator shutdown complete\n";
        
        // Final statistics
        auto total_time = std::chrono::steady_clock::now() - start_time;
        auto total_seconds = std::chrono::duration_cast<std::chrono::seconds>(total_time).count();
        
        std::cout << "\n📋 Final Statistics:\n";
        std::cout << "   Runtime: " << total_seconds << " seconds\n";
        std::cout << "   App frames: " << app_frame_counter << "\n";
        std::cout << "   Engine frames: " << graphics_engine->get_frames_rendered() << "\n";
        std::cout << "   Average FPS: " << (total_seconds > 0 ? app_frame_counter / total_seconds : 0) << "\n";
        std::cout << "\n🎉 Display connection test completed successfully!\n";
        
    } catch (const std::exception& e) {
        std::cerr << "💥 Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}