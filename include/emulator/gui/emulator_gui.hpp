#pragma once

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/types.hpp"

#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <atomic>
#include <mutex>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

// Forward declarations
class MainWindow;
class GUIManager;
class StyleManager;

/**
 * @brief Modern GUI application for M5Stack Tab5 Emulator
 * 
 * Features:
 * - Professional development environment interface
 * - Real-time hardware monitoring and control
 * - Integrated firmware management and debugging
 * - Responsive design with dockable panels
 * - Cross-platform compatibility
 */
class EmulatorGUI {
public:
    enum class Theme {
        Light,
        Dark,
        System
    };

    struct GUIConfig {
        u32 window_width = 1400;
        u32 window_height = 900;
        bool maximized = false;
        Theme theme = Theme::System;
        std::string last_firmware_path = "";
        std::string last_config_path = "";
        bool auto_start_emulator = false;
        bool show_welcome_dialog = true;
        float ui_scale = 1.0f;
    };

    EmulatorGUI();
    ~EmulatorGUI();

    // Application lifecycle
    Result<void> initialize(int argc, char* argv[]);
    Result<void> run();
    void shutdown();

    // Configuration management
    void load_gui_config();
    void save_gui_config() const;
    const GUIConfig& get_gui_config() const { return gui_config_; }
    void set_gui_config(const GUIConfig& config);

    // Emulator integration
    EmulatorCore* get_emulator_core() const { return emulator_core_.get(); }
    bool is_emulator_running() const;
    Result<void> load_firmware(const std::string& elf_path);
    Result<void> start_emulator();
    Result<void> stop_emulator();
    Result<void> reset_emulator();

    // Event handling
    void register_emulator_callback(const std::string& event, std::function<void()> callback);
    void emit_emulator_event(const std::string& event);
    
    // Delightful experience features
    void trigger_celebration(const std::string& achievement);
    void show_easter_egg(const std::string& trigger);
    void pulse_element(const std::string& element_id);
    void shake_on_error(const std::string& message);
    
    // Animation system
    struct AnimationState {
        std::chrono::steady_clock::time_point start_time;
        float duration = 1.0f;
        bool active = false;
        std::string type;
        std::map<std::string, float> properties;
    };
    
    std::map<std::string, AnimationState> active_animations_;
    void update_animations();
    void start_animation(const std::string& id, const std::string& type, float duration, std::map<std::string, float> properties = {});
    
    // Achievement system for developer delight
    enum class Achievement {
        FIRST_FIRMWARE_LOAD,
        THOUSAND_CYCLES,
        GPIO_MASTER,
        SPEED_DEMON,
        SERIAL_MONITOR_PRO,
        EASTER_EGG_HUNTER
    };
    
    void unlock_achievement(Achievement achievement);
    bool is_achievement_unlocked(Achievement achievement) const;
    std::set<Achievement> unlocked_achievements_;

private:
    // Core components
    std::unique_ptr<EmulatorCore> emulator_core_;
    std::unique_ptr<MainWindow> main_window_;
    std::unique_ptr<GUIManager> gui_manager_;
    std::unique_ptr<StyleManager> style_manager_;
    
    // Delightful experience components
    std::unique_ptr<class PersonalityManager> personality_manager_;
    std::unique_ptr<class ParticleSystem> particle_system_;
    std::unique_ptr<class SoundManager> sound_manager_;  // Optional audio feedback
    
    // Startup animation state
    bool startup_complete_ = false;
    std::chrono::steady_clock::time_point startup_time_;
    
    // Konami code detection
#ifndef NO_GRAPHICS
    std::vector<int> konami_sequence_ = {SDLK_UP, SDLK_UP, SDLK_DOWN, SDLK_DOWN, SDLK_LEFT, SDLK_RIGHT, SDLK_LEFT, SDLK_RIGHT, SDLK_b, SDLK_a};
    std::vector<int> input_sequence_;
    void check_konami_code(int keycode);
#else
    std::vector<int> konami_sequence_;
    std::vector<int> input_sequence_;
    void check_konami_code(int keycode);
#endif
    
    // Loading states for delightful feedback
    struct LoadingState {
        bool active = false;
        std::string message = "";
        float progress = 0.0f;
        std::chrono::steady_clock::time_point start_time;
        std::vector<std::string> messages = {
            "Initializing ESP32-P4 cores...",
            "Calibrating GPIO pins...",
            "Syncing I2C peripherals...",
            "Warming up the oscillators...",
            "Teaching registers to count...",
            "Convincing electrons to cooperate...",
            "Loading firmware with care...",
            "Almost there, just debugging the bugs..."
        };
    } loading_state_;

    // Configuration
    Configuration emulator_config_;
    GUIConfig gui_config_;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
    
    // Event system
    std::mutex callback_mutex_;
    std::map<std::string, std::vector<std::function<void()>>> event_callbacks_;

#ifndef NO_GRAPHICS
    SDL_Event current_event_;
#endif

    // Initialization helpers
    Result<void> init_sdl();
    Result<void> init_emulator_core();
    Result<void> init_gui_components();
    Result<void> setup_signal_handlers();
    
    // Main application loop
    void event_loop();
#ifndef NO_GRAPHICS
    void handle_sdl_event(const SDL_Event& event);
#endif
    void update_gui();
    void render_gui();

    // Configuration helpers
    std::string get_config_path() const;
    void apply_theme(Theme theme);

    // Error handling
    void handle_critical_error(const std::string& error);
    void show_error_dialog(const std::string& title, const std::string& message);
};

/**
 * @brief GUI component manager for organizing UI elements
 */
class GUIManager {
public:
    explicit GUIManager(EmulatorGUI& parent);
    ~GUIManager();

    Result<void> initialize();
    void update();
    void render();
    void shutdown();

    // Component access
    template<typename T>
    std::shared_ptr<T> get_component(const std::string& name) const;
    
    template<typename T>
    void register_component(const std::string& name, std::shared_ptr<T> component);

    // Layout management
    void save_layout(const std::string& name);
    void load_layout(const std::string& name);
    void reset_layout();

private:
    EmulatorGUI& parent_;
    std::map<std::string, std::shared_ptr<void>> components_;
    
    Result<void> setup_default_layout();
};

/**
 * @brief Modern styling system with theme support
 */
class StyleManager {
public:
    struct ColorPalette {
        // M5Stack Brand Colors with Delightful Variations
        u32 primary = 0xFF6600;           // M5Stack Orange
        u32 primary_dark = 0xE55100;      // Darker Orange
        u32 secondary = 0x2196F3;         // Complementary Blue
        u32 accent = 0xFF9800;            // Vibrant Accent
        u32 background = 0xF8F9FA;        // Warm Light Gray
        u32 surface = 0xFFFFFF;           // Pure White
        u32 surface_elevated = 0xFEFEFE;  // Slightly Elevated
        u32 error = 0xFF5722;             // Friendly Red
        u32 success = 0x4CAF50;           // Success Green
        u32 warning = 0xFFC107;           // Cheerful Yellow
        u32 text_primary = 0x212121;      // Primary Text
        u32 text_secondary = 0x757575;    // Secondary Text
        u32 text_hint = 0x9E9E9E;         // Hint Text
        u32 border = 0xE1E5E9;            // Soft Border
        u32 divider = 0xECEFF1;           // Gentle Divider
        
        // Delightful Animation Colors
        u32 celebration = 0xFF4081;       // Pink Celebration
        u32 pulse_start = 0xFFE0B2;       // Warm Pulse Start
        u32 pulse_end = 0xFF6600;         // M5Stack Orange Pulse End
        u32 loading_gradient_1 = 0xFF9800; // Loading Animation Color 1
        u32 loading_gradient_2 = 0xFFB74D; // Loading Animation Color 2
    };

    struct Metrics {
        u32 padding_xs = 4;
        u32 padding_sm = 8;
        u32 padding_md = 16;
        u32 padding_lg = 24;
        u32 padding_xl = 32;
        u32 border_radius = 6;
        u32 border_width = 1;
        float font_size_sm = 12.0f;
        float font_size_md = 14.0f;
        float font_size_lg = 16.0f;
        float font_size_xl = 18.0f;
    };

    explicit StyleManager(EmulatorGUI::Theme theme = EmulatorGUI::Theme::System);
    
    void set_theme(EmulatorGUI::Theme theme);
    EmulatorGUI::Theme get_theme() const { return current_theme_; }
    
    const ColorPalette& get_colors() const { return colors_; }
    const Metrics& get_metrics() const { return metrics_; }
    
    void set_scale_factor(float scale);
    float get_scale_factor() const { return scale_factor_; }

private:
    EmulatorGUI::Theme current_theme_;
    ColorPalette colors_;
    Metrics metrics_;
    float scale_factor_ = 1.0f;
    
    void apply_light_theme();
    void apply_dark_theme();
    void apply_system_theme();
    void scale_metrics();
};

} // namespace m5tab5::emulator::gui