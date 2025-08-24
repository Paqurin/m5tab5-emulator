#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator {
    class SdlRenderer;
}

namespace m5tab5::emulator::gui {

// Forward declarations
class PersonalityManager;

/**
 * @brief Professional SDL2-based menu system for M5Stack Tab5 emulator
 * 
 * Features:
 * - Professional menu bar with File/Edit/Emulator/Debug/View/Tools/Help
 * - Keyboard shortcuts (Ctrl+O, F5, etc.)
 * - SDL2-compatible rendering using rectangles and text
 * - Integration with PersonalityManager for delightful responses
 * - M5Stack design aesthetic
 * - Hover effects and visual feedback
 */
class MenuBar {
public:
    struct MenuItem {
        std::string text;
        std::string shortcut;
        std::string action;
        bool enabled = true;
        bool separator_after = false;
        std::vector<MenuItem> submenu;
        
        MenuItem() = default;
        MenuItem(const std::string& t, const std::string& s, const std::string& a, bool e = true, bool sep = false) 
            : text(t), shortcut(s), action(a), enabled(e), separator_after(sep) {}
    };

    struct MenuBarState {
        bool menu_open = false;
        size_t active_menu = 0;
        size_t active_item = 0;
        bool mouse_over_menu = false;
        std::chrono::steady_clock::time_point hover_start;
        std::string status_message;
    };

    // Menu action callback
    using ActionCallback = std::function<void(const std::string& action, const MenuItem& item)>;

    explicit MenuBar(m5tab5::emulator::SdlRenderer* renderer, PersonalityManager* personality = nullptr);
    ~MenuBar();

    Result<void> initialize();
    void update();
    void render();
    void shutdown();

    // Event handling
#ifndef NO_GRAPHICS
    bool handle_key_event(const SDL_KeyboardEvent& event);
    bool handle_mouse_event(const SDL_MouseButtonEvent& event);
    bool handle_mouse_motion(const SDL_MouseMotionEvent& event);
#endif

    // Menu configuration
    void set_action_callback(ActionCallback callback);
    void set_menu_enabled(const std::string& menu_name, bool enabled);
    void set_item_enabled(const std::string& action, bool enabled);
    
    // State access
    bool is_menu_open() const { return state_.menu_open; }
    std::string get_status_message() const { return state_.status_message; }
    
    // Visual configuration
    void set_theme_colors(u32 background, u32 text, u32 hover, u32 disabled);

private:
    m5tab5::emulator::SdlRenderer* renderer_;
    PersonalityManager* personality_;
    ActionCallback action_callback_;
    
    // Menu structure
    std::vector<MenuItem> menus_;
    std::map<std::string, bool> menu_enabled_;
    std::map<std::string, bool> item_enabled_;
    
    // Visual state
    MenuBarState state_;
    
    // Theme colors (M5Stack aesthetic)
    u32 background_color_ = 0x1a1a1a;  // Dark background
    u32 text_color_ = 0xffffff;        // White text
    u32 hover_color_ = 0x0066cc;       // M5Stack blue
    u32 disabled_color_ = 0x666666;    // Gray for disabled
    u32 separator_color_ = 0x333333;   // Dark gray separators
    
    // Layout constants
    static constexpr u32 MENU_HEIGHT = 30;
    static constexpr u32 MENU_PADDING = 15;
    static constexpr u32 ITEM_HEIGHT = 25;
    static constexpr u32 SUBMENU_WIDTH = 200;
    static constexpr u32 SEPARATOR_HEIGHT = 8;
    
    // Menu setup
    void setup_file_menu();
    void setup_edit_menu();
    void setup_emulator_menu();
    void setup_debug_menu();
    void setup_view_menu();
    void setup_tools_menu();
    void setup_help_menu();
    
    // Rendering
    void render_menu_bar();
    void render_dropdown_menu(size_t menu_index);
    void render_menu_item(const MenuItem& item, u32 x, u32 y, u32 width, bool highlighted = false);
    void render_separator(u32 x, u32 y, u32 width);
    
    // Interaction
    size_t get_menu_at_position(i32 x, i32 y);
    size_t get_item_at_position(size_t menu_index, i32 x, i32 y);
    void execute_action(const std::string& action, const MenuItem& item);
    void close_menu();
    void show_personality_feedback(const std::string& action);
    
    // Keyboard shortcuts
    void register_shortcuts();
    bool handle_shortcut(const SDL_KeyboardEvent& event);
    std::map<u32, std::string> shortcuts_; // SDL_Keycode -> action
    
    // Helper functions
    u32 calculate_text_width(const std::string& text);
    void draw_text_centered(const std::string& text, u32 x, u32 y, u32 width, u32 color);
    bool is_point_in_rect(i32 px, i32 py, i32 x, i32 y, u32 w, u32 h);
};

/**
 * @brief Context menu system for right-click actions
 */
class ContextMenu {
public:
    struct ContextItem {
        std::string text;
        std::string action;
        bool enabled = true;
        bool separator_after = false;
        
        ContextItem() = default;
        ContextItem(const std::string& t, const std::string& a, bool e = true, bool sep = false)
            : text(t), action(a), enabled(e), separator_after(sep) {}
    };

    explicit ContextMenu(m5tab5::emulator::SdlRenderer* renderer);
    ~ContextMenu();

    void show(i32 x, i32 y, const std::vector<ContextItem>& items);
    void hide();
    void render();
    
#ifndef NO_GRAPHICS
    bool handle_mouse_event(const SDL_MouseButtonEvent& event);
#endif

private:
    m5tab5::emulator::SdlRenderer* renderer_;
    std::vector<ContextItem> items_;
    i32 position_x_, position_y_;
    bool visible_;
    size_t highlighted_item_;
};

} // namespace m5tab5::emulator::gui