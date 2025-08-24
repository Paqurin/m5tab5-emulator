#pragma once

#include "emulator/gui/emulator_gui.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <vector>
#include <functional>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

// Forward declarations
class MenuBar;
class ToolBar;
class StatusBar;
class DockManager;
class EmulatorDisplay;
class ControlPanel;
class HardwareMonitor;
class LogViewer;
class MemoryInspector;

/**
 * @brief Main application window with professional IDE-style layout
 * 
 * Layout Structure:
 * - Menu bar with comprehensive actions
 * - Tool bar with quick access buttons
 * - Central display area showing M5Stack Tab5 screen
 * - Dockable panels for controls, monitoring, and debugging
 * - Status bar with real-time information
 */
class MainWindow {
public:
    struct WindowState {
#ifndef NO_GRAPHICS
        i32 x = SDL_WINDOWPOS_CENTERED;
        i32 y = SDL_WINDOWPOS_CENTERED;
#else
        i32 x = 100;
        i32 y = 100;
#endif
        u32 width = 1400;
        u32 height = 900;
        bool maximized = false;
        bool fullscreen = false;
    };

    explicit MainWindow(EmulatorGUI& parent);
    ~MainWindow();

    Result<void> initialize();
    void update();
    void render();
    void shutdown();

    // Window management
    void show();
    void hide();
    bool is_visible() const { return visible_; }
    
    void set_window_state(const WindowState& state);
    WindowState get_window_state() const;
    
    void toggle_fullscreen();
    void center_window();

    // Component access
    MenuBar* get_menu_bar() const { return menu_bar_.get(); }
    ToolBar* get_tool_bar() const { return tool_bar_.get(); }
    StatusBar* get_status_bar() const { return status_bar_.get(); }
    DockManager* get_dock_manager() const { return dock_manager_.get(); }
    EmulatorDisplay* get_emulator_display() const { return emulator_display_.get(); }

    // Event handling
#ifndef NO_GRAPHICS
    void handle_key_event(const SDL_KeyboardEvent& event);
    void handle_mouse_event(const SDL_MouseButtonEvent& event);
    void handle_window_event(const SDL_WindowEvent& event);
#endif
    
    // UI state
    void set_emulator_running(bool running);
    void set_firmware_loaded(bool loaded);
    void update_status(const std::string& message);

private:
    EmulatorGUI& parent_;
    
    // SDL components
#ifndef NO_GRAPHICS
    SDL_Window* window_;
#else
    void* window_;
#endif
    
    // UI components
    std::unique_ptr<MenuBar> menu_bar_;
    std::unique_ptr<ToolBar> tool_bar_;
    std::unique_ptr<StatusBar> status_bar_;
    std::unique_ptr<DockManager> dock_manager_;
    std::unique_ptr<EmulatorDisplay> emulator_display_;
    
    // Dockable panels
    std::unique_ptr<ControlPanel> control_panel_;
    std::unique_ptr<HardwareMonitor> hardware_monitor_;
    std::unique_ptr<LogViewer> log_viewer_;
    std::unique_ptr<MemoryInspector> memory_inspector_;
    
    // State
    bool visible_;
    bool initialized_;
    WindowState window_state_;
    
    // Layout management
    void setup_layout();
    void create_dock_panels();
    void setup_shortcuts();
    
    // Event handlers
    void handle_menu_action(const std::string& action);
    void handle_toolbar_action(const std::string& action);
    
    Result<void> create_window();
    void cleanup_window();
};

/**
 * @brief Professional menu bar with comprehensive application actions
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
    };

    explicit MenuBar(MainWindow& parent);
    ~MenuBar();

    void render();
    void handle_action(const std::string& action);
    
    void set_menu_enabled(const std::string& menu, bool enabled);
    void set_item_enabled(const std::string& action, bool enabled);

private:
    MainWindow& parent_;
    std::vector<MenuItem> menus_;
    
    void setup_file_menu();
    void setup_edit_menu();
    void setup_emulator_menu();
    void setup_debug_menu();
    void setup_view_menu();
    void setup_tools_menu();
    void setup_help_menu();
    
    void render_menu_item(const MenuItem& item);
    void execute_action(const std::string& action);
};

/**
 * @brief Quick access toolbar with essential controls
 */
class ToolBar {
public:
    struct ToolButton {
        std::string icon;
        std::string tooltip;
        std::string action;
        bool enabled = true;
        bool toggle = false;
        bool toggled = false;
        bool separator_after = false;
    };

    explicit ToolBar(MainWindow& parent);
    ~ToolBar();

    void render();
    void handle_action(const std::string& action);
    
    void set_button_enabled(const std::string& action, bool enabled);
    void set_button_toggled(const std::string& action, bool toggled);

private:
    MainWindow& parent_;
    std::vector<ToolButton> buttons_;
    
    void setup_buttons();
    void render_button(const ToolButton& button);
};

/**
 * @brief Status bar showing real-time emulator information
 */
class StatusBar {
public:
    struct StatusItem {
        std::string text;
        u32 color = 0x000000;
        bool permanent = false;
    };

    explicit StatusBar(MainWindow& parent);
    ~StatusBar();

    void render();
    void update();
    
    void set_status(const std::string& text, u32 color = 0x000000);
    void set_permanent_status(const std::string& key, const std::string& text, u32 color = 0x000000);
    void clear_status();
    
    void show_progress(const std::string& text, float progress);
    void hide_progress();

private:
    MainWindow& parent_;
    
    // Status items
    StatusItem temporary_status_;
    std::map<std::string, StatusItem> permanent_items_;
    
    // Progress indication
    bool show_progress_bar_;
    std::string progress_text_;
    float progress_value_;
    
    void render_status_item(const StatusItem& item);
    void update_emulator_stats();
};

/**
 * @brief Central display area showing M5Stack Tab5 screen with touch support
 */
class EmulatorDisplay {
public:
    explicit EmulatorDisplay(MainWindow& parent);
    ~EmulatorDisplay();

    Result<void> initialize();
    void render();
    void shutdown();
    
    // Display control
    void set_scale_factor(float scale);
    float get_scale_factor() const { return scale_factor_; }
    
    void set_rotation(u32 degrees); // 0, 90, 180, 270
    u32 get_rotation() const { return rotation_; }
    
    // Touch handling
#ifndef NO_GRAPHICS
    void handle_mouse_down(const SDL_MouseButtonEvent& event);
    void handle_mouse_up(const SDL_MouseButtonEvent& event);
    void handle_mouse_motion(const SDL_MouseMotionEvent& event);
#endif
    
    // Screenshot functionality
    Result<void> save_screenshot(const std::string& path);

private:
    MainWindow& parent_;
    
    // Display properties
    float scale_factor_;
    u32 rotation_;
    bool touch_active_;
    
    // Touch state
    struct TouchPoint {
        float x, y;
        bool active;
        u32 timestamp;
    };
    TouchPoint touch_state_;
    
    // Rendering
    void render_display_border();
    void render_emulator_screen();
    void convert_mouse_to_display_coords(i32 mouse_x, i32 mouse_y, float& display_x, float& display_y);
    void send_touch_event(float x, float y, bool pressed);
};

} // namespace m5tab5::emulator::gui