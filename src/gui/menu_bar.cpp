#include "emulator/gui/menu_bar.hpp"
#include "emulator/gui/personality_manager.hpp"
#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cstring>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

MenuBar::MenuBar(m5tab5::emulator::SdlRenderer* renderer, PersonalityManager* personality)
    : renderer_(renderer), personality_(personality) {
    LOG_DEBUG("Creating MenuBar with renderer: {}, personality: {}", 
              (void*)renderer, (void*)personality);
}

MenuBar::~MenuBar() {
    shutdown();
}

Result<void> MenuBar::initialize() {
    if (!renderer_) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Renderer not provided to MenuBar"));
    }

    LOG_INFO("🎮 Initializing professional menu bar with M5Stack aesthetic");

    // Set up all menus
    setup_file_menu();
    setup_edit_menu(); 
    setup_emulator_menu();
    setup_debug_menu();
    setup_view_menu();
    setup_tools_menu();
    setup_help_menu();

    // Register keyboard shortcuts
    register_shortcuts();

    LOG_INFO("✨ Menu bar initialized with {} menus and {} shortcuts", 
             menus_.size(), shortcuts_.size());

    if (personality_) {
        personality_->unlock_achievement(PersonalityManager::Achievement::FIRST_BOOT);
    }

    return {};
}

void MenuBar::update() {
    // Update hover effects and animations
    if (state_.mouse_over_menu) {
        auto now = std::chrono::steady_clock::now();
        auto hover_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - state_.hover_start).count();
        
        // Show tooltip after 1 second of hover
        if (hover_duration > 1000 && state_.active_menu < menus_.size()) {
            const auto& menu = menus_[state_.active_menu];
            state_.status_message = "Menu: " + menu.text;
        }
    } else {
        state_.status_message.clear();
    }
}

void MenuBar::render() {
    if (!renderer_) return;

    // Render menu bar background
    render_menu_bar();

    // Render active dropdown menu
    if (state_.menu_open && state_.active_menu < menus_.size()) {
        render_dropdown_menu(state_.active_menu);
    }
}

void MenuBar::shutdown() {
    LOG_DEBUG("Shutting down MenuBar");
    menus_.clear();
    shortcuts_.clear();
    state_ = MenuBarState{};
}

bool MenuBar::handle_key_event(const SDL_KeyboardEvent& event) {
#ifndef NO_GRAPHICS
    if (event.type == SDL_KEYDOWN) {
        // Handle keyboard shortcuts
        if (handle_shortcut(event)) {
            return true;
        }
        
        // Handle menu navigation
        if (state_.menu_open) {
            switch (event.keysym.sym) {
                case SDLK_ESCAPE:
                    close_menu();
                    return true;
                case SDLK_LEFT:
                    if (state_.active_menu > 0) {
                        state_.active_menu--;
                    }
                    return true;
                case SDLK_RIGHT:
                    if (state_.active_menu < menus_.size() - 1) {
                        state_.active_menu++;
                    }
                    return true;
                case SDLK_RETURN:
                    // Execute highlighted item
                    if (state_.active_menu < menus_.size() && 
                        state_.active_item < menus_[state_.active_menu].submenu.size()) {
                        const auto& item = menus_[state_.active_menu].submenu[state_.active_item];
                        if (item.enabled) {
                            execute_action(item.action, item);
                        }
                        close_menu();
                    }
                    return true;
            }
        }
    }
#endif
    return false; // Event not handled
}

bool MenuBar::handle_mouse_event(const SDL_MouseButtonEvent& event) {
#ifndef NO_GRAPHICS
    if (event.type == SDL_MOUSEBUTTONDOWN) {
        if (event.y <= static_cast<i32>(MENU_HEIGHT)) {
            // Click in menu bar area
            size_t menu_idx = get_menu_at_position(event.x, event.y);
            if (menu_idx < menus_.size()) {
                if (state_.menu_open && state_.active_menu == menu_idx) {
                    // Close if same menu clicked
                    close_menu();
                } else {
                    // Open menu
                    state_.menu_open = true;
                    state_.active_menu = menu_idx;
                    state_.active_item = 0;
                    
                    if (personality_) {
                        show_personality_feedback("menu_open");
                    }
                    
                    LOG_DEBUG("Opened menu: {}", menus_[menu_idx].text);
                }
                return true;
            }
        } else if (state_.menu_open) {
            // Click in dropdown area
            size_t item_idx = get_item_at_position(state_.active_menu, event.x, event.y);
            if (item_idx < menus_[state_.active_menu].submenu.size()) {
                const auto& item = menus_[state_.active_menu].submenu[item_idx];
                if (item.enabled && !item.action.empty()) {
                    execute_action(item.action, item);
                    close_menu();
                    return true;
                }
            } else {
                // Click outside menu - close it
                close_menu();
            }
        }
    }
#endif
    return false;
}

bool MenuBar::handle_mouse_motion(const SDL_MouseMotionEvent& event) {
#ifndef NO_GRAPHICS
    bool was_over_menu = state_.mouse_over_menu;
    state_.mouse_over_menu = (event.y <= static_cast<i32>(MENU_HEIGHT));
    
    if (state_.mouse_over_menu && !was_over_menu) {
        state_.hover_start = std::chrono::steady_clock::now();
    }
    
    if (state_.menu_open) {
        // Update active item based on mouse position
        size_t item_idx = get_item_at_position(state_.active_menu, event.x, event.y);
        if (item_idx < menus_[state_.active_menu].submenu.size()) {
            state_.active_item = item_idx;
        }
    }
#endif
    return state_.menu_open; // Return true if we're consuming the event
}

void MenuBar::set_action_callback(ActionCallback callback) {
    action_callback_ = std::move(callback);
}

void MenuBar::set_menu_enabled(const std::string& menu_name, bool enabled) {
    menu_enabled_[menu_name] = enabled;
}

void MenuBar::set_item_enabled(const std::string& action, bool enabled) {
    item_enabled_[action] = enabled;
}

void MenuBar::set_theme_colors(u32 background, u32 text, u32 hover, u32 disabled) {
    background_color_ = background;
    text_color_ = text;
    hover_color_ = hover;
    disabled_color_ = disabled;
}

void MenuBar::setup_file_menu() {
    MenuItem file_menu("File", "", "");
    
    file_menu.submenu.emplace_back("Open Firmware...", "Ctrl+O", "file_open_firmware");
    file_menu.submenu.emplace_back("Recent Files", "", "file_recent", true, true);
    file_menu.submenu.emplace_back("Save Configuration", "Ctrl+S", "file_save_config");
    file_menu.submenu.emplace_back("Load Configuration", "Ctrl+L", "file_load_config", true, true);
    file_menu.submenu.emplace_back("Exit", "Ctrl+Q", "file_exit");
    
    menus_.push_back(std::move(file_menu));
}

void MenuBar::setup_edit_menu() {
    MenuItem edit_menu("Edit", "", "");
    
    edit_menu.submenu.emplace_back("Undo", "Ctrl+Z", "edit_undo");
    edit_menu.submenu.emplace_back("Redo", "Ctrl+Y", "edit_redo", true, true);
    edit_menu.submenu.emplace_back("Copy", "Ctrl+C", "edit_copy");
    edit_menu.submenu.emplace_back("Paste", "Ctrl+V", "edit_paste", true, true);
    edit_menu.submenu.emplace_back("Preferences", "Ctrl+,", "edit_preferences");
    
    menus_.push_back(std::move(edit_menu));
}

void MenuBar::setup_emulator_menu() {
    MenuItem emulator_menu("Emulator", "", "");
    
    emulator_menu.submenu.emplace_back("Start", "F5", "emulator_start");
    emulator_menu.submenu.emplace_back("Stop", "Shift+F5", "emulator_stop");
    emulator_menu.submenu.emplace_back("Pause", "F6", "emulator_pause");
    emulator_menu.submenu.emplace_back("Resume", "F7", "emulator_resume", true, true);
    emulator_menu.submenu.emplace_back("Reset", "Ctrl+R", "emulator_reset", true, true);
    emulator_menu.submenu.emplace_back("Power Cycle", "Ctrl+Shift+R", "emulator_power_cycle");
    
    menus_.push_back(std::move(emulator_menu));
}

void MenuBar::setup_debug_menu() {
    MenuItem debug_menu("Debug", "", "");
    
    debug_menu.submenu.emplace_back("Start Debugging", "F9", "debug_start");
    debug_menu.submenu.emplace_back("Stop Debugging", "Shift+F9", "debug_stop", true, true);
    debug_menu.submenu.emplace_back("Step Over", "F10", "debug_step_over");
    debug_menu.submenu.emplace_back("Step Into", "F11", "debug_step_into");
    debug_menu.submenu.emplace_back("Step Out", "Shift+F11", "debug_step_out", true, true);
    debug_menu.submenu.emplace_back("Toggle Breakpoint", "F2", "debug_toggle_breakpoint");
    debug_menu.submenu.emplace_back("Clear All Breakpoints", "", "debug_clear_breakpoints");
    
    menus_.push_back(std::move(debug_menu));
}

void MenuBar::setup_view_menu() {
    MenuItem view_menu("View", "", "");
    
    view_menu.submenu.emplace_back("Control Panel", "F4", "view_control_panel");
    view_menu.submenu.emplace_back("GPIO Viewer", "F3", "view_gpio_viewer");
    view_menu.submenu.emplace_back("Memory Inspector", "Ctrl+M", "view_memory_inspector");
    view_menu.submenu.emplace_back("Log Viewer", "Ctrl+G", "view_log_viewer", true, true);
    view_menu.submenu.emplace_back("Hardware Monitor", "Ctrl+H", "view_hardware_monitor");
    view_menu.submenu.emplace_back("Performance Stats", "Ctrl+P", "view_performance", true, true);
    view_menu.submenu.emplace_back("Fullscreen", "F11", "view_fullscreen");
    
    menus_.push_back(std::move(view_menu));
}

void MenuBar::setup_tools_menu() {
    MenuItem tools_menu("Tools", "", "");
    
    tools_menu.submenu.emplace_back("Firmware Manager", "F2", "tools_firmware_manager");
    tools_menu.submenu.emplace_back("Pin Configurator", "", "tools_pin_config");
    tools_menu.submenu.emplace_back("I2C Scanner", "", "tools_i2c_scanner", true, true);
    tools_menu.submenu.emplace_back("UART Monitor", "", "tools_uart_monitor");
    tools_menu.submenu.emplace_back("Logic Analyzer", "", "tools_logic_analyzer", true, true);
    tools_menu.submenu.emplace_back("Export Log", "", "tools_export_log");
    tools_menu.submenu.emplace_back("Take Screenshot", "F12", "tools_screenshot");
    
    menus_.push_back(std::move(tools_menu));
}

void MenuBar::setup_help_menu() {
    MenuItem help_menu("Help", "", "");
    
    help_menu.submenu.emplace_back("User Guide", "F1", "help_user_guide");
    help_menu.submenu.emplace_back("Keyboard Shortcuts", "Ctrl+?", "help_shortcuts");
    help_menu.submenu.emplace_back("ESP32-P4 Reference", "", "help_esp32_reference", true, true);
    help_menu.submenu.emplace_back("M5Stack Documentation", "", "help_m5stack_docs");
    help_menu.submenu.emplace_back("Report Issue", "", "help_report_issue", true, true);
    help_menu.submenu.emplace_back("About", "", "help_about");
    
    menus_.push_back(std::move(help_menu));
}

void MenuBar::render_menu_bar() {
    if (!renderer_) return;

    // Draw menu bar background
    auto result = renderer_->draw_rect(0, 0, renderer_->get_width(), MENU_HEIGHT, background_color_);
    if (!result) {
        LOG_WARN("Failed to draw menu bar background: {}", result.error().to_string());
        return;
    }

    // Draw menu items
    u32 x_offset = MENU_PADDING;
    for (size_t i = 0; i < menus_.size(); ++i) {
        const auto& menu = menus_[i];
        
        // Check if menu is enabled
        bool enabled = true;
        if (menu_enabled_.find(menu.text) != menu_enabled_.end()) {
            enabled = menu_enabled_[menu.text];
        }
        
        // Calculate menu width
        u32 menu_width = calculate_text_width(menu.text) + (MENU_PADDING * 2);
        
        // Highlight if active
        u32 bg_color = background_color_;
        u32 text_color = enabled ? text_color_ : disabled_color_;
        
        if (state_.menu_open && state_.active_menu == i) {
            bg_color = hover_color_;
            text_color = 0xffffff;
        } else if (state_.mouse_over_menu && get_menu_at_position(x_offset + menu_width/2, MENU_HEIGHT/2) == i) {
            bg_color = hover_color_ & 0x7f7f7f7f; // Semi-transparent hover
        }
        
        // Draw menu background if highlighted
        if (bg_color != background_color_) {
            renderer_->draw_rect(x_offset - MENU_PADDING/2, 0, menu_width, MENU_HEIGHT, bg_color);
        }
        
        // Draw menu text
        draw_text_centered(menu.text, x_offset, 0, menu_width - MENU_PADDING, text_color);
        
        x_offset += menu_width;
    }
    
    // Draw bottom border
    renderer_->draw_rect(0, MENU_HEIGHT - 1, renderer_->get_width(), 1, separator_color_);
}

void MenuBar::render_dropdown_menu(size_t menu_index) {
    if (menu_index >= menus_.size() || !renderer_) return;
    
    const auto& menu = menus_[menu_index];
    if (menu.submenu.empty()) return;
    
    // Calculate dropdown position
    u32 x_offset = MENU_PADDING;
    for (size_t i = 0; i < menu_index; ++i) {
        x_offset += calculate_text_width(menus_[i].text) + (MENU_PADDING * 2);
    }
    
    u32 dropdown_y = MENU_HEIGHT;
    u32 dropdown_height = 0;
    
    // Calculate required height
    for (const auto& item : menu.submenu) {
        if (item.separator_after) {
            dropdown_height += ITEM_HEIGHT + SEPARATOR_HEIGHT;
        } else {
            dropdown_height += ITEM_HEIGHT;
        }
    }
    
    // Draw dropdown background with shadow effect
    renderer_->draw_rect(x_offset + 2, dropdown_y + 2, SUBMENU_WIDTH, dropdown_height, 0x000000); // Shadow
    renderer_->draw_rect(x_offset, dropdown_y, SUBMENU_WIDTH, dropdown_height, background_color_);
    renderer_->draw_rect(x_offset, dropdown_y, SUBMENU_WIDTH, 1, separator_color_); // Top border
    
    // Render menu items
    u32 item_y = dropdown_y;
    for (size_t i = 0; i < menu.submenu.size(); ++i) {
        const auto& item = menu.submenu[i];
        bool highlighted = (state_.active_item == i);
        
        render_menu_item(item, x_offset, item_y, SUBMENU_WIDTH, highlighted);
        
        item_y += ITEM_HEIGHT;
        if (item.separator_after) {
            render_separator(x_offset, item_y, SUBMENU_WIDTH);
            item_y += SEPARATOR_HEIGHT;
        }
    }
}

void MenuBar::render_menu_item(const MenuItem& item, u32 x, u32 y, u32 width, bool highlighted) {
    if (!renderer_) return;
    
    // Check if item is enabled
    bool enabled = item.enabled;
    if (item_enabled_.find(item.action) != item_enabled_.end()) {
        enabled = item_enabled_[item.action];
    }
    
    // Draw background if highlighted
    if (highlighted && enabled) {
        renderer_->draw_rect(x + 2, y + 1, width - 4, ITEM_HEIGHT - 2, hover_color_);
    }
    
    // Draw text
    u32 text_color = enabled ? text_color_ : disabled_color_;
    if (highlighted && enabled) {
        text_color = 0xffffff;
    }
    
    renderer_->draw_text(x + 8, y + (ITEM_HEIGHT - 12) / 2, item.text, text_color);
    
    // Draw shortcut text (right-aligned)
    if (!item.shortcut.empty()) {
        u32 shortcut_width = calculate_text_width(item.shortcut);
        renderer_->draw_text(x + width - shortcut_width - 8, y + (ITEM_HEIGHT - 12) / 2, 
                           item.shortcut, text_color & 0xcccccc);
    }
}

void MenuBar::render_separator(u32 x, u32 y, u32 width) {
    if (!renderer_) return;
    
    renderer_->draw_rect(x + 8, y + SEPARATOR_HEIGHT/2 - 1, width - 16, 1, separator_color_);
}

size_t MenuBar::get_menu_at_position(i32 x, i32 y) {
    if (y > static_cast<i32>(MENU_HEIGHT)) return menus_.size();
    
    u32 x_offset = MENU_PADDING;
    for (size_t i = 0; i < menus_.size(); ++i) {
        u32 menu_width = calculate_text_width(menus_[i].text) + (MENU_PADDING * 2);
        if (x >= static_cast<i32>(x_offset) && x < static_cast<i32>(x_offset + menu_width)) {
            return i;
        }
        x_offset += menu_width;
    }
    
    return menus_.size(); // Not found
}

size_t MenuBar::get_item_at_position(size_t menu_index, i32 x, i32 y) {
    if (menu_index >= menus_.size()) return SIZE_MAX;
    
    // Calculate menu position
    u32 x_offset = MENU_PADDING;
    for (size_t i = 0; i < menu_index; ++i) {
        x_offset += calculate_text_width(menus_[i].text) + (MENU_PADDING * 2);
    }
    
    // Check if within dropdown bounds
    if (x < static_cast<i32>(x_offset) || x >= static_cast<i32>(x_offset + SUBMENU_WIDTH) ||
        y <= static_cast<i32>(MENU_HEIGHT)) {
        return SIZE_MAX;
    }
    
    // Find item at position
    u32 item_y = MENU_HEIGHT;
    for (size_t i = 0; i < menus_[menu_index].submenu.size(); ++i) {
        if (y >= static_cast<i32>(item_y) && y < static_cast<i32>(item_y + ITEM_HEIGHT)) {
            return i;
        }
        
        item_y += ITEM_HEIGHT;
        if (menus_[menu_index].submenu[i].separator_after) {
            item_y += SEPARATOR_HEIGHT;
        }
    }
    
    return SIZE_MAX;
}

void MenuBar::execute_action(const std::string& action, const MenuItem& item) {
    LOG_INFO("🎯 Menu action executed: {} ({})", action, item.text);
    
    if (personality_) {
        show_personality_feedback(action);
    }
    
    if (action_callback_) {
        action_callback_(action, item);
    }
}

void MenuBar::close_menu() {
    state_.menu_open = false;
    state_.active_menu = 0;
    state_.active_item = 0;
}

void MenuBar::show_personality_feedback(const std::string& action) {
    if (!personality_) return;
    
    // Show delightful responses for certain actions
    if (action == "file_open_firmware") {
        LOG_INFO("✨ {}", "Ready to load some amazing firmware!");
    } else if (action == "emulator_start") {
        LOG_INFO("🚀 {}", personality_->get_success_celebration());
    } else if (action == "help_about") {
        LOG_INFO("💡 {}", "M5Stack Tab5 Emulator - Making ESP32-P4 development delightful!");
    } else if (action == "menu_open") {
        LOG_DEBUG("🎮 Menu opened with personality!");
    }
}

void MenuBar::register_shortcuts() {
#ifndef NO_GRAPHICS
    // File menu shortcuts
    shortcuts_[SDLK_o | (KMOD_CTRL << 16)] = "file_open_firmware";
    shortcuts_[SDLK_s | (KMOD_CTRL << 16)] = "file_save_config";
    shortcuts_[SDLK_l | (KMOD_CTRL << 16)] = "file_load_config";
    shortcuts_[SDLK_q | (KMOD_CTRL << 16)] = "file_exit";
    
    // Edit menu shortcuts
    shortcuts_[SDLK_z | (KMOD_CTRL << 16)] = "edit_undo";
    shortcuts_[SDLK_y | (KMOD_CTRL << 16)] = "edit_redo";
    shortcuts_[SDLK_c | (KMOD_CTRL << 16)] = "edit_copy";
    shortcuts_[SDLK_v | (KMOD_CTRL << 16)] = "edit_paste";
    
    // Emulator shortcuts
    shortcuts_[SDLK_F5] = "emulator_start";
    shortcuts_[SDLK_F5 | (KMOD_SHIFT << 16)] = "emulator_stop";
    shortcuts_[SDLK_F6] = "emulator_pause";
    shortcuts_[SDLK_F7] = "emulator_resume";
    shortcuts_[SDLK_r | (KMOD_CTRL << 16)] = "emulator_reset";
    
    // View shortcuts
    shortcuts_[SDLK_F3] = "view_gpio_viewer";
    shortcuts_[SDLK_F4] = "view_control_panel";
    shortcuts_[SDLK_F11] = "view_fullscreen";
    shortcuts_[SDLK_F12] = "tools_screenshot";
    
    // Debug shortcuts
    shortcuts_[SDLK_F9] = "debug_start";
    shortcuts_[SDLK_F10] = "debug_step_over";
    shortcuts_[SDLK_F11] = "debug_step_into";
    shortcuts_[SDLK_F2] = "debug_toggle_breakpoint";
    
    // Help shortcuts
    shortcuts_[SDLK_F1] = "help_user_guide";
#endif
    
    LOG_DEBUG("Registered {} keyboard shortcuts", shortcuts_.size());
}

bool MenuBar::handle_shortcut(const SDL_KeyboardEvent& event) {
#ifndef NO_GRAPHICS
    u32 key_combo = event.keysym.sym;
    
    // Add modifier keys to the combination
    if (event.keysym.mod & KMOD_CTRL) {
        key_combo |= (KMOD_CTRL << 16);
    }
    if (event.keysym.mod & KMOD_SHIFT) {
        key_combo |= (KMOD_SHIFT << 16);
    }
    if (event.keysym.mod & KMOD_ALT) {
        key_combo |= (KMOD_ALT << 16);
    }
    
    auto it = shortcuts_.find(key_combo);
    if (it != shortcuts_.end()) {
        LOG_DEBUG("Keyboard shortcut triggered: {} -> {}", key_combo, it->second);
        
        // Find the corresponding menu item for context
        MenuItem dummy_item;
        dummy_item.action = it->second;
        dummy_item.text = "Keyboard Shortcut";
        
        execute_action(it->second, dummy_item);
        return true;
    }
#endif
    
    return false;
}

u32 MenuBar::calculate_text_width(const std::string& text) {
    // Simple approximation: 8 pixels per character
    // In a real implementation, this would measure actual font metrics
    return static_cast<u32>(text.length() * 8);
}

void MenuBar::draw_text_centered(const std::string& text, u32 x, u32 y, u32 width, u32 color) {
    if (!renderer_) return;
    
    u32 text_width = calculate_text_width(text);
    u32 text_x = x + (width - text_width) / 2;
    u32 text_y = y + (MENU_HEIGHT - 12) / 2; // Center vertically
    
    renderer_->draw_text(text_x, text_y, text, color);
}

bool MenuBar::is_point_in_rect(i32 px, i32 py, i32 x, i32 y, u32 w, u32 h) {
    return px >= x && px < x + static_cast<i32>(w) && 
           py >= y && py < y + static_cast<i32>(h);
}

// Context Menu Implementation
ContextMenu::ContextMenu(m5tab5::emulator::SdlRenderer* renderer) 
    : renderer_(renderer), position_x_(0), position_y_(0), visible_(false), highlighted_item_(0) {
}

ContextMenu::~ContextMenu() {
    hide();
}

void ContextMenu::show(i32 x, i32 y, const std::vector<ContextItem>& items) {
    items_ = items;
    position_x_ = x;
    position_y_ = y;
    visible_ = true;
    highlighted_item_ = 0;
    
    LOG_DEBUG("Context menu shown at ({}, {}) with {} items", x, y, items.size());
}

void ContextMenu::hide() {
    visible_ = false;
    items_.clear();
}

void ContextMenu::render() {
    if (!visible_ || !renderer_ || items_.empty()) return;
    
    const u32 item_height = 25;
    const u32 menu_width = 180;
    const u32 menu_height = static_cast<u32>(items_.size() * item_height);
    
    // Draw background with shadow
    renderer_->draw_rect(position_x_ + 2, position_y_ + 2, menu_width, menu_height, 0x000000);
    renderer_->draw_rect(position_x_, position_y_, menu_width, menu_height, 0x2a2a2a);
    
    // Draw border
    renderer_->draw_rect(position_x_, position_y_, menu_width, 1, 0x555555);
    renderer_->draw_rect(position_x_, position_y_ + menu_height - 1, menu_width, 1, 0x555555);
    renderer_->draw_rect(position_x_, position_y_, 1, menu_height, 0x555555);
    renderer_->draw_rect(position_x_ + menu_width - 1, position_y_, 1, menu_height, 0x555555);
    
    // Draw items
    for (size_t i = 0; i < items_.size(); ++i) {
        const auto& item = items_[i];
        u32 item_y = position_y_ + static_cast<u32>(i * item_height);
        
        // Highlight if selected
        if (highlighted_item_ == i && item.enabled) {
            renderer_->draw_rect(position_x_ + 1, item_y + 1, menu_width - 2, item_height - 2, 0x0066cc);
        }
        
        // Draw text
        u32 text_color = item.enabled ? 0xffffff : 0x666666;
        renderer_->draw_text(position_x_ + 8, item_y + (item_height - 12) / 2, item.text, text_color);
    }
}

#ifndef NO_GRAPHICS
bool ContextMenu::handle_mouse_event(const SDL_MouseButtonEvent& event) {
    if (!visible_) return false;
    
    if (event.type == SDL_MOUSEBUTTONDOWN) {
        const u32 item_height = 25;
        const u32 menu_width = 180;
        const u32 menu_height = static_cast<u32>(items_.size() * item_height);
        
        // Check if click is within menu bounds
        if (event.x >= position_x_ && event.x < position_x_ + static_cast<i32>(menu_width) &&
            event.y >= position_y_ && event.y < position_y_ + static_cast<i32>(menu_height)) {
            
            // Calculate clicked item
            size_t item_index = static_cast<size_t>((event.y - position_y_) / item_height);
            if (item_index < items_.size() && items_[item_index].enabled) {
                LOG_INFO("Context menu action: {}", items_[item_index].action);
                // Execute action here
                hide();
                return true;
            }
        } else {
            // Click outside - hide menu
            hide();
        }
    }
    
    return visible_; // Consume events while visible
}
#endif

} // namespace m5tab5::emulator::gui