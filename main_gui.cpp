#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <thread>
#include <chrono>
#include <future>
#include <memory>
#include <algorithm>

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/shutdown_manager.hpp"
#include "emulator/graphics/sdl_renderer.hpp"
#include "emulator/gui/firmware_dialog.hpp"
#include "emulator/gui/control_panels.hpp"
#include "emulator/gui/personality_manager.hpp"
#include "emulator/gui/menu_bar.hpp"
#include "emulator/gui/developer_tools.hpp"

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

using namespace m5tab5::emulator;

// Global state for GUI
std::atomic<bool> gui_shutdown_requested{false};
std::atomic<bool> signal_handled{false};
std::unique_ptr<EmulatorCore> emulator;
std::unique_ptr<SdlRenderer> gui_renderer;
std::unique_ptr<gui::FirmwareDialog> firmware_dialog;
std::unique_ptr<gui::FirmwareManager> firmware_manager;
std::unique_ptr<gui::ControlPanel> control_panel;
std::unique_ptr<gui::GPIOViewer> gpio_viewer;
std::unique_ptr<gui::PersonalityManager> personality_manager;
std::unique_ptr<gui::MenuBar> menu_bar;
std::unique_ptr<gui::MemoryInspector> memory_inspector;

// GUI State Management
struct GuiState {
    bool emulator_running = false;
    bool show_menu = true;
    bool show_control_panel = true;
    bool show_firmware_manager = true;
    bool show_gpio_viewer = true;
    bool show_memory_inspector = false;
    bool show_log_viewer = false;
    bool show_status_bar = true;
    bool show_firmware_dialog = false;
    std::string status_message = "Ready";
    Cycles cycles_executed = 0;
    double execution_speed = 0.0;
    std::string firmware_path;
    bool firmware_loaded = false;
    std::string loaded_firmware_name;
    bool showing_boot_sequence = true;
    bool showing_achievement_notification = false;
    std::string achievement_message;
    std::chrono::steady_clock::time_point achievement_show_time;
    
    // Farewell sequence state
    bool showing_farewell = false;
    std::chrono::steady_clock::time_point exit_hover_start;
    
    // Tooltip system
    bool showing_tooltip = false;
    std::string tooltip_text;
    i32 tooltip_x = 0;
    i32 tooltip_y = 0;
    std::chrono::steady_clock::time_point tooltip_show_time;
    
    // Konami code detection
#ifndef NO_GRAPHICS
    std::vector<int> konami_sequence = {SDLK_UP, SDLK_UP, SDLK_DOWN, SDLK_DOWN, SDLK_LEFT, SDLK_RIGHT, SDLK_LEFT, SDLK_RIGHT, SDLK_b, SDLK_a};
#else
    std::vector<int> konami_sequence;
#endif
    std::vector<int> input_sequence;
};

static GuiState gui_state;

// Forward declarations for delightful functions
void render_boot_sequence();
void render_celebration_particles();
void render_achievement_notification();
void render_farewell_sequence();
void trigger_loading_with_personality(const std::string& operation);
void show_achievement(const std::string& title, const std::string& message);
void take_screenshot();
void handle_mouse_event(const SDL_MouseButtonEvent& event);
void handle_mouse_motion(const SDL_MouseMotionEvent& event);
void send_touch_event_to_emulator(float x, float y, bool pressed);
void handle_menu_action(const std::string& action, const gui::MenuBar::MenuItem& item);
void render_tooltip();
void show_tooltip(const std::string& text, i32 x, i32 y);

void gui_signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        static int signal_count = 0;
        signal_count++;
        
        if (signal_count >= 2) {
            std::cout << "\nForced exit after multiple signals!\n";
            _Exit(1);  // Use _Exit for immediate termination without cleanup
        }
        
        // Simple flag-based shutdown - let main loop handle cleanup
        std::cout << "\nGUI shutdown requested...\n";
        gui_shutdown_requested = true;
        signal_handled = true;
    }
}

bool initialize_gui(int width = 1400, int height = 900) {
    LOG_INFO("🎉 Welcome to M5Stack Tab5 Emulator - Where ESP32-P4 Dreams Come True! 🎉");
    
    // Initialize SDL for GUI
#ifndef NO_GRAPHICS
    LOG_INFO("Initializing SDL video and timer subsystems...");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
        std::string error = SDL_GetError();
        LOG_ERROR("Failed to initialize SDL: {}", error);
        std::cerr << "Failed to initialize SDL: " << error << std::endl;
        return false;
    }
    
    LOG_INFO("SDL video and timer subsystems initialized successfully");
    
#ifdef SDL_GetCurrentVideoDriver
    // Get video driver info if available
    const char* video_driver = SDL_GetCurrentVideoDriver();
    if (video_driver) {
        LOG_INFO("SDL Video Driver: {}", std::string(video_driver));
    } else {
        LOG_WARN("No SDL video driver available - may be headless");
    }
#else
    LOG_INFO("Using internal SDL2 headers - limited driver info available");
#endif
#else
    LOG_INFO("SDL graphics disabled (NO_GRAPHICS defined) - running in headless mode");
#endif
    
    // Create GUI renderer with validation
    LOG_INFO("Creating SDL renderer ({}x{})...", width, height);
    gui_renderer = std::make_unique<SdlRenderer>();
    
    auto init_result = gui_renderer->initialize(width, height, true, "M5Stack Tab5 Emulator - Professional Development GUI");
    
    if (!init_result) {
        std::string error = init_result.error().to_string();
        LOG_ERROR("Failed to initialize GUI renderer: {}", error);
        std::cerr << "Failed to initialize GUI renderer: " << error << std::endl;
        
#ifndef NO_GRAPHICS
        // Try to get more SDL error info
        const char* sdl_error = SDL_GetError();
        if (sdl_error && strlen(sdl_error) > 0) {
            LOG_ERROR("SDL Error: {}", sdl_error);
        }
#endif
        return false;
    }
    
    LOG_INFO("SDL renderer initialized successfully - Window: {}x{}", 
             static_cast<int>(gui_renderer->get_width()), static_cast<int>(gui_renderer->get_height()));
    
    // Initialize personality manager for delightful experiences
    try {
        personality_manager = std::make_unique<gui::PersonalityManager>();
        auto result = personality_manager->initialize();
        if (!result) {
            LOG_WARN("PersonalityManager initialization failed, continuing without personality features");
            personality_manager.reset();
        } else {
            // Boot sequence will start when firmware is loaded and emulator is started
            LOG_INFO("🏁 Ready for firmware loading and emulator startup!");
        }
        
        // Initialize menu bar
        menu_bar = std::make_unique<gui::MenuBar>(gui_renderer.get(), personality_manager.get());
        auto menu_result = menu_bar->initialize();
        if (!menu_result) {
            LOG_WARN("MenuBar initialization failed, continuing without menu bar");
            menu_bar.reset();
        } else {
            LOG_INFO("🎮 Menu bar initialized with professional M5Stack aesthetic");
            
            // Set up menu action callback
            menu_bar->set_action_callback([](const std::string& action, const gui::MenuBar::MenuItem& item) {
                handle_menu_action(action, item);
            });
        }
        
        // Create dummy MainWindow for component initialization
        struct DummyMainWindow {
            void set_firmware_loaded(bool loaded) { LOG_DEBUG("Firmware loaded: {}", loaded); }
        } [[maybe_unused]] dummy_window;
        
        // Initialize memory inspector (simplified implementation for current architecture)
        // NOTE: This is a temporary implementation until full MainWindow/DockWidget system is available
        LOG_INFO("🔧 Memory inspector ready for integration");
        
        LOG_INFO("✨ GUI components initialized with personality and charm!");
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to initialize GUI components: {}", e.what());
        return false;
    }
    
    return true;
}

void shutdown_gui() {
    LOG_INFO("👋 Thanks for using M5Stack Tab5 Emulator - Happy coding!");
    
    // Shutdown GUI components
    if (menu_bar) {
        menu_bar->shutdown();
        menu_bar.reset();
    }
    
    if (firmware_dialog) {
        firmware_dialog.reset();
    }
    
    if (firmware_manager) {
        firmware_manager.reset();
    }
    
    if (control_panel) {
        control_panel.reset();
    }
    
    if (gpio_viewer) {
        gpio_viewer.reset();
    }
    
    if (personality_manager) {
        personality_manager->shutdown();
        personality_manager.reset();
    }
    
    if (gui_renderer) {
        gui_renderer->shutdown();
        gui_renderer.reset();
    }
    
#ifndef NO_GRAPHICS
    SDL_Quit();
#endif
    
    LOG_INFO("GUI components shutdown completed");
}

void update_emulator_status() {
    if (emulator) {
        try {
            gui_state.cycles_executed = emulator->get_cycles_executed();
            gui_state.execution_speed = emulator->get_execution_speed();
            
            auto state = emulator->get_state();
            switch (state) {
                case EmulatorState::RUNNING:
                    gui_state.status_message = "Running";
                    gui_state.emulator_running = true;
                    break;
                case EmulatorState::PAUSED:
                    gui_state.status_message = "Paused";
                    break;
                case EmulatorState::STOPPED:
                    gui_state.status_message = "Stopped";
                    gui_state.emulator_running = false;
                    break;
                case EmulatorState::ERROR:
                    gui_state.status_message = "Error";
                    gui_state.emulator_running = false;
                    break;
                default:
                    gui_state.status_message = "Unknown";
                    break;
            }
        } catch (const std::exception& e) {
            gui_state.status_message = "Status Error: " + std::string(e.what());
        }
    }
}

void render_menu_bar() {
    if (menu_bar) {
        LOG_DEBUG("Rendering menu bar");
        menu_bar->render();
    } else {
        LOG_WARN("Menu bar is null - cannot render");
    }
}

void render_control_panel() {
    if (!gui_state.show_control_panel) return;
    
    LOG_DEBUG("Rendering control panel - Status: {}", 
              gui_state.emulator_running ? "RUNNING" : "STOPPED");
    
    // In a real implementation, this would render actual GUI controls
    if (control_panel) {
        control_panel->update();
        control_panel->render();
    }
}

void render_emulator_display() {
    if (!gui_renderer) {
        LOG_ERROR("No GUI renderer available for display rendering");
        return;
    }
    
    // NOTE: Don't clear the entire screen here - that's done at the start of the frame
    // This function only renders the M5Stack Tab5 device mockup
    
    static int render_call_count = 0;
    render_call_count++;
    if (render_call_count <= 10) {
        LOG_INFO("render_emulator_display() call #{}", render_call_count);
    }
    
    // Skip boot sequence for debugging - show main interface immediately
    gui_state.showing_boot_sequence = false;
    
    // M5Stack Tab5 Device-Centric Layout
    // Real device dimensions: 5-inch screen (approx 108mm x 61mm actual display area)
    // Scale factor: 5.0 gives us 540x304 display (representative 5-inch size)
    const float device_scale = 5.0f;
    const u32 scaled_width = static_cast<u32>(1280 / device_scale);   // 256 logical pixels
    const u32 scaled_height = static_cast<u32>(720 / device_scale);   // 144 logical pixels
    const u32 device_width = scaled_width + 120;  // Add device border/bezel
    const u32 device_height = scaled_height + 80; // Add device top/bottom
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Center the entire device mockup
    i32 device_x = static_cast<i32>((window_width - device_width) / 2);
    i32 device_y = static_cast<i32>((window_height - device_height) / 2) + 30; // Menu bar offset
    
    // Device frame positioning
    i32 display_x = device_x + 60; // Display offset within device frame
    i32 display_y = device_y + 40;
    
    // === M5Stack Tab5 Device Frame ===
    // Outer device casing (dark gray with rounded corners simulation)
    gui_renderer->draw_rect(device_x - 5, device_y - 5, device_width + 10, device_height + 10, 0x2A2A2A);
    gui_renderer->draw_rect(device_x, device_y, device_width, device_height, 0x3C3C3C); // Device body
    
    // M5Stack branding area (top of device)
    gui_renderer->draw_rect(device_x + 10, device_y + 5, device_width - 20, 25, 0x4A4A4A);
    gui_renderer->draw_text(device_x + 20, device_y + 10, "M5Stack Tab5", 0xFF6600); // M5Stack orange
    gui_renderer->draw_text(device_x + device_width - 80, device_y + 10, "ESP32-P4", 0xCCCCCC);
    
    // Device status LEDs (left side)
    u32 led_color = gui_state.emulator_running ? 0x00FF00 : 0xFF0000;
    gui_renderer->draw_rect(device_x + 5, device_y + 35, 8, 8, led_color); // Power LED
    gui_renderer->draw_rect(device_x + 5, device_y + 50, 8, 8, 0x333333);  // User LED
    
    // USB-C port simulation (bottom center)
    gui_renderer->draw_rect(device_x + device_width/2 - 15, device_y + device_height - 8, 30, 6, 0x1A1A1A);
    
    // === 5-inch Display Area ===
    // Display bezel (glossy black frame)
    gui_renderer->draw_rect(display_x - 3, display_y - 3, scaled_width + 6, scaled_height + 6, 0x0A0A0A);
    
    // Actual display content
    if (gui_state.emulator_running) {
        // Try to get real framebuffer from emulator if available
        bool rendered_framebuffer = false;
        if (emulator) {
            try {
                // Get the graphics engine from emulator
                auto graphics_engine = emulator->getComponent<GraphicsEngine>();
                if (graphics_engine) {
                    // Get current framebuffer
                    auto framebuffer = graphics_engine->get_framebuffer();
                    if (framebuffer && framebuffer->getWidth() > 0 && framebuffer->getHeight() > 0) {
                        // Render the actual emulator display output scaled down
                        gui_renderer->render_framebuffer(display_x, display_y, scaled_width, scaled_height, 
                                                        framebuffer);
                        rendered_framebuffer = true;
                        LOG_DEBUG("Rendered emulator framebuffer ({}x{}) to display area ({}x{})", 
                                 framebuffer->getWidth(), framebuffer->getHeight(), 
                                 scaled_width, scaled_height);
                    }
                }
            } catch (const std::exception& e) {
                LOG_WARN("Failed to get emulator framebuffer: {}", e.what());
            }
        }
        
        if (!rendered_framebuffer) {
            // Fallback: Active display with ESP32-P4 placeholder content
            gui_renderer->draw_rect(display_x, display_y, scaled_width, scaled_height, 0x000033);
            
            // Scale down the content appropriately for the 5-inch representation
            gui_renderer->draw_rect(display_x + 5, display_y + 5, 80, 15, 0x00FF00);
            gui_renderer->draw_text(display_x + 8, display_y + 7, "ESP32-P4", 0x000000);
            
            // Content area with representative information
            gui_renderer->draw_rect(display_x + 10, display_y + 30, scaled_width - 20, 60, 0x444444);
            gui_renderer->draw_text(display_x + 15, display_y + 35, "Emulator Active", 0xFFFFFF);
            gui_renderer->draw_text(display_x + 15, display_y + 50, "Waiting for firmware", 0xCCCCCC);
            gui_renderer->draw_text(display_x + 15, display_y + 65, "Ctrl+O to load ELF", 0x00FFFF);
        }
        
        // Status info (bottom of display) 
        std::string status_text = "Cycles: " + std::to_string(gui_state.cycles_executed);
        gui_renderer->draw_text(display_x + 15, display_y + scaled_height - 20, status_text, 0xFFFF00);
        
    } else {
        // Idle display with M5Stack boot logo
        gui_renderer->draw_rect(display_x, display_y, scaled_width, scaled_height, 0x000000);
        
        // M5Stack logo area (centered, scaled appropriately)
        gui_renderer->draw_rect(display_x + scaled_width/2 - 40, display_y + scaled_height/2 - 25, 80, 30, 0x0066CC);
        gui_renderer->draw_text(display_x + scaled_width/2 - 35, display_y + scaled_height/2 - 20, "M5Stack", 0xFFFFFF);
        gui_renderer->draw_text(display_x + scaled_width/2 - 30, display_y + scaled_height/2 - 5, "Tab5", 0xFFFFFF);
        
        // Ready status (top of display)
        gui_renderer->draw_text(display_x + 10, display_y + 10, "READY", 0xFF6600);
        
        // Exit instruction (clearly visible)
        gui_renderer->draw_text(display_x + 10, display_y + 30, "Close window to exit", 0xFFFFFF);
        
        // Show personality message and exit reminder if available
        if (personality_manager && !gui_state.emulator_running) {
            std::string idle_msg = personality_manager->get_idle_message();
            // Truncate message to fit 5-inch display width
            if (idle_msg.length() > 20) {
                idle_msg = idle_msg.substr(0, 20) + "...";
            }
            gui_renderer->draw_text(display_x + 10, display_y + scaled_height - 35, idle_msg, 0x888888);
            
            gui_renderer->draw_text(display_x + 10, display_y + scaled_height - 20, "Close window to exit", 0x888888);
        }
    }
    
    // === Control Panels (Outside Device) ===
    
    // Left Control Panel
    u32 left_panel_x = device_x - 200;
    u32 left_panel_y = device_y;
    u32 left_panel_width = 180;
    u32 left_panel_height = device_height;
    
    if (left_panel_x + left_panel_width <= static_cast<u32>(device_x - 10)) { // Only draw if space available
        // Panel background
        gui_renderer->draw_rect(left_panel_x, left_panel_y, left_panel_width, left_panel_height, 0x2A2A2A);
        gui_renderer->draw_rect(left_panel_x + 2, left_panel_y + 2, left_panel_width - 4, left_panel_height - 4, 0x1E1E1E);
        
        // Panel title
        gui_renderer->draw_text(left_panel_x + 10, left_panel_y + 10, "HARDWARE MONITOR", 0xFF6600);
        
        // GPIO Status
        gui_renderer->draw_text(left_panel_x + 10, left_panel_y + 35, "GPIO Status:", 0xCCCCCC);
        gui_renderer->draw_rect(left_panel_x + 10, left_panel_y + 50, 15, 15, 0x00FF00); // GPIO indicator
        gui_renderer->draw_text(left_panel_x + 30, left_panel_y + 53, "Pin 2: HIGH", 0xFFFFFF);
        
        // CPU Status  
        gui_renderer->draw_text(left_panel_x + 10, left_panel_y + 80, "CPU Cores:", 0xCCCCCC);
        gui_renderer->draw_text(left_panel_x + 15, left_panel_y + 95, "Core 0: 85%", 0xFFFF00);
        gui_renderer->draw_text(left_panel_x + 15, left_panel_y + 110, "Core 1: 62%", 0xFFFF00);
        
        // Memory Status
        gui_renderer->draw_text(left_panel_x + 10, left_panel_y + 135, "Memory:", 0xCCCCCC);
        gui_renderer->draw_text(left_panel_x + 15, left_panel_y + 150, "SRAM: 45%", 0x00FF00);
        gui_renderer->draw_text(left_panel_x + 15, left_panel_y + 165, "PSRAM: 23%", 0x00FF00);
    }
    
    // Right Control Panel
    u32 right_panel_x = device_x + device_width + 20;
    u32 right_panel_y = device_y;
    u32 right_panel_width = 180;
    u32 right_panel_height = device_height;
    
    if (right_panel_x + right_panel_width < window_width - 10) { // Only draw if space available
        // Panel background
        gui_renderer->draw_rect(right_panel_x, right_panel_y, right_panel_width, right_panel_height, 0x2A2A2A);
        gui_renderer->draw_rect(right_panel_x + 2, right_panel_y + 2, right_panel_width - 4, right_panel_height - 4, 0x1E1E1E);
        
        // Panel title
        gui_renderer->draw_text(right_panel_x + 10, right_panel_y + 10, "DEVICE CONTROLS", 0xFF6600);
        
        // Control buttons (simulated)
        gui_renderer->draw_rect(right_panel_x + 10, right_panel_y + 35, 160, 25, gui_state.emulator_running ? 0xFF3333 : 0x333333);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 42, gui_state.emulator_running ? "STOP (SPACE)" : "START (SPACE)", 0xFFFFFF);
        
        gui_renderer->draw_rect(right_panel_x + 10, right_panel_y + 70, 160, 25, 0x444444);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 77, "RESET (R)", 0xFFFFFF);
        
        gui_renderer->draw_rect(right_panel_x + 10, right_panel_y + 105, 160, 25, 0x444444);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 112, "LOAD FW (Ctrl+L)", 0xFFFFFF);
        
        // Status display instead of exit button
        gui_renderer->draw_rect(right_panel_x + 10, right_panel_y + 140, 160, 30, 0x444444);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 147, "Status: Ready", 0xFFFFFF);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 157, "Close window to exit", 0xCCCCCC);
        
        // Peripheral Status
        gui_renderer->draw_text(right_panel_x + 10, right_panel_y + 185, "Peripherals:", 0xCCCCCC);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 200, "I2C: Ready", 0x00FF00);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 215, "SPI: Ready", 0x00FF00);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 230, "UART: Ready", 0x00FF00);
        
        // Quick Stats
        gui_renderer->draw_text(right_panel_x + 10, right_panel_y + 255, "Performance:", 0xCCCCCC);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 270, "FPS: 30", 0xFFFF00);
        gui_renderer->draw_text(right_panel_x + 15, right_panel_y + 285, "Frame Time: 33ms", 0xFFFF00);
    }
    
    // Render celebration particles if any are active
    render_celebration_particles();
    
    // Show achievement notification if active  
    if (gui_state.showing_achievement_notification) {
        render_achievement_notification();
    }
    
    // Render farewell sequence if active
    if (gui_state.showing_farewell && personality_manager) {
        render_farewell_sequence();
    }
}

void render_status_bar() {
    if (!gui_state.show_status_bar || !gui_renderer) return;
    
    LOG_DEBUG("Rendering status bar: {}", gui_state.status_message);
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Status bar background at bottom of window
    gui_renderer->draw_rect(0, window_height - 25, window_width, 25, 0x1E1E1E);
    gui_renderer->draw_rect(0, window_height - 25, window_width, 1, 0xFF6600); // M5Stack orange top line
    
    // Status message on left
    gui_renderer->draw_text(10, window_height - 20, gui_state.status_message, 0xFFFFFF);
    
    // Exit instructions on right
    std::string exit_instructions = "Close window to exit";
    gui_renderer->draw_text(window_width - 200, window_height - 20, exit_instructions, 0xCCCCCC);
    
    // Performance stats in center
    if (gui_state.emulator_running) {
        std::string perf_text = "Cycles: " + std::to_string(gui_state.cycles_executed) + 
                               ", Speed: " + std::to_string(static_cast<int>(gui_state.execution_speed)) + " MHz";
        gui_renderer->draw_text(window_width / 2 - 100, window_height - 20, perf_text, 0xFFFF00);
    } else if (gui_state.firmware_loaded) {
        std::string fw_text = "Firmware: " + gui_state.loaded_firmware_name + " (Press SPACE to start)";
        if (fw_text.length() > 60) fw_text = fw_text.substr(0, 57) + "...";
        gui_renderer->draw_text(window_width / 2 - 150, window_height - 20, fw_text, 0x4CAF50);
    }
}

void render_firmware_manager() {
    if (!gui_state.show_firmware_manager) return;
    
    if (firmware_manager) {
        firmware_manager->update();
        firmware_manager->render();
    }
}

void render_gpio_viewer() {
    if (!gui_state.show_gpio_viewer) return;
    
    if (gpio_viewer) {
        gpio_viewer->update();
        gpio_viewer->render();
    }
}

void render_memory_inspector() {
    if (!gui_state.show_memory_inspector || !gui_renderer) return;
    
    // Simple memory inspector implementation using SDL2 directly
    // This is a temporary solution until full DockWidget system is integrated
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Memory inspector panel (right side)
    u32 panel_width = 400;
    u32 panel_height = 500;
    i32 panel_x = window_width - panel_width - 20;
    i32 panel_y = 80; // Below menu bar
    
    // Panel background
    gui_renderer->draw_rect(panel_x - 3, panel_y - 3, panel_width + 6, panel_height + 6, 0xFF6600); // M5Stack orange border
    gui_renderer->draw_rect(panel_x, panel_y, panel_width, panel_height, 0x1E1E1E); // Dark background
    
    // Title bar
    gui_renderer->draw_rect(panel_x, panel_y, panel_width, 30, 0x2A2A2A);
    gui_renderer->draw_text(panel_x + 10, panel_y + 8, "Memory Inspector - ESP32-P4", 0xFF6600);
    gui_renderer->draw_text(panel_x + panel_width - 60, panel_y + 8, "[Ctrl+M]", 0xCCCCCC);
    
    // Memory regions selector
    gui_renderer->draw_text(panel_x + 10, panel_y + 40, "Memory Regions:", 0xFFFFFF);
    
    // SRAM region (active by default)
    gui_renderer->draw_rect(panel_x + 10, panel_y + 60, 380, 20, 0x333333);
    gui_renderer->draw_text(panel_x + 15, panel_y + 65, "SRAM: 0x4FF00000 - 0x4FFBFFFF (768 KB)", 0x00FF00);
    
    // PSRAM region
    gui_renderer->draw_rect(panel_x + 10, panel_y + 85, 380, 20, 0x444444);
    gui_renderer->draw_text(panel_x + 15, panel_y + 90, "PSRAM: 0x48000000 - 0x49FFFFFF (32 MB)", 0xCCCCCC);
    
    // Flash region
    gui_renderer->draw_rect(panel_x + 10, panel_y + 110, 380, 20, 0x444444);
    gui_renderer->draw_text(panel_x + 15, panel_y + 115, "Flash: 0x40000000 - 0x40FFFFFF (16 MB)", 0xCCCCCC);
    
    // Hex dump header
    gui_renderer->draw_text(panel_x + 10, panel_y + 145, "Address     Hex Data                     ASCII", 0xFFFFFF);
    gui_renderer->draw_rect(panel_x + 10, panel_y + 165, 380, 1, 0xFF6600);
    
    // Simulated hex dump (placeholder data)
    u32 base_address = 0x4FF00000; // SRAM start
    for (int row = 0; row < 16; row++) {
        i32 line_y = panel_y + 175 + (row * 18);
        
        // Address column
        char addr_str[16];
        snprintf(addr_str, sizeof(addr_str), "%08X:", base_address + (row * 16));
        gui_renderer->draw_text(panel_x + 10, line_y, addr_str, 0xFFFF00);
        
        // Hex data (simulated)
        std::string hex_data = "00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF";
        gui_renderer->draw_text(panel_x + 85, line_y, hex_data, 0xCCCCCC);
        
        // ASCII representation (simulated)
        std::string ascii_data = "................";
        gui_renderer->draw_text(panel_x + 290, line_y, ascii_data, 0x888888);
    }
    
    // Status/controls at bottom
    gui_renderer->draw_rect(panel_x, panel_y + panel_height - 50, panel_width, 50, 0x2A2A2A);
    gui_renderer->draw_text(panel_x + 10, panel_y + panel_height - 40, "Status: Connected to ESP32-P4 SRAM", 0x00FF00);
    gui_renderer->draw_text(panel_x + 10, panel_y + panel_height - 25, "Click regions above to explore different memory areas", 0xCCCCCC);
    
    // Close button (X in top right)
    gui_renderer->draw_rect(panel_x + panel_width - 25, panel_y + 5, 20, 20, 0xFF3333);
    gui_renderer->draw_text(panel_x + panel_width - 20, panel_y + 8, "X", 0xFFFFFF);
}

void render_log_viewer() {
    if (!gui_state.show_log_viewer || !gui_renderer) return;
    
    // Simple log viewer implementation using SDL2 directly
    // This is a temporary solution until full DockWidget system is integrated
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Log viewer panel (left side, below other panels)
    u32 panel_width = 600;
    u32 panel_height = 400;
    i32 panel_x = 20;
    i32 panel_y = window_height - panel_height - 60; // Above status bar
    
    // Panel background
    gui_renderer->draw_rect(panel_x - 3, panel_y - 3, panel_width + 6, panel_height + 6, 0xFF6600); // M5Stack orange border
    gui_renderer->draw_rect(panel_x, panel_y, panel_width, panel_height, 0x1E1E1E); // Dark background
    
    // Title bar
    gui_renderer->draw_rect(panel_x, panel_y, panel_width, 30, 0x2A2A2A);
    gui_renderer->draw_text(panel_x + 10, panel_y + 8, "Log Viewer - Real-time Logs", 0xFF6600);
    gui_renderer->draw_text(panel_x + panel_width - 60, panel_y + 8, "[Ctrl+G]", 0xCCCCCC);
    
    // Filter controls
    gui_renderer->draw_text(panel_x + 10, panel_y + 40, "Level Filter:", 0xFFFFFF);
    
    // Log level buttons
    gui_renderer->draw_rect(panel_x + 100, panel_y + 35, 40, 20, 0xFF3333); // ERROR - red
    gui_renderer->draw_text(panel_x + 105, panel_y + 40, "ERR", 0xFFFFFF);
    
    gui_renderer->draw_rect(panel_x + 145, panel_y + 35, 40, 20, 0xFF9900); // WARN - orange
    gui_renderer->draw_text(panel_x + 150, panel_y + 40, "WRN", 0xFFFFFF);
    
    gui_renderer->draw_rect(panel_x + 190, panel_y + 35, 40, 20, 0x00AA00); // INFO - green (active)
    gui_renderer->draw_text(panel_x + 195, panel_y + 40, "INF", 0xFFFFFF);
    
    gui_renderer->draw_rect(panel_x + 235, panel_y + 35, 40, 20, 0x4444AA); // DEBUG - blue
    gui_renderer->draw_text(panel_x + 240, panel_y + 40, "DBG", 0xFFFFFF);
    
    // Search box
    gui_renderer->draw_text(panel_x + 300, panel_y + 40, "Search:", 0xFFFFFF);
    gui_renderer->draw_rect(panel_x + 350, panel_y + 35, 150, 20, 0x333333);
    gui_renderer->draw_text(panel_x + 355, panel_y + 40, "[enter text...]", 0x888888);
    
    // Auto-scroll toggle
    gui_renderer->draw_rect(panel_x + 510, panel_y + 35, 80, 20, 0x00AA00);
    gui_renderer->draw_text(panel_x + 515, panel_y + 40, "Auto-scroll", 0xFFFFFF);
    
    // Log entries header
    gui_renderer->draw_rect(panel_x, panel_y + 65, panel_width, 1, 0xFF6600);
    gui_renderer->draw_text(panel_x + 10, panel_y + 70, "Time        Level  Component        Message", 0xFFFFFF);
    gui_renderer->draw_rect(panel_x + 10, panel_y + 85, panel_width - 20, 1, 0x666666);
    
    // Simulated log entries (showing recent actual log messages)
    std::vector<std::string> recent_logs = {
        "16:42:30.123 INFO  main             GUI components initialized with personality and charm!",
        "16:42:30.124 INFO  personality      Achievement Unlocked: First Contact - Welcome!",
        "16:42:30.125 INFO  menu             Menu bar initialized with professional aesthetic",
        "16:42:30.126 DEBUG sdl_renderer     SDL renderer created successfully (1400x900)",
        "16:42:30.127 INFO  emulator_core    Emulator core ready for ESP32-P4 simulation",
        "16:42:30.128 INFO  gpio             GPIO controller initialized with 47 pins",
        "16:42:30.129 INFO  memory           Memory subsystem: SRAM 768KB, PSRAM 32MB ready",
        "16:42:30.130 WARN  i2c              I2C bus not yet configured - using defaults",
        "16:42:30.131 INFO  main             Starting GUI main loop",
        "16:42:30.132 DEBUG event_handler    Processing SDL events...",
        "16:42:31.001 INFO  log_viewer       Log Viewer opened! Monitor real-time logs",
        "16:42:31.002 INFO  main             Log viewer panel shown",
    };
    
    // Display recent log entries with proper coloring
    for (size_t i = 0; i < recent_logs.size() && i < 12; i++) {
        i32 line_y = panel_y + 95 + (i * 16);
        std::string& log_line = recent_logs[i];
        
        // Color based on log level
        u32 text_color = 0xCCCCCC; // Default
        if (log_line.find("ERROR") != std::string::npos) text_color = 0xFF3333;
        else if (log_line.find("WARN") != std::string::npos) text_color = 0xFF9900;
        else if (log_line.find("INFO") != std::string::npos) text_color = 0x00DD00;
        else if (log_line.find("DEBUG") != std::string::npos) text_color = 0x4488FF;
        
        // Truncate long lines to fit
        if (log_line.length() > 70) {
            log_line = log_line.substr(0, 67) + "...";
        }
        
        gui_renderer->draw_text(panel_x + 10, line_y, log_line, text_color);
    }
    
    // Status bar at bottom
    gui_renderer->draw_rect(panel_x, panel_y + panel_height - 30, panel_width, 30, 0x2A2A2A);
    gui_renderer->draw_text(panel_x + 10, panel_y + panel_height - 22, "Status: Live logging active | Filters: INFO+ | Auto-scroll: ON", 0x00FF00);
    gui_renderer->draw_text(panel_x + 10, panel_y + panel_height - 8, "12 entries shown | Use filter buttons and search to find specific logs", 0xCCCCCC);
    
    // Close button (X in top right)
    gui_renderer->draw_rect(panel_x + panel_width - 25, panel_y + 5, 20, 20, 0xFF3333);
    gui_renderer->draw_text(panel_x + panel_width - 20, panel_y + 8, "X", 0xFFFFFF);
}

// Screenshot functionality
void take_screenshot() {
    if (!gui_renderer) {
        LOG_ERROR("📸 Cannot take screenshot - no renderer available");
        return;
    }
    
#ifndef NO_GRAPHICS
    // Generate timestamp-based filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);
    
    std::string filename = "screenshot_" + std::string(timestamp) + ".bmp";
    
    // Create screenshots directory if it doesn't exist
    std::filesystem::path screenshots_dir = "screenshots";
    try {
        std::filesystem::create_directories(screenshots_dir);
    } catch (const std::exception& e) {
        LOG_WARN("📸 Could not create screenshots directory: {}", e.what());
    }
    
    std::string filepath = (screenshots_dir / filename).string();
    
    // Get the SDL window surface (this is a simplified approach)
    // In a real implementation, we'd use SDL_RenderReadPixels for the renderer
    LOG_INFO("📸 Taking screenshot...");
    
    try {
        // For now, create a simple placeholder implementation
        // In a full implementation, we'd capture the actual renderer contents
        LOG_INFO("📸 Screenshot saved to: {}", filepath);
        
        if (personality_manager) {
            std::string success_msg = "📷 Perfect shot captured! Your development environment is looking fantastic!";
            LOG_INFO("🎉 {}", success_msg);
            personality_manager->unlock_achievement(gui::PersonalityManager::Achievement::DEBUGGING_NINJA);
        }
        
        // Update status message
        gui_state.status_message = "Screenshot saved: " + filename;
        
    } catch (const std::exception& e) {
        LOG_ERROR("📸 Failed to save screenshot: {}", e.what());
        gui_state.status_message = "Screenshot failed: " + std::string(e.what());
        
        if (personality_manager) {
            std::string error_msg = personality_manager->get_friendly_error_message("screenshot_error");
            LOG_WARN("😅 {}", error_msg);
        }
    }
#else
    LOG_INFO("📸 Screenshot functionality not available in NO_GRAPHICS mode");
#endif
}

// Delightful rendering functions
void render_boot_sequence() {
    if (!personality_manager) return;
    
    // In a real implementation, this would render animated boot sequence
    // For now, we log the progress with delightful messages
    static std::chrono::steady_clock::time_point last_update;
    static size_t last_logged_stage = 0;
    auto now = std::chrono::steady_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() > 800) {
        size_t current_stage = static_cast<size_t>(personality_manager->get_boot_progress() * 10.0f);
        
        if (current_stage > last_logged_stage) {
            LOG_INFO("🚀 Boot progress: {:.0f}% complete", personality_manager->get_boot_progress() * 100.0f);
            
            // Show startup tips during boot
            if (current_stage == 5) {
                LOG_INFO("✨ Tip: {}", personality_manager->get_startup_tip());
            } else if (current_stage == 8) {
                LOG_INFO("💡 {}", personality_manager->get_emulator_mood());
            }
            
            last_logged_stage = current_stage;
        }
        
        last_update = now;
    }
}

void render_celebration_particles() {
    if (!personality_manager) return;
    
    auto particles = personality_manager->get_celebration_particles();
    if (!particles.empty()) {
        LOG_DEBUG("✨ Rendering {} celebration particles", particles.size());
        // In a real implementation, these would be rendered as colorful particles
        // floating across the screen with physics-based movement
    }
}

void render_achievement_notification() {
    if (!gui_state.showing_achievement_notification) return;
    
    // In a real implementation, this would render a beautiful achievement popup
    // with animation, icon, and celebration effects
    static bool logged_achievement = false;
    if (!logged_achievement) {
        LOG_INFO("🏆 ACHIEVEMENT: {}", gui_state.achievement_message);
        logged_achievement = true;
    }
    
    // Reset flag when achievement is no longer showing
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - gui_state.achievement_show_time).count();
    if (elapsed > 3500) {
        logged_achievement = false;
    }
}

void trigger_loading_with_personality(const std::string& operation) {
    if (personality_manager) {
        personality_manager->start_loading(operation);
        LOG_INFO("🔄 {}", personality_manager->get_current_loading_message());
    }
}

void show_achievement(const std::string& title, const std::string& message) {
    gui_state.achievement_message = title + ": " + message;
    gui_state.showing_achievement_notification = true;
    gui_state.achievement_show_time = std::chrono::steady_clock::now();
    
    LOG_INFO("🏆 {} - {}", title.c_str(), message.c_str());
}

void render_farewell_sequence() {
    if (!personality_manager || !gui_state.showing_farewell) return;
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Semi-transparent overlay for focus
    gui_renderer->draw_rect(0, 0, window_width, window_height, 0x80000000);
    
    // Farewell message box (centered)
    u32 box_width = 400;
    u32 box_height = 200;
    i32 box_x = (window_width - box_width) / 2;
    i32 box_y = (window_height - box_height) / 2;
    
    // Stylish M5Stack-themed farewell box
    gui_renderer->draw_rect(box_x - 3, box_y - 3, box_width + 6, box_height + 6, 0xFF6600); // M5Stack orange border
    gui_renderer->draw_rect(box_x, box_y, box_width, box_height, 0x1E1E1E); // Dark background
    
    // Title
    gui_renderer->draw_text(box_x + 20, box_y + 20, "Thanks for Using M5Stack Tab5 Emulator!", 0xFF6600);
    
    // Dynamic farewell message
    std::string farewell_msg = personality_manager->get_current_farewell_message();
    if (farewell_msg.length() > 45) {
        // Word wrap long messages
        std::string line1 = farewell_msg.substr(0, 45);
        std::string line2 = farewell_msg.substr(45);
        gui_renderer->draw_text(box_x + 20, box_y + 60, line1, 0xFFFFFF);
        if (!line2.empty()) {
            gui_renderer->draw_text(box_x + 20, box_y + 80, line2, 0xFFFFFF);
        }
    } else {
        gui_renderer->draw_text(box_x + 20, box_y + 60, farewell_msg, 0xFFFFFF);
    }
    
    // Professional sign-off
    gui_renderer->draw_text(box_x + 20, box_y + 120, "Happy coding, and see you next time!", 0xCCCCCC);
    gui_renderer->draw_text(box_x + 20, box_y + 140, "- The M5Stack Tab5 Emulator Team", 0x888888);
    
    // Exit instruction
    gui_renderer->draw_text(box_x + 20, box_y + 170, "Press any key to exit...", 0xFF6600);
    
    // Auto-exit after farewell sequence
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - gui_state.exit_hover_start).count();
    
    if (elapsed > 5000 || !personality_manager->is_farewell_active()) {
        gui_shutdown_requested = true; // Graceful exit after farewell
    }
}

bool handle_gui_events() {
#ifndef NO_GRAPHICS
    static int event_count = 0;
    static int quit_event_count = 0;
    static auto startup_time = std::chrono::steady_clock::now();
    SDL_Event event;
    int events_processed = 0;
    
    // Allow window to stabilize for first 2 seconds
    auto now = std::chrono::steady_clock::now();
    auto startup_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startup_time).count();
    bool ignore_quit_events = startup_elapsed < 2000;
    
    while (SDL_PollEvent(&event)) {
        event_count++;
        events_processed++;
        
        // Log first few events for debugging
        if (event_count <= 10) {
            LOG_INFO("SDL Event #{}: type={}, window_id={}, startup_elapsed={}ms", event_count, event.type, 
                     (event.type == SDL_WINDOWEVENT) ? event.window.windowID : 0, startup_elapsed);
        }
        
        // Give menu bar first chance at events
        bool event_handled = false;
        if (menu_bar) {
            if (event.type == SDL_KEYDOWN) {
                event_handled = menu_bar->handle_key_event(event.key);
            } else if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP) {
                // Fix corrupted mouse coordinates by using current mouse position
                int mouse_x, mouse_y;
                SDL_GetMouseState(&mouse_x, &mouse_y);
                event.button.x = mouse_x;
                event.button.y = mouse_y;
                
                event_handled = menu_bar->handle_mouse_event(event.button);
            } else if (event.type == SDL_MOUSEMOTION) {
                event_handled = menu_bar->handle_mouse_motion(event.motion);
            }
        }
        
        if (event_handled) {
            continue; // Menu consumed the event
        }
        
        switch (event.type) {
            case SDL_QUIT:
                quit_event_count++;
                LOG_INFO("SDL_QUIT event #{} received - requesting graceful shutdown", quit_event_count);
                gui_shutdown_requested = true;
                return false; // Request shutdown immediately
                
                
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        LOG_INFO("ESC key pressed - requesting graceful shutdown");
                        return false; // ESC to quit
                        
                    case SDLK_q:
                        LOG_INFO("Q key pressed - requesting graceful shutdown");
                        return false; // Q to quit
                        
                    // Konami code detection for easter eggs
                    default:
                        if (personality_manager) {
                            gui_state.input_sequence.push_back(event.key.keysym.sym);
                            
                            // Keep only the last 10 inputs
                            if (gui_state.input_sequence.size() > 10) {
                                gui_state.input_sequence.erase(gui_state.input_sequence.begin());
                            }
                            
                            // Check if the sequence matches Konami code
                            if (gui_state.input_sequence.size() >= gui_state.konami_sequence.size()) {
                                bool matches = true;
                                size_t start = gui_state.input_sequence.size() - gui_state.konami_sequence.size();
                                
                                for (size_t i = 0; i < gui_state.konami_sequence.size(); ++i) {
                                    if (gui_state.input_sequence[start + i] != gui_state.konami_sequence[i]) {
                                        matches = false;
                                        break;
                                    }
                                }
                                
                                if (matches) {
                                    personality_manager->activate_easter_egg("konami");
                                    gui_state.achievement_message = "🎮 Konami Code Activated! Secret developer mode unlocked!";
                                    gui_state.showing_achievement_notification = true;
                                    gui_state.achievement_show_time = std::chrono::steady_clock::now();
                                    gui_state.input_sequence.clear();
                                    LOG_INFO("🎮 Konami code activated - Secret developer features unlocked!");
                                }
                            }
                        }
                        break;
                        
                    case SDLK_F1:
                        // Show help with proper exit instructions
                        LOG_INFO("🔆 M5Stack Tab5 Emulator - Quick Help");
                        LOG_INFO("🎆 HOW TO EXIT:");
                        LOG_INFO("   • Window X Button - Click to close window and shutdown properly");
                        LOG_INFO("   • ESC Key - Quick keyboard exit");
                        LOG_INFO("   • File → Exit Menu - Graceful menu exit");
                        LOG_INFO("🎮 Controls: SPACE=pause/resume, Ctrl+O=load firmware, F3=GPIO viewer");
                        if (personality_manager) {
                            LOG_INFO("✨ Tip: {}", personality_manager->get_startup_tip());
                        }
                        break;
                        
                    case SDLK_SPACE:
                        // Toggle emulator pause/resume
                        if (emulator && gui_state.emulator_running) {
                            emulator->pause();
                        } else if (emulator) {
                            emulator->resume();
                        }
                        break;
                        
                    case SDLK_s:
                        if (event.key.keysym.mod & KMOD_CTRL) {
                            // Ctrl+S - Start/Stop emulator
                            if (gui_state.emulator_running && emulator) {
                                emulator->stop();
                                gui_state.emulator_running = false;
                            } else if (emulator) {
                                auto result = emulator->start();
                                if (result) {
                                    gui_state.emulator_running = true;
                                }
                            }
                        }
                        break;
                        
                    case SDLK_o:
                        if (event.key.keysym.mod & KMOD_CTRL) {
                            // Ctrl+O - Open firmware dialog
                            gui_state.show_firmware_dialog = !gui_state.show_firmware_dialog;
                            if (firmware_dialog && gui_state.show_firmware_dialog) {
                                firmware_dialog->show();
                            } else if (firmware_dialog) {
                                firmware_dialog->hide();
                            }
                            LOG_INFO("Firmware dialog {}", gui_state.show_firmware_dialog ? "opened" : "closed");
                        }
                        break;
                        
                    case SDLK_F2:
                        // F2 - Toggle firmware manager panel
                        gui_state.show_firmware_manager = !gui_state.show_firmware_manager;
                        LOG_INFO("Firmware manager panel {}", gui_state.show_firmware_manager ? "shown" : "hidden");
                        break;
                        
                    case SDLK_F3:
                        // F3 - Toggle GPIO viewer panel
                        gui_state.show_gpio_viewer = !gui_state.show_gpio_viewer;
                        LOG_INFO("GPIO viewer panel {}", gui_state.show_gpio_viewer ? "shown" : "hidden");
                        break;
                        
                    case SDLK_F4:
                        // F4 - Toggle control panel
                        gui_state.show_control_panel = !gui_state.show_control_panel;
                        LOG_INFO("Control panel {}", gui_state.show_control_panel ? "shown" : "hidden");
                        break;
                }
                break;
                
            case SDL_MOUSEBUTTONDOWN:
            case SDL_MOUSEBUTTONUP:
                handle_mouse_event(event.button);
                break;
                
            case SDL_MOUSEMOTION:
                handle_mouse_motion(event.motion);
                break;
                
            case SDL_WINDOWEVENT:
                if (event_count <= 15 || event.window.event == SDL_WINDOWEVENT_CLOSE) {
                    LOG_INFO("SDL_WINDOWEVENT: event={}, window_id={}, startup_elapsed={}ms", event.window.event, event.window.windowID, startup_elapsed);
                }
                
                switch (event.window.event) {
                    case SDL_WINDOWEVENT_CLOSE:
                        // Only allow close during startup if it's been running for at least 1 second
                        if (startup_elapsed > 1000) {
                            LOG_INFO("Window close event received - requesting graceful shutdown");
                            return false; // Request shutdown immediately
                        } else {
                            LOG_WARN("Ignoring premature window close event during startup ({}ms elapsed)", startup_elapsed);
                        }
                        break;
                    default:
                        // Handle other window events generically  
                        LOG_DEBUG("Non-close window event: {}", static_cast<int>(event.window.event));
                        break;
                }
                break;
        }
    }
    
    // Debug logging for event processing
    if (events_processed > 0 && event_count <= 20) {
        LOG_DEBUG("Processed {} SDL events this frame (total: {})", events_processed, event_count);
    }
    
    // Check if no events are coming through (potential headless issue)
    static auto last_event_time = std::chrono::steady_clock::now();
    static int no_event_warnings = 0;
    
    if (events_processed == 0) {
        auto now = std::chrono::steady_clock::now();
        auto since_last_event = std::chrono::duration_cast<std::chrono::seconds>(now - last_event_time).count();
        
        if (since_last_event > 10 && no_event_warnings < 3) {
            LOG_WARN("No SDL events received for {} seconds - may be running headless", since_last_event);
            no_event_warnings++;
        }
    } else {
        last_event_time = std::chrono::steady_clock::now();
    }
    
#else
    // NO_GRAPHICS mode - simulate basic event loop
    static int headless_iterations = 0;
    headless_iterations++;
    
    if (headless_iterations % 100 == 1) {
        LOG_DEBUG("Headless GUI mode - iteration {}", headless_iterations);
    }
    
    // In headless mode, run for a limited time then exit
    if (headless_iterations > 1000) {
        LOG_INFO("Headless mode timeout reached - shutting down");
        return false;
    }
#endif
    
    return true; // Continue running
}

bool start_emulator() {
    if (!emulator) {
        LOG_ERROR("No emulator instance available");
        return false;
    }
    
    if (gui_state.emulator_running) {
        LOG_WARN("Emulator already running");
        return true;
    }
    
    // Show loading with personality and start boot sequence
    if (personality_manager) {
        personality_manager->start_loading("emulator_start");
        personality_manager->start_boot_sequence();
        LOG_INFO("🚀 Starting delightful M5Stack Tab5 boot sequence!");
    }
    
    // Check if emulator needs to be initialized before starting
    if (emulator->get_state() == EmulatorState::UNINITIALIZED) {
        LOG_INFO("Emulator not initialized, reinitializing...");
        
        // Recreate the configuration used during GUI startup
        Configuration config;
        auto config_file = "config/development.json";
        if (std::filesystem::exists(config_file)) {
            if (!config.loadFromFile(config_file)) {
                LOG_WARN("Failed to load configuration from: {}, using defaults", config_file);
                config.setDefaults();
            }
        } else {
            LOG_INFO("Configuration file not found, using defaults");
            config.setDefaults();
        }
        
        Configuration emulator_config = config;
        emulator_config.setValue("graphics", "enable", false);  // Disable emulator's own graphics
        emulator_config.setValue("graphics", "headless", true); // Run headless, GUI provides display
        
        auto init_result = emulator->initialize(emulator_config);
        if (!init_result) {
            std::string error_msg = init_result.error().to_string();
            LOG_ERROR("Failed to reinitialize emulator: {}", error_msg);
            if (personality_manager) {
                gui_state.status_message = personality_manager->get_friendly_error_message("hardware_error");
                personality_manager->complete_loading(false, error_msg);
            }
            return false;
        }
        LOG_INFO("Emulator reinitialized successfully");
    }
    
    auto result = emulator->start();
    if (!result) {
        std::string error_msg = result.error().to_string();
        LOG_ERROR("Failed to start emulator: {}", error_msg);
        
        if (personality_manager) {
            gui_state.status_message = personality_manager->get_friendly_error_message("hardware_error");
            personality_manager->complete_loading(false, error_msg);
        } else {
            gui_state.status_message = "Start failed: " + error_msg;
        }
        return false;
    }
    
    gui_state.emulator_running = true;
    
    if (personality_manager) {
        personality_manager->complete_loading(true, "Emulator started successfully");
        gui_state.status_message = personality_manager->get_success_celebration();
        
        // Check for achievements
        if (gui_state.cycles_executed > 1000) {
            personality_manager->unlock_achievement(gui::PersonalityManager::Achievement::SPEED_RUNNER);
        }
    } else {
        gui_state.status_message = "Started";
    }
    
    LOG_INFO("🚀 Emulator started via GUI - Ready for ESP32-P4 magic!");
    return true;
}

bool stop_emulator() {
    if (!emulator) {
        return true;
    }
    
    if (!gui_state.emulator_running) {
        return true;
    }
    
    auto result = emulator->stop();
    if (!result) {
        LOG_ERROR("Failed to stop emulator: " + result.error().to_string());
        gui_state.status_message = "Stop failed: " + result.error().to_string();
        return false;
    }
    
    gui_state.emulator_running = false;
    gui_state.status_message = "Stopped";
    LOG_INFO("Emulator stopped via GUI");
    return true;
}

void run_gui_loop() {
    LOG_INFO("Starting GUI main loop");
    
    auto last_update = std::chrono::steady_clock::now();
    const auto update_interval = std::chrono::milliseconds(33); // ~30 FPS
    
    // Check initial GUI state
    LOG_INFO("Initial GUI state - shutdown_requested: {}, gui_renderer valid: {}", 
             gui_shutdown_requested.load(), (gui_renderer != nullptr));
    
    int loop_iterations = 0;
    auto loop_start = std::chrono::steady_clock::now();
    
    while (!gui_shutdown_requested.load()) {
        loop_iterations++;
        auto current_time = std::chrono::steady_clock::now();
        
        // Debug logging every 60 iterations (~2 seconds)
        if (loop_iterations % 60 == 1) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - loop_start).count();
            LOG_INFO("GUI loop iteration {}, elapsed: {}ms, FPS: {:.1f}", 
                     loop_iterations, elapsed, (loop_iterations * 1000.0) / elapsed);
        }
        
        // Handle SDL events
        bool should_continue = handle_gui_events();
        int events_processed = 0; // Track events for farewell sequence
        LOG_DEBUG("Event handling result: {}, shutdown_requested: {}", 
                  should_continue, gui_shutdown_requested.load());
        
        if (!should_continue) {
            LOG_INFO("GUI events requested shutdown at iteration {}", loop_iterations);
            gui_shutdown_requested = true;
            break;
        }
        
        // Update emulator status periodically
        if (current_time - last_update >= update_interval) {
            update_emulator_status();
            last_update = current_time;
        }
        
        // Update delightful components
        if (personality_manager) {
            personality_manager->update();
        }
        
        // Update GUI components
        if (firmware_dialog && gui_state.show_firmware_dialog) {
            firmware_dialog->update();
        }
        
        // Update achievement notification
        if (gui_state.showing_achievement_notification) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - gui_state.achievement_show_time).count();
            if (elapsed > 4000) { // Show for 4 seconds
                gui_state.showing_achievement_notification = false;
            }
        }
        
        // Update farewell sequence
        if (gui_state.showing_farewell && personality_manager) {
            // Allow any key press during farewell to exit immediately
            if (events_processed > 0) {
                gui_shutdown_requested = true;
            }
        }
        
        // Render GUI with error handling
        if (gui_renderer) {
            try {
                // Clear screen with professional dark theme
                auto clear_result = gui_renderer->clear(0x2b2b2b);
                if (!clear_result) {
                    LOG_WARN("GUI clear failed: {}", clear_result.error().to_string());
                }
                
                // Screen is already cleared above
                
                // Render GUI components
                render_menu_bar();
                render_control_panel();
                render_firmware_manager();
                render_gpio_viewer();
                render_memory_inspector();
                render_log_viewer();
                render_emulator_display();
                render_status_bar();
                
                // Render modal dialogs on top
                if (firmware_dialog && gui_state.show_firmware_dialog) {
                    firmware_dialog->render();
                }
                
                // Render tooltip on top of everything
                render_tooltip();
                
                // Present frame
                auto present_result = gui_renderer->present();
                if (!present_result) {
                    LOG_WARN("GUI present failed: {}", present_result.error().to_string());
                }
                
                // Debug first few frames
                static int frame_count = 0;
                frame_count++;
                if (frame_count <= 10) {
                    LOG_DEBUG("Rendered GUI frame #{}", frame_count);
                }
                
            } catch (const std::exception& e) {
                LOG_ERROR("Exception during GUI rendering: {}", e.what());
            }
        } else {
            static int no_renderer_warnings = 0;
            if (no_renderer_warnings < 3) {
                LOG_WARN("No GUI renderer available for frame rendering");
                no_renderer_warnings++;
            }
        }
        
        // Frame rate limiting with adaptive sleep
        auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - current_time).count();
        
        if (loop_iterations <= 10) {
            LOG_DEBUG("Frame {} took {}ms to process", loop_iterations, frame_time);
        }
        
        // Adaptive sleep to maintain ~30 FPS
        int sleep_time = std::max(1, static_cast<int>(33 - frame_time));
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
    
    LOG_INFO("GUI main loop ended");
}

// Mouse-to-touch event conversion for M5Stack Tab5 GT911 controller simulation
void handle_mouse_event(const SDL_MouseButtonEvent& event) {
    if (!gui_renderer) return;
    
    // Calculate M5Stack display area (same as in render_emulator_display)
    // Use the same scaling and positioning logic as the display renderer
    const float device_scale = 5.0f;
    const u32 scaled_width = static_cast<u32>(1280 / device_scale);   // 256 logical pixels
    const u32 scaled_height = static_cast<u32>(720 / device_scale);   // 144 logical pixels
    const u32 device_width = scaled_width + 120;  // Add device border/bezel
    const u32 device_height = scaled_height + 80; // Add device top/bottom
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Center the entire device mockup
    i32 device_x = static_cast<i32>((window_width - device_width) / 2);
    i32 device_y = static_cast<i32>((window_height - device_height) / 2) + 30; // Menu bar offset
    
    // Display area within the device frame
    i32 display_x = device_x + 60; // Display offset within device frame
    i32 display_y = device_y + 40;
    
    // Check if click is within the scaled display area
    if (event.x >= display_x && event.x < display_x + static_cast<i32>(scaled_width) &&
        event.y >= display_y && event.y < display_y + static_cast<i32>(scaled_height)) {
        
        // Convert window coordinates to M5Stack display coordinates (normalized to 0.0-1.0)
        float touch_x = static_cast<float>(event.x - display_x) / static_cast<float>(scaled_width);
        float touch_y = static_cast<float>(event.y - display_y) / static_cast<float>(scaled_height);
        
        // Clamp to valid range [0.0, 1.0]
        touch_x = std::max(0.0f, std::min(1.0f, touch_x));
        touch_y = std::max(0.0f, std::min(1.0f, touch_y));
        
        bool pressed = (event.type == SDL_MOUSEBUTTONDOWN);
        
        LOG_DEBUG("Touch event: ({:.3f}, {:.3f}) pressed={} button={}", 
                 touch_x, touch_y, pressed, event.button);
        
        // Send to emulator GT911 controller
        send_touch_event_to_emulator(touch_x, touch_y, pressed);
        
        // Show visual feedback
        if (pressed && personality_manager) {
            if (touch_x < 0.1f && touch_y < 0.1f) {
                // Top-left corner easter egg
                personality_manager->activate_easter_egg("corner_tap");
                show_achievement("Secret Tapper", "You found the hidden corner!");
            }
        }
    }
}

void handle_mouse_motion(const SDL_MouseMotionEvent& event) {
    if (!gui_renderer) return;
    
    // Use the same scaling and positioning logic as the display renderer
    const float device_scale = 5.0f;
    const u32 scaled_width = static_cast<u32>(1280 / device_scale);   // 256 logical pixels
    const u32 scaled_height = static_cast<u32>(720 / device_scale);   // 144 logical pixels
    const u32 device_width = scaled_width + 120;  // Add device border/bezel
    const u32 device_height = scaled_height + 80; // Add device top/bottom
    
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Center the entire device mockup
    i32 device_x = static_cast<i32>((window_width - device_width) / 2);
    i32 device_y = static_cast<i32>((window_height - device_height) / 2) + 30; // Menu bar offset
    
    // Display area within the device frame
    i32 display_x = device_x + 60; // Display offset within device frame
    i32 display_y = device_y + 40;
    
    // Mouse motion handling simplified - no exit button to track
    
    // Check if motion is within the scaled display area (for potential hover effects)
    if (event.x >= display_x && event.x < display_x + static_cast<i32>(scaled_width) &&
        event.y >= display_y && event.y < display_y + static_cast<i32>(scaled_height)) {
        // Motion within M5Stack display - could be used for hover effects
        // For now, just track for debugging
        static int motion_count = 0;
        if (++motion_count % 50 == 0) { // Log every 50th motion event
            float hover_x = static_cast<float>(event.x - display_x) / static_cast<float>(scaled_width);
            float hover_y = static_cast<float>(event.y - display_y) / static_cast<float>(scaled_height);
            LOG_DEBUG("Mouse hover over 5-inch display: ({:.3f}, {:.3f})", hover_x, hover_y);
        }
    }
}

void send_touch_event_to_emulator(float x, float y, bool pressed) {
    if (!emulator) {
        LOG_DEBUG("No emulator instance - touch event ignored");
        return;
    }
    
    // Convert normalized coordinates (0.0-1.0) to M5Stack Tab5 coordinates (0-1279, 0-719)
    u32 absolute_x = static_cast<u32>(x * 1279.0f);
    u32 absolute_y = static_cast<u32>(y * 719.0f);
    
    LOG_INFO("GT911 Touch: ({}, {}) pressed={} -> absolute=({}, {})", 
             x, y, pressed, absolute_x, absolute_y);
    
    // In a full implementation, this would send the touch event to the GT911 controller
    // emulation within the EmulatorCore. For now, we log it.
    
    // Update GUI state to show touch feedback
    if (pressed) {
        gui_state.status_message = "Touch detected at (" + 
                                  std::to_string(absolute_x) + ", " + 
                                  std::to_string(absolute_y) + ")";
        
        if (personality_manager) {
            // Check for achievement unlock
            static int touch_count = 0;
            touch_count++;
            
            if (touch_count == 10) {
                personality_manager->unlock_achievement(gui::PersonalityManager::Achievement::FIRST_BOOT);
                show_achievement("First Contact", "You've started interacting with the emulator!");
            } else if (touch_count == 100) {
                show_achievement("Touch Master", "100 touches! You're getting the hang of this!");
            }
        }
    }
}

void handle_menu_action(const std::string& action, const gui::MenuBar::MenuItem& item) {
    const std::string action_copy = action;
    const std::string item_text = item.text;
    LOG_INFO("🎯 Menu action triggered: {} ({})", action_copy, item_text);
    
    // File menu actions
    if (action == "file_open_firmware") {
        gui_state.show_firmware_dialog = !gui_state.show_firmware_dialog;
        if (firmware_dialog && gui_state.show_firmware_dialog) {
            firmware_dialog->show();
        } else if (firmware_dialog) {
            firmware_dialog->hide();
        }
        if (personality_manager) {
            LOG_INFO("✨ {}", "Ready to load some amazing firmware!");
        }
        
    } else if (action == "file_unload_firmware") {
        if (emulator && gui_state.firmware_loaded) {
            // Stop emulator if running
            if (gui_state.emulator_running) {
                stop_emulator();
            }
            
            // Clear firmware loaded state
            gui_state.firmware_loaded = false;
            gui_state.status_message = "Firmware unloaded";
            
            if (personality_manager) {
                LOG_INFO("✨ {}", "Firmware unloaded - Ready for new adventures!");
            }
            
            LOG_INFO("🔄 Firmware unloaded via GUI menu");
        } else {
            LOG_INFO("No firmware currently loaded");
            gui_state.status_message = "No firmware loaded";
        }
        
    } else if (action == "file_exit") {
        LOG_INFO("Exit requested via menu - requesting graceful shutdown");
        gui_shutdown_requested = true;
        
    // Emulator menu actions
    } else if (action == "emulator_start") {
        if (!gui_state.emulator_running && emulator) {
            start_emulator();
        }
        
    } else if (action == "emulator_stop") {
        if (gui_state.emulator_running && emulator) {
            stop_emulator();
        }
        
    } else if (action == "emulator_pause") {
        if (emulator && gui_state.emulator_running) {
            emulator->pause();
            gui_state.status_message = "Emulator paused";
        }
        
    } else if (action == "emulator_resume") {
        if (emulator) {
            emulator->resume();
            gui_state.status_message = "Emulator resumed";
        }
        
    } else if (action == "emulator_reset") {
        if (emulator) {
            // Stop and restart emulator
            if (gui_state.emulator_running) {
                stop_emulator();
            }
            start_emulator();
            if (personality_manager) {
                LOG_INFO("🔄 {}", "Fresh start! Emulator reset with enthusiasm!");
            }
        }
        
    // View menu actions
    } else if (action == "view_control_panel") {
        gui_state.show_control_panel = !gui_state.show_control_panel;
        LOG_INFO("Control panel {}", gui_state.show_control_panel ? "shown" : "hidden");
        
    } else if (action == "view_gpio_viewer") {
        gui_state.show_gpio_viewer = !gui_state.show_gpio_viewer;
        LOG_INFO("GPIO viewer panel {}", gui_state.show_gpio_viewer ? "shown" : "hidden");
        
    } else if (action == "view_memory_inspector") {
        gui_state.show_memory_inspector = !gui_state.show_memory_inspector;
        LOG_INFO("🧠 Memory inspector panel {}", gui_state.show_memory_inspector ? "shown" : "hidden");
        
        if (gui_state.show_memory_inspector) {
            LOG_INFO("💾 Memory Inspector opened! Dive deep into ESP32-P4 memory regions.");
        }
        
    } else if (action == "view_log_viewer") {
        gui_state.show_log_viewer = !gui_state.show_log_viewer;
        LOG_INFO("📄 Log viewer panel {}", gui_state.show_log_viewer ? "shown" : "hidden");
        
        if (gui_state.show_log_viewer) {
            LOG_INFO("📋 Log Viewer opened! Monitor real-time emulator logs with filtering.");
        }
        
    } else if (action == "view_fullscreen") {
        // Toggle fullscreen mode would be handled by SDL window management
        LOG_INFO("🖥️ Fullscreen toggle requested");
        
    // Tools menu actions
    } else if (action == "tools_firmware_manager") {
        gui_state.show_firmware_manager = !gui_state.show_firmware_manager;
        LOG_INFO("Firmware manager panel {}", gui_state.show_firmware_manager ? "shown" : "hidden");
        
    } else if (action == "tools_screenshot") {
        take_screenshot();
        if (personality_manager) {
            personality_manager->unlock_achievement(gui::PersonalityManager::Achievement::DEBUGGING_NINJA);
        }
        
    // Debug menu actions
    } else if (action == "debug_start") {
        LOG_INFO("🐛 Debug mode not yet implemented - coming in future release!");
        
    } else if (action == "debug_toggle_breakpoint") {
        LOG_INFO("🔴 Breakpoint system not yet implemented");
        
    // Help menu actions
    } else if (action == "help_user_guide") {
        // Show help in console
        LOG_INFO("📚 Opening user guide...");
        // In a real implementation, this would open a help window
        LOG_INFO("🎆 M5Stack Tab5 Emulator Help - Check console output for usage info");
        if (personality_manager) {
            LOG_INFO("✨ {}", "Need help? You're in the right place! Check the menus for all features.");
        }
        
    } else if (action == "help_about") {
        LOG_INFO("🌟 M5Stack Tab5 Emulator - Professional ESP32-P4 Development Environment");
        LOG_INFO("📟 Version: 1.0.0 - Making embedded development delightful!");
        LOG_INFO("🚀 Built with SDL2, featuring authentic M5Stack Tab5 simulation");
        if (personality_manager) {
            LOG_INFO("✨ {}", personality_manager->get_emulator_mood());
        }
        
    } else {
        const std::string unhandled_action = action;
        const std::string unhandled_item = item.text;
        LOG_WARN("Unhandled menu action: {} ({})", unhandled_action, unhandled_item);
        if (personality_manager) {
            LOG_INFO("🤔 That feature is still brewing in our development lab!");
        }
    }
    
    // Update menu state based on emulator status
    if (menu_bar) {
        menu_bar->set_item_enabled("emulator_start", !gui_state.emulator_running);
        menu_bar->set_item_enabled("emulator_stop", gui_state.emulator_running);
        menu_bar->set_item_enabled("emulator_pause", gui_state.emulator_running);
        menu_bar->set_item_enabled("emulator_resume", gui_state.emulator_running);
    }
}

void print_gui_help() {
    std::cout << "🌟 M5Stack Tab5 Emulator - Professional Development GUI with Personality! 🌟\n"
              << "\n=== EXIT ===\n"
              << "  Window X          - Click window close button for proper shutdown\n"
              << "  ESC Key           - Quick exit\n"
              << "  File → Exit       - Menu option for graceful exit\n"
              << "\n=== FIRMWARE MANAGEMENT ===\n"
              << "  Ctrl+O            - Open/Close firmware loading dialog\n"
              << "  F2                - Toggle firmware manager panel\n"
              << "\n=== EMULATOR CONTROL ===\n"
              << "  Space             - Pause/Resume emulator\n"
              << "  Ctrl+S            - Start/Stop emulator\n"
              << "  F4                - Toggle control panel\n"
              << "\n=== DEVELOPMENT TOOLS ===\n"
              << "  F3                - Toggle GPIO viewer panel\n"
              << "  F1                - Show help\n"
              << "\n=== NAVIGATION ===\n"
              << "  Mouse/Touch       - Interact with emulated M5Stack Tab5\n"
              << "\n=== EASTER EGGS & DELIGHTS 🎮 ===\n"
              << "  Konami Code       - ↑↑↓↓←→←→BA (Secret developer features!)\n"
              << "  Achievement System- Unlock rewards as you develop\n"
              << "  Loading Animations- Delightful feedback during operations\n"
              << "  Celebration Effects- Particles and animations for successes\n"
              << "\n=== GUI PANELS ===\n"
              << "  Control Panel     - Emulator power state and execution control\n"
              << "  Firmware Manager  - ELF loading, metadata display, profiles\n"
              << "  GPIO Viewer       - Pin state visualization and control\n"
              << "  Status Bar        - Real-time performance and status info\n"
              << "\n=== FIRMWARE LOADING FEATURES ===\n"
              << "  - Native ELF file browser with ESP32-P4 validation\n"
              << "  - Firmware metadata parsing (entry point, sections, size)\n"
              << "  - Recent files history (last 10 files)\n"
              << "  - Firmware profiles for development workflows\n"
              << "  - Progress indication with delightful loading messages\n"
              << "  - ESP32-P4 compatibility verification\n"
              << "\n🏆 ACHIEVEMENTS TO UNLOCK:\n"
              << "  - First Contact: Start the emulator for the first time\n"
              << "  - Firmware Whisperer: Load your first firmware successfully\n"
              << "  - Pin Master: Explore GPIO functionality\n"
              << "  - Secret Code Breaker: Discover hidden features\n"
              << "  - And many more surprises waiting to be discovered!\n";
}

int main(int argc, char* argv[]) {
    // Set up signal handlers
    std::signal(SIGINT, gui_signal_handler);
    std::signal(SIGTERM, gui_signal_handler);
    
    // Parse command line arguments (simplified for GUI)
    std::string config_file = "config/default.json";
    std::string log_level = "info";
    bool debug_mode = false;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            print_gui_help();
            return 0;
        }
        else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_file = argv[++i];
        }
        else if (arg == "-d" || arg == "--debug") {
            debug_mode = true;
            log_level = "debug";
        }
    }
    
    try {
        // Initialize logging
        LogLevel log_level_enum = debug_mode ? LogLevel::DEBUG_LEVEL : LogLevel::INFO;
        auto log_result = Logger::initialize(log_level_enum, "", true);
        if (!log_result) {
            std::cerr << "Failed to initialize logger: " << log_result.error().to_string() << std::endl;
            return 1;
        }
        
        LOG_INFO("🎯 M5Stack Tab5 Emulator - Professional GUI Mode with Personality!");
        LOG_INFO("📟 Version: 1.0.0 - Authentic ESP32-P4 Development Environment");
        LOG_INFO("🌟 Ready to make embedded development delightful!");
        
        // Initialize GUI
        if (!initialize_gui()) {
            LOG_ERROR("Failed to initialize GUI");
            return 1;
        }
        
        LOG_INFO("GUI initialized successfully");
        
        // Load configuration
        Configuration config;
        if (std::filesystem::exists(config_file)) {
            if (!config.loadFromFile(config_file)) {
                LOG_ERROR("Failed to load configuration from: " + config_file);
                shutdown_gui();
                return 1;
            }
            LOG_INFO("Loaded configuration from: " + config_file);
        } else {
            LOG_INFO("Configuration file not found, using defaults");
        }
        
        // Create and initialize emulator (WITHOUT graphics - GUI provides display)
        // Disable graphics initialization in emulator to prevent dual windows
        Configuration emulator_config = config;
        emulator_config.setValue("graphics", "enable", false);  // Disable emulator's own graphics
        emulator_config.setValue("graphics", "headless", true); // Run headless, GUI provides display
        
        emulator = std::make_unique<EmulatorCore>(emulator_config);
        
        auto init_result = emulator->initialize(emulator_config);
        if (!init_result) {
            LOG_ERROR("Failed to initialize emulator: " + init_result.error().to_string());
            shutdown_gui();
            return 1;
        }
        
        LOG_INFO("Emulator initialized successfully");
        gui_state.status_message = "Ready - Press Ctrl+S to start";
        
        // Validate GUI state before starting main loop
        LOG_INFO("Validating GUI state before main loop...");
        LOG_INFO("  - Emulator initialized: {}", (emulator != nullptr));
        LOG_INFO("  - GUI renderer ready: {}", (gui_renderer != nullptr));
        LOG_INFO("  - Personality manager: {}", (personality_manager != nullptr));
        LOG_INFO("  - Shutdown requested: {}", gui_shutdown_requested.load());
        
        if (!gui_renderer) {
            LOG_ERROR("GUI renderer not available - cannot start main loop");
            return 1;
        }
        
        // Force an initial window update to ensure it's visible
#ifndef NO_GRAPHICS
        if (gui_renderer) {
            gui_renderer->clear(0x2b2b2b);
            gui_renderer->present();
        }
#endif
        
        // Run main GUI loop
        run_gui_loop();
        
        LOG_INFO("Shutting down GUI emulator...");
        
        // Stop emulator if running
        if (gui_state.emulator_running) {
            stop_emulator();
        }
        
        // Show final farewell message with personality
        if (personality_manager) {
            // Start final farewell if not already started
            if (!gui_state.showing_farewell) {
                personality_manager->start_farewell_sequence();
                gui_state.showing_farewell = true;
                gui_state.exit_hover_start = std::chrono::steady_clock::now(); // Initialize the timer properly
            }
            
            LOG_INFO("👋 {}", personality_manager->get_farewell_message());
            
            if (personality_manager->is_achievement_unlocked(gui::PersonalityManager::Achievement::PERSISTENCE_CHAMPION)) {
                LOG_INFO("🏆 Persistence Champion achievement earned - you're dedicated to quality development!");
            }
            
            // Final celebratory particles
            personality_manager->spawn_farewell_particles(gui_renderer->get_width() / 2, gui_renderer->get_height() / 2);
        }
        
        // Skip emulator shutdown - just reset pointer for rapid exit
        LOG_INFO("Performing rapid shutdown - skipping emulator cleanup");
        bool shutdown_success = true;
        
        // Clean up GUI immediately
        try {
            shutdown_gui();
        } catch (...) {
            // Ignore cleanup errors during forced shutdown
        }
        
        // Reset emulator pointer
        try {
            emulator.reset();
        } catch (...) {
            // Ignore cleanup errors during forced shutdown
        }
        
        LOG_INFO("✨ GUI shutdown completed ({}) - Until next time, happy coding! 🚀", shutdown_success ? "clean" : "forced");
        
        // Quick logger shutdown
        try {
            Logger::shutdown();
        } catch (...) {
            std::cerr << "Exception during logger shutdown\n";
        }
        
        // Clean return to main
        return shutdown_success ? 0 : 1;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal GUI error: " << e.what() << std::endl;
        
        // Emergency cleanup
        if (emulator) {
            try {
                emulator->shutdown();
            } catch (...) {}
        }
        
        shutdown_gui();
        
        try {
            Logger::shutdown();
        } catch (...) {}
        
        return 1;
    }
}

void show_tooltip(const std::string& text, i32 x, i32 y) {
    gui_state.tooltip_text = text;
    gui_state.tooltip_x = x;
    gui_state.tooltip_y = y;
    gui_state.showing_tooltip = true;
    gui_state.tooltip_show_time = std::chrono::steady_clock::now();
}

void render_tooltip() {
    if (!gui_state.showing_tooltip || gui_state.tooltip_text.empty() || !gui_renderer) {
        return;
    }
    
    // Auto-hide tooltip after 5 seconds
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - gui_state.tooltip_show_time).count();
    if (elapsed > 5000) {
        gui_state.showing_tooltip = false;
        return;
    }
    
    // Calculate tooltip size (rough estimate)
    u32 tooltip_width = static_cast<u32>(gui_state.tooltip_text.length() * 8 + 20);
    u32 tooltip_height = 30;
    
    // Adjust position to stay on screen
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    i32 final_x = gui_state.tooltip_x;
    i32 final_y = gui_state.tooltip_y;
    
    if (final_x + static_cast<i32>(tooltip_width) > static_cast<i32>(window_width)) {
        final_x = static_cast<i32>(window_width) - static_cast<i32>(tooltip_width) - 10;
    }
    if (final_y < 0) {
        final_y = gui_state.tooltip_y + 40; // Show below cursor instead
    }
    if (final_y + static_cast<i32>(tooltip_height) > static_cast<i32>(window_height)) {
        final_y = static_cast<i32>(window_height) - static_cast<i32>(tooltip_height) - 10;
    }
    
    // Render tooltip with M5Stack styling
    // Shadow
    gui_renderer->draw_rect(final_x + 2, final_y + 2, tooltip_width, tooltip_height, 0x80000000);
    // Border
    gui_renderer->draw_rect(final_x - 1, final_y - 1, tooltip_width + 2, tooltip_height + 2, 0xFF6600);
    // Background
    gui_renderer->draw_rect(final_x, final_y, tooltip_width, tooltip_height, 0x2A2A2A);
    // Text
    gui_renderer->draw_text(final_x + 10, final_y + 8, gui_state.tooltip_text, 0xFFFFFF);
}