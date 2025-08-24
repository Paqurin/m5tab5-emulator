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

// GUI State Management
struct GuiState {
    bool emulator_running = false;
    bool show_menu = true;
    bool show_control_panel = true;
    bool show_firmware_manager = true;
    bool show_gpio_viewer = true;
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
void trigger_loading_with_personality(const std::string& operation);
void show_achievement(const std::string& title, const std::string& message);
void handle_mouse_event(const SDL_MouseButtonEvent& event);
void handle_mouse_motion(const SDL_MouseMotionEvent& event);
void send_touch_event_to_emulator(float x, float y, bool pressed);
void handle_menu_action(const std::string& action, const gui::MenuBar::MenuItem& item);

void gui_signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        bool expected = false;
        if (!signal_handled.compare_exchange_strong(expected, true)) {
            return;
        }
        
        std::cout << "\nGUI shutdown requested...\n";
        gui_shutdown_requested = true;
        
        auto& shutdown_mgr = utils::ShutdownManager::instance();
        shutdown_mgr.request_shutdown();
        
        // Stop emulator if running
        if (emulator && gui_state.emulator_running) {
            try {
                auto result = emulator->stop();
                if (!result) {
                    std::cerr << "Failed to stop emulator: " << result.error().to_string() << std::endl;
                }
                gui_state.emulator_running = false;
            } catch (...) {
                std::cerr << "Exception during emergency stop\n";
            }
        }
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
            // Start the delightful boot sequence
            personality_manager->start_boot_sequence();
            LOG_INFO("🚀 Starting delightful M5Stack Tab5 boot sequence!");
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
        } dummy_window;
        
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
        menu_bar->render();
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
    
    // Clear with M5Stack-themed gradient background - make it more visible
    auto clear_result = gui_renderer->clear(0x003366); // Brighter blue background
    if (!clear_result) {
        LOG_ERROR("Failed to clear screen: {}", clear_result.error().to_string());
        return;
    }
    
    static int render_call_count = 0;
    render_call_count++;
    if (render_call_count <= 10) {
        LOG_INFO("render_emulator_display() call #{}", render_call_count);
    }
    
    // Show boot sequence if still active
    if (personality_manager && gui_state.showing_boot_sequence && !personality_manager->is_boot_complete()) {
        render_boot_sequence();
        return;
    }
    
    gui_state.showing_boot_sequence = false;
    
    // Draw some test rectangles to verify rendering is working
    static bool drew_test_rectangles = false;
    if (render_call_count <= 5) {
        LOG_INFO("Drawing test rectangles for render verification");
        
        // Draw large, obvious colored rectangles
        auto test_result = gui_renderer->draw_rect(50, 50, 200, 100, 0xFF0000); // Red rectangle
        if (test_result) {
            LOG_INFO("Successfully drew red test rectangle");
        } else {
            LOG_ERROR("Failed to draw red test rectangle: {}", test_result.error().to_string());
        }
        
        gui_renderer->draw_rect(300, 50, 200, 100, 0x00FF00); // Green rectangle
        gui_renderer->draw_rect(550, 50, 200, 100, 0x0000FF); // Blue rectangle
        drew_test_rectangles = true;
    }
    
    // Draw M5Stack Tab5 emulator display area (1280x720 centered)
    u32 display_width = 1280;
    u32 display_height = 720;
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    // Center the display in the window
    i32 display_x = static_cast<i32>((window_width - display_width) / 2);
    i32 display_y = static_cast<i32>((window_height - display_height) / 2);
    
    // Ensure minimum positioning
    if (display_x < 10) display_x = 10;
    if (display_y < 50) display_y = 50; // Leave space for menu bar
    
    // Draw display border (M5Stack style)
    gui_renderer->draw_rect(display_x - 2, display_y - 2, display_width + 4, display_height + 4, 0x333333);
    
    // Draw the actual display area
    if (gui_state.emulator_running) {
        // When emulator is running, show a gradient pattern
        gui_renderer->draw_rect(display_x, display_y, display_width, display_height, 0x000033);
        
        // Draw some activity indicators
        u32 indicator_color = 0x00FF00; // Green for running
        gui_renderer->draw_rect(display_x + 10, display_y + 10, 200, 30, indicator_color);
        gui_renderer->draw_text(display_x + 15, display_y + 15, "ESP32-P4 RUNNING", 0x000000);
        
        // Draw some mock display content
        gui_renderer->draw_rect(display_x + 50, display_y + 100, 400, 200, 0x444444);
        gui_renderer->draw_text(display_x + 60, display_y + 110, "M5Stack Tab5 Emulator Display", 0xFFFFFF);
        gui_renderer->draw_text(display_x + 60, display_y + 130, "Resolution: 1280x720", 0xCCCCCC);
        gui_renderer->draw_text(display_x + 60, display_y + 150, "Touch: GT911 Controller", 0xCCCCCC);
        gui_renderer->draw_text(display_x + 60, display_y + 170, "Status: Emulator Active", 0x00FF00);
        
        // Draw status info
        std::string status_text = "Cycles: " + std::to_string(gui_state.cycles_executed);
        gui_renderer->draw_text(display_x + 60, display_y + 220, status_text, 0xFFFF00);
        
    } else {
        // When emulator is stopped, show idle screen
        gui_renderer->draw_rect(display_x, display_y, display_width, display_height, 0x111111);
        
        // M5Stack logo area (simulated)
        gui_renderer->draw_rect(display_x + display_width/2 - 100, display_y + display_height/2 - 50, 200, 100, 0x0066CC);
        gui_renderer->draw_text(display_x + display_width/2 - 80, display_y + display_height/2 - 30, "M5Stack Tab5", 0xFFFFFF);
        gui_renderer->draw_text(display_x + display_width/2 - 90, display_y + display_height/2 - 10, "ESP32-P4 Emulator", 0xCCCCCC);
        
        // Status indicator
        gui_renderer->draw_rect(display_x + 10, display_y + 10, 150, 30, 0xFF6600);
        gui_renderer->draw_text(display_x + 15, display_y + 15, "READY - Press Ctrl+S", 0xFFFFFF);
        
        // Show personality message if available
        if (personality_manager && !gui_state.emulator_running) {
            std::string idle_msg = personality_manager->get_idle_message();
            gui_renderer->draw_text(display_x + 60, display_y + display_height - 100, idle_msg, 0x888888);
        }
    }
    
    // Render celebration particles if any are active
    render_celebration_particles();
    
    // Show achievement notification if active
    if (gui_state.showing_achievement_notification) {
        render_achievement_notification();
    }
}

void render_status_bar() {
    if (!gui_state.show_status_bar) return;
    
    LOG_DEBUG("Rendering status bar: {}", gui_state.status_message);
    
    // Display firmware status
    if (gui_state.firmware_loaded) {
        LOG_DEBUG("  Firmware: {} loaded", gui_state.loaded_firmware_name);
    } else {
        LOG_DEBUG("  No firmware loaded");
    }
    
    // Display execution stats
    LOG_DEBUG("  Cycles: {}, Speed: {:.2f} MHz", 
              gui_state.cycles_executed, gui_state.execution_speed);
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
    
    LOG_INFO("🏆 {} - {}", title, message);
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
                LOG_INFO("SDL_QUIT event #{} received (startup_elapsed={}ms, ignore={})", 
                         quit_event_count, startup_elapsed, ignore_quit_events);
                
                // Ignore quit events for the first 2 seconds to let window stabilize
                if (ignore_quit_events && quit_event_count <= 3) {
                    LOG_WARN("Ignoring SDL_QUIT event #{} during startup stabilization period", quit_event_count);
                    break; // Ignore this quit event
                } else {
                    LOG_INFO("Processing SDL_QUIT event - requesting graceful shutdown");
                    return false; // Request shutdown
                }
                
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        return false; // ESC to quit
                        
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
                        
                    case SDLK_F1:
                        // Show help
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
                LOG_DEBUG("SDL_WINDOWEVENT: event={}, startup_elapsed={}ms", event.window.event, startup_elapsed);
                
                switch (event.window.event) {
                    case SDL_WINDOWEVENT_CLOSE:
                        LOG_INFO("Window close event received (startup_elapsed={}ms, ignore={})", startup_elapsed, ignore_quit_events);
                        
                        // Ignore window close events during startup stabilization
                        if (ignore_quit_events) {
                            LOG_WARN("Ignoring SDL_WINDOWEVENT_CLOSE during startup stabilization period");
                            break; // Ignore this close event
                        } else {
                            LOG_INFO("Processing window close event - requesting shutdown");
                            return false;
                        }
                        break;
                    default:
                        // Handle other window events generically
                        LOG_DEBUG("Window event: {}", static_cast<int>(event.window.event));
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
    
    // Show loading with personality
    if (personality_manager) {
        personality_manager->start_loading("emulator_start");
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
        
        // Render GUI with error handling
        if (gui_renderer) {
            try {
                // Clear screen with professional dark theme
                auto clear_result = gui_renderer->clear(0x2b2b2b);
                if (!clear_result) {
                    LOG_WARN("GUI clear failed: {}", clear_result.error().to_string());
                }
                
                // Render GUI components
                render_menu_bar();
                render_control_panel();
                render_firmware_manager();
                render_gpio_viewer();
                render_emulator_display();
                render_status_bar();
                
                // Render modal dialogs on top
                if (firmware_dialog && gui_state.show_firmware_dialog) {
                    firmware_dialog->render();
                }
                
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
    u32 display_width = 1280;
    u32 display_height = 720;
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    i32 display_x = static_cast<i32>((window_width - display_width) / 2);
    i32 display_y = static_cast<i32>((window_height - display_height) / 2);
    
    if (display_x < 10) display_x = 10;
    if (display_y < 50) display_y = 50;
    
    // Check if click is within display area
    if (event.x >= display_x && event.x < display_x + static_cast<i32>(display_width) &&
        event.y >= display_y && event.y < display_y + static_cast<i32>(display_height)) {
        
        // Convert window coordinates to M5Stack display coordinates
        float touch_x = static_cast<float>(event.x - display_x) / static_cast<float>(display_width);
        float touch_y = static_cast<float>(event.y - display_y) / static_cast<float>(display_height);
        
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
    // For now, just log motion over the display area
    if (!gui_renderer) return;
    
    // Same display area calculation
    u32 display_width = 1280;
    u32 display_height = 720;
    u32 window_width = gui_renderer->get_width();
    u32 window_height = gui_renderer->get_height();
    
    i32 display_x = static_cast<i32>((window_width - display_width) / 2);
    i32 display_y = static_cast<i32>((window_height - display_height) / 2);
    
    if (display_x < 10) display_x = 10;
    if (display_y < 50) display_y = 50;
    
    // Check if motion is within display area (for potential hover effects)
    if (event.x >= display_x && event.x < display_x + static_cast<i32>(display_width) &&
        event.y >= display_y && event.y < display_y + static_cast<i32>(display_height)) {
        // Motion within M5Stack display - could be used for hover effects
        // For now, just track for debugging
        static int motion_count = 0;
        if (++motion_count % 50 == 0) { // Log every 50th motion event
            float hover_x = static_cast<float>(event.x - display_x) / static_cast<float>(display_width);
            float hover_y = static_cast<float>(event.y - display_y) / static_cast<float>(display_height);
            LOG_DEBUG("Mouse hover over display: ({:.3f}, {:.3f})", hover_x, hover_y);
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
        
    } else if (action == "file_exit") {
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
        LOG_INFO("📊 Memory inspector not yet implemented - coming soon!");
        
    } else if (action == "view_log_viewer") {
        LOG_INFO("📄 Log viewer not yet implemented - coming soon!");
        
    } else if (action == "view_fullscreen") {
        // Toggle fullscreen mode would be handled by SDL window management
        LOG_INFO("🖥️ Fullscreen toggle requested");
        
    // Tools menu actions
    } else if (action == "tools_firmware_manager") {
        gui_state.show_firmware_manager = !gui_state.show_firmware_manager;
        LOG_INFO("Firmware manager panel {}", gui_state.show_firmware_manager ? "shown" : "hidden");
        
    } else if (action == "tools_screenshot") {
        LOG_INFO("📸 Screenshot functionality not yet implemented");
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
              << "  ESC               - Exit GUI\n"
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
        
        // Create and initialize emulator
        emulator = std::make_unique<EmulatorCore>(config);
        
        auto init_result = emulator->initialize(config);
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
        
        // Show farewell message with personality
        if (personality_manager) {
            LOG_INFO("👋 {}", "Thanks for using M5Stack Tab5 Emulator! See you next time!");
            if (personality_manager->is_achievement_unlocked(gui::PersonalityManager::Achievement::PERSISTENCE_CHAMPION)) {
                LOG_INFO("🏆 You've earned the Persistence Champion achievement - way to go!");
            }
        }
        
        // Shutdown with timeout protection
        bool shutdown_success = false;
        try {
            auto future = std::async(std::launch::async, [&emulator]() -> Result<void> {
                return emulator->shutdown();
            });
            
            if (future.wait_for(std::chrono::milliseconds(3000)) == std::future_status::ready) {
                auto shutdown_result = future.get();
                if (!shutdown_result) {
                    LOG_WARN("Emulator shutdown warning: {}", shutdown_result.error().to_string());
                } else {
                    shutdown_success = true;
                }
            } else {
                LOG_ERROR("Emulator shutdown timed out, forcing exit");
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during emulator shutdown: {}", e.what());
        }
        
        // Clean up GUI
        shutdown_gui();
        
        // Reset emulator pointer
        emulator.reset();
        
        LOG_INFO("✨ GUI shutdown completed ({}) - Until next time, happy coding! 🚀", shutdown_success ? "clean" : "forced");
        
        // Shutdown logger with timeout
        try {
            auto future = std::async(std::launch::async, []() {
                Logger::shutdown();
            });
            
            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
                std::cerr << "Logger shutdown timed out\n";
            }
        } catch (...) {
            std::cerr << "Exception during logger shutdown\n";
        }
        
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