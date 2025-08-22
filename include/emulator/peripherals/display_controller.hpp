#pragma once

#include "emulator/peripherals/peripheral_base.hpp"
#include "emulator/graphics/framebuffer.hpp"
#include "emulator/graphics/touch_input.hpp"

#include <SDL2/SDL.h>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>

namespace m5tab5::emulator {

/**
 * @brief Display controller emulating the 5-inch 1280×720 MIPI-DSI display
 * 
 * Features:
 * - SDL2-based rendering
 * - GT911 touch controller emulation
 * - Real-time display updates
 * - Color space conversion
 * - Display configuration registers
 */
class DisplayController : public PeripheralBase {
public:
    struct DisplayConfig {
        uint32_t width = DISPLAY_WIDTH;
        uint32_t height = DISPLAY_HEIGHT;
        uint32_t refresh_rate = 60; // Hz
        bool enable_vsync = true;
        bool enable_touch = true;
        float scale_factor = 1.0f;
        std::string window_title = "M5Tab5 Emulator";
    };

    // Display registers (MIPI-DSI controller)
    enum class Register : Address {
        CTRL = 0x40020000,          // Display control
        STATUS = 0x40020004,        // Display status
        FRAMEBUFFER_ADDR = 0x40020008, // Framebuffer base address
        WIDTH = 0x4002000C,         // Display width
        HEIGHT = 0x40020010,        // Display height
        FORMAT = 0x40020014,        // Pixel format
        VSYNC_COUNT = 0x40020018,   // VSYNC counter
        HSYNC_COUNT = 0x4002001C,   // HSYNC counter
        BACKLIGHT = 0x40020020,     // Backlight control
        POWER = 0x40020024,         // Power management
        
        // Touch controller registers (GT911)
        TOUCH_STATUS = 0x40021000,  // Touch status
        TOUCH_POINT_COUNT = 0x40021004, // Number of touch points
        TOUCH_POINT_0 = 0x40021008, // First touch point data
        TOUCH_POINT_1 = 0x40021018, // Second touch point data (multi-touch)
        TOUCH_CONFIG = 0x40021020,  // Touch configuration
        TOUCH_CALIBRATION = 0x40021024, // Touch calibration data
    };

    // Control register bits
    enum ControlBits : uint32_t {
        ENABLE = 1 << 0,
        RESET = 1 << 1,
        VSYNC_ENABLE = 1 << 2,
        HSYNC_ENABLE = 1 << 3,
        BACKLIGHT_ENABLE = 1 << 4,
        TOUCH_ENABLE = 1 << 5,
        DMA_ENABLE = 1 << 6,
        INTERRUPT_ENABLE = 1 << 7
    };

    // Pixel formats
    enum class PixelFormat : uint32_t {
        RGB565 = 0,
        RGB888 = 1,
        ARGB8888 = 2,
        MONO = 3
    };

    explicit DisplayController(const DisplayConfig& config);
    ~DisplayController();

    // PeripheralBase implementation
    std::string getName() const override { return "display"; }
    EmulatorError initialize() override;
    EmulatorError reset() override;
    EmulatorError tick(ClockCycle cycle) override;
    
    EmulatorError readRegister(Address address, uint32_t& value) override;
    EmulatorError writeRegister(Address address, uint32_t value) override;
    
    std::vector<Address> getRegisterAddresses() const override;
    std::vector<uint32_t> getInterruptIds() const override;

    // Display operations
    EmulatorError updateDisplay();
    EmulatorError setFramebuffer(Address fb_address);
    EmulatorError setPixelFormat(PixelFormat format);
    EmulatorError setBacklight(uint8_t brightness); // 0-255

    // Touch input simulation
    EmulatorError injectTouchEvent(int x, int y, bool pressed);
    EmulatorError injectMultiTouch(const std::vector<TouchPoint>& points);

    // SDL integration
    SDL_Window* getSDLWindow() { return window_; }
    SDL_Renderer* getSDLRenderer() { return renderer_; }
    bool isWindowOpen() const { return window_open_.load(); }

    // Debug and inspection
    EmulatorError saveScreenshot(const std::string& filename);
    Framebuffer& getFramebuffer() { return *framebuffer_; }
    const TouchInput& getTouchInput() const { return *touch_input_; }

private:
    // SDL management
    EmulatorError initializeSDL();
    void shutdownSDL();
    void handleSDLEvents();
    
    // Rendering
    void renderingLoop();
    EmulatorError renderFrame();
    EmulatorError convertFramebuffer();

    // Touch processing
    void processTouchInput();
    EmulatorError updateTouchRegisters();

    // Register handling
    EmulatorError handleControlRegister(uint32_t value);
    EmulatorError handleFramebufferAddress(uint32_t value);
    EmulatorError handlePixelFormat(uint32_t value);

    // Interrupt generation
    void triggerVSyncInterrupt();
    void triggerTouchInterrupt();

    // Configuration
    DisplayConfig config_;

    // SDL components
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    SDL_Texture* display_texture_ = nullptr;
    std::atomic<bool> window_open_{false};

    // Display state
    std::unique_ptr<Framebuffer> framebuffer_;
    Address framebuffer_address_ = 0;
    PixelFormat pixel_format_ = PixelFormat::RGB888;
    uint8_t backlight_brightness_ = 255;
    bool display_enabled_ = false;

    // Touch controller
    std::unique_ptr<TouchInput> touch_input_;
    bool touch_enabled_ = false;

    // Register state
    std::unordered_map<Address, uint32_t> registers_;

    // Timing control
    std::thread rendering_thread_;
    std::atomic<bool> should_stop_{false};
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    uint32_t frame_count_ = 0;
    uint32_t vsync_count_ = 0;

    // Performance monitoring
    struct DisplayStatistics {
        uint64_t frames_rendered = 0;
        uint64_t vsync_interrupts = 0;
        uint64_t touch_events = 0;
        double average_fps = 0.0;
        uint64_t pixel_transfers = 0;
    };

    DisplayStatistics display_stats_;
    mutable std::mutex stats_mutex_;

    // Interrupt IDs
    static constexpr uint32_t VSYNC_INTERRUPT_ID = 16;
    static constexpr uint32_t TOUCH_INTERRUPT_ID = 17;
    static constexpr uint32_t DISPLAY_ERROR_INTERRUPT_ID = 18;
};

} // namespace m5tab5::emulator