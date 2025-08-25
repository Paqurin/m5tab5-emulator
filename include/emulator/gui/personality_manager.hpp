#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <string>
#include <vector>
#include <map>
#include <random>
#include <chrono>
#include <functional>
#include <set>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

/**
 * @brief Personality Manager adds delightful and whimsical elements to the GUI
 * 
 * This system creates memorable moments and personality in the M5Stack Tab5 emulator:
 * - Startup boot sequence animations
 * - Loading states with personality
 * - Error messages that make users smile
 * - Achievement celebrations
 * - Easter eggs and surprises
 * - Hardware-themed interactions
 */
class PersonalityManager {
public:
    PersonalityManager();
    ~PersonalityManager();
    
    Result<void> initialize();
    void update();
    void shutdown();
    
    // Startup Experience
    void start_boot_sequence();
    void update_boot_sequence();
    bool is_boot_complete() const { return boot_complete_; }
    
    // Loading States with Personality
    void start_loading(const std::string& operation);
    void update_loading_message();
    void complete_loading(bool success, const std::string& result = "");
    bool is_loading() const { return loading_active_; }
    std::string get_current_loading_message() const { return current_loading_message_; }
    float get_loading_progress() const { return loading_progress_; }
    float get_boot_progress() const { return calculate_boot_progress(); }
    
    // Error Handling with Personality
    std::string get_friendly_error_message(const std::string& error_code) const;
    void show_error_with_personality(const std::string& error);
    
    // Achievement System
    enum class Achievement {
        FIRST_BOOT,
        FIRST_FIRMWARE,
        GPIO_EXPLORER,
        SERIAL_WHISPERER,
        I2C_MASTER,
        SPEED_RUNNER,
        DEBUGGING_NINJA,
        KONAMI_DISCOVERER,
        HARDWARE_WHISPERER,
        PERSISTENCE_CHAMPION
    };
    
    void unlock_achievement(Achievement achievement);
    bool is_achievement_unlocked(Achievement achievement) const;
    std::string get_achievement_message(Achievement achievement) const;
    void trigger_achievement_celebration(Achievement achievement);
    
    // Easter Eggs and Surprises
    void activate_easter_egg(const std::string& trigger);
    std::vector<std::string> get_available_easter_eggs() const;
    
    // Hardware-themed Interactions
    std::string get_gpio_pin_personality(u8 pin) const;
    std::string get_i2c_device_greeting(u8 address) const;
    std::string get_firmware_loading_quote() const;
    std::string get_emulator_mood() const;
    
    // Random Delightful Messages
    std::string get_startup_tip() const;
    std::string get_idle_message() const;
    std::string get_success_celebration() const;
    
    // Exit Experience with Delight
    std::string get_farewell_message() const;
    std::string get_exit_confirmation_message() const;
    void start_farewell_sequence();
    bool is_farewell_active() const { return farewell_active_; }
    std::string get_current_farewell_message() const { return current_farewell_message_; };
    
    // Animation Hooks for UI
    struct Particle {
        float x, y;
        float vx, vy;
        u32 color;
        float life;
        float size;
    };
    
    std::vector<Particle> get_celebration_particles() const { return celebration_particles_; }
    void clear_particles() { celebration_particles_.clear(); }
    void spawn_farewell_particles(float x, float y);
    
private:
    // Boot sequence state
    bool boot_complete_ = false;
    std::chrono::steady_clock::time_point boot_start_time_;
    size_t boot_stage_ = 0;
    std::vector<std::string> boot_messages_ = {
        "M5Stack Tab5 Emulator starting up...",
        "Initializing ESP32-P4 dual cores @ 400MHz...",
        "Calibrating 768KB SRAM + 32MB PSRAM...",
        "Configuring 1280x720 IPS display...",
        "Warming up GPIO pins 0-47...",
        "Syncing I2C, SPI, UART peripherals...",
        "Loading ES8388 audio codec...",
        "Initializing BMI270 6-axis IMU...",
        "Connecting SC2356 2MP camera...",
        "Enabling WiFi 6 + Bluetooth LE...",
        "Ready for authentic ESP32-P4 development!"
    };
    
    // Loading state
    bool loading_active_ = false;
    std::string current_loading_message_;
    float loading_progress_ = 0.0f;
    std::chrono::steady_clock::time_point loading_start_time_;
    size_t loading_message_index_ = 0;
    
    // Farewell sequence state
    bool farewell_active_ = false;
    std::string current_farewell_message_;
    std::chrono::steady_clock::time_point farewell_start_time_;
    size_t farewell_stage_ = 0;
    
    std::vector<std::string> firmware_loading_messages_ = {
        "Examining ELF headers with a magnifying glass...",
        "Teaching RISC-V instructions to the CPU...",
        "Arranging memory sections like a librarian...",
        "Convincing Flash memory to accept new data...",
        "Initializing stack pointer with confidence...",
        "Validating entry point coordinates...",
        "Synchronizing with ESP32-P4 timing...",
        "Almost ready - just polishing the bits!"
    };
    
    std::vector<std::string> general_loading_messages_ = {
        "Calibrating quantum flux capacitors...",
        "Organizing electron traffic patterns...",
        "Consulting the silicon oracle...",
        "Translating binary to human intentions...",
        "Optimizing for maximum developer happiness...",
        "Loading with the power of positive thinking..."
    };
    
    // Farewell messages for delightful exits
    std::vector<std::string> farewell_messages_ = {
        "Thanks for spending time with the M5Stack Tab5 Emulator!",
        "Your code adventures were epic - see you next time!",
        "The ESP32-P4 cores will miss you - until next boot!",
        "Farewell, embedded explorer! Happy coding out there!",
        "Thanks for making development delightful today!",
        "The GPIO pins are waving goodbye! Come back soon!",
        "May your pull-up resistors always be strong!",
        "Until next time - keep the silicon spirits high!"
    };
    
    std::vector<std::string> exit_confirmation_messages_ = {
        "Ready to wrap up this coding session?",
        "Time to save your progress and head out?",
        "All done with embedded adventures for now?",
        "Finished debugging the universe today?",
        "Ready to let the ESP32-P4 rest?",
        "Time to close this chapter of silicon magic?",
        "Heading out? Thanks for the great session!",
        "Done exploring the hardware today?",
        "Ready to compile your memories?",
        "Time for the GPIO pins to take a break?",
        "Finished your embedded masterpiece?",
        "Ready to power down gracefully?"
    };
    
    // Achievement tracking
    std::set<Achievement> unlocked_achievements_;
    std::map<Achievement, std::string> achievement_names_ = {
        {Achievement::FIRST_BOOT, "First Contact"},
        {Achievement::FIRST_FIRMWARE, "Firmware Whisperer"},
        {Achievement::GPIO_EXPLORER, "Pin Master"},
        {Achievement::SERIAL_WHISPERER, "UART Virtuoso"},
        {Achievement::I2C_MASTER, "Bus Captain"},
        {Achievement::SPEED_RUNNER, "Clock Cycle Champion"},
        {Achievement::DEBUGGING_NINJA, "Bug Hunter Supreme"},
        {Achievement::KONAMI_DISCOVERER, "Secret Code Breaker"},
        {Achievement::HARDWARE_WHISPERER, "Silicon Sage"},
        {Achievement::PERSISTENCE_CHAMPION, "Never Give Up Award"}
    };
    
    // Easter egg triggers
    std::map<std::string, std::string> easter_eggs_ = {
        {"konami", "You found the secret developer mode! All debugging tools unlocked."},
        {"m5stack", "M5Stack: Making IoT development delightful since 2017!"},
        {"esp32", "ESP32: The chip that changed everything. Now with RISC-V power!"},
        {"riscv", "RISC-V: Free and open instruction set architecture for everyone!"},
        {"gpio", "GPIO: General Purpose Input/Output - The Swiss Army knife of embedded systems!"},
        {"uart", "UART: Universal Asynchronous Receiver Transmitter - The postal service of chips!"}
    };
    
    // Hardware personality
    std::vector<std::string> gpio_personalities_ = {
        "Digital Pin - Ready for action!",
        "Analog Input - Sensing the world!",
        "PWM Output - Smooth as silk!",
        "Interrupt Pin - Always alert!",
        "Communication Pin - Let's talk!",
        "Power Pin - Energetic and reliable!"
    };
    
    std::vector<std::string> i2c_greetings_ = {
        "Hello! I'm your friendly I2C device.",
        "Ready to exchange data at the speed of awesome!",
        "Two wires, unlimited possibilities!",
        "Pull-up resistors: engaged. Communication: ready!",
        "SDA and SCL are standing by for your commands!"
    };
    
    // Success celebrations
    std::vector<std::string> success_messages_ = {
        "Fantastic! That worked perfectly!",
        "Success! The electrons are dancing with joy!",
        "Wonderful! Everything is running smoothly!",
        "Excellent! You're becoming a hardware wizard!",
        "Amazing! The ESP32-P4 is purring like a happy cat!",
        "Outstanding! Even the oscillators are impressed!",
        "Brilliant! You've made the silicon smile!"
    };
    
    // Friendly error messages
    std::map<std::string, std::vector<std::string>> friendly_errors_ = {
        {"file_not_found", {
            "Hmm, that file seems to be playing hide and seek!",
            "File not found - did it go for a coffee break?",
            "Oops! That file must be in another castle."
        }},
        {"invalid_format", {
            "That file format is speaking a different language!",
            "File format mismatch - we need an ESP32-P4 ELF file.",
            "This file isn't quite what we expected. ELF files only, please!"
        }},
        {"hardware_error", {
            "The hardware is having a moment - let's give it some space.",
            "Hardware hiccup detected - time for a gentle reset!",
            "Something's not quite right with the hardware. Let's troubleshoot together!"
        }}
    };
    
    // Particle system for celebrations
    std::vector<Particle> celebration_particles_;
    void spawn_celebration_particles(float x, float y, u32 color);
    void update_particles();
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<float> float_dist_;
    std::uniform_int_distribution<size_t> size_dist_;
    
    // Helper functions
    std::string get_random_message(const std::vector<std::string>& messages) const;
    float calculate_boot_progress() const;
    void advance_loading_stage();
    void update_farewell_sequence();
};

} // namespace m5tab5::emulator::gui