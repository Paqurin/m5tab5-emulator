#include "emulator/gui/personality_manager.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"
#include <algorithm>
#include <cmath>

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::gui;

PersonalityManager::PersonalityManager()
    : rng_(std::chrono::steady_clock::now().time_since_epoch().count())
    , float_dist_(0.0f, 1.0f)
    , size_dist_(0, 100) {
}

PersonalityManager::~PersonalityManager() {
    shutdown();
}

Result<void> PersonalityManager::initialize() {
    LOG_INFO("PersonalityManager initializing - preparing delightful experiences!");
    
    // Initialize random number generator
    rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
    
    // Set up initial state
    boot_complete_ = false;
    loading_active_ = false;
    celebration_particles_.clear();
    
    LOG_DEBUG("PersonalityManager initialized with {} easter eggs and {} achievements",
              easter_eggs_.size(), achievement_names_.size());
    
    return {};
}

void PersonalityManager::update() {
    if (!boot_complete_) {
        update_boot_sequence();
    }
    
    if (loading_active_) {
        update_loading_message();
    }
    
    update_particles();
}

void PersonalityManager::shutdown() {
    LOG_DEBUG("PersonalityManager shutting down - thanks for the memories!");
    celebration_particles_.clear();
}

// Boot Sequence Implementation
void PersonalityManager::start_boot_sequence() {
    LOG_INFO("Starting delightful boot sequence!");
    boot_start_time_ = std::chrono::steady_clock::now();
    boot_stage_ = 0;
    boot_complete_ = false;
}

void PersonalityManager::update_boot_sequence() {
    if (boot_complete_) return;
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - boot_start_time_).count();
    
    // Each boot stage takes about 800ms for dramatic effect
    size_t target_stage = std::min(elapsed / 800, static_cast<long>(boot_messages_.size() - 1));
    
    if (target_stage > boot_stage_) {
        boot_stage_ = target_stage;
        LOG_INFO("Boot stage {}: {}", boot_stage_, boot_messages_[boot_stage_]);
        
        // Spawn some celebratory particles for major milestones
        if (boot_stage_ == 3 || boot_stage_ == 7 || boot_stage_ == boot_messages_.size() - 1) {
            spawn_celebration_particles(400.0f, 300.0f, 0xFF6600); // M5Stack Orange
        }
    }
    
    // Complete boot sequence
    if (boot_stage_ >= boot_messages_.size() - 1 && elapsed > boot_messages_.size() * 800) {
        boot_complete_ = true;
        LOG_INFO("Boot sequence completed - M5Stack Tab5 Emulator ready for awesome development!");
        unlock_achievement(Achievement::FIRST_BOOT);
    }
}

float PersonalityManager::calculate_boot_progress() const {
    if (boot_complete_) return 1.0f;
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - boot_start_time_).count();
    
    float total_time = boot_messages_.size() * 800.0f;
    return std::min(elapsed / total_time, 1.0f);
}

// Loading States Implementation
void PersonalityManager::start_loading(const std::string& operation) {
    loading_active_ = true;
    loading_start_time_ = std::chrono::steady_clock::now();
    loading_message_index_ = 0;
    loading_progress_ = 0.0f;
    
    // Choose appropriate message set based on operation
    if (operation.find("firmware") != std::string::npos || operation.find("elf") != std::string::npos) {
        current_loading_message_ = get_random_message(firmware_loading_messages_);
    } else {
        current_loading_message_ = get_random_message(general_loading_messages_);
    }
    
    LOG_INFO("Started loading with personality: {}", current_loading_message_);
}

void PersonalityManager::update_loading_message() {
    if (!loading_active_) return;
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - loading_start_time_).count();
    
    // Update progress (simulate progress over 3 seconds)
    loading_progress_ = std::min(elapsed / 3000.0f, 0.95f); // Don't reach 100% until complete
    
    // Change message every 1.5 seconds for variety
    if (elapsed > 1500 && loading_message_index_ == 0) {
        loading_message_index_ = 1;
        current_loading_message_ = get_random_message(firmware_loading_messages_);
        LOG_DEBUG("Updated loading message: {}", current_loading_message_);
    }
}

void PersonalityManager::complete_loading(bool success, const std::string& result) {
    loading_active_ = false;
    loading_progress_ = 1.0f;
    
    if (success) {
        current_loading_message_ = get_random_message(success_messages_);
        spawn_celebration_particles(700.0f, 400.0f, 0x4CAF50); // Success Green
        LOG_INFO("Loading completed with success: {}", current_loading_message_);
    } else {
        current_loading_message_ = "Don't worry, we'll figure this out together!";
        LOG_WARN("Loading completed with issues: {}", result);
    }
}

// Error Handling with Personality
std::string PersonalityManager::get_friendly_error_message(const std::string& error_code) const {
    auto it = friendly_errors_.find(error_code);
    if (it != friendly_errors_.end() && !it->second.empty()) {
        return get_random_message(it->second);
    }
    
    // Default friendly error message
    std::vector<std::string> defaults = {
        "Oops! Something unexpected happened, but we can fix this!",
        "Hmm, that didn't go as planned. Let's try a different approach!",
        "No worries! Even the best developers encounter surprises.",
        "Plot twist! Let's debug this together like the pros we are!"
    };
    
    return get_random_message(defaults);
}

void PersonalityManager::show_error_with_personality(const std::string& error) {
    LOG_WARN("Showing friendly error: {}", get_friendly_error_message(error));
    // In a full implementation, this would trigger UI animations like a gentle shake
}

// Achievement System Implementation
void PersonalityManager::unlock_achievement(Achievement achievement) {
    if (unlocked_achievements_.find(achievement) != unlocked_achievements_.end()) {
        return; // Already unlocked
    }
    
    unlocked_achievements_.insert(achievement);
    
    std::string name = achievement_names_.at(achievement);
    std::string message = get_achievement_message(achievement);
    
    LOG_INFO("Achievement Unlocked: {} - {}", name, message);
    
    // Trigger celebration
    trigger_achievement_celebration(achievement);
}

bool PersonalityManager::is_achievement_unlocked(Achievement achievement) const {
    return unlocked_achievements_.find(achievement) != unlocked_achievements_.end();
}

std::string PersonalityManager::get_achievement_message(Achievement achievement) const {
    switch (achievement) {
        case Achievement::FIRST_BOOT:
            return "Welcome to the M5Stack Tab5 Emulator! Your journey in ESP32-P4 development begins now.";
        case Achievement::FIRST_FIRMWARE:
            return "You've loaded your first firmware! The ESP32-P4 is ready to run your code.";
        case Achievement::GPIO_EXPLORER:
            return "GPIO Pin Master! You've discovered the power of digital I/O control.";
        case Achievement::SERIAL_WHISPERER:
            return "UART Communication Expert! You speak the language of serial data.";
        case Achievement::I2C_MASTER:
            return "I2C Bus Captain! Two wires, unlimited communication possibilities.";
        case Achievement::SPEED_RUNNER:
            return "Clock Cycle Champion! Your emulator is running at impressive speeds.";
        case Achievement::DEBUGGING_NINJA:
            return "Debug Mode Activated! You've unlocked advanced development tools.";
        case Achievement::KONAMI_DISCOVERER:
            return "Secret Code Master! You've found the hidden developer features.";
        case Achievement::HARDWARE_WHISPERER:
            return "Silicon Sage! You understand the deep mysteries of embedded hardware.";
        case Achievement::PERSISTENCE_CHAMPION:
            return "Never Give Up! Your persistence in development is truly admirable.";
        default:
            return "Achievement unlocked! You're becoming an embedded systems expert!";
    }
}

void PersonalityManager::trigger_achievement_celebration(Achievement achievement) {
    // Spawn celebration particles
    spawn_celebration_particles(700.0f, 200.0f, 0xFF4081); // Celebration Pink
    spawn_celebration_particles(600.0f, 250.0f, 0xFF6600); // M5Stack Orange
    spawn_celebration_particles(800.0f, 180.0f, 0x4CAF50); // Success Green
    
    LOG_DEBUG("Triggered celebration for achievement: {}", achievement_names_.at(achievement));
}

// Easter Eggs and Surprises
void PersonalityManager::activate_easter_egg(const std::string& trigger) {
    auto it = easter_eggs_.find(trigger);
    if (it != easter_eggs_.end()) {
        LOG_INFO("Easter egg activated: {} - {}", trigger, it->second);
        spawn_celebration_particles(500.0f, 300.0f, 0xFFB74D);
        
        if (trigger == "konami") {
            unlock_achievement(Achievement::KONAMI_DISCOVERER);
        }
    }
}

std::vector<std::string> PersonalityManager::get_available_easter_eggs() const {
    std::vector<std::string> triggers;
    for (const auto& egg : easter_eggs_) {
        triggers.push_back(egg.first);
    }
    return triggers;
}

// Hardware-themed Interactions
std::string PersonalityManager::get_gpio_pin_personality(u8 pin) const {
    // Give each GPIO pin a unique personality based on its number
    size_t personality_index = pin % gpio_personalities_.size();
    return "GPIO " + std::to_string(pin) + ": " + gpio_personalities_[personality_index];
}

std::string PersonalityManager::get_i2c_device_greeting(u8 address) const {
    size_t greeting_index = address % i2c_greetings_.size();
    return "Device 0x" + std::to_string(address) + ": " + i2c_greetings_[greeting_index];
}

std::string PersonalityManager::get_firmware_loading_quote() const {
    std::vector<std::string> quotes = {
        "Loading firmware is like teaching a computer to dream.",
        "Every great embedded project starts with a single ELF file.",
        "Code is poetry, and the ESP32-P4 is your stage.",
        "Firmware: where software meets silicon magic.",
        "In embedded systems, every byte counts and every cycle matters.",
        "The ESP32-P4: RISC-V power meets M5Stack innovation."
    };
    return get_random_message(quotes);
}

std::string PersonalityManager::get_emulator_mood() const {
    std::vector<std::string> moods = {
        "The emulator is feeling energetic and ready for action!",
        "ESP32-P4 cores are humming with anticipation.",
        "All systems nominal - ready for embedded adventures!",
        "The silicon is in a great mood today!",
        "Emulator status: Happy and hardware-ready!",
        "Clock domains synchronized - feeling fantastic!"
    };
    return get_random_message(moods);
}

// Random Delightful Messages
std::string PersonalityManager::get_startup_tip() const {
    std::vector<std::string> tips = {
        "Tip: Use Ctrl+O to quickly load firmware files!",
        "Pro tip: F3 toggles the GPIO viewer for real-time pin monitoring.",
        "Did you know? The M5Stack Tab5 has 47 GPIO pins ready for action!",
        "Hint: Try the Konami code for secret developer features!",
        "Fun fact: The ESP32-P4 runs dual RISC-V cores at 400MHz!",
        "Reminder: The emulator supports authentic M5Stack hardware simulation.",
        "Tip: Press Space to pause/resume the emulator during execution."
    };
    return get_random_message(tips);
}

std::string PersonalityManager::get_idle_message() const {
    std::vector<std::string> messages = {
        "Ready when you are! Load some firmware to get started.",
        "The ESP32-P4 cores are patiently waiting for instructions.",
        "All quiet on the embedded front. Time to code something amazing!",
        "Emulator ready - let's bring some hardware to life!",
        "Standing by for your next brilliant embedded project.",
        "The GPIO pins are curious about what you'll connect next!"
    };
    return get_random_message(messages);
}

std::string PersonalityManager::get_success_celebration() const {
    return get_random_message(success_messages_);
}

// Particle System for Celebrations
void PersonalityManager::spawn_celebration_particles(float x, float y, u32 color) {
    const int particle_count = 20;
    
    for (int i = 0; i < particle_count; ++i) {
        Particle particle;
        
        // Random position around the spawn point
        float angle = float_dist_(rng_) * 2.0f * M_PI;
        float radius = float_dist_(rng_) * 20.0f;
        
        particle.x = x + std::cos(angle) * radius;
        particle.y = y + std::sin(angle) * radius;
        
        // Random velocity
        particle.vx = (float_dist_(rng_) - 0.5f) * 200.0f;
        particle.vy = (float_dist_(rng_) - 0.5f) * 200.0f - 50.0f; // Slight upward bias
        
        particle.color = color;
        particle.life = 1.0f + float_dist_(rng_) * 2.0f; // 1-3 seconds
        particle.size = 2.0f + float_dist_(rng_) * 4.0f;
        
        celebration_particles_.push_back(particle);
    }
    
    LOG_DEBUG("Spawned {} celebration particles at ({}, {})", particle_count, x, y);
}

void PersonalityManager::update_particles() {
    const float dt = 1.0f / 60.0f; // Assume 60 FPS
    const float gravity = 200.0f;
    
    // Update existing particles
    for (auto& particle : celebration_particles_) {
        if (particle.life > 0.0f) {
            // Physics update
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.vy += gravity * dt; // Apply gravity
            
            // Reduce life
            particle.life -= dt;
            
            // Fade out as life decreases
            if (particle.life < 0.5f) {
                float alpha = particle.life / 0.5f;
                // In a real implementation, you'd modify the alpha channel of the color
            }
        }
    }
    
    // Remove dead particles
    celebration_particles_.erase(
        std::remove_if(celebration_particles_.begin(), celebration_particles_.end(),
            [](const Particle& p) { return p.life <= 0.0f; }),
        celebration_particles_.end());
}

// Helper Functions
std::string PersonalityManager::get_random_message(const std::vector<std::string>& messages) const {
    if (messages.empty()) return "";
    
    std::uniform_int_distribution<size_t> dist(0, messages.size() - 1);
    return messages[dist(const_cast<std::mt19937&>(rng_))];
}