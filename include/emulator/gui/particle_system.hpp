#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <vector>
#include <chrono>
#include <random>

#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator::gui {

/**
 * @brief Simple particle system for delightful celebration effects
 * 
 * Creates beautiful particle animations for:
 * - Achievement unlocks
 * - Successful operations
 * - Loading completions
 * - Easter egg activations
 */
class ParticleSystem {
public:
    struct Particle {
        float x, y;           // Position
        float vx, vy;         // Velocity
        float ax, ay;         // Acceleration
        u32 color;            // RGBA color
        float size;           // Size in pixels
        float life;           // Remaining life (0.0 to 1.0)
        float initial_life;   // Original life value
        float rotation;       // Rotation angle
        float rotation_speed; // Rotation velocity
        
        bool is_alive() const { return life > 0.0f; }
        float get_alpha() const { return life / initial_life; }
    };
    
    enum class EmissionType {
        BURST,        // All particles at once
        CONTINUOUS,   // Particles over time
        FOUNTAIN,     // Upward spray
        EXPLOSION,    // Radial burst
        CONFETTI,     // Falling celebration
        SPARKLE       // Twinkling effects
    };
    
    ParticleSystem();
    ~ParticleSystem();
    
    Result<void> initialize();
    void update(float delta_time);
    void render(); // In a real implementation, this would use SDL2 or OpenGL
    void shutdown();
    
    // Particle emission
    void emit_burst(float x, float y, EmissionType type, u32 color, int count = 20);
    void emit_continuous(float x, float y, EmissionType type, u32 color, float rate = 10.0f);
    void emit_achievement_celebration(float x, float y);
    void emit_loading_completion(float x, float y, bool success);
    void emit_easter_egg_sparkle(float x, float y);
    
    // Control
    void clear_all_particles();
    void pause() { paused_ = true; }
    void resume() { paused_ = false; }
    bool is_active() const { return !particles_.empty(); }
    
    // Configuration
    void set_gravity(float gravity) { gravity_ = gravity; }
    void set_air_resistance(float resistance) { air_resistance_ = resistance; }
    void set_max_particles(size_t max) { max_particles_ = max; }
    
    // Access for rendering
    const std::vector<Particle>& get_particles() const { return particles_; }
    
private:
    std::vector<Particle> particles_;
    size_t max_particles_ = 1000;
    bool paused_ = false;
    
    // Physics parameters
    float gravity_ = 200.0f;           // Pixels per second squared
    float air_resistance_ = 0.98f;     // Velocity multiplier per frame
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<float> float_dist_;
    std::uniform_int_distribution<int> int_dist_;
    
    // Emission parameters
    struct EmissionConfig {
        float x, y;
        EmissionType type;
        u32 color;
        float rate;
        float last_emission_time;
        bool active;
    };
    std::vector<EmissionConfig> continuous_emitters_;
    
    // Helper functions
    Particle create_particle(float x, float y, EmissionType type, u32 color);
    void update_particle(Particle& particle, float delta_time);
    void remove_dead_particles();
    
    // Emission type specific creation
    Particle create_burst_particle(float x, float y, u32 color);
    Particle create_fountain_particle(float x, float y, u32 color);
    Particle create_explosion_particle(float x, float y, u32 color);
    Particle create_confetti_particle(float x, float y, u32 color);
    Particle create_sparkle_particle(float x, float y, u32 color);
    
    // Color utilities
    u32 create_celebration_color();
    u32 create_success_color();
    u32 create_error_color();
    u32 create_sparkle_color();
    
    // M5Stack themed colors
    static constexpr u32 M5STACK_ORANGE = 0xFF6600FF;
    static constexpr u32 M5STACK_BLUE = 0x2196F3FF;
    static constexpr u32 CELEBRATION_PINK = 0xFF4081FF;
    static constexpr u32 SUCCESS_GREEN = 0x4CAF50FF;
    static constexpr u32 WARNING_YELLOW = 0xFFC107FF;
    static constexpr u32 ERROR_RED = 0xFF5722FF;
};

} // namespace m5tab5::emulator::gui