#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/es8388_codec.hpp"
#include "emulator/peripherals/microphone_array.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#ifndef NO_GRAPHICS
#ifdef INTERNAL_SDL2_HEADERS
#include "emulator/graphics/internal_sdl2.h"
#else
#include <SDL2/SDL.h>
#endif
#endif

namespace m5tab5::emulator {

enum class AudioStreamDirection : u8 {
    PLAYBACK = 0,
    RECORDING = 1,
    DUPLEX = 2
};

enum class AudioBufferMode : u8 {
    BLOCKING = 0,
    NON_BLOCKING = 1,
    CALLBACK = 2
};

struct AudioStreamConfig {
    AudioSampleRate sample_rate = AudioSampleRate::RATE_44100;
    AudioBitDepth bit_depth = AudioBitDepth::BITS_16;
    u8 channels = 2;                    // Stereo
    u32 buffer_size_frames = 1024;      // Buffer size in frames
    u32 buffer_count = 4;               // Number of buffers
    AudioStreamDirection direction = AudioStreamDirection::DUPLEX;
    AudioBufferMode buffer_mode = AudioBufferMode::CALLBACK;
    float volume = 1.0f;                // Master volume (0.0-1.0)
    bool auto_start = true;             // Start streaming automatically
};

struct AudioBuffer {
    std::vector<i16> data;
    size_t frames;
    u8 channels;
    std::chrono::steady_clock::time_point timestamp;
    bool is_valid;
};

struct AudioLatencyInfo {
    u32 input_latency_ms;
    u32 output_latency_ms;
    u32 total_latency_ms;
    u32 buffer_underruns;
    u32 buffer_overruns;
    double cpu_usage_percent;
};

struct AudioPipelineStatistics {
    u64 frames_played = 0;
    u64 frames_recorded = 0;
    u64 buffer_underruns = 0;
    u64 buffer_overruns = 0;
    u64 callback_errors = 0;
    u64 format_conversions = 0;
    double average_cpu_usage = 0.0;
    double peak_cpu_usage = 0.0;
    u32 current_latency_ms = 0;
    double sample_rate_actual = 0.0;
};

using AudioCallback = std::function<void(const AudioBuffer& input, AudioBuffer& output)>;
using AudioErrorCallback = std::function<void(const std::string& error_message)>;

class AudioPipeline {
public:
    static constexpr size_t MAX_BUFFER_SIZE_FRAMES = 8192;
    static constexpr size_t MIN_BUFFER_SIZE_FRAMES = 64;
    static constexpr size_t MAX_BUFFER_COUNT = 16;
    static constexpr size_t MIN_BUFFER_COUNT = 2;
    
    AudioPipeline();
    ~AudioPipeline();

    Result<void> initialize(const Configuration& config, 
                           ES8388Codec* codec, 
                           MicrophoneArray* microphone_array);
    Result<void> shutdown();

    // Stream configuration
    Result<void> configure_stream(const AudioStreamConfig& config);
    Result<AudioStreamConfig> get_stream_config() const;
    Result<void> set_sample_rate(AudioSampleRate sample_rate);
    Result<void> set_buffer_size(u32 buffer_size_frames);
    Result<void> set_volume(float volume);
    
    // Stream control
    Result<void> start_stream();
    Result<void> stop_stream();
    Result<void> pause_stream(bool pause);
    
    // Callback-based interface
    Result<void> set_audio_callback(AudioCallback callback);
    Result<void> set_error_callback(AudioErrorCallback callback);
    
    // Blocking I/O interface
    Result<void> write_audio(const std::vector<i16>& samples);
    Result<std::vector<i16>> read_audio(size_t max_frames = 0);
    
    // Non-blocking I/O interface
    Result<bool> try_write_audio(const std::vector<i16>& samples);
    Result<std::vector<i16>> try_read_audio();
    
    // Buffer management
    Result<size_t> get_write_available() const;
    Result<size_t> get_read_available() const;
    Result<void> flush_buffers();
    
    // Latency and performance
    Result<AudioLatencyInfo> get_latency_info() const;
    Result<bool> is_stream_active() const;
    Result<double> get_cpu_load() const;
    
    // Audio effects and processing
    Result<void> enable_echo_cancellation(bool enable);
    Result<void> enable_noise_suppression(bool enable);
    Result<void> enable_automatic_gain_control(bool enable, float target_level = -12.0f);
    
    // Format conversion utilities
    static std::vector<i16> convert_float_to_int16(const std::vector<float>& samples);
    static std::vector<float> convert_int16_to_float(const std::vector<i16>& samples);
    static std::vector<i16> resample_audio(const std::vector<i16>& samples, 
                                          u32 input_rate, u32 output_rate);
    
    bool is_initialized() const { return initialized_; }
    const AudioPipelineStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct SDL_AudioState {
#ifndef NO_GRAPHICS
        SDL_AudioDeviceID playback_device;
        SDL_AudioDeviceID recording_device;
        SDL_AudioSpec playback_spec;
        SDL_AudioSpec recording_spec;
#else
        u32 playback_device;
        u32 recording_device;
        u32 playback_spec;
        u32 recording_spec;
#endif
        bool playback_active;
        bool recording_active;
    };

    // SDL audio callbacks
#ifndef NO_GRAPHICS
    static void sdl_playback_callback(void* userdata, Uint8* stream, int len);
    static void sdl_recording_callback(void* userdata, Uint8* stream, int len);
#else
    static void sdl_playback_callback(void* userdata, u8* stream, int len);
    static void sdl_recording_callback(void* userdata, u8* stream, int len);
#endif
    
    // Internal processing methods
#ifndef NO_GRAPHICS
    void process_playback_callback(Uint8* stream, int len);
    void process_recording_callback(Uint8* stream, int len);
#else
    void process_playback_callback(u8* stream, int len);
    void process_recording_callback(u8* stream, int len);
#endif
    
    void audio_processing_thread();
    void update_statistics();
    void apply_audio_effects(std::vector<i16>& samples);
    void handle_buffer_underrun();
    void handle_buffer_overrun();
    
    // Format conversion helpers
#ifndef NO_GRAPHICS
    void convert_samples_to_sdl_format(const std::vector<i16>& input, 
                                      Uint8* output, const SDL_AudioSpec& spec);
    void convert_samples_from_sdl_format(const Uint8* input, 
                                        std::vector<i16>& output, const SDL_AudioSpec& spec);
#else
    void convert_samples_to_sdl_format(const std::vector<i16>& input, 
                                      u8* output, u32 spec);
    void convert_samples_from_sdl_format(const u8* input, 
                                        std::vector<i16>& output, u32 spec);
#endif
    
    bool initialized_;
    AudioStreamConfig stream_config_;
    
    ES8388Codec* codec_;
    MicrophoneArray* microphone_array_;
    
    SDL_AudioState sdl_state_;
    
    // Audio buffers
    std::queue<AudioBuffer> playback_buffer_queue_;
    std::queue<AudioBuffer> recording_buffer_queue_;
    
    // Callback interface
    AudioCallback audio_callback_;
    AudioErrorCallback error_callback_;
    
    // Processing thread
    std::unique_ptr<std::thread> processing_thread_;
    std::atomic<bool> processing_active_;
    
    // Audio effects
    bool echo_cancellation_enabled_;
    bool noise_suppression_enabled_;
    bool agc_enabled_;
    float agc_target_level_;
    
    // Statistics and monitoring
    AudioPipelineStatistics statistics_;
    std::chrono::steady_clock::time_point last_stats_update_;
    std::chrono::steady_clock::time_point stream_start_time_;
    
    mutable std::mutex pipeline_mutex_;
    std::condition_variable buffer_condition_;
};

}  // namespace m5tab5::emulator