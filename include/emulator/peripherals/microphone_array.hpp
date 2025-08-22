#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/es8388_codec.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <chrono>
#include <complex>

namespace m5tab5::emulator {

enum class MicrophoneType : u8 {
    OMNIDIRECTIONAL = 0,
    CARDIOID = 1,
    BIDIRECTIONAL = 2,
    SHOTGUN = 3
};

enum class BeamformingMode : u8 {
    DISABLED = 0,
    FIXED_BEAM = 1,
    ADAPTIVE_BEAM = 2,
    NOISE_CANCELLATION = 3,
    VOICE_ENHANCEMENT = 4
};

enum class NoiseProfile : u8 {
    SILENT = 0,
    QUIET_ROOM = 1,
    OFFICE = 2,
    STREET = 3,
    RESTAURANT = 4,
    MACHINERY = 5,
    CUSTOM = 255
};

struct MicrophoneElement {
    u8 element_id;
    float x_position_mm;      // X position relative to array center
    float y_position_mm;      // Y position relative to array center  
    float z_position_mm;      // Z position relative to array center
    MicrophoneType type;
    float sensitivity_dbfs;   // Sensitivity in dBFS
    float frequency_response[32]; // Frequency response bins (20Hz-20kHz)
    bool enabled;
    float gain_db;
    bool muted;
};

struct AudioSource {
    float azimuth_degrees;    // 0-360 degrees (0 = front)
    float elevation_degrees;  // -90 to +90 degrees
    float distance_meters;    // Distance from array
    float amplitude;          // Source amplitude (0.0-1.0)
    float frequency_hz;       // Primary frequency
    std::string source_type;  // "voice", "music", "noise", etc.
    bool active;
};

struct BeamformingParameters {
    BeamformingMode mode;
    float target_azimuth;     // Target direction in degrees
    float target_elevation;   // Target elevation in degrees
    float beam_width;         // Beam width in degrees
    float noise_threshold;    // Noise gate threshold
    float adaptation_rate;    // Adaptation speed (0.0-1.0)
    bool wind_noise_reduction;
    bool echo_cancellation;
};

struct MicrophoneStatistics {
    u64 samples_processed = 0;
    u64 frames_captured = 0;
    u64 beamforming_updates = 0;
    u64 voice_detections = 0;
    u64 noise_events = 0;
    double average_input_level[2] = {0.0, 0.0}; // Left, Right
    double snr_estimate_db = 0.0;
    double background_noise_level = 0.0;
    float dominant_frequency_hz = 0.0;
    float spatial_resolution_degrees = 0.0;
};

class MicrophoneArray {
public:
    static constexpr size_t MAX_MICROPHONES = 2;  // Dual microphone array
    static constexpr size_t MAX_AUDIO_SOURCES = 8;
    static constexpr size_t FFT_SIZE = 512;
    static constexpr float ARRAY_SPACING_MM = 14.0f; // Typical spacing for M5Stack
    static constexpr u32 DEFAULT_SAMPLE_RATE = 44100;
    
    MicrophoneArray();
    ~MicrophoneArray();

    Result<void> initialize(const Configuration& config, ES8388Codec* codec);
    Result<void> shutdown();

    // Microphone configuration
    Result<void> configure_microphone(u8 mic_id, MicrophoneType type, float sensitivity_dbfs);
    Result<void> set_microphone_gain(u8 mic_id, float gain_db);
    Result<void> mute_microphone(u8 mic_id, bool mute);
    Result<void> enable_microphone(u8 mic_id, bool enable);
    
    // Audio source simulation
    Result<void> add_audio_source(const AudioSource& source);
    Result<void> remove_audio_source(u8 source_id);
    Result<void> update_audio_source(u8 source_id, const AudioSource& source);
    Result<std::vector<AudioSource>> get_active_sources() const;
    
    // Beamforming control
    Result<void> configure_beamforming(const BeamformingParameters& params);
    Result<void> set_beam_direction(float azimuth, float elevation);
    Result<void> enable_beamforming(bool enable);
    Result<void> calibrate_array();
    
    // Noise simulation
    Result<void> set_noise_profile(NoiseProfile profile, float level_db = -40.0f);
    Result<void> add_custom_noise(const std::vector<float>& noise_spectrum);
    Result<void> enable_wind_noise(bool enable, float wind_speed_ms = 0.0f);
    
    // Audio processing
    Result<std::vector<i16>> capture_frame(size_t samples_per_channel);
    Result<void> process_beamforming(std::vector<i16>& left_channel, std::vector<i16>& right_channel);
    Result<float> estimate_direction_of_arrival();
    Result<float> measure_snr();
    
    // Voice activity detection
    Result<bool> detect_voice_activity();
    Result<float> get_voice_probability();
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    size_t get_microphone_count() const { return microphones_.size(); }
    const MicrophoneStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    void initialize_microphone_array();
    void simulate_acoustic_environment(std::vector<i16>& left_samples, std::vector<i16>& right_samples);
    void apply_spatial_filtering(const AudioSource& source, std::vector<i16>& left_out, std::vector<i16>& right_out);
    void add_background_noise(std::vector<i16>& left_samples, std::vector<i16>& right_samples);
    void apply_frequency_response(u8 mic_id, std::vector<i16>& samples);
    
    // Signal processing helpers
    std::vector<std::complex<float>> compute_fft(const std::vector<i16>& samples);
    std::vector<i16> compute_ifft(const std::vector<std::complex<float>>& spectrum);
    float compute_cross_correlation(const std::vector<i16>& left, const std::vector<i16>& right);
    float estimate_time_delay(const std::vector<i16>& left, const std::vector<i16>& right);
    
    // Beamforming algorithms
    void apply_delay_and_sum_beamforming(std::vector<i16>& left, std::vector<i16>& right);
    void apply_adaptive_beamforming(std::vector<i16>& left, std::vector<i16>& right);
    void apply_noise_cancellation(std::vector<i16>& left, std::vector<i16>& right);
    
    // Voice activity detection
    float compute_zero_crossing_rate(const std::vector<i16>& samples);
    float compute_spectral_centroid(const std::vector<i16>& samples);
    float compute_energy_ratio(const std::vector<i16>& samples);
    
    bool initialized_;
    std::vector<MicrophoneElement> microphones_;
    std::vector<AudioSource> audio_sources_;
    BeamformingParameters beamforming_;
    NoiseProfile noise_profile_;
    float noise_level_db_;
    
    ES8388Codec* codec_;
    
    // Processing state
    std::vector<float> noise_spectrum_;
    std::vector<i16> left_buffer_;
    std::vector<i16> right_buffer_;
    std::vector<float> beamforming_weights_;
    
    // Voice activity detection state
    std::vector<float> vad_history_;
    float voice_threshold_;
    float noise_floor_;
    
    // Wind noise simulation
    bool wind_noise_enabled_;
    float wind_speed_;
    std::vector<float> wind_filter_state_;
    
    MicrophoneStatistics statistics_;
    
    mutable std::mutex array_mutex_;
};

}  // namespace m5tab5::emulator