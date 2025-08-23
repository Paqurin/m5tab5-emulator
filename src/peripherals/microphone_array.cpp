#include "emulator/peripherals/microphone_array.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <numeric>

namespace m5tab5::emulator {

DECLARE_LOGGER("MicrophoneArray");

MicrophoneArray::MicrophoneArray()
    : initialized_(false),
      noise_profile_(NoiseProfile::QUIET_ROOM),
      noise_level_db_(-40.0f),
      codec_(nullptr),
      voice_threshold_(0.5f),
      noise_floor_(0.1f),
      wind_noise_enabled_(false),
      wind_speed_(0.0f) {
    
    microphones_.reserve(MAX_MICROPHONES);
    audio_sources_.reserve(MAX_AUDIO_SOURCES);
    beamforming_weights_.resize(MAX_MICROPHONES, 1.0f);
    vad_history_.resize(10, 0.0f); // 10-frame VAD history
    
    COMPONENT_LOG_DEBUG("Microphone array created");
}

MicrophoneArray::~MicrophoneArray() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Microphone array destroyed");
}

Result<void> MicrophoneArray::initialize(const Configuration& config, ES8388Codec* codec) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Microphone array already initialized"));
    }
    
    if (!codec) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Audio codec required"));
    }
    
    COMPONENT_LOG_INFO("Initializing microphone array");
    
    codec_ = codec;
    
    // Initialize microphone elements
    initialize_microphone_array();
    
    // Set default beamforming parameters
    beamforming_.mode = BeamformingMode::DISABLED;
    beamforming_.target_azimuth = 0.0f;    // Front-facing
    beamforming_.target_elevation = 0.0f;  // Horizontal
    beamforming_.beam_width = 60.0f;       // 60-degree beam width
    beamforming_.noise_threshold = -30.0f;
    beamforming_.adaptation_rate = 0.1f;
    beamforming_.wind_noise_reduction = false;
    beamforming_.echo_cancellation = false;
    
    // Initialize noise spectrum
    noise_spectrum_.resize(FFT_SIZE / 2, 0.0f);
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Microphone array initialized successfully");
    
    return {};
}

Result<void> MicrophoneArray::shutdown() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down microphone array");
    
    // Clear all sources
    audio_sources_.clear();
    
    // Clear buffers
    left_buffer_.clear();
    right_buffer_.clear();
    noise_spectrum_.clear();
    
    codec_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Microphone array shutdown completed");
    return {};
}

Result<void> MicrophoneArray::configure_microphone(u8 mic_id, MicrophoneType type, float sensitivity_dbfs) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (mic_id >= MAX_MICROPHONES) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid microphone ID"));
    }
    
    if (sensitivity_dbfs > 0.0f || sensitivity_dbfs < -60.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Sensitivity out of range (-60dBFS to 0dBFS)"));
    }
    
    if (mic_id < microphones_.size()) {
        microphones_[mic_id].type = type;
        microphones_[mic_id].sensitivity_dbfs = sensitivity_dbfs;
        
        COMPONENT_LOG_DEBUG("Microphone {} configured: type={}, sensitivity={:.1f}dBFS",
                           mic_id, static_cast<u8>(type), sensitivity_dbfs);
    }
    
    return {};
}

Result<void> MicrophoneArray::set_microphone_gain(u8 mic_id, float gain_db) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (mic_id >= MAX_MICROPHONES || mic_id >= microphones_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid microphone ID"));
    }
    
    if (gain_db < -20.0f || gain_db > 40.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Gain out of range (-20dB to +40dB)"));
    }
    
    microphones_[mic_id].gain_db = gain_db;
    
    COMPONENT_LOG_DEBUG("Microphone {} gain set to {:.1f}dB", mic_id, gain_db);
    return {};
}

Result<void> MicrophoneArray::mute_microphone(u8 mic_id, bool mute) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (mic_id >= MAX_MICROPHONES || mic_id >= microphones_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid microphone ID"));
    }
    
    microphones_[mic_id].muted = mute;
    
    COMPONENT_LOG_DEBUG("Microphone {} {}", mic_id, mute ? "muted" : "unmuted");
    return {};
}

Result<void> MicrophoneArray::enable_microphone(u8 mic_id, bool enable) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (mic_id >= MAX_MICROPHONES || mic_id >= microphones_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid microphone ID"));
    }
    
    microphones_[mic_id].enabled = enable;
    
    COMPONENT_LOG_DEBUG("Microphone {} {}", mic_id, enable ? "enabled" : "disabled");
    return {};
}

Result<void> MicrophoneArray::add_audio_source(const AudioSource& source) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (audio_sources_.size() >= MAX_AUDIO_SOURCES) {
        return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
            "Maximum audio sources reached"));
    }
    
    audio_sources_.push_back(source);
    
    COMPONENT_LOG_DEBUG("Audio source added: azimuth={:.1f}°, elevation={:.1f}°, distance={:.1f}m, type={}",
                       source.azimuth_degrees, source.elevation_degrees, 
                       source.distance_meters, source.source_type);
    
    return {};
}

Result<void> MicrophoneArray::remove_audio_source(u8 source_id) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (source_id >= audio_sources_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid source ID"));
    }
    
    audio_sources_.erase(audio_sources_.begin() + source_id);
    
    COMPONENT_LOG_DEBUG("Audio source {} removed", source_id);
    return {};
}

Result<void> MicrophoneArray::update_audio_source(u8 source_id, const AudioSource& source) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (source_id >= audio_sources_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid source ID"));
    }
    
    audio_sources_[source_id] = source;
    
    COMPONENT_LOG_DEBUG("Audio source {} updated", source_id);
    return {};
}

Result<std::vector<AudioSource>> MicrophoneArray::get_active_sources() const {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    std::vector<AudioSource> active_sources;
    for (const auto& source : audio_sources_) {
        if (source.active) {
            active_sources.push_back(source);
        }
    }
    
    return active_sources;
}

Result<void> MicrophoneArray::configure_beamforming(const BeamformingParameters& params) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    beamforming_ = params;
    
    COMPONENT_LOG_INFO("Beamforming configured: mode={}, target={}°/{:.1f}°, beam_width={:.1f}°",
                      static_cast<u8>(params.mode), params.target_azimuth, 
                      params.target_elevation, params.beam_width);
    
    return {};
}

Result<void> MicrophoneArray::set_beam_direction(float azimuth, float elevation) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    beamforming_.target_azimuth = std::fmod(azimuth, 360.0f);
    if (beamforming_.target_azimuth < 0.0f) {
        beamforming_.target_azimuth += 360.0f;
    }
    
    beamforming_.target_elevation = std::clamp(elevation, -90.0f, 90.0f);
    
    COMPONENT_LOG_DEBUG("Beam direction set to azimuth={:.1f}°, elevation={:.1f}°",
                       beamforming_.target_azimuth, beamforming_.target_elevation);
    
    return {};
}

Result<void> MicrophoneArray::enable_beamforming(bool enable) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (enable && beamforming_.mode == BeamformingMode::DISABLED) {
        beamforming_.mode = BeamformingMode::FIXED_BEAM;
    } else if (!enable) {
        beamforming_.mode = BeamformingMode::DISABLED;
    }
    
    COMPONENT_LOG_DEBUG("Beamforming {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> MicrophoneArray::calibrate_array() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    // Simple calibration: measure background noise and adjust thresholds
    noise_floor_ = statistics_.background_noise_level;
    voice_threshold_ = noise_floor_ + 10.0f; // 10dB above noise floor
    
    COMPONENT_LOG_INFO("Array calibrated: noise_floor={:.2f}, voice_threshold={:.2f}",
                      noise_floor_, voice_threshold_);
    
    return {};
}

Result<void> MicrophoneArray::set_noise_profile(NoiseProfile profile, float level_db) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (level_db > -10.0f || level_db < -80.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Noise level out of range (-80dB to -10dB)"));
    }
    
    noise_profile_ = profile;
    noise_level_db_ = level_db;
    
    // Generate noise spectrum based on profile
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    switch (profile) {
        case NoiseProfile::SILENT:
            std::fill(noise_spectrum_.begin(), noise_spectrum_.end(), 0.0f);
            break;
            
        case NoiseProfile::QUIET_ROOM:
            for (size_t i = 0; i < noise_spectrum_.size(); ++i) {
                // Pink noise with low-frequency emphasis
                float freq = static_cast<float>(i) / noise_spectrum_.size();
                noise_spectrum_[i] = std::pow(freq + 0.1f, -0.5f) * 0.1f;
            }
            break;
            
        case NoiseProfile::OFFICE:
            for (size_t i = 0; i < noise_spectrum_.size(); ++i) {
                // Mid-frequency emphasis (keyboard, conversation)
                float freq = static_cast<float>(i) / noise_spectrum_.size();
                noise_spectrum_[i] = std::exp(-std::pow((freq - 0.3f) * 5.0f, 2.0f)) * 0.3f;
            }
            break;
            
        case NoiseProfile::STREET:
            for (size_t i = 0; i < noise_spectrum_.size(); ++i) {
                // Broadband with low-frequency emphasis
                float freq = static_cast<float>(i) / noise_spectrum_.size();
                noise_spectrum_[i] = (std::pow(freq + 0.05f, -0.3f) + 0.2f) * 0.4f;
            }
            break;
            
        case NoiseProfile::RESTAURANT:
            for (size_t i = 0; i < noise_spectrum_.size(); ++i) {
                // Speech frequencies with broadband background
                float freq = static_cast<float>(i) / noise_spectrum_.size();
                float speech_band = std::exp(-std::pow((freq - 0.25f) * 8.0f, 2.0f));
                noise_spectrum_[i] = (speech_band * 0.5f + 0.2f) * 0.4f;
            }
            break;
            
        case NoiseProfile::MACHINERY:
            for (size_t i = 0; i < noise_spectrum_.size(); ++i) {
                // Harmonic structure with fundamental around 50Hz
                float freq = static_cast<float>(i) / noise_spectrum_.size();
                float harmonic = 0.0f;
                for (int h = 1; h <= 10; ++h) {
                    float harm_freq = 50.0f * h / 20000.0f; // Normalize to 0-1
                    if (std::abs(freq - harm_freq) < 0.01f) {
                        harmonic += 1.0f / h;
                    }
                }
                noise_spectrum_[i] = harmonic * 0.6f;
            }
            break;
            
        default:
            break;
    }
    
    COMPONENT_LOG_INFO("Noise profile set: type={}, level={:.1f}dB",
                      static_cast<u8>(profile), level_db);
    
    return {};
}

Result<void> MicrophoneArray::add_custom_noise(const std::vector<float>& noise_spectrum) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (noise_spectrum.size() != FFT_SIZE / 2) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Noise spectrum size must match FFT_SIZE/2"));
    }
    
    noise_spectrum_ = noise_spectrum;
    noise_profile_ = NoiseProfile::CUSTOM;
    
    COMPONENT_LOG_DEBUG("Custom noise spectrum loaded");
    return {};
}

Result<void> MicrophoneArray::enable_wind_noise(bool enable, float wind_speed_ms) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    wind_noise_enabled_ = enable;
    wind_speed_ = std::clamp(wind_speed_ms, 0.0f, 50.0f); // 0-50 m/s
    
    if (enable) {
        // Initialize wind filter state
        wind_filter_state_.resize(4, 0.0f); // Simple 4-pole filter
    } else {
        wind_filter_state_.clear();
    }
    
    COMPONENT_LOG_DEBUG("Wind noise {}: speed={:.1f}m/s", 
                       enable ? "enabled" : "disabled", wind_speed_);
    
    return {};
}

Result<std::vector<i16>> MicrophoneArray::capture_frame(size_t samples_per_channel) {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (samples_per_channel == 0 || samples_per_channel > 4096) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid sample count"));
    }
    
    // Prepare stereo buffer (interleaved L/R)
    std::vector<i16> stereo_samples(samples_per_channel * 2);
    left_buffer_.resize(samples_per_channel);
    right_buffer_.resize(samples_per_channel);
    
    // Simulate acoustic environment
    simulate_acoustic_environment(left_buffer_, right_buffer_);
    
    // Apply beamforming if enabled
    if (beamforming_.mode != BeamformingMode::DISABLED) {
        process_beamforming(left_buffer_, right_buffer_);
    }
    
    // Interleave samples
    for (size_t i = 0; i < samples_per_channel; ++i) {
        stereo_samples[i * 2] = left_buffer_[i];
        stereo_samples[i * 2 + 1] = right_buffer_[i];
    }
    
    // Update statistics
    statistics_.samples_processed += samples_per_channel * 2;
    statistics_.frames_captured++;
    
    // Calculate input levels
    if (!left_buffer_.empty()) {
        float left_sum = 0.0f, right_sum = 0.0f;
        for (size_t i = 0; i < samples_per_channel; ++i) {
            left_sum += std::abs(left_buffer_[i]) / 32768.0f;
            right_sum += std::abs(right_buffer_[i]) / 32768.0f;
        }
        statistics_.average_input_level[0] = left_sum / samples_per_channel;
        statistics_.average_input_level[1] = right_sum / samples_per_channel;
    }
    
    return stereo_samples;
}

Result<void> MicrophoneArray::process_beamforming(std::vector<i16>& left_channel, std::vector<i16>& right_channel) {
    if (beamforming_.mode == BeamformingMode::DISABLED) {
        return {};
    }
    
    switch (beamforming_.mode) {
        case BeamformingMode::FIXED_BEAM:
            apply_delay_and_sum_beamforming(left_channel, right_channel);
            break;
            
        case BeamformingMode::ADAPTIVE_BEAM:
            apply_adaptive_beamforming(left_channel, right_channel);
            break;
            
        case BeamformingMode::NOISE_CANCELLATION:
            apply_noise_cancellation(left_channel, right_channel);
            break;
            
        case BeamformingMode::VOICE_ENHANCEMENT:
            apply_adaptive_beamforming(left_channel, right_channel);
            apply_noise_cancellation(left_channel, right_channel);
            break;
            
        default:
            break;
    }
    
    statistics_.beamforming_updates++;
    return {};
}

Result<float> MicrophoneArray::estimate_direction_of_arrival() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (left_buffer_.empty() || right_buffer_.empty()) {
        return unexpected(MAKE_ERROR(INSUFFICIENT_DATA,
            "No audio data available"));
    }
    
    // Estimate DOA using cross-correlation and time delay
    float time_delay = estimate_time_delay(left_buffer_, right_buffer_);
    
    // Convert time delay to angle (simplified)
    float sound_speed = 343.0f; // m/s at 20°C
    float max_delay = ARRAY_SPACING_MM / 1000.0f / sound_speed; // Maximum possible delay
    float normalized_delay = std::clamp(time_delay / max_delay, -1.0f, 1.0f);
    
    // Convert to angle (0° = front, positive = right side)
    float angle = std::asin(normalized_delay) * 180.0f / M_PI;
    
    return angle;
}

Result<float> MicrophoneArray::measure_snr() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (left_buffer_.empty()) {
        return unexpected(MAKE_ERROR(INSUFFICIENT_DATA,
            "No audio data available"));
    }
    
    // Simple SNR estimation using spectral analysis
    auto spectrum = compute_fft(left_buffer_);
    
    float signal_power = 0.0f;
    float noise_power = 0.0f;
    
    // Voice band: 300Hz - 3400Hz (approximate)
    size_t voice_start = (300 * spectrum.size()) / (DEFAULT_SAMPLE_RATE / 2);
    size_t voice_end = (3400 * spectrum.size()) / (DEFAULT_SAMPLE_RATE / 2);
    
    for (size_t i = 0; i < spectrum.size(); ++i) {
        float power = std::norm(spectrum[i]);
        
        if (i >= voice_start && i <= voice_end) {
            signal_power += power;
        } else {
            noise_power += power;
        }
    }
    
    float snr_linear = signal_power / std::max(noise_power, 1e-10f);
    float snr_db = 10.0f * std::log10(snr_linear);
    
    statistics_.snr_estimate_db = snr_db;
    return snr_db;
}

Result<bool> MicrophoneArray::detect_voice_activity() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (left_buffer_.empty()) {
        return false;
    }
    
    // Compute VAD features
    float energy = 0.0f;
    for (i16 sample : left_buffer_) {
        energy += static_cast<float>(sample * sample);
    }
    energy /= left_buffer_.size();
    energy = std::sqrt(energy) / 32768.0f;
    
    float zcr = compute_zero_crossing_rate(left_buffer_);
    float spectral_centroid = compute_spectral_centroid(left_buffer_);
    
    // Simple VAD decision based on energy and spectral features
    float vad_score = 0.0f;
    
    if (energy > voice_threshold_) {
        vad_score += 0.4f;
    }
    
    if (zcr > 0.1f && zcr < 0.3f) { // Typical voice ZCR range
        vad_score += 0.3f;
    }
    
    if (spectral_centroid > 500.0f && spectral_centroid < 4000.0f) {
        vad_score += 0.3f;
    }
    
    // Update VAD history
    vad_history_.erase(vad_history_.begin());
    vad_history_.push_back(vad_score);
    
    // Average over history for stability
    float avg_vad = std::accumulate(vad_history_.begin(), vad_history_.end(), 0.0f) / vad_history_.size();
    
    bool voice_detected = avg_vad > 0.5f;
    
    if (voice_detected) {
        statistics_.voice_detections++;
    }
    
    return voice_detected;
}

Result<float> MicrophoneArray::get_voice_probability() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Microphone array not initialized"));
    }
    
    if (vad_history_.empty()) {
        return 0.0f;
    }
    
    return std::accumulate(vad_history_.begin(), vad_history_.end(), 0.0f) / vad_history_.size();
}

void MicrophoneArray::update() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Update background noise level estimate
    if (!left_buffer_.empty()) {
        float noise_estimate = 0.0f;
        for (i16 sample : left_buffer_) {
            noise_estimate += std::abs(sample) / 32768.0f;
        }
        noise_estimate /= left_buffer_.size();
        
        // Exponential moving average
        statistics_.background_noise_level = 0.95f * statistics_.background_noise_level + 0.05f * noise_estimate;
    }
    
    // Update spatial resolution estimate
    statistics_.spatial_resolution_degrees = 180.0f / M_PI * 
        std::asin(343.0f / (DEFAULT_SAMPLE_RATE * ARRAY_SPACING_MM / 1000.0f));
}

void MicrophoneArray::initialize_microphone_array() {
    microphones_.clear();
    microphones_.resize(MAX_MICROPHONES);
    
    // Left microphone
    microphones_[0].element_id = 0;
    microphones_[0].x_position_mm = -ARRAY_SPACING_MM / 2.0f;
    microphones_[0].y_position_mm = 0.0f;
    microphones_[0].z_position_mm = 0.0f;
    microphones_[0].type = MicrophoneType::OMNIDIRECTIONAL;
    microphones_[0].sensitivity_dbfs = -26.0f; // Typical MEMS mic
    microphones_[0].enabled = true;
    microphones_[0].gain_db = 0.0f;
    microphones_[0].muted = false;
    
    // Right microphone  
    microphones_[1].element_id = 1;
    microphones_[1].x_position_mm = ARRAY_SPACING_MM / 2.0f;
    microphones_[1].y_position_mm = 0.0f;
    microphones_[1].z_position_mm = 0.0f;
    microphones_[1].type = MicrophoneType::OMNIDIRECTIONAL;
    microphones_[1].sensitivity_dbfs = -26.0f;
    microphones_[1].enabled = true;
    microphones_[1].gain_db = 0.0f;
    microphones_[1].muted = false;
    
    // Initialize frequency response (flat response for now)
    for (auto& mic : microphones_) {
        std::fill(mic.frequency_response, mic.frequency_response + 32, 1.0f);
    }
}

void MicrophoneArray::simulate_acoustic_environment(std::vector<i16>& left_samples, std::vector<i16>& right_samples) {
    // Start with silence
    std::fill(left_samples.begin(), left_samples.end(), 0);
    std::fill(right_samples.begin(), right_samples.end(), 0);
    
    // Add audio sources
    for (const auto& source : audio_sources_) {
        if (!source.active) continue;
        
        std::vector<i16> source_left(left_samples.size());
        std::vector<i16> source_right(right_samples.size());
        
        apply_spatial_filtering(source, source_left, source_right);
        
        // Mix into main channels
        for (size_t i = 0; i < left_samples.size(); ++i) {
            int32_t left_sum = left_samples[i] + source_left[i];
            int32_t right_sum = right_samples[i] + source_right[i];
            
            left_samples[i] = static_cast<i16>(std::clamp(left_sum, -32768, 32767));
            right_samples[i] = static_cast<i16>(std::clamp(right_sum, -32768, 32767));
        }
    }
    
    // Add background noise
    add_background_noise(left_samples, right_samples);
    
    // Apply microphone characteristics
    if (!microphones_[0].muted && microphones_[0].enabled) {
        apply_frequency_response(0, left_samples);
        
        // Apply gain
        float gain_linear = std::pow(10.0f, microphones_[0].gain_db / 20.0f);
        for (i16& sample : left_samples) {
            sample = static_cast<i16>(std::clamp(sample * gain_linear, -32768.0f, 32767.0f));
        }
    } else {
        std::fill(left_samples.begin(), left_samples.end(), 0);
    }
    
    if (!microphones_[1].muted && microphones_[1].enabled) {
        apply_frequency_response(1, right_samples);
        
        float gain_linear = std::pow(10.0f, microphones_[1].gain_db / 20.0f);
        for (i16& sample : right_samples) {
            sample = static_cast<i16>(std::clamp(sample * gain_linear, -32768.0f, 32767.0f));
        }
    } else {
        std::fill(right_samples.begin(), right_samples.end(), 0);
    }
}

void MicrophoneArray::apply_spatial_filtering(const AudioSource& source, std::vector<i16>& left_out, std::vector<i16>& right_out) {
    // Generate test signal based on source type
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    for (size_t i = 0; i < left_out.size(); ++i) {
        float signal = 0.0f;
        
        if (source.source_type == "voice") {
            // Simulate speech-like signal
            signal = std::sin(2.0f * M_PI * source.frequency_hz * i / DEFAULT_SAMPLE_RATE) * 
                    (0.5f + 0.5f * std::sin(2.0f * M_PI * 3.0f * i / DEFAULT_SAMPLE_RATE)); // AM modulation
        } else if (source.source_type == "music") {
            // Simulate complex harmonic content
            signal = 0.5f * std::sin(2.0f * M_PI * source.frequency_hz * i / DEFAULT_SAMPLE_RATE) +
                    0.3f * std::sin(2.0f * M_PI * source.frequency_hz * 2.0f * i / DEFAULT_SAMPLE_RATE) +
                    0.1f * std::sin(2.0f * M_PI * source.frequency_hz * 3.0f * i / DEFAULT_SAMPLE_RATE);
        } else {
            // White noise
            std::normal_distribution<float> noise_dist(0.0f, 1.0f);
            signal = noise_dist(gen) * 0.1f;
        }
        
        // Apply amplitude and distance attenuation
        float amplitude = source.amplitude / (source.distance_meters * source.distance_meters);
        signal *= amplitude;
        
        // Apply spatial positioning (simplified HRTF)
        float azimuth_rad = source.azimuth_degrees * M_PI / 180.0f;
        float left_gain = 0.5f * (1.0f + std::cos(azimuth_rad));
        float right_gain = 0.5f * (1.0f - std::cos(azimuth_rad));
        
        // Add simple time delay based on angle
        // (In a real implementation, this would use fractional delays)
        left_out[i] = static_cast<i16>(signal * left_gain * 32767.0f);
        right_out[i] = static_cast<i16>(signal * right_gain * 32767.0f);
    }
}

void MicrophoneArray::add_background_noise(std::vector<i16>& left_samples, std::vector<i16>& right_samples) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> noise_dist(0.0f, 1.0f);
    
    float noise_amplitude = std::pow(10.0f, noise_level_db_ / 20.0f);
    
    for (size_t i = 0; i < left_samples.size(); ++i) {
        float noise = noise_dist(gen) * noise_amplitude * 32767.0f;
        
        int32_t left_sum = left_samples[i] + static_cast<i16>(noise);
        int32_t right_sum = right_samples[i] + static_cast<i16>(noise * 0.8f); // Slightly different for stereo
        
        left_samples[i] = static_cast<i16>(std::clamp(left_sum, -32768, 32767));
        right_samples[i] = static_cast<i16>(std::clamp(right_sum, -32768, 32767));
    }
    
    statistics_.noise_events++;
}

void MicrophoneArray::apply_frequency_response(u8 mic_id, std::vector<i16>& samples) {
    // Apply simple frequency response (in a real implementation, this would use proper filtering)
    if (mic_id >= microphones_.size()) return;
    
    // Apply sensitivity
    float sensitivity_linear = std::pow(10.0f, microphones_[mic_id].sensitivity_dbfs / 20.0f);
    for (i16& sample : samples) {
        sample = static_cast<i16>(std::clamp(sample * sensitivity_linear, -32768.0f, 32767.0f));
    }
}

// Signal processing helper implementations
std::vector<std::complex<float>> MicrophoneArray::compute_fft(const std::vector<i16>& samples) {
    // Simplified FFT implementation (in practice, use FFTW or similar)
    std::vector<std::complex<float>> result(samples.size());
    
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = std::complex<float>(static_cast<float>(samples[i]), 0.0f);
    }
    
    // This would be replaced with actual FFT implementation
    return result;
}

std::vector<i16> MicrophoneArray::compute_ifft(const std::vector<std::complex<float>>& spectrum) {
    // Simplified IFFT implementation
    std::vector<i16> result(spectrum.size());
    
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = static_cast<i16>(std::clamp(spectrum[i].real(), -32768.0f, 32767.0f));
    }
    
    return result;
}

float MicrophoneArray::compute_cross_correlation(const std::vector<i16>& left, const std::vector<i16>& right) {
    if (left.size() != right.size()) return 0.0f;
    
    float correlation = 0.0f;
    float left_energy = 0.0f;
    float right_energy = 0.0f;
    
    for (size_t i = 0; i < left.size(); ++i) {
        correlation += left[i] * right[i];
        left_energy += left[i] * left[i];
        right_energy += right[i] * right[i];
    }
    
    float norm = std::sqrt(left_energy * right_energy);
    return norm > 0.0f ? correlation / norm : 0.0f;
}

float MicrophoneArray::estimate_time_delay(const std::vector<i16>& left, const std::vector<i16>& right) {
    // Simplified time delay estimation using cross-correlation
    // In practice, this would use GCC-PHAT or similar algorithms
    
    float max_correlation = 0.0f;
    int best_delay = 0;
    int max_delay = std::min(static_cast<int>(left.size()) / 4, 10); // Limit search range
    
    for (int delay = -max_delay; delay <= max_delay; ++delay) {
        float correlation = 0.0f;
        int valid_samples = 0;
        
        for (size_t i = 0; i < left.size(); ++i) {
            int right_idx = static_cast<int>(i) + delay;
            if (right_idx >= 0 && right_idx < static_cast<int>(right.size())) {
                correlation += left[i] * right[right_idx];
                valid_samples++;
            }
        }
        
        if (valid_samples > 0) {
            correlation /= valid_samples;
            if (std::abs(correlation) > std::abs(max_correlation)) {
                max_correlation = correlation;
                best_delay = delay;
            }
        }
    }
    
    return static_cast<float>(best_delay) / DEFAULT_SAMPLE_RATE;
}

void MicrophoneArray::apply_delay_and_sum_beamforming(std::vector<i16>& left, std::vector<i16>& right) {
    // Simple delay-and-sum beamforming
    float target_azimuth_rad = beamforming_.target_azimuth * M_PI / 180.0f;
    
    // Calculate required delay for steering
    float sound_speed = 343.0f; // m/s
    float delay_samples = (ARRAY_SPACING_MM / 1000.0f) * std::sin(target_azimuth_rad) * DEFAULT_SAMPLE_RATE / sound_speed;
    
    int delay_int = static_cast<int>(std::round(delay_samples));
    
    // Apply delay to right channel (simplified)
    if (delay_int > 0) {
        for (int i = right.size() - 1; i >= delay_int; --i) {
            right[i] = right[i - delay_int];
        }
        for (int i = 0; i < delay_int; ++i) {
            right[i] = 0;
        }
    } else if (delay_int < 0) {
        for (size_t i = 0; i < left.size() + delay_int; ++i) {
            left[i] = left[i - delay_int];
        }
        for (int i = left.size() + delay_int; i < static_cast<int>(left.size()); ++i) {
            left[i] = 0;
        }
    }
    
    // Sum the channels with weighting
    for (size_t i = 0; i < left.size(); ++i) {
        int32_t sum = (left[i] + right[i]) / 2;
        left[i] = static_cast<i16>(std::clamp(sum, -32768, 32767));
        right[i] = left[i]; // Output same signal to both channels
    }
}

void MicrophoneArray::apply_adaptive_beamforming(std::vector<i16>& left, std::vector<i16>& right) {
    // Simplified adaptive beamforming (LMS-based)
    float adaptation_rate = beamforming_.adaptation_rate;
    
    for (size_t i = 0; i < left.size(); ++i) {
        // Current output
        float output = beamforming_weights_[0] * left[i] + beamforming_weights_[1] * right[i];
        
        // Error signal (simplified - assumes desired signal is in target direction)
        float error = output; // In practice, this would be more sophisticated
        
        // Update weights
        beamforming_weights_[0] -= adaptation_rate * error * left[i] / 32768.0f;
        beamforming_weights_[1] -= adaptation_rate * error * right[i] / 32768.0f;
        
        // Normalize weights
        float weight_sum = std::abs(beamforming_weights_[0]) + std::abs(beamforming_weights_[1]);
        if (weight_sum > 2.0f) {
            beamforming_weights_[0] /= weight_sum / 2.0f;
            beamforming_weights_[1] /= weight_sum / 2.0f;
        }
        
        // Apply beamformed output
        i16 beamformed = static_cast<i16>(std::clamp(output, -32768.0f, 32767.0f));
        left[i] = beamformed;
        right[i] = beamformed;
    }
}

void MicrophoneArray::apply_noise_cancellation(std::vector<i16>& left, std::vector<i16>& right) {
    // Simple spectral subtraction for noise cancellation
    for (size_t i = 0; i < left.size(); ++i) {
        // Estimate noise level
        float noise_estimate = statistics_.background_noise_level * 32768.0f;
        
        // Apply noise gate
        if (std::abs(left[i]) < noise_estimate * 2.0f) {
            left[i] = static_cast<i16>(left[i] * 0.1f); // Attenuate noise
        }
        
        if (std::abs(right[i]) < noise_estimate * 2.0f) {
            right[i] = static_cast<i16>(right[i] * 0.1f);
        }
    }
}

float MicrophoneArray::compute_zero_crossing_rate(const std::vector<i16>& samples) {
    if (samples.size() < 2) return 0.0f;
    
    int zero_crossings = 0;
    for (size_t i = 1; i < samples.size(); ++i) {
        if ((samples[i-1] >= 0 && samples[i] < 0) || (samples[i-1] < 0 && samples[i] >= 0)) {
            zero_crossings++;
        }
    }
    
    return static_cast<float>(zero_crossings) / (samples.size() - 1);
}

float MicrophoneArray::compute_spectral_centroid(const std::vector<i16>& samples) {
    auto spectrum = compute_fft(samples);
    
    float weighted_sum = 0.0f;
    float magnitude_sum = 0.0f;
    
    for (size_t i = 0; i < spectrum.size() / 2; ++i) {
        float magnitude = std::abs(spectrum[i]);
        float frequency = static_cast<float>(i) * DEFAULT_SAMPLE_RATE / spectrum.size();
        
        weighted_sum += frequency * magnitude;
        magnitude_sum += magnitude;
    }
    
    return magnitude_sum > 0.0f ? weighted_sum / magnitude_sum : 0.0f;
}

float MicrophoneArray::compute_energy_ratio(const std::vector<i16>& samples) {
    if (samples.empty()) return 0.0f;
    
    float total_energy = 0.0f;
    float high_energy = 0.0f;
    
    // Simple energy calculation
    for (i16 sample : samples) {
        float energy = static_cast<float>(sample * sample);
        total_energy += energy;
        
        // High-frequency energy (approximation)
        if (std::abs(sample) > 16384) { // Upper half of dynamic range
            high_energy += energy;
        }
    }
    
    return total_energy > 0.0f ? high_energy / total_energy : 0.0f;
}

void MicrophoneArray::clear_statistics() {
    std::lock_guard<std::mutex> lock(array_mutex_);
    statistics_ = {};
}

void MicrophoneArray::dump_status() const {
    std::lock_guard<std::mutex> lock(array_mutex_);
    
    COMPONENT_LOG_INFO("=== Microphone Array Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Microphones: {}/{} active", 
                          std::count_if(microphones_.begin(), microphones_.end(), 
                                       [](const auto& mic) { return mic.enabled; }),
                          microphones_.size());
        
        COMPONENT_LOG_INFO("Audio sources: {}/{} active",
                          std::count_if(audio_sources_.begin(), audio_sources_.end(),
                                       [](const auto& src) { return src.active; }),
                          audio_sources_.size());
        
        COMPONENT_LOG_INFO("Beamforming: mode={}, target={:.1f}°/{:.1f}°, width={:.1f}°",
                          static_cast<u8>(beamforming_.mode),
                          beamforming_.target_azimuth, beamforming_.target_elevation,
                          beamforming_.beam_width);
        
        COMPONENT_LOG_INFO("Noise profile: type={}, level={:.1f}dB",
                          static_cast<u8>(noise_profile_), noise_level_db_);
        
        COMPONENT_LOG_INFO("Wind noise: enabled={}, speed={:.1f}m/s",
                          wind_noise_enabled_, wind_speed_);
        
        for (size_t i = 0; i < microphones_.size(); ++i) {
            const auto& mic = microphones_[i];
            COMPONENT_LOG_INFO("  Mic{}: enabled={}, gain={:.1f}dB, muted={}, sensitivity={:.1f}dBFS",
                              i, mic.enabled, mic.gain_db, mic.muted, mic.sensitivity_dbfs);
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Samples processed: {}", statistics_.samples_processed);
        COMPONENT_LOG_INFO("  Frames captured: {}", statistics_.frames_captured);
        COMPONENT_LOG_INFO("  Beamforming updates: {}", statistics_.beamforming_updates);
        COMPONENT_LOG_INFO("  Voice detections: {}", statistics_.voice_detections);
        COMPONENT_LOG_INFO("  Input levels: L={:.3f}, R={:.3f}",
                          statistics_.average_input_level[0], statistics_.average_input_level[1]);
        COMPONENT_LOG_INFO("  SNR estimate: {:.1f}dB", statistics_.snr_estimate_db);
        COMPONENT_LOG_INFO("  Background noise: {:.3f}", statistics_.background_noise_level);
        COMPONENT_LOG_INFO("  Spatial resolution: {:.1f}°", statistics_.spatial_resolution_degrees);
    }
}

}  // namespace m5tab5::emulator