#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/peripherals/es8388_codec.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <atomic>
#include <array>
#include <complex>
#include <algorithm>

namespace m5tab5::emulator {

enum class AECMode : u8 {
    DISABLED = 0,
    BASIC_AEC = 1,
    ADVANCED_AEC = 2,
    NOISE_SUPPRESSION = 3,
    FULL_AEC_NS = 4
};

enum class MicrophoneInput : u8 {
    MIC1_SINGLE = 0,
    MIC2_SINGLE = 1,
    DUAL_MIC_STEREO = 2,
    DUAL_MIC_BEAMFORMING = 3,
    DISABLED = 255
};

enum class AECGain : u8 {
    GAIN_0DB = 0,
    GAIN_6DB = 1,
    GAIN_12DB = 2,
    GAIN_18DB = 3,
    GAIN_24DB = 4,
    GAIN_30DB = 5
};

enum class NoiseSuppressionLevel : u8 {
    OFF = 0,
    LOW = 1,
    MODERATE = 2,
    HIGH = 3,
    VERY_HIGH = 4
};

enum class ES7210InterruptType : u8 {
    ADC_DATA_READY = 0x01,
    AEC_CONVERGED = 0x02,
    NOISE_LEVEL_HIGH = 0x04,
    MICROPHONE_FAULT = 0x08,
    OVERLOAD_DETECT = 0x10,
    CLOCK_ERROR = 0x20,
    FIFO_OVERFLOW = 0x40,
    THERMAL_WARNING = 0x80
};

struct AECParameters {
    float echo_delay_ms = 8.0f;                  // Echo delay estimation (ms)
    float adaptation_rate = 0.1f;                // AEC adaptation speed (0.01-1.0)
    float suppression_factor = 0.3f;             // Echo suppression factor (0.1-0.9)
    float noise_gate_threshold = -60.0f;        // Noise gate threshold (dBFS)
    float comfort_noise_level = -65.0f;         // Comfort noise level (dBFS)
    bool enable_double_talk_detection = true;   // Detect simultaneous speech
    bool enable_howling_suppression = true;     // Prevent acoustic feedback
    float filter_length_ms = 64.0f;            // Adaptive filter length (ms)
};

struct MicrophoneCalibration {
    float mic1_gain_compensation = 1.0f;        // Individual mic gain matching
    float mic2_gain_compensation = 1.0f;
    float mic1_phase_delay = 0.0f;              // Phase alignment (samples)
    float mic2_phase_delay = 0.0f;
    float sensitivity_matching = 1.0f;          // Sensitivity matching factor
    float frequency_response_correction[32] = {1.0f}; // EQ correction per band
};

struct BeamformingConfig {
    float mic_spacing_mm = 65.0f;               // Physical microphone spacing
    float beam_angle_degrees = 0.0f;           // Main beam direction (±90°)
    float beam_width_degrees = 60.0f;          // Beam width (3dB point)
    float null_depth_db = -20.0f;              // Null steering depth
    bool enable_adaptive_beamforming = true;   // Dynamic beam steering
    bool enable_wind_noise_reduction = true;   // Wind noise suppression
};

struct AECStatistics {
    u64 samples_processed = 0;
    u64 echo_samples_cancelled = 0;
    u64 noise_samples_suppressed = 0;
    u64 double_talk_detections = 0;
    u64 microphone_faults = 0;
    u64 overload_events = 0;
    float current_echo_return_loss_db = -20.0f;  // Echo cancellation quality
    float current_noise_level_dbfs = -60.0f;     // Background noise level
    float current_speech_level_dbfs = -30.0f;    // Speech signal level
    float adaptation_convergence = 0.0f;         // AEC convergence (0-1)
    double average_processing_load = 0.0;        // CPU load percentage
};

class ES7210_AEC {
public:
    static constexpr u8 I2C_ADDRESS = 0x40;     // ES7210 I2C address
    static constexpr size_t AUDIO_BUFFER_SIZE = 2048;
    static constexpr size_t MAX_CHANNELS = 4;    // 4-channel ADC
    static constexpr u32 DEFAULT_SAMPLE_RATE = 16000; // Optimal for voice
    static constexpr size_t AEC_FILTER_TAPS = 512;   // Adaptive filter length
    
    ES7210_AEC();
    ~ES7210_AEC();

    Result<void> initialize(const Configuration& config, 
                           InterruptController* interrupt_controller,
                           I2CController* i2c_controller,
                           ES8388Codec* main_codec = nullptr);
    Result<void> shutdown();

    // Configuration
    Result<void> set_microphone_input(MicrophoneInput input);
    Result<void> set_sample_rate(u32 sample_rate);
    Result<void> set_input_gain(AECGain gain);
    Result<void> configure_aec(AECMode mode, const AECParameters& params);
    Result<void> configure_noise_suppression(NoiseSuppressionLevel level);
    Result<void> configure_beamforming(const BeamformingConfig& config);
    
    // Calibration
    Result<void> start_microphone_calibration();
    Result<bool> is_calibration_complete() const;
    Result<void> apply_calibration(const MicrophoneCalibration& calibration);
    Result<MicrophoneCalibration> get_calibration() const;
    Result<void> save_calibration();
    Result<void> load_calibration();
    
    // Audio processing
    Result<void> start_capture();
    Result<void> stop_capture();
    Result<void> enable_aec(bool enable);
    Result<void> enable_noise_suppression(bool enable);
    Result<void> enable_beamforming(bool enable);
    
    // Data interface
    Result<std::vector<i16>> read_processed_audio(size_t max_samples = 0);
    Result<std::vector<i16>> read_raw_audio(u8 channel, size_t max_samples = 0);
    Result<void> set_reference_audio(const std::vector<i16>& reference_samples);
    
    // Real-time processing control
    Result<void> set_adaptation_rate(float rate);
    Result<void> reset_aec_filter();
    Result<void> freeze_adaptation(bool freeze);
    Result<float> get_echo_return_loss() const;
    Result<float> get_noise_suppression_level() const;
    
    // Advanced features
    Result<bool> detect_double_talk() const;
    Result<void> set_comfort_noise_level(float level_dbfs);
    Result<void> enable_howling_suppression(bool enable);
    Result<void> set_beam_direction(float angle_degrees);
    
    // Voice activity detection
    Result<bool> is_speech_detected() const;
    Result<float> get_speech_probability() const;
    Result<void> set_vad_sensitivity(float sensitivity);
    
    // I2C register interface
    Result<void> write_register(u8 reg_addr, u8 value);
    Result<u8> read_register(u8 reg_addr) const;
    
    // Interrupt handling
    Result<void> enable_interrupt(ES7210InterruptType interrupt_type);
    Result<void> disable_interrupt(ES7210InterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_capturing() const { return capture_active_; }
    bool is_aec_enabled() const { return aec_enabled_; }
    AECMode get_aec_mode() const { return aec_mode_; }
    const AECStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct ES7210Registers {
        u8 reset_control = 0x00;       // 0x00 - Reset and power control
        u8 clock_control = 0x30;       // 0x01 - Clock configuration
        u8 record_control1 = 0x34;     // 0x02 - Recording control 1
        u8 record_control2 = 0x07;     // 0x03 - Recording control 2
        u8 record_control3 = 0x20;     // 0x04 - Recording control 3
        u8 mic_gain_1 = 0x2E;          // 0x05 - Microphone 1 gain
        u8 mic_gain_2 = 0x2E;          // 0x06 - Microphone 2 gain
        u8 mic_gain_3 = 0x2E;          // 0x07 - Microphone 3 gain (unused)
        u8 mic_gain_4 = 0x2E;          // 0x08 - Microphone 4 gain (unused)
        u8 mic_bias = 0x41;            // 0x09 - Microphone bias control
        u8 format_control = 0x00;      // 0x0A - Audio format control
        u8 hpf_control = 0x04;         // 0x0B - High-pass filter control
        u8 aec_control = 0x00;         // 0x0C - AEC control register
        u8 noise_gate = 0x12;          // 0x0D - Noise gate control
        u8 interrupt_mask = 0xFF;      // 0x0E - Interrupt mask
        u8 interrupt_status = 0x00;    // 0x0F - Interrupt status (read-only)
    };

    struct AECFilter {
        std::array<float, AEC_FILTER_TAPS> coefficients;
        std::array<float, AEC_FILTER_TAPS> reference_delay_line;
        std::array<float, AEC_FILTER_TAPS> error_delay_line;
        float adaptation_step_size = 0.01f;
        float convergence_factor = 0.0f;
        size_t delay_line_index = 0;
        bool is_converged = false;
    };

    struct NoiseProcessor {
        std::array<float, 256> noise_spectrum;
        std::array<float, 256> speech_spectrum;
        std::array<float, 256> suppression_gains;
        float noise_floor_estimate = -60.0f;
        float speech_presence_probability = 0.0f;
        bool vad_active = false;
    };

    struct BeamformerState {
        std::array<std::array<float, 256>, 2> mic_delay_buffers; // Per microphone
        float beam_weights[2] = {0.707f, 0.707f}; // Stereo weights
        float steering_angle = 0.0f;
        float null_angles[4] = {0.0f}; // Null steering directions
        bool adaptive_enabled = false;
    };

    // Core processing methods
    void process_audio_pipeline();
    void capture_microphone_data();
    void apply_echo_cancellation(std::vector<i16>& samples);
    void apply_noise_suppression(std::vector<i16>& samples);
    void apply_beamforming(std::vector<i16>& left_samples, std::vector<i16>& right_samples);
    
    // AEC algorithm implementation
    void lms_adaptive_filter(const std::vector<float>& reference, 
                           const std::vector<float>& input,
                           std::vector<float>& output);
    void nlms_adaptive_filter(const std::vector<float>& reference, 
                            const std::vector<float>& input,
                            std::vector<float>& output);
    void update_filter_coefficients(const std::vector<float>& error_signal,
                                  const std::vector<float>& reference_signal);
    
    // Noise suppression algorithms
    void spectral_subtraction(std::vector<float>& samples);
    void wiener_filtering(std::vector<float>& samples);
    void update_noise_estimate(const std::vector<float>& samples);
    
    // Beamforming algorithms
    void delay_and_sum_beamforming(const std::vector<float>& mic1_samples,
                                 const std::vector<float>& mic2_samples,
                                 std::vector<float>& output_samples);
    void adaptive_beamforming(const std::vector<float>& mic1_samples,
                            const std::vector<float>& mic2_samples,
                            std::vector<float>& output_samples);
    void calculate_beam_weights(float target_angle);
    
    // Voice activity detection
    void update_voice_activity_detection(const std::vector<float>& samples);
    float calculate_speech_probability(const std::vector<float>& samples);
    bool detect_double_talk_condition(const std::vector<float>& near_end,
                                    const std::vector<float>& far_end);
    
    // Signal processing utilities
    void fft_forward(const std::vector<float>& time_domain, 
                    std::vector<std::complex<float>>& frequency_domain);
    void fft_inverse(const std::vector<std::complex<float>>& frequency_domain,
                    std::vector<float>& time_domain);
    float calculate_signal_level_dbfs(const std::vector<float>& samples);
    void apply_window_function(std::vector<float>& samples, const std::string& window_type);
    
    // Calibration algorithms
    void auto_gain_calibration();
    void phase_alignment_calibration();
    void frequency_response_calibration();
    
    // Simulation helpers
    void simulate_microphone_input(std::vector<i16>& samples, u8 channel);
    void simulate_echo_path(const std::vector<i16>& reference, 
                          std::vector<i16>& echo_samples);
    void add_background_noise(std::vector<i16>& samples, float noise_level_dbfs);
    
    void trigger_interrupt(ES7210InterruptType interrupt_type);
    float db_to_linear(float db) const;
    float linear_to_db(float linear) const;
    i16 float_to_i16(float sample) const;
    float i16_to_float(i16 sample) const;

    bool initialized_;
    bool capture_active_;
    bool aec_enabled_;
    bool noise_suppression_enabled_;
    bool beamforming_enabled_;
    
    MicrophoneInput input_mode_;
    AECMode aec_mode_;
    NoiseSuppressionLevel noise_suppression_level_;
    u32 sample_rate_;
    AECGain input_gain_;
    
    ES7210Registers registers_;
    InterruptController* interrupt_controller_;
    I2CController* i2c_controller_;
    ES8388Codec* main_codec_;
    
    // Processing state
    AECFilter aec_filter_;
    NoiseProcessor noise_processor_;
    BeamformerState beamformer_state_;
    AECParameters aec_parameters_;
    BeamformingConfig beamforming_config_;
    MicrophoneCalibration calibration_;
    
    // Audio buffers
    std::queue<i16> processed_buffer_;
    std::queue<i16> raw_buffer_ch1_;
    std::queue<i16> raw_buffer_ch2_;
    std::queue<i16> reference_buffer_;
    
    // Calibration state
    bool calibration_in_progress_;
    std::chrono::steady_clock::time_point calibration_start_;
    std::vector<std::vector<i16>> calibration_samples_;
    
    // Timing and synchronization
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_process_time_;
    u32 samples_per_update_;
    
    AECStatistics statistics_;
    
    mutable std::mutex aec_mutex_;
};

}  // namespace m5tab5::emulator