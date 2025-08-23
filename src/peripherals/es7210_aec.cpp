#include "emulator/peripherals/es7210_aec.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <complex>
#include <numeric>

namespace m5tab5::emulator {

DECLARE_LOGGER("ES7210_AEC");

ES7210_AEC::ES7210_AEC()
    : initialized_(false),
      capture_active_(false),
      aec_enabled_(false),
      noise_suppression_enabled_(false),
      beamforming_enabled_(false),
      input_mode_(MicrophoneInput::DUAL_MIC_STEREO),
      aec_mode_(AECMode::DISABLED),
      noise_suppression_level_(NoiseSuppressionLevel::OFF),
      sample_rate_(DEFAULT_SAMPLE_RATE),
      input_gain_(AECGain::GAIN_12DB),
      interrupt_controller_(nullptr),
      i2c_controller_(nullptr),
      main_codec_(nullptr),
      calibration_in_progress_(false),
      samples_per_update_(0) {
    
    // Initialize AEC filter
    aec_filter_.coefficients.fill(0.0f);
    aec_filter_.reference_delay_line.fill(0.0f);
    aec_filter_.error_delay_line.fill(0.0f);
    
    // Initialize noise processor
    noise_processor_.noise_spectrum.fill(-60.0f);
    noise_processor_.speech_spectrum.fill(-30.0f);
    noise_processor_.suppression_gains.fill(1.0f);
    
    // Initialize beamformer
    beamformer_state_.mic_delay_buffers[0].fill(0.0f);
    beamformer_state_.mic_delay_buffers[1].fill(0.0f);
    
    // Set default AEC parameters
    aec_parameters_.echo_delay_ms = 8.0f;
    aec_parameters_.adaptation_rate = 0.1f;
    aec_parameters_.suppression_factor = 0.3f;
    aec_parameters_.noise_gate_threshold = -60.0f;
    aec_parameters_.comfort_noise_level = -65.0f;
    
    COMPONENT_LOG_DEBUG("ES7210 AEC controller created");
}

ES7210_AEC::~ES7210_AEC() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("ES7210 AEC controller destroyed");
}

Result<void> ES7210_AEC::initialize(const Configuration& config,
                                   InterruptController* interrupt_controller,
                                   I2CController* i2c_controller,
                                   ES8388Codec* main_codec) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "ES7210 AEC already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing ES7210 AEC controller");
    
    interrupt_controller_ = interrupt_controller;
    i2c_controller_ = i2c_controller;
    main_codec_ = main_codec;
    
    // Reset all registers to default values
    registers_ = {};
    
    // Clear audio buffers
    while (!processed_buffer_.empty()) processed_buffer_.pop();
    while (!raw_buffer_ch1_.empty()) raw_buffer_ch1_.pop();
    while (!raw_buffer_ch2_.empty()) raw_buffer_ch2_.pop();
    while (!reference_buffer_.empty()) reference_buffer_.pop();
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_process_time_ = last_update_;
    samples_per_update_ = sample_rate_ / 60; // 60 FPS updates
    
    // Clear statistics
    statistics_ = {};
    
    // Configure for dual microphone operation
    registers_.record_control1 = 0x34; // Enable CH1 and CH2
    registers_.record_control2 = 0x07; // 16-bit, I2S format
    registers_.mic_gain_1 = 0x2E;      // +12dB gain
    registers_.mic_gain_2 = 0x2E;      // +12dB gain
    
    initialized_ = true;
    COMPONENT_LOG_INFO("ES7210 AEC controller initialized successfully");
    
    return {};
}

Result<void> ES7210_AEC::shutdown() {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down ES7210 AEC controller");
    
    // Stop audio capture
    capture_active_ = false;
    aec_enabled_ = false;
    noise_suppression_enabled_ = false;
    beamforming_enabled_ = false;
    
    // Clear buffers
    while (!processed_buffer_.empty()) processed_buffer_.pop();
    while (!raw_buffer_ch1_.empty()) raw_buffer_ch1_.pop();
    while (!raw_buffer_ch2_.empty()) raw_buffer_ch2_.pop();
    while (!reference_buffer_.empty()) reference_buffer_.pop();
    
    interrupt_controller_ = nullptr;
    i2c_controller_ = nullptr;
    main_codec_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("ES7210 AEC controller shutdown completed");
    return {};
}

Result<void> ES7210_AEC::set_microphone_input(MicrophoneInput input) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    if (capture_active_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot change input mode while capture is active"));
    }
    
    input_mode_ = input;
    
    // Update register configuration based on input mode
    switch (input) {
        case MicrophoneInput::MIC1_SINGLE:
            registers_.record_control1 = 0x30; // Enable CH1 only
            beamforming_enabled_ = false;
            break;
        case MicrophoneInput::MIC2_SINGLE:
            registers_.record_control1 = 0x34; // Enable CH2 only  
            beamforming_enabled_ = false;
            break;
        case MicrophoneInput::DUAL_MIC_STEREO:
            registers_.record_control1 = 0x34; // Enable CH1 and CH2
            beamforming_enabled_ = false;
            break;
        case MicrophoneInput::DUAL_MIC_BEAMFORMING:
            registers_.record_control1 = 0x34; // Enable CH1 and CH2
            beamforming_enabled_ = true;
            break;
        case MicrophoneInput::DISABLED:
            registers_.record_control1 = 0x00; // Disable all channels
            break;
    }
    
    COMPONENT_LOG_INFO("ES7210 microphone input set to {}", static_cast<u8>(input));
    return {};
}

Result<void> ES7210_AEC::configure_aec(AECMode mode, const AECParameters& params) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    aec_mode_ = mode;
    aec_parameters_ = params;
    
    // Configure AEC register based on mode
    switch (mode) {
        case AECMode::DISABLED:
            registers_.aec_control = 0x00;
            aec_enabled_ = false;
            break;
        case AECMode::BASIC_AEC:
            registers_.aec_control = 0x01;
            aec_enabled_ = true;
            break;
        case AECMode::ADVANCED_AEC:
            registers_.aec_control = 0x03;
            aec_enabled_ = true;
            break;
        case AECMode::NOISE_SUPPRESSION:
            registers_.aec_control = 0x04;
            aec_enabled_ = false;
            noise_suppression_enabled_ = true;
            break;
        case AECMode::FULL_AEC_NS:
            registers_.aec_control = 0x07;
            aec_enabled_ = true;
            noise_suppression_enabled_ = true;
            break;
    }
    
    // Reset AEC filter with new parameters
    aec_filter_.adaptation_step_size = params.adaptation_rate;
    aec_filter_.convergence_factor = 0.0f;
    aec_filter_.coefficients.fill(0.0f);
    
    COMPONENT_LOG_INFO("ES7210 AEC configured: mode={}, adaptation_rate={:.3f}",
                      static_cast<u8>(mode), params.adaptation_rate);
    
    return {};
}

Result<void> ES7210_AEC::start_capture() {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    if (input_mode_ == MicrophoneInput::DISABLED) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Cannot start capture with disabled microphone input"));
    }
    
    capture_active_ = true;
    registers_.record_control1 |= 0x80; // Set capture enable bit
    
    // Clear buffers
    while (!processed_buffer_.empty()) processed_buffer_.pop();
    while (!raw_buffer_ch1_.empty()) raw_buffer_ch1_.pop();
    while (!raw_buffer_ch2_.empty()) raw_buffer_ch2_.pop();
    
    COMPONENT_LOG_INFO("ES7210 audio capture started");
    return {};
}

Result<void> ES7210_AEC::stop_capture() {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    capture_active_ = false;
    registers_.record_control1 &= ~0x80; // Clear capture enable bit
    
    COMPONENT_LOG_INFO("ES7210 audio capture stopped");
    return {};
}

Result<std::vector<i16>> ES7210_AEC::read_processed_audio(size_t max_samples) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    if (!capture_active_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Audio capture not active"));
    }
    
    std::vector<i16> samples;
    size_t count = (max_samples == 0) ? processed_buffer_.size() : 
                   std::min(max_samples, processed_buffer_.size());
    
    samples.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        if (!processed_buffer_.empty()) {
            samples.push_back(processed_buffer_.front());
            processed_buffer_.pop();
        }
    }
    
    return samples;
}

Result<void> ES7210_AEC::set_reference_audio(const std::vector<i16>& reference_samples) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    // Add reference samples to buffer for AEC processing
    for (i16 sample : reference_samples) {
        if (reference_buffer_.size() >= AUDIO_BUFFER_SIZE) {
            reference_buffer_.pop(); // Remove oldest sample
        }
        reference_buffer_.push(sample);
    }
    
    return {};
}

Result<void> ES7210_AEC::write_register(u8 reg_addr, u8 value) {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    // Update the appropriate register (simplified implementation)
    switch (reg_addr) {
        case 0x00: registers_.reset_control = value; break;
        case 0x01: registers_.clock_control = value; break;
        case 0x02: registers_.record_control1 = value; break;
        case 0x03: registers_.record_control2 = value; break;
        case 0x04: registers_.record_control3 = value; break;
        case 0x05: registers_.mic_gain_1 = value; break;
        case 0x06: registers_.mic_gain_2 = value; break;
        case 0x0C: registers_.aec_control = value; break;
        case 0x0D: registers_.noise_gate = value; break;
        case 0x0E: registers_.interrupt_mask = value; break;
        default:
            COMPONENT_LOG_WARN("ES7210 write to unimplemented register 0x{:02X} = 0x{:02X}", 
                              reg_addr, value);
            break;
    }
    
    return {};
}

Result<u8> ES7210_AEC::read_register(u8 reg_addr) const {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES7210 AEC not initialized"));
    }
    
    switch (reg_addr) {
        case 0x00: return registers_.reset_control;
        case 0x01: return registers_.clock_control;
        case 0x02: return registers_.record_control1;
        case 0x03: return registers_.record_control2;
        case 0x04: return registers_.record_control3;
        case 0x05: return registers_.mic_gain_1;
        case 0x06: return registers_.mic_gain_2;
        case 0x0C: return registers_.aec_control;
        case 0x0D: return registers_.noise_gate;
        case 0x0E: return registers_.interrupt_mask;
        case 0x0F: return registers_.interrupt_status;
        default:
            COMPONENT_LOG_WARN("ES7210 read from unimplemented register 0x{:02X}", reg_addr);
            return 0x00;
    }
}

void ES7210_AEC::update() {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    if (!initialized_ || !capture_active_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);
    last_update_ = now;
    
    // Update audio processing at sample rate intervals
    auto process_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_process_time_);
    u32 process_period_us = 1000000 / (sample_rate_ / samples_per_update_);
    
    if (process_elapsed.count() >= process_period_us) {
        process_audio_pipeline();
        last_process_time_ = now;
    }
}

void ES7210_AEC::process_audio_pipeline() {
    // Capture microphone data
    capture_microphone_data();
    
    if (!raw_buffer_ch1_.empty() && !raw_buffer_ch2_.empty()) {
        // Get samples for processing
        std::vector<i16> left_samples, right_samples;
        
        size_t samples_to_process = std::min({
            static_cast<size_t>(samples_per_update_),
            raw_buffer_ch1_.size(), 
            raw_buffer_ch2_.size()
        });
        
        for (size_t i = 0; i < samples_to_process; ++i) {
            left_samples.push_back(raw_buffer_ch1_.front());
            right_samples.push_back(raw_buffer_ch2_.front());
            raw_buffer_ch1_.pop();
            raw_buffer_ch2_.pop();
        }
        
        // Apply beamforming if enabled
        if (beamforming_enabled_) {
            apply_beamforming(left_samples, right_samples);
        }
        
        // Use left channel as primary for further processing
        std::vector<i16> processed_samples = left_samples;
        
        // Apply AEC if enabled
        if (aec_enabled_) {
            apply_echo_cancellation(processed_samples);
        }
        
        // Apply noise suppression if enabled
        if (noise_suppression_enabled_) {
            apply_noise_suppression(processed_samples);
        }
        
        // Add processed samples to output buffer
        for (i16 sample : processed_samples) {
            if (processed_buffer_.size() >= AUDIO_BUFFER_SIZE) {
                processed_buffer_.pop(); // Remove oldest sample
                statistics_.overload_events++;
            }
            processed_buffer_.push(sample);
        }
        
        statistics_.samples_processed += processed_samples.size();
    }
}

void ES7210_AEC::capture_microphone_data() {
    // Simulate dual microphone input
    std::vector<i16> mic1_samples(samples_per_update_);
    std::vector<i16> mic2_samples(samples_per_update_);
    
    simulate_microphone_input(mic1_samples, 0);
    simulate_microphone_input(mic2_samples, 1);
    
    // Add samples to raw buffers
    for (size_t i = 0; i < samples_per_update_; ++i) {
        if (raw_buffer_ch1_.size() < AUDIO_BUFFER_SIZE) {
            raw_buffer_ch1_.push(mic1_samples[i]);
        }
        if (raw_buffer_ch2_.size() < AUDIO_BUFFER_SIZE) {
            raw_buffer_ch2_.push(mic2_samples[i]);
        }
    }
}

void ES7210_AEC::apply_echo_cancellation(std::vector<i16>& samples) {
    // Convert to float for processing
    std::vector<float> float_samples;
    for (i16 sample : samples) {
        float_samples.push_back(i16_to_float(sample));
    }
    
    // Get reference signal (from main codec if available)
    std::vector<float> reference_samples;
    size_t ref_samples_needed = std::min(samples.size(), reference_buffer_.size());
    
    for (size_t i = 0; i < ref_samples_needed; ++i) {
        reference_samples.push_back(i16_to_float(reference_buffer_.front()));
        reference_buffer_.pop();
    }
    
    // Pad with zeros if not enough reference samples
    while (reference_samples.size() < float_samples.size()) {
        reference_samples.push_back(0.0f);
    }
    
    // Apply NLMS adaptive filter
    std::vector<float> output_samples;
    nlms_adaptive_filter(reference_samples, float_samples, output_samples);
    
    // Convert back to i16
    for (size_t i = 0; i < samples.size() && i < output_samples.size(); ++i) {
        samples[i] = float_to_i16(output_samples[i]);
    }
    
    statistics_.echo_samples_cancelled += samples.size();
}

void ES7210_AEC::apply_noise_suppression(std::vector<i16>& samples) {
    // Convert to float for processing
    std::vector<float> float_samples;
    for (i16 sample : samples) {
        float_samples.push_back(i16_to_float(sample));
    }
    
    // Apply spectral subtraction
    spectral_subtraction(float_samples);
    
    // Convert back to i16
    for (size_t i = 0; i < samples.size(); ++i) {
        samples[i] = float_to_i16(float_samples[i]);
    }
    
    statistics_.noise_samples_suppressed += samples.size();
}

void ES7210_AEC::apply_beamforming(std::vector<i16>& left_samples, std::vector<i16>& right_samples) {
    // Convert to float for processing
    std::vector<float> left_float, right_float, output_float;
    
    for (size_t i = 0; i < left_samples.size(); ++i) {
        left_float.push_back(i16_to_float(left_samples[i]));
        right_float.push_back(i16_to_float(right_samples[i]));
    }
    
    // Apply delay-and-sum beamforming
    delay_and_sum_beamforming(left_float, right_float, output_float);
    
    // Update left channel with beamformed output
    for (size_t i = 0; i < left_samples.size() && i < output_float.size(); ++i) {
        left_samples[i] = float_to_i16(output_float[i]);
    }
}

void ES7210_AEC::nlms_adaptive_filter(const std::vector<float>& reference, 
                                     const std::vector<float>& input,
                                     std::vector<float>& output) {
    output.resize(input.size());
    
    for (size_t n = 0; n < input.size(); ++n) {
        // Calculate filter output
        float filter_output = 0.0f;
        for (size_t k = 0; k < std::min(n + 1, AEC_FILTER_TAPS); ++k) {
            if (n >= k && k < reference.size()) {
                filter_output += aec_filter_.coefficients[k] * reference[n - k];
            }
        }
        
        // Calculate error signal
        float error = input[n] - filter_output;
        output[n] = error;
        
        // Update filter coefficients (NLMS algorithm)
        float power = 1e-6f; // Small regularization constant
        for (size_t k = 0; k < std::min(n + 1, AEC_FILTER_TAPS); ++k) {
            if (n >= k && k < reference.size()) {
                power += reference[n - k] * reference[n - k];
            }
        }
        
        float step_size = aec_filter_.adaptation_step_size / power;
        
        for (size_t k = 0; k < std::min(n + 1, AEC_FILTER_TAPS); ++k) {
            if (n >= k && k < reference.size()) {
                aec_filter_.coefficients[k] += step_size * error * reference[n - k];
            }
        }
    }
}

void ES7210_AEC::spectral_subtraction(std::vector<float>& samples) {
    // Simple noise gate implementation
    float threshold_linear = db_to_linear(aec_parameters_.noise_gate_threshold);
    
    for (float& sample : samples) {
        float abs_sample = std::abs(sample);
        if (abs_sample < threshold_linear) {
            // Apply suppression
            sample *= aec_parameters_.suppression_factor;
        }
    }
}

void ES7210_AEC::delay_and_sum_beamforming(const std::vector<float>& mic1_samples,
                                          const std::vector<float>& mic2_samples,
                                          std::vector<float>& output_samples) {
    output_samples.resize(mic1_samples.size());
    
    // Simple delay-and-sum beamforming
    for (size_t i = 0; i < mic1_samples.size(); ++i) {
        float sample1 = mic1_samples[i] * beamformer_state_.beam_weights[0];
        float sample2 = mic2_samples[i] * beamformer_state_.beam_weights[1];
        output_samples[i] = (sample1 + sample2) * 0.5f; // Average
    }
}

void ES7210_AEC::simulate_microphone_input(std::vector<i16>& samples, u8 channel) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> noise_dist(0.0f, 0.05f);
    static std::uniform_real_distribution<float> signal_dist(-0.2f, 0.2f);
    
    for (size_t i = 0; i < samples.size(); ++i) {
        // Simulate environmental noise + occasional voice signal
        float signal = noise_dist(gen);
        
        // Add occasional voice-like signal bursts
        if (gen() % 2000 < 5) {
            signal += signal_dist(gen);
        }
        
        // Add slight delay/phase difference for second microphone
        if (channel == 1) {
            signal *= 0.95f; // Slight amplitude difference
        }
        
        samples[i] = float_to_i16(signal);
    }
}

void ES7210_AEC::trigger_interrupt(ES7210InterruptType interrupt_type) {
    if (interrupt_controller_) {
        // Map ES7210 interrupts to system interrupts
        interrupt_controller_->trigger_interrupt(InterruptType::I2C0); // ES7210 uses I2C
    }
    
    // Update interrupt status register
    registers_.interrupt_status |= static_cast<u8>(interrupt_type);
}

float ES7210_AEC::db_to_linear(float db) const {
    return std::pow(10.0f, db / 20.0f);
}

float ES7210_AEC::linear_to_db(float linear) const {
    return 20.0f * std::log10(std::max(linear, 1e-6f));
}

i16 ES7210_AEC::float_to_i16(float sample) const {
    return static_cast<i16>(std::clamp(sample * 32767.0f, -32768.0f, 32767.0f));
}

float ES7210_AEC::i16_to_float(i16 sample) const {
    return static_cast<float>(sample) / 32768.0f;
}

void ES7210_AEC::clear_statistics() {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    statistics_ = {};
}

void ES7210_AEC::dump_status() const {
    std::lock_guard<std::mutex> lock(aec_mutex_);
    
    COMPONENT_LOG_INFO("=== ES7210 AEC Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Capture active: {}", capture_active_);
        COMPONENT_LOG_INFO("AEC enabled: {}", aec_enabled_);
        COMPONENT_LOG_INFO("Noise suppression: {}", noise_suppression_enabled_);
        COMPONENT_LOG_INFO("Beamforming: {}", beamforming_enabled_);
        COMPONENT_LOG_INFO("Input mode: {}", static_cast<u8>(input_mode_));
        COMPONENT_LOG_INFO("Sample rate: {} Hz", sample_rate_);
        COMPONENT_LOG_INFO("Input gain: {}", static_cast<u8>(input_gain_));
        
        COMPONENT_LOG_INFO("Buffers: processed={}, raw_ch1={}, raw_ch2={}, reference={}",
                          processed_buffer_.size(), raw_buffer_ch1_.size(),
                          raw_buffer_ch2_.size(), reference_buffer_.size());
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Samples processed: {}", statistics_.samples_processed);
        COMPONENT_LOG_INFO("  Echo samples cancelled: {}", statistics_.echo_samples_cancelled);
        COMPONENT_LOG_INFO("  Noise samples suppressed: {}", statistics_.noise_samples_suppressed);
        COMPONENT_LOG_INFO("  Double talk detections: {}", statistics_.double_talk_detections);
        COMPONENT_LOG_INFO("  Processing load: {:.1f}%", statistics_.average_processing_load);
        COMPONENT_LOG_INFO("  Echo return loss: {:.1f} dB", statistics_.current_echo_return_loss_db);
    }
}

}  // namespace m5tab5::emulator