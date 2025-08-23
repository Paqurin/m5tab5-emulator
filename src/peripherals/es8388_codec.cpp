#include "emulator/peripherals/es8388_codec.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace m5tab5::emulator {

DECLARE_LOGGER("ES8388Codec");

ES8388Codec::ES8388Codec()
    : initialized_(false),
      powered_on_(false),
      low_power_mode_(false),
      format_(AudioFormat::I2S_PHILIPS),
      sample_rate_(AudioSampleRate::RATE_44100),
      bit_depth_(AudioBitDepth::BITS_16),
      left_input_(AudioInput::MIC1),
      right_input_(AudioInput::MIC2),
      output_destination_(AudioOutput::HEADPHONE),
      interrupt_controller_(nullptr),
      playback_active_(false),
      recording_active_(false),
      headphone_connected_(true),
      microphone_connected_(true),
      samples_per_update_(0) {
    
    COMPONENT_LOG_DEBUG("ES8388 codec created");
}

ES8388Codec::~ES8388Codec() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("ES8388 codec destroyed");
}

Result<void> ES8388Codec::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "ES8388 codec already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing ES8388 audio codec");
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset all registers to default values
    registers_ = {};
    
    // Clear audio buffers
    while (!playback_buffer_.empty()) playback_buffer_.pop();
    while (!recording_buffer_.empty()) recording_buffer_.pop();
    
    // Reset mixer to default values
    mixer_ = {};
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_sample_time_ = last_update_;
    samples_per_update_ = static_cast<u32>(sample_rate_) / 60; // 60 FPS updates
    
    // Clear statistics
    statistics_ = {};
    
    // Power on by default
    powered_on_ = true;
    registers_.chip_power = 0x00; // All blocks powered on
    
    initialized_ = true;
    COMPONENT_LOG_INFO("ES8388 codec initialized successfully");
    
    return {};
}

Result<void> ES8388Codec::shutdown() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down ES8388 codec");
    
    // Stop audio streaming
    playback_active_ = false;
    recording_active_ = false;
    
    // Power off
    powered_on_ = false;
    
    // Clear buffers
    while (!playback_buffer_.empty()) playback_buffer_.pop();
    while (!recording_buffer_.empty()) recording_buffer_.pop();
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("ES8388 codec shutdown completed");
    return {};
}

Result<void> ES8388Codec::configure_format(AudioFormat format, AudioSampleRate sample_rate, AudioBitDepth bit_depth) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (playback_active_ || recording_active_) {
        return unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot change format while audio is active"));
    }
    
    format_ = format;
    sample_rate_ = sample_rate;
    bit_depth_ = bit_depth;
    
    // Update ADC control registers
    registers_.adc_control4 = (static_cast<u8>(format) << 1) | 
                             (static_cast<u8>(bit_depth) == 16 ? 0x00 :
                              static_cast<u8>(bit_depth) == 18 ? 0x40 :
                              static_cast<u8>(bit_depth) == 20 ? 0x80 :
                              static_cast<u8>(bit_depth) == 24 ? 0xC0 : 0x00);
    
    // Update DAC control registers
    registers_.dac_control1 = registers_.adc_control4; // Same format for DAC
    
    // Calculate samples per update based on sample rate
    samples_per_update_ = static_cast<u32>(sample_rate_) / 60;
    
    COMPONENT_LOG_INFO("ES8388 codec format configured: {} Hz, {} bits, format={}",
                      static_cast<u32>(sample_rate_), static_cast<u8>(bit_depth_), 
                      static_cast<u8>(format_));
    
    return {};
}

Result<void> ES8388Codec::set_input_source(AudioInput left, AudioInput right) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    left_input_ = left;
    right_input_ = right;
    
    // Update ADC input selection registers
    registers_.adc_control2 = (static_cast<u8>(left) << 4) | static_cast<u8>(right);
    
    COMPONENT_LOG_DEBUG("ES8388 input sources: left={}, right={}", 
                       static_cast<u8>(left), static_cast<u8>(right));
    
    return {};
}

Result<void> ES8388Codec::set_output_destination(AudioOutput output) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    output_destination_ = output;
    
    // Update DAC output routing registers
    switch (output) {
        case AudioOutput::HEADPHONE:
            registers_.dac_control16 = 0x00; // Route to headphone
            break;
        case AudioOutput::SPEAKER:
            registers_.dac_control16 = 0x09; // Route to speaker
            break;
        case AudioOutput::LINEOUT:
            registers_.dac_control16 = 0x06; // Route to line out
            break;
        case AudioOutput::DISABLED:
            registers_.dac_control16 = 0xFF; // Disable output
            break;
    }
    
    COMPONENT_LOG_DEBUG("ES8388 output destination set to {}", static_cast<u8>(output));
    return {};
}

Result<void> ES8388Codec::set_input_gain(AudioInput input, float gain_db) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (gain_db < -12.0f || gain_db > 30.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Input gain out of range (-12dB to +30dB)"));
    }
    
    if (static_cast<u8>(input) < 4) {
        mixer_.input_gain[static_cast<u8>(input)] = db_to_linear(gain_db);
        
        // Update register (simplified mapping)
        u8 gain_reg = static_cast<u8>(std::clamp((gain_db + 12.0f) * 255.0f / 42.0f, 0.0f, 255.0f));
        
        switch (input) {
            case AudioInput::MIC1:
                registers_.adc_control8 = gain_reg;
                break;
            case AudioInput::MIC2:
                registers_.adc_control9 = gain_reg;
                break;
            case AudioInput::LINEIN1:
                registers_.adc_control10 = gain_reg;
                break;
            case AudioInput::LINEIN2:
                registers_.adc_control11 = gain_reg;
                break;
            default:
                break;
        }
    }
    
    COMPONENT_LOG_DEBUG("ES8388 input gain set: input={}, gain={:.1f}dB", 
                       static_cast<u8>(input), gain_db);
    
    return {};
}

Result<void> ES8388Codec::set_output_volume(AudioOutput output, float volume_percent) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (volume_percent < 0.0f || volume_percent > 100.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Volume out of range (0-100%)"));
    }
    
    float volume_linear = volume_percent / 100.0f;
    
    if (static_cast<u8>(output) < 3) {
        mixer_.output_volume[static_cast<u8>(output)] = volume_linear;
        statistics_.volume_changes++;
        
        // Update volume registers
        u8 volume_reg = static_cast<u8>(volume_linear * 63); // 0-63 range
        
        switch (output) {
            case AudioOutput::HEADPHONE:
                registers_.lout1_volume = volume_reg;
                registers_.rout1_volume = volume_reg;
                break;
            case AudioOutput::SPEAKER:
                registers_.lout2_volume = volume_reg;
                registers_.rout2_volume = volume_reg;
                break;
            case AudioOutput::LINEOUT:
                // Line out uses DAC control registers (LOUT1/ROUT1 volume)
                registers_.dac_control17 = volume_reg;
                registers_.dac_control18 = volume_reg;
                break;
            default:
                break;
        }
    }
    
    COMPONENT_LOG_DEBUG("ES8388 output volume set: output={}, volume={:.1f}%", 
                       static_cast<u8>(output), volume_percent);
    
    return {};
}

Result<void> ES8388Codec::mute_input(AudioInput input, bool mute) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (static_cast<u8>(input) < 4) {
        mixer_.input_mute[static_cast<u8>(input)] = mute;
        statistics_.mute_events++;
    }
    
    COMPONENT_LOG_DEBUG("ES8388 input {}: input={}", mute ? "muted" : "unmuted", 
                       static_cast<u8>(input));
    
    return {};
}

Result<void> ES8388Codec::mute_output(AudioOutput output, bool mute) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (static_cast<u8>(output) < 3) {
        mixer_.output_mute[static_cast<u8>(output)] = mute;
        statistics_.mute_events++;
        
        // Update mute bits in volume registers
        u8 mute_bit = mute ? 0x80 : 0x00;
        
        switch (output) {
            case AudioOutput::HEADPHONE:
                registers_.lout1_volume = (registers_.lout1_volume & 0x7F) | mute_bit;
                registers_.rout1_volume = (registers_.rout1_volume & 0x7F) | mute_bit;
                break;
            case AudioOutput::SPEAKER:
                registers_.lout2_volume = (registers_.lout2_volume & 0x7F) | mute_bit;
                registers_.rout2_volume = (registers_.rout2_volume & 0x7F) | mute_bit;
                break;
            default:
                break;
        }
    }
    
    COMPONENT_LOG_DEBUG("ES8388 output {}: output={}", mute ? "muted" : "unmuted", 
                       static_cast<u8>(output));
    
    return {};
}

Result<void> ES8388Codec::set_master_volume(float volume_percent) {
    // Set volume for all active outputs
    auto result = set_output_volume(output_destination_, volume_percent);
    if (!result) {
        return result;
    }
    
    COMPONENT_LOG_DEBUG("ES8388 master volume set to {:.1f}%", volume_percent);
    return {};
}

Result<void> ES8388Codec::set_bass_control(float gain_db, bool boost_enable) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (gain_db < -12.0f || gain_db > 12.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Bass gain out of range (-12dB to +12dB)"));
    }
    
    mixer_.bass_gain = gain_db;
    mixer_.bass_boost = boost_enable;
    
    // Update bass control register
    u8 bass_reg = static_cast<u8>((gain_db + 12.0f) * 15.0f / 24.0f) | (boost_enable ? 0x80 : 0x00);
    registers_.dac_control12 = bass_reg;
    
    COMPONENT_LOG_DEBUG("ES8388 bass control: gain={:.1f}dB, boost={}", gain_db, boost_enable);
    return {};
}

Result<void> ES8388Codec::set_treble_control(float gain_db, bool boost_enable) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (gain_db < -12.0f || gain_db > 12.0f) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Treble gain out of range (-12dB to +12dB)"));
    }
    
    mixer_.treble_gain = gain_db;
    mixer_.treble_boost = boost_enable;
    
    // Update treble control register
    u8 treble_reg = static_cast<u8>((gain_db + 12.0f) * 15.0f / 24.0f) | (boost_enable ? 0x80 : 0x00);
    registers_.dac_control13 = treble_reg;
    
    COMPONENT_LOG_DEBUG("ES8388 treble control: gain={:.1f}dB, boost={}", gain_db, boost_enable);
    return {};
}

Result<void> ES8388Codec::power_on() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    powered_on_ = true;
    registers_.chip_power = 0x00; // Power on all blocks
    registers_.adc_power = 0x00;
    registers_.dac_power = 0x00;
    
    COMPONENT_LOG_INFO("ES8388 codec powered on");
    return {};
}

Result<void> ES8388Codec::power_off() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    // Stop audio streaming before powering off
    playback_active_ = false;
    recording_active_ = false;
    
    powered_on_ = false;
    registers_.chip_power = 0xFF; // Power off all blocks
    registers_.adc_power = 0xFF;
    registers_.dac_power = 0xFF;
    
    COMPONENT_LOG_INFO("ES8388 codec powered off");
    return {};
}

Result<void> ES8388Codec::set_power_mode(bool low_power) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    low_power_mode_ = low_power;
    registers_.chip_low_power = low_power ? 0x01 : 0x00;
    
    COMPONENT_LOG_DEBUG("ES8388 low power mode {}", low_power ? "enabled" : "disabled");
    return {};
}

Result<void> ES8388Codec::start_playback() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (!powered_on_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Codec not powered on"));
    }
    
    playback_active_ = true;
    registers_.dac_power &= ~0x01; // Enable DAC
    
    COMPONENT_LOG_INFO("ES8388 playback started");
    return {};
}

Result<void> ES8388Codec::stop_playback() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    playback_active_ = false;
    registers_.dac_power |= 0x01; // Disable DAC
    
    COMPONENT_LOG_INFO("ES8388 playback stopped");
    return {};
}

Result<void> ES8388Codec::start_recording() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (!powered_on_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Codec not powered on"));
    }
    
    recording_active_ = true;
    registers_.adc_power &= ~0x01; // Enable ADC
    
    COMPONENT_LOG_INFO("ES8388 recording started");
    return {};
}

Result<void> ES8388Codec::stop_recording() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    recording_active_ = false;
    registers_.adc_power |= 0x01; // Disable ADC
    
    COMPONENT_LOG_INFO("ES8388 recording stopped");
    return {};
}

Result<void> ES8388Codec::write_audio_data(const std::vector<i16>& samples) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (!playback_active_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Playback not active"));
    }
    
    // Check for buffer overflow
    if (playback_buffer_.size() + samples.size() > AUDIO_BUFFER_SIZE) {
        statistics_.overrun_errors++;
        trigger_interrupt(CodecInterruptType::PLAYBACK_OVERRUN);
        return unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
            "Playback buffer overflow"));
    }
    
    // Add samples to playback buffer
    for (i16 sample : samples) {
        playback_buffer_.push(sample);
    }
    
    return {};
}

Result<std::vector<i16>> ES8388Codec::read_audio_data(size_t max_samples) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    if (!recording_active_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Recording not active"));
    }
    
    std::vector<i16> samples;
    size_t count = (max_samples == 0) ? recording_buffer_.size() : 
                   std::min(max_samples, recording_buffer_.size());
    
    samples.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        if (!recording_buffer_.empty()) {
            samples.push_back(recording_buffer_.front());
            recording_buffer_.pop();
        }
    }
    
    return samples;
}

bool ES8388Codec::is_headphone_connected() const {
    return headphone_connected_.load();
}

bool ES8388Codec::is_microphone_connected() const {
    return microphone_connected_.load();
}

Result<void> ES8388Codec::write_register(u8 reg_addr, u8 value) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    // Update the appropriate register (simplified implementation)
    switch (reg_addr) {
        case 0x00: registers_.chip_control1 = value; break;
        case 0x01: registers_.chip_control2 = value; break;
        case 0x02: 
            registers_.chip_power = value;
            powered_on_ = (value == 0x00);
            break;
        case 0x03: registers_.adc_power = value; break;
        case 0x04: registers_.dac_power = value; break;
        // Add more register cases as needed
        default:
            COMPONENT_LOG_WARN("ES8388 write to unimplemented register 0x{:02X} = 0x{:02X}", 
                              reg_addr, value);
            break;
    }
    
    return {};
}

Result<u8> ES8388Codec::read_register(u8 reg_addr) const {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    switch (reg_addr) {
        case 0x00: return registers_.chip_control1;
        case 0x01: return registers_.chip_control2;
        case 0x02: return registers_.chip_power;
        case 0x03: return registers_.adc_power;
        case 0x04: return registers_.dac_power;
        // Add more register cases as needed
        default:
            COMPONENT_LOG_WARN("ES8388 read from unimplemented register 0x{:02X}", reg_addr);
            return 0x00;
    }
}

Result<void> ES8388Codec::enable_interrupt(CodecInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    // Enable interrupt in register (simplified)
    registers_.chip_control1 |= static_cast<u8>(interrupt_type);
    
    COMPONENT_LOG_DEBUG("ES8388 interrupt {} enabled", static_cast<u8>(interrupt_type));
    return {};
}

Result<void> ES8388Codec::disable_interrupt(CodecInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ES8388 codec not initialized"));
    }
    
    // Disable interrupt in register (simplified)
    registers_.chip_control1 &= ~static_cast<u8>(interrupt_type);
    
    COMPONENT_LOG_DEBUG("ES8388 interrupt {} disabled", static_cast<u8>(interrupt_type));
    return {};
}

void ES8388Codec::update() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    if (!initialized_ || !powered_on_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);
    last_update_ = now;
    
    // Update audio processing at sample rate intervals
    auto sample_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_sample_time_);
    u32 sample_period_us = 1000000 / static_cast<u32>(sample_rate_);
    
    if (sample_elapsed.count() >= sample_period_us * samples_per_update_) {
        process_audio_pipeline();
        last_sample_time_ = now;
    }
    
    check_jack_detection();
}

void ES8388Codec::update_playback() {
    if (!playback_active_ || playback_buffer_.empty()) {
        return;
    }
    
    std::vector<i16> samples;
    size_t samples_to_process = std::min(static_cast<size_t>(samples_per_update_), 
                                        playback_buffer_.size());
    
    for (size_t i = 0; i < samples_to_process; ++i) {
        samples.push_back(playback_buffer_.front());
        playback_buffer_.pop();
    }
    
    // Apply mixer and EQ settings
    apply_mixer_settings(samples);
    apply_eq_settings(samples);
    
    statistics_.frames_played += samples.size() / MAX_CHANNELS;
    
    // Calculate average playback level
    if (!samples.empty()) {
        float level_sum = 0.0f;
        for (i16 sample : samples) {
            level_sum += std::abs(sample) / 32768.0f;
        }
        statistics_.average_playback_level = level_sum / samples.size();
    }
    
    // Check for underrun
    if (playback_buffer_.size() < samples_per_update_ / 4) {
        statistics_.underrun_errors++;
        trigger_interrupt(CodecInterruptType::PLAYBACK_UNDERRUN);
    }
}

void ES8388Codec::update_recording() {
    if (!recording_active_) {
        return;
    }
    
    std::vector<i16> samples(samples_per_update_ * MAX_CHANNELS);
    simulate_microphone_input(samples);
    
    // Apply input gain and mute
    for (size_t i = 0; i < samples.size(); i += MAX_CHANNELS) {
        if (!mixer_.input_mute[static_cast<u8>(left_input_)]) {
            samples[i] = static_cast<i16>(samples[i] * mixer_.input_gain[static_cast<u8>(left_input_)]);
        } else {
            samples[i] = 0;
        }
        
        if (i + 1 < samples.size() && !mixer_.input_mute[static_cast<u8>(right_input_)]) {
            samples[i + 1] = static_cast<i16>(samples[i + 1] * mixer_.input_gain[static_cast<u8>(right_input_)]);
        } else if (i + 1 < samples.size()) {
            samples[i + 1] = 0;
        }
    }
    
    // Add to recording buffer
    for (i16 sample : samples) {
        if (recording_buffer_.size() >= AUDIO_BUFFER_SIZE) {
            statistics_.overrun_errors++;
            trigger_interrupt(CodecInterruptType::RECORD_OVERRUN);
            recording_buffer_.pop(); // Remove oldest sample
        }
        recording_buffer_.push(sample);
    }
    
    statistics_.frames_recorded += samples.size() / MAX_CHANNELS;
    
    // Calculate average record level
    if (!samples.empty()) {
        float level_sum = 0.0f;
        for (i16 sample : samples) {
            level_sum += std::abs(sample) / 32768.0f;
        }
        statistics_.average_record_level = level_sum / samples.size();
    }
}

void ES8388Codec::process_audio_pipeline() {
    update_playback();
    update_recording();
}

void ES8388Codec::apply_mixer_settings(std::vector<i16>& samples) {
    if (mixer_.output_mute[static_cast<u8>(output_destination_)]) {
        std::fill(samples.begin(), samples.end(), 0);
        return;
    }
    
    float volume = mixer_.output_volume[static_cast<u8>(output_destination_)];
    for (i16& sample : samples) {
        sample = static_cast<i16>(sample * volume);
    }
}

void ES8388Codec::apply_eq_settings(std::vector<i16>& samples) {
    // Simplified EQ implementation
    if (mixer_.bass_gain != 0.0f || mixer_.treble_gain != 0.0f) {
        float bass_multiplier = db_to_linear(mixer_.bass_gain);
        float treble_multiplier = db_to_linear(mixer_.treble_gain);
        
        for (size_t i = 0; i < samples.size(); ++i) {
            float sample_f = static_cast<float>(samples[i]);
            
            // Simple bass boost (low-pass emphasis)
            if (mixer_.bass_boost && mixer_.bass_gain > 0.0f) {
                sample_f *= bass_multiplier;
            }
            
            // Simple treble boost (high-pass emphasis)
            if (mixer_.treble_boost && mixer_.treble_gain > 0.0f) {
                sample_f *= treble_multiplier;
            }
            
            samples[i] = static_cast<i16>(std::clamp(sample_f, -32768.0f, 32767.0f));
        }
    }
}

void ES8388Codec::trigger_interrupt(CodecInterruptType interrupt_type) {
    if (interrupt_controller_) {
        // Map codec interrupts to system interrupts
        interrupt_controller_->trigger_interrupt(InterruptType::I2C0); // Codec uses I2C
    }
}

void ES8388Codec::simulate_microphone_input(std::vector<i16>& samples) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> noise_dist(0.0f, 0.1f);
    static std::uniform_real_distribution<float> signal_dist(-0.3f, 0.3f);
    
    for (size_t i = 0; i < samples.size(); ++i) {
        // Simulate environmental noise + occasional signal
        float signal = noise_dist(gen);
        
        // Add occasional stronger signal bursts
        if (gen() % 1000 < 10) {
            signal += signal_dist(gen);
        }
        
        samples[i] = static_cast<i16>(signal * 32767.0f);
    }
}

void ES8388Codec::check_jack_detection() {
    // Simulate jack detection changes occasionally
    static auto last_check = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_check);
    
    if (elapsed.count() >= 5) { // Check every 5 seconds
        // Randomly toggle connection status (for simulation)
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        if (gen() % 100 < 5) { // 5% chance to change status
            headphone_connected_.store(!headphone_connected_.load());
            trigger_interrupt(CodecInterruptType::JACK_DETECT);
        }
        
        last_check = now;
    }
}

float ES8388Codec::db_to_linear(float db) const {
    return std::pow(10.0f, db / 20.0f);
}

float ES8388Codec::linear_to_db(float linear) const {
    return 20.0f * std::log10(std::max(linear, 1e-6f));
}

void ES8388Codec::clear_statistics() {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    statistics_ = {};
}

void ES8388Codec::dump_status() const {
    std::lock_guard<std::mutex> lock(codec_mutex_);
    
    COMPONENT_LOG_INFO("=== ES8388 Codec Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Power: on={}, low_power={}", powered_on_, low_power_mode_);
        COMPONENT_LOG_INFO("Format: {} Hz, {} bits, format={}",
                          static_cast<u32>(sample_rate_), static_cast<u8>(bit_depth_), 
                          static_cast<u8>(format_));
        
        COMPONENT_LOG_INFO("Audio: playback={}, recording={}", playback_active_, recording_active_);
        COMPONENT_LOG_INFO("Inputs: left={}, right={}", 
                          static_cast<u8>(left_input_), static_cast<u8>(right_input_));
        COMPONENT_LOG_INFO("Output: {}", static_cast<u8>(output_destination_));
        
        COMPONENT_LOG_INFO("Jack detection: headphone={}, microphone={}", 
                          headphone_connected_.load(), microphone_connected_.load());
        
        COMPONENT_LOG_INFO("Buffers: playback={}/{}, recording={}/{}", 
                          playback_buffer_.size(), AUDIO_BUFFER_SIZE,
                          recording_buffer_.size(), AUDIO_BUFFER_SIZE);
        
        COMPONENT_LOG_INFO("Volume: HP={:.1f}%, SPK={:.1f}%, LINE={:.1f}%",
                          mixer_.output_volume[0] * 100.0f,
                          mixer_.output_volume[1] * 100.0f,
                          mixer_.output_volume[2] * 100.0f);
        
        COMPONENT_LOG_INFO("EQ: bass={:.1f}dB (boost={}), treble={:.1f}dB (boost={})",
                          mixer_.bass_gain, mixer_.bass_boost,
                          mixer_.treble_gain, mixer_.treble_boost);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Frames played: {}", statistics_.frames_played);
        COMPONENT_LOG_INFO("  Frames recorded: {}", statistics_.frames_recorded);
        COMPONENT_LOG_INFO("  Underrun errors: {}", statistics_.underrun_errors);
        COMPONENT_LOG_INFO("  Overrun errors: {}", statistics_.overrun_errors);
        COMPONENT_LOG_INFO("  Average playback level: {:.3f}", statistics_.average_playback_level);
        COMPONENT_LOG_INFO("  Average record level: {:.3f}", statistics_.average_record_level);
        COMPONENT_LOG_INFO("  Volume changes: {}", statistics_.volume_changes);
        COMPONENT_LOG_INFO("  Mute events: {}", statistics_.mute_events);
    }
}

}  // namespace m5tab5::emulator