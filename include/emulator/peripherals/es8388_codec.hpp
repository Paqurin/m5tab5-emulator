#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <atomic>

namespace m5tab5::emulator {

enum class AudioFormat : u8 {
    I2S_PHILIPS = 0,
    I2S_LEFT_JUSTIFIED = 1,
    I2S_RIGHT_JUSTIFIED = 2,
    PCM_MODE_A = 3,
    PCM_MODE_B = 4,
    DSP_MODE = 5
};

enum class AudioSampleRate : u32 {
    RATE_8000 = 8000,
    RATE_11025 = 11025,
    RATE_16000 = 16000,
    RATE_22050 = 22050,
    RATE_32000 = 32000,
    RATE_44100 = 44100,
    RATE_48000 = 48000,
    RATE_88200 = 88200,
    RATE_96000 = 96000
};

enum class AudioBitDepth : u8 {
    BITS_16 = 16,
    BITS_18 = 18,
    BITS_20 = 20,
    BITS_24 = 24,
    BITS_32 = 32
};

enum class AudioInput : u8 {
    MIC1 = 0,
    MIC2 = 1,
    LINEIN1 = 2,
    LINEIN2 = 3,
    DIFFERENTIAL = 4,
    DISABLED = 255
};

enum class AudioOutput : u8 {
    HEADPHONE = 0,
    SPEAKER = 1,
    LINEOUT = 2,
    DISABLED = 255
};

enum class CodecInterruptType : u8 {
    PLAYBACK_UNDERRUN = 0x01,
    PLAYBACK_OVERRUN = 0x02,
    RECORD_UNDERRUN = 0x04,
    RECORD_OVERRUN = 0x08,
    JACK_DETECT = 0x10,
    THERMAL_WARNING = 0x20,
    POWER_FAULT = 0x40,
    MUTE_COMPLETE = 0x80
};

struct AudioMixer {
    float input_gain[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // MIC1, MIC2, LINE1, LINE2
    float output_volume[3] = {0.8f, 0.8f, 0.8f};      // HP, SPK, LINEOUT
    bool input_mute[4] = {false, false, false, false};
    bool output_mute[3] = {false, false, false};
    float bass_gain = 0.0f;    // -12dB to +12dB
    float treble_gain = 0.0f;  // -12dB to +12dB
    bool bass_boost = false;
    bool treble_boost = false;
};

struct AudioStatistics {
    u64 frames_played = 0;
    u64 frames_recorded = 0;
    u64 underrun_errors = 0;
    u64 overrun_errors = 0;
    u64 mute_events = 0;
    u64 volume_changes = 0;
    double average_playback_level = 0.0;
    double average_record_level = 0.0;
};

class ES8388Codec {
public:
    static constexpr u8 I2C_ADDRESS = 0x10;  // ES8388 I2C address
    static constexpr size_t AUDIO_BUFFER_SIZE = 4096;
    static constexpr size_t MAX_CHANNELS = 2; // Stereo
    static constexpr u32 DEFAULT_SAMPLE_RATE = 44100;
    static constexpr u8 DEFAULT_BIT_DEPTH = 16;
    
    ES8388Codec();
    ~ES8388Codec();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    // Configuration
    Result<void> configure_format(AudioFormat format, AudioSampleRate sample_rate, AudioBitDepth bit_depth);
    Result<void> set_input_source(AudioInput left, AudioInput right);
    Result<void> set_output_destination(AudioOutput output);
    
    // Volume and mixing control
    Result<void> set_input_gain(AudioInput input, float gain_db);
    Result<void> set_output_volume(AudioOutput output, float volume_percent);
    Result<void> mute_input(AudioInput input, bool mute);
    Result<void> mute_output(AudioOutput output, bool mute);
    Result<void> set_master_volume(float volume_percent);
    
    // EQ control
    Result<void> set_bass_control(float gain_db, bool boost_enable);
    Result<void> set_treble_control(float gain_db, bool boost_enable);
    
    // Power management
    Result<void> power_on();
    Result<void> power_off();
    Result<void> set_power_mode(bool low_power);
    
    // Audio streaming
    Result<void> start_playback();
    Result<void> stop_playback();
    Result<void> start_recording();
    Result<void> stop_recording();
    
    // Audio data interface
    Result<void> write_audio_data(const std::vector<i16>& samples);
    Result<std::vector<i16>> read_audio_data(size_t max_samples = 0);
    
    // Jack detection
    bool is_headphone_connected() const;
    bool is_microphone_connected() const;
    
    // I2C register interface
    Result<void> write_register(u8 reg_addr, u8 value);
    Result<u8> read_register(u8 reg_addr) const;
    
    // Interrupt handling
    Result<void> enable_interrupt(CodecInterruptType interrupt_type);
    Result<void> disable_interrupt(CodecInterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_playing() const { return playback_active_; }
    bool is_recording() const { return recording_active_; }
    AudioFormat get_format() const { return format_; }
    AudioSampleRate get_sample_rate() const { return sample_rate_; }
    const AudioStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct ES8388Registers {
        u8 chip_control1 = 0x00;      // 0x00 - Chip control 1
        u8 chip_control2 = 0x00;      // 0x01 - Chip control 2
        u8 chip_power = 0xFF;         // 0x02 - Chip power management
        u8 adc_power = 0xFF;          // 0x03 - ADC power management
        u8 dac_power = 0xC0;          // 0x04 - DAC power management
        u8 chip_low_power = 0x00;     // 0x05 - Chip low power modes
        u8 adc_control1 = 0x88;       // 0x06 - ADC control 1
        u8 adc_control2 = 0x00;       // 0x07 - ADC control 2
        u8 adc_control3 = 0x02;       // 0x08 - ADC control 3
        u8 adc_control4 = 0x0C;       // 0x09 - ADC control 4
        u8 adc_control5 = 0x02;       // 0x0A - ADC control 5
        u8 adc_control6 = 0x00;       // 0x0B - ADC control 6
        u8 adc_control7 = 0x00;       // 0x0C - ADC control 7
        u8 adc_control8 = 0x00;       // 0x0D - ADC control 8
        u8 adc_control9 = 0x00;       // 0x0E - ADC control 9
        u8 adc_control10 = 0x00;      // 0x0F - ADC control 10
        u8 adc_control11 = 0x00;      // 0x10 - ADC control 11
        u8 adc_control12 = 0x00;      // 0x11 - ADC control 12
        u8 adc_control13 = 0x00;      // 0x12 - ADC control 13
        u8 adc_control14 = 0x00;      // 0x13 - ADC control 14
        u8 dac_control1 = 0x00;       // 0x14 - DAC control 1
        u8 dac_control2 = 0x02;       // 0x15 - DAC control 2
        u8 dac_control3 = 0x00;       // 0x16 - DAC control 3
        u8 dac_control4 = 0x00;       // 0x17 - DAC control 4
        u8 dac_control5 = 0x00;       // 0x18 - DAC control 5
        u8 dac_control6 = 0x00;       // 0x19 - DAC control 6
        u8 dac_control7 = 0x00;       // 0x1A - DAC control 7
        u8 dac_control8 = 0x00;       // 0x1B - DAC control 8
        u8 dac_control9 = 0x00;       // 0x1C - DAC control 9
        u8 dac_control10 = 0x00;      // 0x1D - DAC control 10
        u8 dac_control11 = 0x00;      // 0x1E - DAC control 11
        u8 dac_control12 = 0x00;      // 0x1F - DAC control 12
        u8 dac_control13 = 0x00;      // 0x20 - DAC control 13
        u8 dac_control14 = 0x00;      // 0x21 - DAC control 14
        u8 dac_control15 = 0x00;      // 0x22 - DAC control 15
        u8 dac_control16 = 0x00;      // 0x23 - DAC control 16
        u8 dac_control17 = 0x90;      // 0x24 - DAC control 17
        u8 dac_control18 = 0x90;      // 0x25 - DAC control 18
        u8 dac_control19 = 0x90;      // 0x26 - DAC control 19
        u8 dac_control20 = 0x90;      // 0x27 - DAC control 20
        u8 dac_control21 = 0x90;      // 0x28 - DAC control 21
        u8 dac_control22 = 0x00;      // 0x29 - DAC control 22
        u8 dac_control23 = 0x00;      // 0x2A - DAC control 23
        u8 lout1_volume = 0x1E;       // 0x2E - LOUT1 volume
        u8 rout1_volume = 0x1E;       // 0x2F - ROUT1 volume
        u8 lout2_volume = 0x1E;       // 0x30 - LOUT2 volume
        u8 rout2_volume = 0x1E;       // 0x31 - ROUT2 volume
    };

    void update_playback();
    void update_recording();
    void process_audio_pipeline();
    void apply_mixer_settings(std::vector<i16>& samples);
    void apply_eq_settings(std::vector<i16>& samples);
    void trigger_interrupt(CodecInterruptType interrupt_type);
    void simulate_microphone_input(std::vector<i16>& samples);
    void check_jack_detection();
    float db_to_linear(float db) const;
    float linear_to_db(float linear) const;

    bool initialized_;
    bool powered_on_;
    bool low_power_mode_;
    AudioFormat format_;
    AudioSampleRate sample_rate_;
    AudioBitDepth bit_depth_;
    
    AudioInput left_input_;
    AudioInput right_input_;
    AudioOutput output_destination_;
    
    ES8388Registers registers_;
    InterruptController* interrupt_controller_;
    
    AudioMixer mixer_;
    
    // Audio streaming state
    bool playback_active_;
    bool recording_active_;
    
    std::queue<i16> playback_buffer_;
    std::queue<i16> recording_buffer_;
    
    // Jack detection simulation
    std::atomic<bool> headphone_connected_;
    std::atomic<bool> microphone_connected_;
    
    // Timing and synchronization
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_sample_time_;
    u32 samples_per_update_;
    
    AudioStatistics statistics_;
    
    mutable std::mutex codec_mutex_;
};

}  // namespace m5tab5::emulator