#include "emulator/audio/audio_pipeline.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("AudioPipeline");

AudioPipeline::AudioPipeline()
    : initialized_(false),
      codec_(nullptr),
      microphone_array_(nullptr),
      processing_active_(false),
      echo_cancellation_enabled_(false),
      noise_suppression_enabled_(false),
      agc_enabled_(false),
      agc_target_level_(-12.0f) {
    
    // Initialize SDL audio state
    sdl_state_ = {};
    sdl_state_.playback_device = 0;
    sdl_state_.recording_device = 0;
    sdl_state_.playback_active = false;
    sdl_state_.recording_active = false;
    
    COMPONENT_LOG_DEBUG("Audio pipeline created");
}

AudioPipeline::~AudioPipeline() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Audio pipeline destroyed");
}

Result<void> AudioPipeline::initialize(const Configuration& config, 
                                      ES8388Codec* codec, 
                                      MicrophoneArray* microphone_array) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Audio pipeline already initialized"));
    }
    
    if (!codec || !microphone_array) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Codec and microphone array required"));
    }
    
    COMPONENT_LOG_INFO("Initializing audio pipeline");
    
    codec_ = codec;
    microphone_array_ = microphone_array;
    
    // Initialize SDL audio subsystem
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
        return std::unexpected(MAKE_ERROR(INITIALIZATION_FAILED,
            "Failed to initialize SDL audio: " + std::string(SDL_GetError())));
    }
    
    // Set default stream configuration
    stream_config_ = {};
    stream_config_.sample_rate = AudioSampleRate::RATE_44100;
    stream_config_.bit_depth = AudioBitDepth::BITS_16;
    stream_config_.channels = 2;
    stream_config_.buffer_size_frames = 1024;
    stream_config_.buffer_count = 4;
    stream_config_.direction = AudioStreamDirection::DUPLEX;
    stream_config_.buffer_mode = AudioBufferMode::CALLBACK;
    stream_config_.volume = 1.0f;
    stream_config_.auto_start = true;
    
    // Clear statistics
    statistics_ = {};
    last_stats_update_ = std::chrono::steady_clock::now();
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Audio pipeline initialized successfully");
    
    return {};
}

Result<void> AudioPipeline::shutdown() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down audio pipeline");
    
    // Stop processing thread
    processing_active_.store(false);
    if (processing_thread_ && processing_thread_->joinable()) {
        processing_thread_->join();
        processing_thread_.reset();
    }
    
    // Close SDL audio devices
    if (sdl_state_.playback_device > 0) {
        SDL_CloseAudioDevice(sdl_state_.playback_device);
        sdl_state_.playback_device = 0;
    }
    
    if (sdl_state_.recording_device > 0) {
        SDL_CloseAudioDevice(sdl_state_.recording_device);
        sdl_state_.recording_device = 0;
    }
    
    // Clear buffers
    while (!playback_buffer_queue_.empty()) {
        playback_buffer_queue_.pop();
    }
    while (!recording_buffer_queue_.empty()) {
        recording_buffer_queue_.pop();
    }
    
    // Quit SDL audio subsystem
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    
    codec_ = nullptr;
    microphone_array_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Audio pipeline shutdown completed");
    return {};
}

Result<void> AudioPipeline::configure_stream(const AudioStreamConfig& config) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (sdl_state_.playback_active || sdl_state_.recording_active) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot configure stream while active"));
    }
    
    // Validate configuration
    if (config.buffer_size_frames < MIN_BUFFER_SIZE_FRAMES || 
        config.buffer_size_frames > MAX_BUFFER_SIZE_FRAMES) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Buffer size out of range"));
    }
    
    if (config.buffer_count < MIN_BUFFER_COUNT || config.buffer_count > MAX_BUFFER_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Buffer count out of range"));
    }
    
    if (config.channels != 1 && config.channels != 2) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Only mono and stereo supported"));
    }
    
    if (config.volume < 0.0f || config.volume > 2.0f) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Volume out of range (0.0-2.0)"));
    }
    
    stream_config_ = config;
    
    COMPONENT_LOG_INFO("Audio stream configured: {} Hz, {} channels, {} frames buffer",
                      static_cast<u32>(config.sample_rate), config.channels, 
                      config.buffer_size_frames);
    
    return {};
}

Result<AudioStreamConfig> AudioPipeline::get_stream_config() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    return stream_config_;
}

Result<void> AudioPipeline::set_sample_rate(AudioSampleRate sample_rate) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (sdl_state_.playback_active || sdl_state_.recording_active) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot change sample rate while stream is active"));
    }
    
    stream_config_.sample_rate = sample_rate;
    
    COMPONENT_LOG_DEBUG("Audio pipeline sample rate set to {} Hz", 
                       static_cast<u32>(sample_rate));
    
    return {};
}

Result<void> AudioPipeline::set_buffer_size(u32 buffer_size_frames) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (buffer_size_frames < MIN_BUFFER_SIZE_FRAMES || 
        buffer_size_frames > MAX_BUFFER_SIZE_FRAMES) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Buffer size out of range"));
    }
    
    if (sdl_state_.playback_active || sdl_state_.recording_active) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot change buffer size while stream is active"));
    }
    
    stream_config_.buffer_size_frames = buffer_size_frames;
    
    COMPONENT_LOG_DEBUG("Audio pipeline buffer size set to {} frames", buffer_size_frames);
    return {};
}

Result<void> AudioPipeline::set_volume(float volume) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (volume < 0.0f || volume > 2.0f) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Volume out of range (0.0-2.0)"));
    }
    
    stream_config_.volume = volume;
    
    COMPONENT_LOG_DEBUG("Audio pipeline volume set to {:.2f}", volume);
    return {};
}

Result<void> AudioPipeline::start_stream() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (sdl_state_.playback_active || sdl_state_.recording_active) {
        return {}; // Already active
    }
    
    COMPONENT_LOG_INFO("Starting audio stream");
    
    // Configure SDL audio specs
    SDL_AudioSpec desired_playback = {};
    desired_playback.freq = static_cast<int>(stream_config_.sample_rate);
    desired_playback.format = AUDIO_S16LSB; // 16-bit signed little-endian
    desired_playback.channels = stream_config_.channels;
    desired_playback.samples = static_cast<Uint16>(stream_config_.buffer_size_frames);
    desired_playback.callback = sdl_playback_callback;
    desired_playback.userdata = this;
    
    SDL_AudioSpec desired_recording = desired_playback;
    desired_recording.callback = sdl_recording_callback;
    
    // Open playback device
    if (stream_config_.direction == AudioStreamDirection::PLAYBACK || 
        stream_config_.direction == AudioStreamDirection::DUPLEX) {
        
        sdl_state_.playback_device = SDL_OpenAudioDevice(nullptr, 0, &desired_playback, 
                                                        &sdl_state_.playback_spec, 
                                                        SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
        
        if (sdl_state_.playback_device == 0) {
            return std::unexpected(MAKE_ERROR(INITIALIZATION_FAILED,
                "Failed to open playback device: " + std::string(SDL_GetError())));
        }
        
        sdl_state_.playback_active = true;
        SDL_PauseAudioDevice(sdl_state_.playback_device, 0); // Unpause
    }
    
    // Open recording device
    if (stream_config_.direction == AudioStreamDirection::RECORDING || 
        stream_config_.direction == AudioStreamDirection::DUPLEX) {
        
        sdl_state_.recording_device = SDL_OpenAudioDevice(nullptr, 1, &desired_recording, 
                                                         &sdl_state_.recording_spec, 
                                                         SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
        
        if (sdl_state_.recording_device == 0) {
            // Close playback device if it was opened
            if (sdl_state_.playback_device > 0) {
                SDL_CloseAudioDevice(sdl_state_.playback_device);
                sdl_state_.playback_device = 0;
                sdl_state_.playback_active = false;
            }
            
            return std::unexpected(MAKE_ERROR(INITIALIZATION_FAILED,
                "Failed to open recording device: " + std::string(SDL_GetError())));
        }
        
        sdl_state_.recording_active = true;
        SDL_PauseAudioDevice(sdl_state_.recording_device, 0); // Unpause
    }
    
    // Start processing thread
    processing_active_.store(true);
    processing_thread_ = std::make_unique<std::thread>(&AudioPipeline::audio_processing_thread, this);
    
    stream_start_time_ = std::chrono::steady_clock::now();
    
    COMPONENT_LOG_INFO("Audio stream started successfully");
    return {};
}

Result<void> AudioPipeline::stop_stream() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    COMPONENT_LOG_INFO("Stopping audio stream");
    
    // Stop processing thread
    processing_active_.store(false);
    buffer_condition_.notify_all();
    
    if (processing_thread_ && processing_thread_->joinable()) {
        pipeline_mutex_.unlock(); // Unlock to allow thread to complete
        processing_thread_->join();
        pipeline_mutex_.lock();
        processing_thread_.reset();
    }
    
    // Close audio devices
    if (sdl_state_.playback_device > 0) {
        SDL_PauseAudioDevice(sdl_state_.playback_device, 1); // Pause
        SDL_CloseAudioDevice(sdl_state_.playback_device);
        sdl_state_.playback_device = 0;
        sdl_state_.playback_active = false;
    }
    
    if (sdl_state_.recording_device > 0) {
        SDL_PauseAudioDevice(sdl_state_.recording_device, 1); // Pause
        SDL_CloseAudioDevice(sdl_state_.recording_device);
        sdl_state_.recording_device = 0;
        sdl_state_.recording_active = false;
    }
    
    // Clear buffers
    while (!playback_buffer_queue_.empty()) {
        playback_buffer_queue_.pop();
    }
    while (!recording_buffer_queue_.empty()) {
        recording_buffer_queue_.pop();
    }
    
    COMPONENT_LOG_INFO("Audio stream stopped");
    return {};
}

Result<void> AudioPipeline::pause_stream(bool pause) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (sdl_state_.playback_device > 0) {
        SDL_PauseAudioDevice(sdl_state_.playback_device, pause ? 1 : 0);
    }
    
    if (sdl_state_.recording_device > 0) {
        SDL_PauseAudioDevice(sdl_state_.recording_device, pause ? 1 : 0);
    }
    
    COMPONENT_LOG_DEBUG("Audio stream {}", pause ? "paused" : "resumed");
    return {};
}

Result<void> AudioPipeline::set_audio_callback(AudioCallback callback) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    audio_callback_ = callback;
    stream_config_.buffer_mode = AudioBufferMode::CALLBACK;
    
    COMPONENT_LOG_DEBUG("Audio callback set");
    return {};
}

Result<void> AudioPipeline::set_error_callback(AudioErrorCallback callback) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    error_callback_ = callback;
    
    COMPONENT_LOG_DEBUG("Error callback set");
    return {};
}

Result<void> AudioPipeline::write_audio(const std::vector<i16>& samples) {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (!sdl_state_.playback_active) {
        return std::unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Playback not active"));
    }
    
    if (samples.empty()) {
        return {};
    }
    
    // Wait for buffer space in blocking mode
    if (stream_config_.buffer_mode == AudioBufferMode::BLOCKING) {
        buffer_condition_.wait(lock, [this] {
            return playback_buffer_queue_.size() < stream_config_.buffer_count || !processing_active_;
        });
    }
    
    if (playback_buffer_queue_.size() >= stream_config_.buffer_count) {
        if (stream_config_.buffer_mode == AudioBufferMode::NON_BLOCKING) {
            return std::unexpected(MAKE_ERROR(BUFFER_OVERFLOW,
                "Playback buffer full"));
        }
        
        // Drop oldest buffer in callback mode
        if (stream_config_.buffer_mode == AudioBufferMode::CALLBACK) {
            playback_buffer_queue_.pop();
            handle_buffer_overrun();
        }
    }
    
    // Create audio buffer
    AudioBuffer buffer;
    buffer.data = samples;
    buffer.frames = samples.size() / stream_config_.channels;
    buffer.channels = stream_config_.channels;
    buffer.timestamp = std::chrono::steady_clock::now();
    buffer.is_valid = true;
    
    playback_buffer_queue_.push(buffer);
    buffer_condition_.notify_one();
    
    return {};
}

Result<std::vector<i16>> AudioPipeline::read_audio(size_t max_frames) {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (!sdl_state_.recording_active) {
        return std::unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Recording not active"));
    }
    
    // Wait for data in blocking mode
    if (stream_config_.buffer_mode == AudioBufferMode::BLOCKING) {
        buffer_condition_.wait(lock, [this] {
            return !recording_buffer_queue_.empty() || !processing_active_;
        });
    }
    
    if (recording_buffer_queue_.empty()) {
        if (stream_config_.buffer_mode == AudioBufferMode::NON_BLOCKING) {
            return std::vector<i16>{};
        } else {
            return std::unexpected(MAKE_ERROR(INSUFFICIENT_DATA,
                "No audio data available"));
        }
    }
    
    std::vector<i16> result;
    size_t frames_collected = 0;
    size_t target_frames = (max_frames == 0) ? SIZE_MAX : max_frames;
    
    while (!recording_buffer_queue_.empty() && frames_collected < target_frames) {
        const auto& buffer = recording_buffer_queue_.front();
        
        size_t frames_to_take = std::min(buffer.frames, target_frames - frames_collected);
        size_t samples_to_take = frames_to_take * stream_config_.channels;
        
        if (result.empty()) {
            result.reserve(samples_to_take);
        }
        
        result.insert(result.end(), buffer.data.begin(), 
                     buffer.data.begin() + samples_to_take);
        
        frames_collected += frames_to_take;
        recording_buffer_queue_.pop();
    }
    
    return result;
}

Result<bool> AudioPipeline::try_write_audio(const std::vector<i16>& samples) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (playback_buffer_queue_.size() >= stream_config_.buffer_count) {
        return false; // Buffer full
    }
    
    auto result = write_audio(samples);
    return result.has_value();
}

Result<std::vector<i16>> AudioPipeline::try_read_audio() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (recording_buffer_queue_.empty()) {
        return std::vector<i16>{};
    }
    
    const auto& buffer = recording_buffer_queue_.front();
    std::vector<i16> result = buffer.data;
    recording_buffer_queue_.pop();
    
    return result;
}

Result<size_t> AudioPipeline::get_write_available() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    return stream_config_.buffer_count - playback_buffer_queue_.size();
}

Result<size_t> AudioPipeline::get_read_available() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    return recording_buffer_queue_.size();
}

Result<void> AudioPipeline::flush_buffers() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    while (!playback_buffer_queue_.empty()) {
        playback_buffer_queue_.pop();
    }
    while (!recording_buffer_queue_.empty()) {
        recording_buffer_queue_.pop();
    }
    
    COMPONENT_LOG_DEBUG("Audio buffers flushed");
    return {};
}

Result<AudioLatencyInfo> AudioPipeline::get_latency_info() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    AudioLatencyInfo info = {};
    
    // Calculate buffer-based latency
    u32 samples_per_ms = static_cast<u32>(stream_config_.sample_rate) / 1000;
    
    if (sdl_state_.playback_active) {
        info.output_latency_ms = (stream_config_.buffer_size_frames * stream_config_.buffer_count) / samples_per_ms;
    }
    
    if (sdl_state_.recording_active) {
        info.input_latency_ms = (stream_config_.buffer_size_frames * stream_config_.buffer_count) / samples_per_ms;
    }
    
    info.total_latency_ms = info.input_latency_ms + info.output_latency_ms;
    info.buffer_underruns = statistics_.buffer_underruns;
    info.buffer_overruns = statistics_.buffer_overruns;
    info.cpu_usage_percent = statistics_.average_cpu_usage;
    
    return info;
}

Result<bool> AudioPipeline::is_stream_active() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    return sdl_state_.playback_active || sdl_state_.recording_active;
}

Result<double> AudioPipeline::get_cpu_load() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    return statistics_.average_cpu_usage;
}

Result<void> AudioPipeline::enable_echo_cancellation(bool enable) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    echo_cancellation_enabled_ = enable;
    
    COMPONENT_LOG_DEBUG("Echo cancellation {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> AudioPipeline::enable_noise_suppression(bool enable) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    noise_suppression_enabled_ = enable;
    
    COMPONENT_LOG_DEBUG("Noise suppression {}", enable ? "enabled" : "disabled");
    return {};
}

Result<void> AudioPipeline::enable_automatic_gain_control(bool enable, float target_level) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Audio pipeline not initialized"));
    }
    
    if (target_level > 0.0f || target_level < -40.0f) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "AGC target level out of range (-40dB to 0dB)"));
    }
    
    agc_enabled_ = enable;
    agc_target_level_ = target_level;
    
    COMPONENT_LOG_DEBUG("Automatic gain control {}: target={:.1f}dB", 
                       enable ? "enabled" : "disabled", target_level);
    
    return {};
}

// Static utility methods
std::vector<i16> AudioPipeline::convert_float_to_int16(const std::vector<float>& samples) {
    std::vector<i16> result;
    result.reserve(samples.size());
    
    for (float sample : samples) {
        float clamped = std::clamp(sample, -1.0f, 1.0f);
        result.push_back(static_cast<i16>(clamped * 32767.0f));
    }
    
    return result;
}

std::vector<float> AudioPipeline::convert_int16_to_float(const std::vector<i16>& samples) {
    std::vector<float> result;
    result.reserve(samples.size());
    
    for (i16 sample : samples) {
        result.push_back(static_cast<float>(sample) / 32768.0f);
    }
    
    return result;
}

std::vector<i16> AudioPipeline::resample_audio(const std::vector<i16>& samples, 
                                              u32 input_rate, u32 output_rate) {
    if (input_rate == output_rate) {
        return samples; // No resampling needed
    }
    
    // Simple linear interpolation resampling
    double ratio = static_cast<double>(output_rate) / input_rate;
    size_t output_size = static_cast<size_t>(samples.size() * ratio);
    
    std::vector<i16> result;
    result.reserve(output_size);
    
    for (size_t i = 0; i < output_size; ++i) {
        double input_index = i / ratio;
        size_t index1 = static_cast<size_t>(input_index);
        size_t index2 = std::min(index1 + 1, samples.size() - 1);
        
        if (index1 < samples.size()) {
            double fraction = input_index - index1;
            double sample1 = samples[index1];
            double sample2 = samples[index2];
            double interpolated = sample1 + fraction * (sample2 - sample1);
            
            result.push_back(static_cast<i16>(std::clamp(interpolated, -32768.0, 32767.0)));
        }
    }
    
    return result;
}

// SDL callback implementations
void AudioPipeline::sdl_playback_callback(void* userdata, Uint8* stream, int len) {
    auto* pipeline = static_cast<AudioPipeline*>(userdata);
    pipeline->process_playback_callback(stream, len);
}

void AudioPipeline::sdl_recording_callback(void* userdata, Uint8* stream, int len) {
    auto* pipeline = static_cast<AudioPipeline*>(userdata);
    pipeline->process_recording_callback(stream, len);
}

void AudioPipeline::process_playback_callback(Uint8* stream, int len) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_ || !codec_) {
        // Fill with silence
        SDL_memset(stream, 0, len);
        return;
    }
    
    auto callback_start = std::chrono::steady_clock::now();
    
    // Calculate required frames
    size_t bytes_per_sample = (sdl_state_.playback_spec.format & 0xFF) / 8;
    size_t samples_needed = len / bytes_per_sample;
    size_t frames_needed = samples_needed / sdl_state_.playback_spec.channels;
    
    std::vector<i16> output_samples(samples_needed, 0);
    
    // Get data from codec
    auto codec_data_result = codec_->read_audio_data(samples_needed);
    if (codec_data_result.has_value() && !codec_data_result.value().empty()) {
        auto codec_data = codec_data_result.value();
        
        // Copy up to available data
        size_t copy_size = std::min(codec_data.size(), output_samples.size());
        std::copy(codec_data.begin(), codec_data.begin() + copy_size, 
                 output_samples.begin());
    } else {
        // Try to get data from playback buffer queue
        if (!playback_buffer_queue_.empty()) {
            const auto& buffer = playback_buffer_queue_.front();
            size_t copy_size = std::min(buffer.data.size(), output_samples.size());
            std::copy(buffer.data.begin(), buffer.data.begin() + copy_size, 
                     output_samples.begin());
            playback_buffer_queue_.pop();
        } else {
            handle_buffer_underrun();
        }
    }
    
    // Apply volume
    for (i16& sample : output_samples) {
        sample = static_cast<i16>(std::clamp(sample * stream_config_.volume, -32768.0f, 32767.0f));
    }
    
    // Apply audio effects
    apply_audio_effects(output_samples);
    
    // Convert to SDL format
    convert_samples_to_sdl_format(output_samples, stream, sdl_state_.playback_spec);
    
    // Update statistics
    statistics_.frames_played += frames_needed;
    
    auto callback_end = std::chrono::steady_clock::now();
    auto callback_time = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
    
    // Update CPU usage estimate
    double callback_duration_ms = callback_time.count() / 1000.0;
    double buffer_duration_ms = (frames_needed * 1000.0) / static_cast<u32>(stream_config_.sample_rate);
    double cpu_usage = (callback_duration_ms / buffer_duration_ms) * 100.0;
    
    statistics_.average_cpu_usage = 0.95 * statistics_.average_cpu_usage + 0.05 * cpu_usage;
    statistics_.peak_cpu_usage = std::max(statistics_.peak_cpu_usage, cpu_usage);
}

void AudioPipeline::process_recording_callback(Uint8* stream, int len) {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    if (!initialized_ || !microphone_array_) {
        return;
    }
    
    // Calculate frames
    size_t bytes_per_sample = (sdl_state_.recording_spec.format & 0xFF) / 8;
    size_t samples_captured = len / bytes_per_sample;
    size_t frames_captured = samples_captured / sdl_state_.recording_spec.channels;
    
    // Convert from SDL format
    std::vector<i16> input_samples;
    convert_samples_from_sdl_format(stream, input_samples, sdl_state_.recording_spec);
    
    // Capture from microphone array
    auto array_data_result = microphone_array_->capture_frame(frames_captured);
    if (array_data_result.has_value() && !array_data_result.value().empty()) {
        input_samples = array_data_result.value();
    }
    
    // Apply audio effects
    apply_audio_effects(input_samples);
    
    // Send to codec
    if (codec_) {
        codec_->write_audio_data(input_samples);
    }
    
    // Add to recording buffer queue
    if (recording_buffer_queue_.size() < stream_config_.buffer_count) {
        AudioBuffer buffer;
        buffer.data = input_samples;
        buffer.frames = frames_captured;
        buffer.channels = stream_config_.channels;
        buffer.timestamp = std::chrono::steady_clock::now();
        buffer.is_valid = true;
        
        recording_buffer_queue_.push(buffer);
        buffer_condition_.notify_one();
    } else {
        handle_buffer_overrun();
    }
    
    // Update statistics
    statistics_.frames_recorded += frames_captured;
}

void AudioPipeline::audio_processing_thread() {
    COMPONENT_LOG_DEBUG("Audio processing thread started");
    
    while (processing_active_.load()) {
        std::unique_lock<std::mutex> lock(pipeline_mutex_);
        
        // Process audio callback if set
        if (audio_callback_ && !playback_buffer_queue_.empty() && 
            stream_config_.buffer_mode == AudioBufferMode::CALLBACK) {
            
            AudioBuffer input_buffer;
            if (!recording_buffer_queue_.empty()) {
                input_buffer = recording_buffer_queue_.front();
                recording_buffer_queue_.pop();
            }
            
            AudioBuffer output_buffer;
            output_buffer.data.resize(stream_config_.buffer_size_frames * stream_config_.channels);
            output_buffer.frames = stream_config_.buffer_size_frames;
            output_buffer.channels = stream_config_.channels;
            output_buffer.timestamp = std::chrono::steady_clock::now();
            
            lock.unlock();
            
            try {
                audio_callback_(input_buffer, output_buffer);
                
                lock.lock();
                if (playback_buffer_queue_.size() < stream_config_.buffer_count) {
                    playback_buffer_queue_.push(output_buffer);
                }
            } catch (const std::exception& e) {
                statistics_.callback_errors++;
                if (error_callback_) {
                    error_callback_("Audio callback error: " + std::string(e.what()));
                }
            }
            
            lock.lock();
        }
        
        // Update statistics periodically
        update_statistics();
        
        // Wait for next processing cycle or shutdown
        buffer_condition_.wait_for(lock, std::chrono::milliseconds(10));
    }
    
    COMPONENT_LOG_DEBUG("Audio processing thread stopped");
}

void AudioPipeline::update_statistics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_);
    
    if (elapsed.count() >= 1000) { // Update every second
        // Calculate actual sample rate
        if (stream_start_time_.time_since_epoch().count() > 0) {
            auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stream_start_time_);
            if (total_elapsed.count() > 0) {
                statistics_.sample_rate_actual = (statistics_.frames_played * 1000.0) / total_elapsed.count();
            }
        }
        
        // Update current latency
        u32 samples_per_ms = static_cast<u32>(stream_config_.sample_rate) / 1000;
        statistics_.current_latency_ms = (stream_config_.buffer_size_frames * stream_config_.buffer_count) / samples_per_ms;
        
        last_stats_update_ = now;
    }
}

void AudioPipeline::apply_audio_effects(std::vector<i16>& samples) {
    if (samples.empty()) return;
    
    // Simple noise suppression
    if (noise_suppression_enabled_) {
        for (i16& sample : samples) {
            if (std::abs(sample) < 1000) { // Simple noise gate
                sample = static_cast<i16>(sample * 0.1f);
            }
        }
    }
    
    // Simple automatic gain control
    if (agc_enabled_) {
        // Calculate RMS level
        float rms = 0.0f;
        for (i16 sample : samples) {
            rms += static_cast<float>(sample * sample);
        }
        rms = std::sqrt(rms / samples.size()) / 32768.0f;
        
        // Calculate gain adjustment
        float target_linear = std::pow(10.0f, agc_target_level_ / 20.0f);
        float gain_adjustment = target_linear / std::max(rms, 0.001f);
        gain_adjustment = std::clamp(gain_adjustment, 0.1f, 10.0f);
        
        // Apply gain
        for (i16& sample : samples) {
            sample = static_cast<i16>(std::clamp(sample * gain_adjustment, -32768.0f, 32767.0f));
        }
    }
}

void AudioPipeline::handle_buffer_underrun() {
    statistics_.buffer_underruns++;
    
    if (error_callback_) {
        error_callback_("Audio buffer underrun detected");
    }
}

void AudioPipeline::handle_buffer_overrun() {
    statistics_.buffer_overruns++;
    
    if (error_callback_) {
        error_callback_("Audio buffer overrun detected");
    }
}

void AudioPipeline::convert_samples_to_sdl_format(const std::vector<i16>& input, 
                                                  Uint8* output, const SDL_AudioSpec& spec) {
    if (spec.format == AUDIO_S16LSB) {
        // Direct copy for 16-bit signed
        size_t bytes_to_copy = std::min(input.size() * sizeof(i16), 
                                       static_cast<size_t>(spec.size));
        std::memcpy(output, input.data(), bytes_to_copy);
    } else {
        // Format conversion would go here for other formats
        SDL_memset(output, 0, spec.size);
    }
    
    statistics_.format_conversions++;
}

void AudioPipeline::convert_samples_from_sdl_format(const Uint8* input, 
                                                    std::vector<i16>& output, const SDL_AudioSpec& spec) {
    if (spec.format == AUDIO_S16LSB) {
        // Direct copy for 16-bit signed
        size_t samples = spec.size / sizeof(i16);
        output.resize(samples);
        std::memcpy(output.data(), input, spec.size);
    } else {
        // Format conversion would go here for other formats
        output.clear();
    }
    
    statistics_.format_conversions++;
}

void AudioPipeline::clear_statistics() {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    statistics_ = {};
}

void AudioPipeline::dump_status() const {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    
    COMPONENT_LOG_INFO("=== Audio Pipeline Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Stream config: {} Hz, {} channels, {} frames, mode={}",
                          static_cast<u32>(stream_config_.sample_rate),
                          stream_config_.channels,
                          stream_config_.buffer_size_frames,
                          static_cast<u8>(stream_config_.buffer_mode));
        
        COMPONENT_LOG_INFO("Direction: {}, Volume: {:.2f}", 
                          static_cast<u8>(stream_config_.direction),
                          stream_config_.volume);
        
        COMPONENT_LOG_INFO("SDL devices: playback={} (active={}), recording={} (active={})",
                          sdl_state_.playback_device, sdl_state_.playback_active,
                          sdl_state_.recording_device, sdl_state_.recording_active);
        
        COMPONENT_LOG_INFO("Buffer queues: playback={}/{}, recording={}/{}",
                          playback_buffer_queue_.size(), stream_config_.buffer_count,
                          recording_buffer_queue_.size(), stream_config_.buffer_count);
        
        COMPONENT_LOG_INFO("Audio effects: echo_cancel={}, noise_suppress={}, agc={} (target={:.1f}dB)",
                          echo_cancellation_enabled_, noise_suppression_enabled_,
                          agc_enabled_, agc_target_level_);
        
        COMPONENT_LOG_INFO("Processing: thread_active={}, callback_set={}",
                          processing_active_.load(), audio_callback_ != nullptr);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Frames played: {}", statistics_.frames_played);
        COMPONENT_LOG_INFO("  Frames recorded: {}", statistics_.frames_recorded);
        COMPONENT_LOG_INFO("  Buffer underruns: {}", statistics_.buffer_underruns);
        COMPONENT_LOG_INFO("  Buffer overruns: {}", statistics_.buffer_overruns);
        COMPONENT_LOG_INFO("  Callback errors: {}", statistics_.callback_errors);
        COMPONENT_LOG_INFO("  Format conversions: {}", statistics_.format_conversions);
        COMPONENT_LOG_INFO("  CPU usage: avg={:.1f}%, peak={:.1f}%",
                          statistics_.average_cpu_usage, statistics_.peak_cpu_usage);
        COMPONENT_LOG_INFO("  Current latency: {} ms", statistics_.current_latency_ms);
        COMPONENT_LOG_INFO("  Actual sample rate: {:.1f} Hz", statistics_.sample_rate_actual);
    }
}

}  // namespace m5tab5::emulator