#include "emulator/peripherals/adc_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>

namespace m5tab5::emulator {

DECLARE_LOGGER("ADCController");

ADCController::ADCController(u8 controller_id)
    : controller_id_(controller_id),
      initialized_(false),
      converting_(false),
      calibrating_(false),
      resolution_(ADCResolution::BITS_12),
      mode_(ADCMode::SINGLE_CONVERSION),
      alignment_(ADCAlignment::RIGHT),
      trigger_(ADCTrigger::SOFTWARE),
      clock_rate_hz_(DEFAULT_CLOCK_RATE),
      interrupt_controller_(nullptr),
      current_sequence_index_(0),
      current_injected_index_(0),
      calibration_duration_us_(1000),
      random_generator_(std::chrono::steady_clock::now().time_since_epoch().count()),
      noise_distribution_(-0.05f, 0.05f) {
    
    channels_.resize(MAX_CHANNELS);
    for (u8 i = 0; i < MAX_CHANNELS; ++i) {
        channels_[i].channel_id = i;
        channels_[i].threshold_high = get_max_value();
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} created", controller_id_);
}

ADCController::~ADCController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("ADC controller {} destroyed", controller_id_);
}

Result<void> ADCController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "ADC controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing ADC controller {}", controller_id_);
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset registers to default values
    registers_ = {};
    registers_.control1 = static_cast<u32>(resolution_) << 24;
    registers_.control2 = 0;
    
    // Reset channels
    for (auto& channel : channels_) {
        channel.enabled = false;
        channel.sampling_time = 1;
        channel.offset = 0;
        channel.gain = 1.0f;
        channel.threshold_low = 0;
        channel.threshold_high = get_max_value();
        channel.watchdog_enabled = false;
        channel.conversions_completed = 0;
        channel.last_value = 0;
    }
    
    // Clear sequences
    regular_sequence_.clear();
    injected_sequence_.clear();
    
    // Clear FIFO
    while (!conversion_fifo_.empty()) {
        conversion_fifo_.pop();
    }
    
    // Reset conversion state
    converting_ = false;
    calibrating_ = false;
    current_sequence_index_ = 0;
    current_injected_index_ = 0;
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("ADC controller {} initialized successfully", controller_id_);
    
    return {};
}

Result<void> ADCController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down ADC controller {}", controller_id_);
    
    // Stop any ongoing conversions
    converting_ = false;
    calibrating_ = false;
    
    // Clear FIFO
    while (!conversion_fifo_.empty()) {
        conversion_fifo_.pop();
    }
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("ADC controller {} shutdown completed", controller_id_);
    return {};
}

Result<void> ADCController::configure(ADCResolution resolution, ADCMode mode, ADCAlignment alignment) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (converting_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot configure ADC while converting"));
    }
    
    resolution_ = resolution;
    mode_ = mode;
    alignment_ = alignment;
    
    // Update control registers
    registers_.control1 = (static_cast<u32>(resolution_) << 24) |
                         (static_cast<u32>(alignment_) << 11);
    registers_.control2 = static_cast<u32>(mode_) << 1;
    
    // Update thresholds for all channels based on new resolution
    u32 max_value = get_max_value();
    for (auto& channel : channels_) {
        if (channel.threshold_high > max_value) {
            channel.threshold_high = max_value;
        }
    }
    
    COMPONENT_LOG_INFO("ADC controller {} configured: {}-bit, mode={}, alignment={}",
                      controller_id_, static_cast<u8>(resolution_),
                      static_cast<u8>(mode_), static_cast<u8>(alignment_));
    
    return {};
}

Result<void> ADCController::set_clock_rate(u32 clock_rate_hz) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (clock_rate_hz < MIN_CLOCK_RATE || clock_rate_hz > MAX_CLOCK_RATE) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Clock rate out of range"));
    }
    
    clock_rate_hz_ = clock_rate_hz;
    
    COMPONENT_LOG_DEBUG("ADC controller {} clock rate set to {} Hz", controller_id_, clock_rate_hz_);
    return {};
}

Result<void> ADCController::set_trigger(ADCTrigger trigger) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    trigger_ = trigger;
    registers_.control2 = (registers_.control2 & ~(0x0F << 24)) | (static_cast<u32>(trigger_) << 24);
    
    COMPONENT_LOG_DEBUG("ADC controller {} trigger set to {}", controller_id_, static_cast<u8>(trigger_));
    return {};
}

Result<void> ADCController::enable_channel(u8 channel, bool enable) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].enabled = enable;
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} {}",
                       controller_id_, channel, enable ? "enabled" : "disabled");
    
    return {};
}

Result<void> ADCController::set_channel_sampling_time(u8 channel, u32 sampling_cycles) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    if (sampling_cycles == 0 || sampling_cycles > 255) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid sampling time"));
    }
    
    channels_[channel].sampling_time = sampling_cycles;
    
    // Update sampling time registers
    if (channel < 10) {
        registers_.sampling_time1 = (registers_.sampling_time1 & ~(0x07 << (channel * 3))) |
                                   (sampling_cycles << (channel * 3));
    } else {
        registers_.sampling_time2 = (registers_.sampling_time2 & ~(0x07 << ((channel - 10) * 3))) |
                                   (sampling_cycles << ((channel - 10) * 3));
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} sampling time set to {} cycles",
                       controller_id_, channel, sampling_cycles);
    
    return {};
}

Result<void> ADCController::set_channel_offset(u8 channel, u32 offset) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    if (offset > get_max_value()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Offset value too large"));
    }
    
    channels_[channel].offset = offset;
    
    if (channel < 4) {
        registers_.offset[channel] = offset;
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} offset set to {}",
                       controller_id_, channel, offset);
    
    return {};
}

Result<void> ADCController::set_channel_gain(u8 channel, float gain) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    if (gain <= 0.0f || gain > 10.0f) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid gain value"));
    }
    
    channels_[channel].gain = gain;
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} gain set to {:.3f}",
                       controller_id_, channel, gain);
    
    return {};
}

Result<void> ADCController::configure_sequence(const std::vector<u8>& channels) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channels.empty() || channels.size() > MAX_SEQUENCE_LENGTH) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid sequence length"));
    }
    
    for (u8 channel : channels) {
        if (channel >= MAX_CHANNELS) {
            return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
                "Invalid channel in sequence"));
        }
    }
    
    regular_sequence_ = channels;
    
    // Update sequence registers
    registers_.sequence1 = channels.size() - 1; // Length
    for (size_t i = 0; i < std::min(channels.size(), static_cast<size_t>(6)); ++i) {
        registers_.sequence1 |= (channels[i] & 0x1F) << (5 * (i + 1));
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} regular sequence configured with {} channels",
                       controller_id_, channels.size());
    
    return {};
}

Result<void> ADCController::configure_injected_sequence(const std::vector<u8>& channels) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channels.size() > 4) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Injected sequence too long (max 4 channels)"));
    }
    
    for (u8 channel : channels) {
        if (channel >= MAX_CHANNELS) {
            return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
                "Invalid channel in injected sequence"));
        }
    }
    
    injected_sequence_ = channels;
    
    // Update injected sequence register
    registers_.injected_sequence = channels.size();
    for (size_t i = 0; i < channels.size(); ++i) {
        registers_.injected_sequence |= (channels[i] & 0x1F) << (5 * (i + 1));
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} injected sequence configured with {} channels",
                       controller_id_, channels.size());
    
    return {};
}

Result<void> ADCController::enable_analog_watchdog(u8 channel, u32 low_threshold, u32 high_threshold) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    u32 max_value = get_max_value();
    if (low_threshold > max_value || high_threshold > max_value || low_threshold >= high_threshold) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid threshold values"));
    }
    
    channels_[channel].watchdog_enabled = true;
    channels_[channel].threshold_low = low_threshold;
    channels_[channel].threshold_high = high_threshold;
    
    registers_.watchdog_low = low_threshold;
    registers_.watchdog_high = high_threshold;
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} watchdog enabled: {}-{}",
                       controller_id_, channel, low_threshold, high_threshold);
    
    return {};
}

Result<void> ADCController::disable_analog_watchdog(u8 channel) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].watchdog_enabled = false;
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} watchdog disabled", controller_id_, channel);
    return {};
}

Result<void> ADCController::start_conversion() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (converting_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "ADC already converting"));
    }
    
    if (regular_sequence_.empty()) {
        return std::unexpected(MAKE_ERROR(INVALID_OPERATION,
            "No conversion sequence configured"));
    }
    
    converting_ = true;
    current_sequence_index_ = 0;
    conversion_start_ = std::chrono::steady_clock::now();
    last_conversion_ = conversion_start_;
    
    registers_.control2 |= 0x01; // Start conversion bit
    
    COMPONENT_LOG_DEBUG("ADC controller {} conversion started", controller_id_);
    return {};
}

Result<void> ADCController::stop_conversion() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    converting_ = false;
    registers_.control2 &= ~0x01; // Clear start conversion bit
    
    COMPONENT_LOG_DEBUG("ADC controller {} conversion stopped", controller_id_);
    return {};
}

Result<u32> ADCController::read_conversion_result(u8 channel) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (channel == 0xFF) {
        // Read last conversion result
        return registers_.data;
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    return channels_[channel].last_value;
}

Result<std::vector<ADCConversion>> ADCController::read_fifo_data(size_t max_count) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    std::vector<ADCConversion> result;
    size_t count = (max_count == 0) ? conversion_fifo_.size() : std::min(max_count, conversion_fifo_.size());
    result.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        if (!conversion_fifo_.empty()) {
            result.push_back(conversion_fifo_.front());
            conversion_fifo_.pop();
        }
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} read {} FIFO entries", controller_id_, result.size());
    return result;
}

Result<void> ADCController::calibrate() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (converting_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_BUSY,
            "Cannot calibrate while converting"));
    }
    
    calibrating_ = true;
    calibration_start_ = std::chrono::steady_clock::now();
    statistics_.calibrations_performed++;
    
    COMPONENT_LOG_INFO("ADC controller {} calibration started", controller_id_);
    return {};
}

Result<bool> ADCController::is_calibration_complete() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    if (!calibrating_) {
        return true;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - calibration_start_);
    
    return elapsed.count() >= calibration_duration_us_;
}

Result<void> ADCController::enable_interrupt(ADCInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    registers_.interrupt_enable |= static_cast<u32>(interrupt_type);
    
    COMPONENT_LOG_DEBUG("ADC controller {} interrupt {} enabled",
                       controller_id_, static_cast<u8>(interrupt_type));
    
    return {};
}

Result<void> ADCController::disable_interrupt(ADCInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    registers_.interrupt_enable &= ~static_cast<u32>(interrupt_type);
    
    COMPONENT_LOG_DEBUG("ADC controller {} interrupt {} disabled",
                       controller_id_, static_cast<u8>(interrupt_type));
    
    return {};
}

Result<void> ADCController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    Address offset = address - (ADC0_BASE_ADDR + controller_id_ * 0x400);
    
    switch (offset) {
        case 0x00: // Control register 1
            registers_.control1 = value;
            resolution_ = static_cast<ADCResolution>((value >> 24) & 0x03);
            alignment_ = static_cast<ADCAlignment>((value >> 11) & 0x01);
            break;
            
        case 0x04: // Control register 2
            registers_.control2 = value;
            mode_ = static_cast<ADCMode>((value >> 1) & 0x03);
            trigger_ = static_cast<ADCTrigger>((value >> 24) & 0x0F);
            
            if (value & 0x01) { // Start conversion
                start_conversion();
            }
            if (value & 0x02) { // Calibration
                calibrate();
            }
            break;
            
        case 0x0C: // Interrupt enable register
            registers_.interrupt_enable = value;
            break;
            
        case 0x10: // Interrupt clear register
            registers_.interrupt_status &= ~value;
            break;
            
        case 0x2C: // Sequence register 1
            registers_.sequence1 = value;
            // Parse sequence from register
            {
                std::vector<u8> sequence;
                u8 length = value & 0x0F;
                for (u8 i = 0; i < length && i < 6; ++i) {
                    u8 channel = (value >> (5 * (i + 1))) & 0x1F;
                    sequence.push_back(channel);
                }
                regular_sequence_ = sequence;
            }
            break;
            
        case 0x38: // Injected sequence register
            registers_.injected_sequence = value;
            // Parse injected sequence from register
            {
                std::vector<u8> sequence;
                u8 length = value & 0x03;
                for (u8 i = 0; i < length; ++i) {
                    u8 channel = (value >> (5 * (i + 1))) & 0x1F;
                    sequence.push_back(channel);
                }
                injected_sequence_ = sequence;
            }
            break;
            
        case 0x44: // Watchdog high threshold
            registers_.watchdog_high = value & get_max_value();
            break;
            
        case 0x48: // Watchdog low threshold
            registers_.watchdog_low = value & get_max_value();
            break;
            
        default:
            if (offset >= 0x4C && offset < 0x5C) {
                // Sampling time registers
                u8 reg_index = (offset - 0x4C) / 4;
                if (reg_index == 0) {
                    registers_.sampling_time1 = value;
                } else {
                    registers_.sampling_time2 = value;
                }
            } else {
                return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                    "Invalid ADC register offset: 0x" + std::to_string(offset)));
            }
            break;
    }
    
    return {};
}

Result<u32> ADCController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "ADC controller not initialized"));
    }
    
    Address offset = address - (ADC0_BASE_ADDR + controller_id_ * 0x400);
    
    switch (offset) {
        case 0x00: // Control register 1
            return registers_.control1;
            
        case 0x04: // Control register 2
            return registers_.control2;
            
        case 0x08: // Status register
            update_status_register();
            return registers_.status;
            
        case 0x0C: // Interrupt enable register
            return registers_.interrupt_enable;
            
        case 0x10: // Interrupt status register
            return registers_.interrupt_status;
            
        case 0x14: // Data register
            return registers_.data;
            
        case 0x2C: // Sequence register 1
            return registers_.sequence1;
            
        case 0x38: // Injected sequence register
            return registers_.injected_sequence;
            
        case 0x44: // Watchdog high threshold
            return registers_.watchdog_high;
            
        case 0x48: // Watchdog low threshold
            return registers_.watchdog_low;
            
        case 0x4C: // Sampling time register 1
            return registers_.sampling_time1;
            
        case 0x50: // Sampling time register 2
            return registers_.sampling_time2;
            
        default:
            return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                "Invalid ADC register offset: 0x" + std::to_string(offset)));
    }
}

void ADCController::update() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Check calibration completion
    if (calibrating_) {
        auto calibration_complete = is_calibration_complete();
        if (calibration_complete.has_value() && calibration_complete.value()) {
            calibrating_ = false;
            trigger_interrupt(ADCInterruptType::CALIBRATION_COMPLETE);
            COMPONENT_LOG_DEBUG("ADC controller {} calibration completed", controller_id_);
        }
    }
    
    if (converting_) {
        update_conversion();
    }
    
    update_status_register();
}

void ADCController::update_conversion() {
    auto now = std::chrono::steady_clock::now();
    
    // Process regular sequence
    if (!regular_sequence_.empty()) {
        process_regular_sequence();
    }
    
    // Process injected sequence (higher priority)
    if (!injected_sequence_.empty()) {
        process_injected_sequence();
    }
    
    // Update conversion rate statistics
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - conversion_start_);
    if (elapsed.count() > 0) {
        statistics_.conversion_rate_sps = (statistics_.total_conversions * 1000.0) / elapsed.count();
    }
}

void ADCController::process_regular_sequence() {
    if (current_sequence_index_ >= regular_sequence_.size()) {
        // Sequence complete
        current_sequence_index_ = 0;
        
        if (mode_ == ADCMode::SINGLE_CONVERSION) {
            converting_ = false;
            registers_.control2 &= ~0x01;
        }
        
        trigger_interrupt(ADCInterruptType::END_OF_SEQUENCE);
        return;
    }
    
    u8 channel = regular_sequence_[current_sequence_index_];
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_conversion_);
    u32 required_time = calculate_conversion_time_us(channel);
    
    if (elapsed.count() >= required_time) {
        u32 value = simulate_channel_value(channel);
        value = apply_channel_processing(channel, value);
        
        complete_conversion(channel, value, false);
        
        current_sequence_index_++;
        last_conversion_ = now;
    }
}

void ADCController::process_injected_sequence() {
    // Simplified injected sequence processing
    if (!injected_sequence_.empty()) {
        u8 channel = injected_sequence_[0];
        u32 value = simulate_channel_value(channel);
        value = apply_channel_processing(channel, value);
        
        complete_conversion(channel, value, true);
    }
}

void ADCController::complete_conversion(u8 channel, u32 value, bool is_injected) {
    channels_[channel].last_value = value;
    channels_[channel].conversions_completed++;
    
    if (!is_injected) {
        registers_.data = value;
        statistics_.regular_conversions++;
    } else {
        statistics_.injected_conversions++;
    }
    
    statistics_.total_conversions++;
    
    // Add to FIFO
    if (conversion_fifo_.size() < FIFO_SIZE) {
        ADCConversion conversion;
        conversion.channel = channel;
        conversion.value = value;
        conversion.timestamp = std::chrono::steady_clock::now();
        conversion.is_injected = is_injected;
        conversion_fifo_.push(conversion);
    } else {
        statistics_.overrun_errors++;
        trigger_interrupt(ADCInterruptType::OVERRUN);
    }
    
    // Check analog watchdog
    check_analog_watchdog(channel, value);
    
    // Trigger completion interrupt
    if (is_injected) {
        trigger_interrupt(ADCInterruptType::INJECTED_COMPLETE);
    } else {
        trigger_interrupt(ADCInterruptType::CONVERSION_COMPLETE);
    }
    
    COMPONENT_LOG_DEBUG("ADC controller {} channel {} conversion: {}",
                       controller_id_, channel, value);
}

void ADCController::trigger_interrupt(ADCInterruptType interrupt_type) {
    registers_.interrupt_status |= static_cast<u32>(interrupt_type);
    
    if (registers_.interrupt_enable & static_cast<u32>(interrupt_type)) {
        if (interrupt_controller_) {
            // Map to appropriate ADC interrupt
            interrupt_controller_->trigger_interrupt(
                static_cast<InterruptType>(static_cast<u8>(InterruptType::DMA_CH0) + 16 + controller_id_));
        }
    }
}

void ADCController::update_status_register() {
    registers_.status = 0;
    
    if (converting_) {
        registers_.status |= 0x01;
    }
    
    if (calibrating_) {
        registers_.status |= 0x02;
    }
    
    if (!conversion_fifo_.empty()) {
        registers_.status |= 0x04; // Data ready
    }
    
    if (conversion_fifo_.size() >= FIFO_SIZE / 2) {
        registers_.status |= 0x08; // FIFO half full
        trigger_interrupt(ADCInterruptType::FIFO_THRESHOLD);
    }
    
    if (conversion_fifo_.size() >= FIFO_SIZE) {
        registers_.status |= 0x10; // FIFO full
    }
}

u32 ADCController::simulate_channel_value(u8 channel) {
    // Simulate different signal types based on channel
    u32 max_value = get_max_value();
    float base_value = 0.0f;
    
    switch (channel) {
        case 0: // DC voltage
            base_value = 0.5f;
            break;
        case 1: // Sine wave
            {
                auto now = std::chrono::steady_clock::now();
                auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()).count();
                base_value = 0.5f + 0.3f * std::sin(2.0 * M_PI * time_us / 1000000.0);
            }
            break;
        case 2: // Triangle wave
            {
                auto now = std::chrono::steady_clock::now();
                auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()).count();
                float t = std::fmod(time_us / 1000000.0, 1.0);
                base_value = 0.5f + 0.3f * (t < 0.5 ? 2.0f * t - 0.5f : 1.5f - 2.0f * t);
            }
            break;
        default: // Random noise
            base_value = 0.5f + noise_distribution_(random_generator_);
            break;
    }
    
    // Add noise
    base_value += noise_distribution_(random_generator_);
    
    // Clamp to valid range
    base_value = std::clamp(base_value, 0.0f, 1.0f);
    
    return static_cast<u32>(base_value * max_value);
}

u32 ADCController::apply_channel_processing(u8 channel, u32 raw_value) {
    float processed = static_cast<float>(raw_value);
    
    // Apply gain
    processed *= channels_[channel].gain;
    
    // Apply offset
    processed += static_cast<float>(channels_[channel].offset);
    
    // Clamp to valid range
    u32 max_value = get_max_value();
    processed = std::clamp(processed, 0.0f, static_cast<float>(max_value));
    
    u32 result = static_cast<u32>(processed);
    
    // Apply alignment
    if (alignment_ == ADCAlignment::LEFT) {
        result <<= (16 - static_cast<u8>(resolution_));
    }
    
    return result;
}

u32 ADCController::calculate_conversion_time_us(u8 channel) const {
    // Conversion time = (sampling time + resolution bits) / clock rate
    u32 total_cycles = channels_[channel].sampling_time + static_cast<u32>(resolution_);
    return (total_cycles * 1000000) / clock_rate_hz_;
}

void ADCController::check_analog_watchdog(u8 channel, u32 value) {
    if (!channels_[channel].watchdog_enabled) {
        return;
    }
    
    if (value < channels_[channel].threshold_low || value > channels_[channel].threshold_high) {
        statistics_.watchdog_events++;
        trigger_interrupt(ADCInterruptType::ANALOG_WATCHDOG);
        
        COMPONENT_LOG_DEBUG("ADC controller {} channel {} watchdog triggered: value={}, range={}-{}",
                           controller_id_, channel, value,
                           channels_[channel].threshold_low, channels_[channel].threshold_high);
    }
}

u32 ADCController::get_max_value() const {
    return (1U << static_cast<u8>(resolution_)) - 1;
}

void ADCController::clear_statistics() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    statistics_ = {};
}

void ADCController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== ADC Controller {} Status ===", controller_id_);
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Configuration: {}-bit, mode={}, alignment={}",
                          static_cast<u8>(resolution_), static_cast<u8>(mode_), static_cast<u8>(alignment_));
        COMPONENT_LOG_INFO("Clock rate: {} Hz", clock_rate_hz_);
        COMPONENT_LOG_INFO("Trigger: {}", static_cast<u8>(trigger_));
        
        COMPONENT_LOG_INFO("Status: converting={}, calibrating={}", converting_, calibrating_);
        COMPONENT_LOG_INFO("FIFO: {}/{}", conversion_fifo_.size(), FIFO_SIZE);
        
        COMPONENT_LOG_INFO("Regular sequence: {} channels", regular_sequence_.size());
        COMPONENT_LOG_INFO("Injected sequence: {} channels", injected_sequence_.size());
        
        COMPONENT_LOG_INFO("Active channels:");
        for (u8 i = 0; i < MAX_CHANNELS; ++i) {
            const auto& ch = channels_[i];
            if (ch.enabled) {
                COMPONENT_LOG_INFO("  CH{}: sampling={} cycles, gain={:.3f}, offset={}, conversions={}",
                                  i, ch.sampling_time, ch.gain, ch.offset, ch.conversions_completed);
            }
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Total conversions: {}", statistics_.total_conversions);
        COMPONENT_LOG_INFO("  Regular conversions: {}", statistics_.regular_conversions);
        COMPONENT_LOG_INFO("  Injected conversions: {}", statistics_.injected_conversions);
        COMPONENT_LOG_INFO("  Conversion rate: {:.2f} SPS", statistics_.conversion_rate_sps);
        COMPONENT_LOG_INFO("  Overrun errors: {}", statistics_.overrun_errors);
        COMPONENT_LOG_INFO("  Watchdog events: {}", statistics_.watchdog_events);
        COMPONENT_LOG_INFO("  Calibrations performed: {}", statistics_.calibrations_performed);
    }
}

}  // namespace m5tab5::emulator