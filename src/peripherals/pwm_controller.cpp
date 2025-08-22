#include "emulator/peripherals/pwm_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("PWMController");

PWMController::PWMController(u8 controller_id)
    : controller_id_(controller_id),
      initialized_(false),
      running_(false),
      frequency_hz_(DEFAULT_FREQUENCY),
      mode_(PWMMode::EDGE_ALIGNED),
      counter_period_(1000),
      interrupt_controller_(nullptr),
      current_counter_(0),
      counter_direction_up_(true) {
    
    channels_.resize(MAX_CHANNELS);
    for (u8 i = 0; i < MAX_CHANNELS; ++i) {
        channels_[i].channel_id = i;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} created", controller_id_);
}

PWMController::~PWMController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("PWM controller {} destroyed", controller_id_);
}

Result<void> PWMController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "PWM controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing PWM controller {}", controller_id_);
    
    interrupt_controller_ = interrupt_controller;
    
    // Reset registers to default values
    registers_ = {};
    registers_.prescaler = 1;
    registers_.period = counter_period_;
    
    // Reset channels
    for (auto& channel : channels_) {
        channel = PWMChannel{};
        channel.channel_id = channel.channel_id; // Preserve channel ID
    }
    
    // Reset timing
    last_update_ = std::chrono::steady_clock::now();
    current_counter_ = 0;
    counter_direction_up_ = true;
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("PWM controller {} initialized successfully", controller_id_);
    
    return {};
}

Result<void> PWMController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down PWM controller {}", controller_id_);
    
    // Stop PWM generation
    running_ = false;
    
    // Disable all channels
    for (auto& channel : channels_) {
        channel.enabled = false;
    }
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("PWM controller {} shutdown completed", controller_id_);
    return {};
}

Result<void> PWMController::configure(u32 frequency_hz, PWMMode mode) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (frequency_hz < MIN_FREQUENCY || frequency_hz > MAX_FREQUENCY) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Frequency out of range"));
    }
    
    frequency_hz_ = frequency_hz;
    mode_ = mode;
    
    // Calculate counter period (microseconds per period)
    counter_period_ = 1000000 / frequency_hz_;
    registers_.period = counter_period_;
    
    // Update control register
    registers_.control = (static_cast<u32>(mode_) << 0) |
                        (running_ ? 0x01 : 0x00);
    
    if (frequency_hz != statistics_.average_frequency_hz) {
        statistics_.frequency_changes++;
        statistics_.average_frequency_hz = frequency_hz;
    }
    
    COMPONENT_LOG_INFO("PWM controller {} configured: {} Hz, mode={}",
                      controller_id_, frequency_hz_, static_cast<u8>(mode_));
    
    return {};
}

Result<void> PWMController::set_period(u32 period_us) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (period_us == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Period cannot be zero"));
    }
    
    counter_period_ = period_us;
    frequency_hz_ = 1000000 / period_us;
    registers_.period = counter_period_;
    
    // Update all channel compare values
    for (auto& channel : channels_) {
        channel.compare_value = calculate_compare_value(channel.channel_id);
        registers_.compare[channel.channel_id] = channel.compare_value;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} period set to {} μs ({} Hz)",
                       controller_id_, period_us, frequency_hz_);
    
    return {};
}

Result<u32> PWMController::get_period() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    return counter_period_;
}

Result<void> PWMController::enable_channel(u8 channel, bool enable) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].enabled = enable;
    
    // Update channel register
    if (enable) {
        registers_.channels[channel] |= 0x01;
    } else {
        registers_.channels[channel] &= ~0x01;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} {}",
                       controller_id_, channel, enable ? "enabled" : "disabled");
    
    return {};
}

Result<void> PWMController::set_duty_cycle(u8 channel, u32 duty_cycle_percent_x100) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    if (duty_cycle_percent_x100 > DUTY_CYCLE_MAX) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Duty cycle out of range"));
    }
    
    if (channels_[channel].duty_cycle != duty_cycle_percent_x100) {
        statistics_.duty_cycle_changes++;
    }
    
    channels_[channel].duty_cycle = duty_cycle_percent_x100;
    channels_[channel].compare_value = calculate_compare_value(channel);
    registers_.compare[channel] = channels_[channel].compare_value;
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} duty cycle set to {:.2f}%",
                       controller_id_, channel, duty_cycle_percent_x100 / 100.0);
    
    return {};
}

Result<void> PWMController::set_compare_value(u8 channel, u32 compare_value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    if (compare_value > counter_period_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Compare value exceeds period"));
    }
    
    channels_[channel].compare_value = compare_value;
    channels_[channel].duty_cycle = (compare_value * DUTY_CYCLE_MAX) / counter_period_;
    registers_.compare[channel] = compare_value;
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} compare value set to {}",
                       controller_id_, channel, compare_value);
    
    return {};
}

Result<void> PWMController::set_polarity(u8 channel, PWMPolarity polarity) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].polarity = polarity;
    
    // Update channel register
    if (polarity == PWMPolarity::ACTIVE_LOW) {
        registers_.channels[channel] |= 0x02;
    } else {
        registers_.channels[channel] &= ~0x02;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} polarity set to {}",
                       controller_id_, channel, static_cast<u8>(polarity));
    
    return {};
}

Result<void> PWMController::set_dead_time(u8 channel, u32 dead_time_ns) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].dead_time_ns = dead_time_ns;
    registers_.dead_time = dead_time_ns;
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} dead time set to {} ns",
                       controller_id_, channel, dead_time_ns);
    
    return {};
}

Result<void> PWMController::enable_complementary(u8 channel, bool enable) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].complementary_enabled = enable;
    
    // Update channel register
    if (enable) {
        registers_.channels[channel] |= 0x04;
    } else {
        registers_.channels[channel] &= ~0x04;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} complementary output {}",
                       controller_id_, channel, enable ? "enabled" : "disabled");
    
    return {};
}

Result<void> PWMController::start() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (running_) {
        return {};
    }
    
    running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    last_update_ = start_time_;
    current_counter_ = 0;
    counter_direction_up_ = true;
    
    // Update control register
    registers_.control |= 0x01;
    
    COMPONENT_LOG_INFO("PWM controller {} started", controller_id_);
    return {};
}

Result<void> PWMController::stop() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    running_ = false;
    
    // Update control register
    registers_.control &= ~0x01;
    
    // Update statistics
    if (start_time_.time_since_epoch().count() > 0) {
        auto now = std::chrono::steady_clock::now();
        auto runtime = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);
        statistics_.total_runtime_ms += runtime.count();
    }
    
    COMPONENT_LOG_INFO("PWM controller {} stopped", controller_id_);
    return {};
}

Result<void> PWMController::reset() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    // Stop if running
    running_ = false;
    
    // Reset counter
    current_counter_ = 0;
    counter_direction_up_ = true;
    registers_.counter = 0;
    
    // Reset all channels
    for (auto& channel : channels_) {
        channel.pulse_count = 0;
    }
    
    COMPONENT_LOG_DEBUG("PWM controller {} reset", controller_id_);
    return {};
}

Result<void> PWMController::enable_interrupt(u8 channel, PWMInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    channels_[channel].interrupt_enabled = true;
    registers_.interrupt_enable |= static_cast<u32>(interrupt_type) << (channel * 4);
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} interrupt {} enabled",
                       controller_id_, channel, static_cast<u8>(interrupt_type));
    
    return {};
}

Result<void> PWMController::disable_interrupt(u8 channel, PWMInterruptType interrupt_type) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    if (channel >= MAX_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid channel number"));
    }
    
    registers_.interrupt_enable &= ~(static_cast<u32>(interrupt_type) << (channel * 4));
    
    // Check if any interrupts are still enabled for this channel
    bool any_enabled = (registers_.interrupt_enable >> (channel * 4)) & 0x0F;
    channels_[channel].interrupt_enabled = any_enabled;
    
    COMPONENT_LOG_DEBUG("PWM controller {} channel {} interrupt {} disabled",
                       controller_id_, channel, static_cast<u8>(interrupt_type));
    
    return {};
}

Result<void> PWMController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    Address offset = address - (PWM0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register
            registers_.control = value;
            if (value & 0x01) {
                start();
            } else {
                stop();
            }
            mode_ = static_cast<PWMMode>((value >> 1) & 0x03);
            break;
            
        case 0x08: // Interrupt enable register
            registers_.interrupt_enable = value;
            break;
            
        case 0x0C: // Interrupt clear register
            registers_.interrupt_status &= ~value;
            break;
            
        case 0x10: // Prescaler register
            registers_.prescaler = value;
            break;
            
        case 0x14: // Period register
            registers_.period = value;
            set_period(value);
            break;
            
        case 0x18: // Counter register (read-only, ignore writes)
            break;
            
        default:
            if (offset >= 0x20 && offset < 0x20 + (MAX_CHANNELS * 4)) {
                // Channel control registers
                u8 channel = (offset - 0x20) / 4;
                registers_.channels[channel] = value;
                
                channels_[channel].enabled = (value & 0x01) != 0;
                channels_[channel].polarity = static_cast<PWMPolarity>((value >> 1) & 0x01);
                channels_[channel].complementary_enabled = (value & 0x04) != 0;
                
            } else if (offset >= 0x40 && offset < 0x40 + (MAX_CHANNELS * 4)) {
                // Compare registers
                u8 channel = (offset - 0x40) / 4;
                registers_.compare[channel] = value;
                set_compare_value(channel, value);
                
            } else {
                return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                    "Invalid PWM register offset: 0x" + std::to_string(offset)));
            }
            break;
    }
    
    return {};
}

Result<u32> PWMController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "PWM controller not initialized"));
    }
    
    Address offset = address - (PWM0_BASE_ADDR + controller_id_ * 0x1000);
    
    switch (offset) {
        case 0x00: // Control register
            return registers_.control;
            
        case 0x04: // Status register
            update_status_register();
            return registers_.status;
            
        case 0x08: // Interrupt enable register
            return registers_.interrupt_enable;
            
        case 0x0C: // Interrupt status register
            return registers_.interrupt_status;
            
        case 0x10: // Prescaler register
            return registers_.prescaler;
            
        case 0x14: // Period register
            return registers_.period;
            
        case 0x18: // Counter register
            return current_counter_;
            
        default:
            if (offset >= 0x20 && offset < 0x20 + (MAX_CHANNELS * 4)) {
                // Channel control registers
                u8 channel = (offset - 0x20) / 4;
                return registers_.channels[channel];
                
            } else if (offset >= 0x40 && offset < 0x40 + (MAX_CHANNELS * 4)) {
                // Compare registers
                u8 channel = (offset - 0x40) / 4;
                return registers_.compare[channel];
                
            } else {
                return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
                    "Invalid PWM register offset: 0x" + std::to_string(offset)));
            }
    }
}

void PWMController::update() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_ || !running_) {
        return;
    }
    
    update_counter();
    process_channels();
    update_status_register();
}

void PWMController::update_counter() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);
    last_update_ = now;
    
    if (elapsed.count() == 0) {
        return;
    }
    
    // Update counter based on elapsed time and frequency
    u32 increment = elapsed.count() * frequency_hz_ / 1000000;
    
    switch (mode_) {
        case PWMMode::EDGE_ALIGNED:
            current_counter_ += increment;
            if (current_counter_ >= counter_period_) {
                current_counter_ = 0;
                statistics_.periods_generated++;
                trigger_interrupt(PWMInterruptType::PERIOD_COMPLETE);
            }
            break;
            
        case PWMMode::CENTER_ALIGNED:
            if (counter_direction_up_) {
                current_counter_ += increment;
                if (current_counter_ >= counter_period_) {
                    current_counter_ = counter_period_;
                    counter_direction_up_ = false;
                }
            } else {
                if (current_counter_ >= increment) {
                    current_counter_ -= increment;
                } else {
                    current_counter_ = 0;
                    counter_direction_up_ = true;
                    statistics_.periods_generated++;
                    trigger_interrupt(PWMInterruptType::PERIOD_COMPLETE);
                }
            }
            break;
            
        case PWMMode::ASYMMETRIC:
            // Similar to center-aligned but with different timing
            current_counter_ += increment;
            if (current_counter_ >= counter_period_) {
                current_counter_ = 0;
                statistics_.periods_generated++;
                trigger_interrupt(PWMInterruptType::PERIOD_COMPLETE);
            }
            break;
    }
    
    registers_.counter = current_counter_;
}

void PWMController::process_channels() {
    for (auto& channel : channels_) {
        if (!channel.enabled) {
            continue;
        }
        
        bool was_active = is_channel_active(channel.channel_id);
        bool is_active = false;
        
        switch (mode_) {
            case PWMMode::EDGE_ALIGNED:
                is_active = current_counter_ < channel.compare_value;
                break;
                
            case PWMMode::CENTER_ALIGNED:
                if (counter_direction_up_) {
                    is_active = current_counter_ < channel.compare_value;
                } else {
                    is_active = current_counter_ > channel.compare_value;
                }
                break;
                
            case PWMMode::ASYMMETRIC:
                is_active = current_counter_ < channel.compare_value;
                break;
        }
        
        // Apply polarity
        if (channel.polarity == PWMPolarity::ACTIVE_LOW) {
            is_active = !is_active;
        }
        
        // Check for compare match interrupt
        if (!was_active && is_active && channel.interrupt_enabled) {
            trigger_interrupt(PWMInterruptType::COMPARE_MATCH, channel.channel_id);
            channel.pulse_count++;
        }
    }
}

void PWMController::trigger_interrupt(PWMInterruptType interrupt_type, u8 channel) {
    registers_.interrupt_status |= static_cast<u32>(interrupt_type) << (channel * 4);
    
    if (registers_.interrupt_enable & (static_cast<u32>(interrupt_type) << (channel * 4))) {
        if (interrupt_controller_) {
            interrupt_controller_->trigger_interrupt(
                static_cast<InterruptType>(static_cast<u8>(InterruptType::TIMER0) + controller_id_));
        }
        statistics_.interrupts_triggered++;
    }
}

void PWMController::update_status_register() {
    registers_.status = 0;
    
    if (running_) {
        registers_.status |= 0x01;
    }
    
    if (current_counter_ == 0) {
        registers_.status |= 0x02; // Counter at zero
    }
    
    if (current_counter_ >= counter_period_) {
        registers_.status |= 0x04; // Counter overflow
    }
    
    // Channel status bits
    for (u8 i = 0; i < MAX_CHANNELS; ++i) {
        if (channels_[i].enabled && is_channel_active(i)) {
            registers_.status |= (1 << (8 + i));
        }
    }
}

u32 PWMController::calculate_compare_value(u8 channel) const {
    return (channels_[channel].duty_cycle * counter_period_) / DUTY_CYCLE_MAX;
}

bool PWMController::is_channel_active(u8 channel) const {
    if (channel >= MAX_CHANNELS || !channels_[channel].enabled) {
        return false;
    }
    
    bool active = current_counter_ < channels_[channel].compare_value;
    
    if (channels_[channel].polarity == PWMPolarity::ACTIVE_LOW) {
        active = !active;
    }
    
    return active;
}

const PWMChannel& PWMController::get_channel(u8 channel) const {
    static const PWMChannel dummy_channel{};
    
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (channel >= MAX_CHANNELS) {
        return dummy_channel;
    }
    
    return channels_[channel];
}

void PWMController::clear_statistics() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    statistics_ = {};
}

void PWMController::dump_status() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== PWM Controller {} Status ===", controller_id_);
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Running: {}", running_);
        COMPONENT_LOG_INFO("Frequency: {} Hz", frequency_hz_);
        COMPONENT_LOG_INFO("Mode: {}", static_cast<u8>(mode_));
        COMPONENT_LOG_INFO("Period: {} μs", counter_period_);
        COMPONENT_LOG_INFO("Current counter: {}", current_counter_);
        
        COMPONENT_LOG_INFO("Channels:");
        for (u8 i = 0; i < MAX_CHANNELS; ++i) {
            const auto& ch = channels_[i];
            if (ch.enabled) {
                COMPONENT_LOG_INFO("  CH{}: {:.2f}%, compare={}, polarity={}, pulses={}",
                                  i, ch.duty_cycle / 100.0, ch.compare_value,
                                  static_cast<u8>(ch.polarity), ch.pulse_count);
            }
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Periods generated: {}", statistics_.periods_generated);
        COMPONENT_LOG_INFO("  Interrupts triggered: {}", statistics_.interrupts_triggered);
        COMPONENT_LOG_INFO("  Duty cycle changes: {}", statistics_.duty_cycle_changes);
        COMPONENT_LOG_INFO("  Frequency changes: {}", statistics_.frequency_changes);
        COMPONENT_LOG_INFO("  Average frequency: {:.2f} Hz", statistics_.average_frequency_hz);
        COMPONENT_LOG_INFO("  Total runtime: {:.2f} ms", statistics_.total_runtime_ms);
    }
}

}  // namespace m5tab5::emulator