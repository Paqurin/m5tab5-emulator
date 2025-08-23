#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("GPIOController");

GPIOController::GPIOController() 
    : direction_register_(0),
      output_register_(0),
      interrupt_enable_(0),
      interrupt_type_(0),
      interrupt_status_(0),
      pullup_enable_(0),
      pulldown_enable_(0),
      last_pwm_update_(0),
      stats_{} {
    COMPONENT_LOG_DEBUG("GPIOController created");
    
    // Initialize all pins to input mode
    for (auto& pin : pin_states_) {
        pin = PinState{};
    }
}

GPIOController::~GPIOController() {
    COMPONENT_LOG_DEBUG("GPIOController destroyed");
}

EmulatorError GPIOController::initialize() {
    COMPONENT_LOG_INFO("Initializing GPIO controller with {} pins", GPIO_PIN_COUNT);
    
    // Reset all pin states
    for (auto& pin : pin_states_) {
        pin = PinState{};
    }
    
    // Reset registers
    direction_register_ = 0;
    output_register_ = 0;
    interrupt_enable_ = 0;
    interrupt_type_ = 0;
    interrupt_status_ = 0;
    pullup_enable_ = 0;
    pulldown_enable_ = 0;
    
    // Reset statistics
    stats_ = {};
    
    COMPONENT_LOG_INFO("GPIO controller initialized successfully");
    return EmulatorError::Success;
}

EmulatorError GPIOController::reset() {
    COMPONENT_LOG_DEBUG("Resetting GPIO controller");
    
    // Reset all pin states to default (input mode)
    for (auto& pin : pin_states_) {
        pin = PinState{};
    }
    
    // Reset all registers
    direction_register_ = 0;
    output_register_ = 0;
    interrupt_enable_ = 0;
    interrupt_type_ = 0;
    interrupt_status_ = 0;
    pullup_enable_ = 0;
    pulldown_enable_ = 0;
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::tick(ClockCycle cycle) {
    // Update PWM outputs if needed
    if (cycle - last_pwm_update_ >= (CPU_FREQ_HZ / PWM_FREQUENCY)) {
        for (uint8_t pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
            if (pin_states_[pin].mode == PinMode::PWM) {
                updatePinOutput(pin);
                stats_.pwm_updates++;
            }
        }
        last_pwm_update_ = cycle;
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::readRegister(Address address, uint32_t& value) {
    Register reg = static_cast<Register>(address);
    
    switch (reg) {
        case Register::GPIO_OUT:
            value = output_register_;
            break;
            
        case Register::GPIO_IN: {
            uint32_t input_value = 0;
            for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
                if (digitalRead(pin)) {
                    input_value |= (1U << pin);
                }
            }
            value = input_value;
            break;
        }
        
        case Register::GPIO_DIR:
            value = direction_register_;
            break;
            
        case Register::GPIO_INT_EN:
            value = interrupt_enable_;
            break;
            
        case Register::GPIO_INT_TYPE:
            value = interrupt_type_;
            break;
            
        case Register::GPIO_INT_STAT:
            value = interrupt_status_;
            break;
            
        case Register::GPIO_PULLUP_EN:
            value = pullup_enable_;
            break;
            
        case Register::GPIO_PULLDOWN_EN:
            value = pulldown_enable_;
            break;
            
        default:
            value = 0;
            return EmulatorError::InvalidAddress;
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::writeRegister(Address address, uint32_t value) {
    Register reg = static_cast<Register>(address);
    
    switch (reg) {
        case Register::GPIO_OUT:
            output_register_ = value;
            return handleOutputRegister(value);
            
        case Register::GPIO_OUT_SET:
            output_register_ |= value;
            return handleOutputRegister(output_register_);
            
        case Register::GPIO_OUT_CLR:
            output_register_ &= ~value;
            return handleOutputRegister(output_register_);
            
        case Register::GPIO_DIR:
            direction_register_ = value;
            return handleDirectionRegister(value);
            
        case Register::GPIO_INT_EN:
            interrupt_enable_ = value;
            return handleInterruptEnable(value);
            
        case Register::GPIO_INT_TYPE:
            interrupt_type_ = value;
            return handleInterruptType(value);
            
        case Register::GPIO_INT_CLR:
            interrupt_status_ &= ~value;
            break;
            
        case Register::GPIO_PULLUP_EN:
            pullup_enable_ = value;
            for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
                enablePullUp(pin, (value & (1U << pin)) != 0);
            }
            break;
            
        case Register::GPIO_PULLDOWN_EN:
            pulldown_enable_ = value;
            for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
                enablePullDown(pin, (value & (1U << pin)) != 0);
            }
            break;
            
        default:
            return EmulatorError::InvalidAddress;
    }
    
    return EmulatorError::Success;
}

std::vector<Address> GPIOController::getRegisterAddresses() const {
    return {
        static_cast<Address>(Register::GPIO_OUT),
        static_cast<Address>(Register::GPIO_OUT_SET),
        static_cast<Address>(Register::GPIO_OUT_CLR),
        static_cast<Address>(Register::GPIO_IN),
        static_cast<Address>(Register::GPIO_DIR),
        static_cast<Address>(Register::GPIO_INT_EN),
        static_cast<Address>(Register::GPIO_INT_TYPE),
        static_cast<Address>(Register::GPIO_INT_STAT),
        static_cast<Address>(Register::GPIO_INT_CLR),
        static_cast<Address>(Register::GPIO_PULLUP_EN),
        static_cast<Address>(Register::GPIO_PULLDOWN_EN),
        static_cast<Address>(Register::GPIO_PWM_CTRL),
        static_cast<Address>(Register::GPIO_ADC_CTRL),
        static_cast<Address>(Register::GPIO_PIN_CTRL)
    };
}

std::vector<uint32_t> GPIOController::getInterruptIds() const {
    return {GPIO_INTERRUPT_ID};
}

EmulatorError GPIOController::setPinMode(uint8_t pin, PinMode mode) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    pin_states_[pin].mode = mode;
    
    // Update direction register
    switch (mode) {
        case PinMode::Input:
        case PinMode::InputPullUp:
        case PinMode::InputPullDown:
        case PinMode::AnalogInput:
            direction_register_ &= ~(1U << pin);
            break;
            
        case PinMode::Output:
        case PinMode::OpenDrain:
        case PinMode::PWM:
            direction_register_ |= (1U << pin);
            break;
    }
    
    COMPONENT_LOG_DEBUG("GPIO pin {} mode set to {}", pin, static_cast<int>(mode));
    return EmulatorError::Success;
}

GPIOController::PinMode GPIOController::getPinMode(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return PinMode::Input;
    }
    return pin_states_[pin].mode;
}

EmulatorError GPIOController::digitalWrite(uint8_t pin, bool level) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    bool previous_level = pin_states_[pin].level;
    pin_states_[pin].level = level;
    
    // Update output register
    if (level) {
        output_register_ |= (1U << pin);
    } else {
        output_register_ &= ~(1U << pin);
    }
    
    // Check for interrupt conditions
    checkPinInterrupt(pin, previous_level, level);
    
    stats_.pin_writes++;
    COMPONENT_LOG_TRACE("GPIO pin {} set to {}", pin, level ? "HIGH" : "LOW");
    
    return EmulatorError::Success;
}

bool GPIOController::digitalRead(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return false;
    }
    
    bool level = pin_states_[pin].level;
    
    // Apply pull-up/pull-down if pin is floating
    if (pin_states_[pin].mode == PinMode::Input) {
        if (isPullUpEnabled(pin)) {
            level = true;
        } else if (isPullDownEnabled(pin)) {
            level = false;
        }
    }
    
    const_cast<GPIOController*>(this)->stats_.pin_reads++;
    return level;
}

EmulatorError GPIOController::setPinInterrupt(uint8_t pin, InterruptMode mode) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    pin_states_[pin].interrupt_mode = mode;
    
    // Update interrupt enable register
    if (mode != InterruptMode::None) {
        interrupt_enable_ |= (1U << pin);
    } else {
        interrupt_enable_ &= ~(1U << pin);
    }
    
    COMPONENT_LOG_DEBUG("GPIO pin {} interrupt mode set to {}", pin, static_cast<int>(mode));
    return EmulatorError::Success;
}

GPIOController::InterruptMode GPIOController::getPinInterrupt(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return InterruptMode::None;
    }
    return pin_states_[pin].interrupt_mode;
}

EmulatorError GPIOController::setPWM(uint8_t pin, uint16_t duty_cycle) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    pin_states_[pin].pwm_duty = std::min(duty_cycle, static_cast<uint16_t>(1023));
    setPinMode(pin, PinMode::PWM);
    
    COMPONENT_LOG_DEBUG("GPIO pin {} PWM duty set to {}", pin, pin_states_[pin].pwm_duty);
    return EmulatorError::Success;
}

uint16_t GPIOController::getPWM(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return 0;
    }
    return pin_states_[pin].pwm_duty;
}

EmulatorError GPIOController::setAnalogValue(uint8_t pin, uint16_t value) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    pin_states_[pin].analog_value = std::min(value, static_cast<uint16_t>(4095));
    setPinMode(pin, PinMode::AnalogInput);
    
    COMPONENT_LOG_DEBUG("GPIO pin {} analog value set to {}", pin, pin_states_[pin].analog_value);
    return EmulatorError::Success;
}

uint16_t GPIOController::getAnalogValue(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return 0;
    }
    
    const_cast<GPIOController*>(this)->stats_.adc_reads++;
    return pin_states_[pin].analog_value;
}

EmulatorError GPIOController::simulatePinChange(uint8_t pin, bool new_level) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    bool previous_level = pin_states_[pin].level;
    pin_states_[pin].level = new_level;
    
    checkPinInterrupt(pin, previous_level, new_level);
    
    COMPONENT_LOG_DEBUG("Simulated pin {} change: {} -> {}", pin, 
                       previous_level ? "HIGH" : "LOW", 
                       new_level ? "HIGH" : "LOW");
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::simulateAnalogInput(uint8_t pin, uint16_t value) {
    return setAnalogValue(pin, value);
}

EmulatorError GPIOController::enablePullUp(uint8_t pin, bool enable) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    if (enable) {
        pullup_enable_ |= (1U << pin);
    } else {
        pullup_enable_ &= ~(1U << pin);
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::enablePullDown(uint8_t pin, bool enable) {
    if (pin >= GPIO_PIN_COUNT) {
        return EmulatorError::InvalidAddress;
    }
    
    if (enable) {
        pulldown_enable_ |= (1U << pin);
    } else {
        pulldown_enable_ &= ~(1U << pin);
    }
    
    return EmulatorError::Success;
}

bool GPIOController::isPullUpEnabled(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return false;
    }
    return (pullup_enable_ & (1U << pin)) != 0;
}

bool GPIOController::isPullDownEnabled(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return false;
    }
    return (pulldown_enable_ & (1U << pin)) != 0;
}

EmulatorError GPIOController::writePort(uint32_t mask, uint32_t value) {
    for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
        if (mask & (1U << pin)) {
            digitalWrite(pin, (value & (1U << pin)) != 0);
        }
    }
    return EmulatorError::Success;
}

uint32_t GPIOController::readPort() const {
    uint32_t value = 0;
    for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
        if (digitalRead(pin)) {
            value |= (1U << pin);
        }
    }
    return value;
}

std::string GPIOController::getPinStatusString(uint8_t pin) const {
    if (pin >= GPIO_PIN_COUNT) {
        return "Invalid pin";
    }
    
    const auto& state = pin_states_[pin];
    std::string mode_str;
    
    switch (state.mode) {
        case PinMode::Input: mode_str = "Input"; break;
        case PinMode::Output: mode_str = "Output"; break;
        case PinMode::InputPullUp: mode_str = "InputPullUp"; break;
        case PinMode::InputPullDown: mode_str = "InputPullDown"; break;
        case PinMode::OpenDrain: mode_str = "OpenDrain"; break;
        case PinMode::AnalogInput: mode_str = "AnalogInput"; break;
        case PinMode::PWM: mode_str = "PWM"; break;
    }
    
    return fmt::format("Pin {}: Mode={}, Level={}, PWM={}, Analog={}",
                      pin, mode_str, state.level ? "HIGH" : "LOW",
                      state.pwm_duty, state.analog_value);
}

std::vector<uint8_t> GPIOController::getConfiguredPins() const {
    std::vector<uint8_t> configured_pins;
    for (uint8_t pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        if (pin_states_[pin].mode != PinMode::Input || 
            pin_states_[pin].interrupt_mode != InterruptMode::None) {
            configured_pins.push_back(pin);
        }
    }
    return configured_pins;
}

// Private methods
void GPIOController::updatePinOutput(uint8_t pin) {
    if (pin >= GPIO_PIN_COUNT) return;
    
    const auto& state = pin_states_[pin];
    if (state.mode == PinMode::PWM) {
        // Simple PWM simulation - could be more sophisticated
        static uint16_t pwm_counter = 0;
        bool pwm_output = (pwm_counter % 1024) < state.pwm_duty;
        
        // Update the actual pin level based on PWM
        const_cast<PinState&>(state).level = pwm_output;
        pwm_counter++;
    }
}

void GPIOController::checkPinInterrupt(uint8_t pin, bool old_level, bool new_level) {
    if (pin >= GPIO_PIN_COUNT) return;
    
    const auto& state = pin_states_[pin];
    if (state.interrupt_mode == InterruptMode::None) return;
    
    bool trigger = false;
    
    switch (state.interrupt_mode) {
        case InterruptMode::Rising:
            trigger = !old_level && new_level;
            break;
        case InterruptMode::Falling:
            trigger = old_level && !new_level;
            break;
        case InterruptMode::Both:
            trigger = old_level != new_level;
            break;
        case InterruptMode::Low:
            trigger = !new_level;
            break;
        case InterruptMode::High:
            trigger = new_level;
            break;
        case InterruptMode::None:
            break;
    }
    
    if (trigger) {
        triggerPinInterrupt(pin);
    }
}

void GPIOController::triggerPinInterrupt(uint8_t pin) {
    if (pin >= GPIO_PIN_COUNT) return;
    
    // Set interrupt status
    interrupt_status_ |= (1U << pin);
    pin_states_[pin].interrupt_pending = true;
    
    stats_.interrupts_generated++;
    
    COMPONENT_LOG_TRACE("GPIO interrupt triggered on pin {}", pin);
}

EmulatorError GPIOController::handleDirectionRegister(uint32_t value) {
    direction_register_ = value;
    
    // Update pin modes based on direction
    for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
        if (value & (1U << pin)) {
            // Output
            if (pin_states_[pin].mode == PinMode::Input ||
                pin_states_[pin].mode == PinMode::InputPullUp ||
                pin_states_[pin].mode == PinMode::InputPullDown) {
                setPinMode(pin, PinMode::Output);
            }
        } else {
            // Input
            if (pin_states_[pin].mode == PinMode::Output ||
                pin_states_[pin].mode == PinMode::OpenDrain ||
                pin_states_[pin].mode == PinMode::PWM) {
                setPinMode(pin, PinMode::Input);
            }
        }
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::handleOutputRegister(uint32_t value) {
    output_register_ = value;
    
    // Update pin levels
    for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
        bool level = (value & (1U << pin)) != 0;
        bool previous_level = pin_states_[pin].level;
        pin_states_[pin].level = level;
        
        checkPinInterrupt(pin, previous_level, level);
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::handleInterruptEnable(uint32_t value) {
    interrupt_enable_ = value;
    
    // Update interrupt modes for pins
    for (uint8_t pin = 0; pin < 32 && pin < GPIO_PIN_COUNT; ++pin) {
        if (!(value & (1U << pin))) {
            pin_states_[pin].interrupt_mode = InterruptMode::None;
        }
    }
    
    return EmulatorError::Success;
}

EmulatorError GPIOController::handleInterruptType(uint32_t value) {
    interrupt_type_ = value;
    // Implementation depends on specific interrupt type encoding
    // For now, just store the value
    return EmulatorError::Success;
}

} // namespace m5tab5::emulator