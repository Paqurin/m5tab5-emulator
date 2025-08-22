#pragma once

#include "emulator/peripherals/peripheral_base.hpp"
#include <array>
#include <functional>

namespace m5tab5::emulator {

/**
 * @brief GPIO controller emulating ESP32-P4's 55 GPIO pins
 * 
 * Features:
 * - 55 configurable GPIO pins
 * - Input/output direction control
 * - Pull-up/pull-down resistors
 * - Interrupt generation on pin changes
 * - PWM output simulation
 * - Analog input simulation (ADC)
 */
class GPIOController : public PeripheralBase {
public:
    enum class PinMode {
        Input,
        Output,
        InputPullUp,
        InputPullDown,
        OpenDrain,
        AnalogInput,
        PWM
    };

    enum class InterruptMode {
        None,
        Rising,
        Falling,
        Both,
        Low,
        High
    };

    struct PinState {
        PinMode mode = PinMode::Input;
        bool level = false;
        InterruptMode interrupt_mode = InterruptMode::None;
        uint16_t pwm_duty = 0;      // 0-1023 for PWM
        uint16_t analog_value = 0;   // 0-4095 for ADC
        bool interrupt_pending = false;
    };

    // GPIO registers
    enum class Register : Address {
        GPIO_OUT = 0x40001000,          // GPIO output register
        GPIO_OUT_SET = 0x40001004,      // Set output bits
        GPIO_OUT_CLR = 0x40001008,      // Clear output bits
        GPIO_IN = 0x4000100C,           // GPIO input register
        GPIO_DIR = 0x40001010,          // Direction register
        GPIO_INT_EN = 0x40001014,       // Interrupt enable
        GPIO_INT_TYPE = 0x40001018,     // Interrupt type
        GPIO_INT_STAT = 0x4000101C,     // Interrupt status
        GPIO_INT_CLR = 0x40001020,      // Interrupt clear
        GPIO_PULLUP_EN = 0x40001024,    // Pull-up enable
        GPIO_PULLDOWN_EN = 0x40001028,  // Pull-down enable
        GPIO_PWM_CTRL = 0x4000102C,     // PWM control
        GPIO_ADC_CTRL = 0x40001030,     // ADC control
        GPIO_PIN_CTRL = 0x40001034      // Individual pin control
    };

    explicit GPIOController();
    ~GPIOController();

    // PeripheralBase implementation
    std::string getName() const override { return "gpio"; }
    EmulatorError initialize() override;
    EmulatorError reset() override;
    EmulatorError tick(ClockCycle cycle) override;
    
    EmulatorError readRegister(Address address, uint32_t& value) override;
    EmulatorError writeRegister(Address address, uint32_t value) override;
    
    std::vector<Address> getRegisterAddresses() const override;
    std::vector<uint32_t> getInterruptIds() const override;

    // GPIO operations
    EmulatorError setPinMode(uint8_t pin, PinMode mode);
    PinMode getPinMode(uint8_t pin) const;
    
    EmulatorError digitalWrite(uint8_t pin, bool level);
    bool digitalRead(uint8_t pin) const;
    
    EmulatorError setPinInterrupt(uint8_t pin, InterruptMode mode);
    InterruptMode getPinInterrupt(uint8_t pin) const;

    // PWM operations
    EmulatorError setPWM(uint8_t pin, uint16_t duty_cycle); // 0-1023
    uint16_t getPWM(uint8_t pin) const;

    // ADC operations
    EmulatorError setAnalogValue(uint8_t pin, uint16_t value); // 0-4095
    uint16_t getAnalogValue(uint8_t pin) const;

    // External pin simulation (for testing/automation)
    EmulatorError simulatePinChange(uint8_t pin, bool new_level);
    EmulatorError simulateAnalogInput(uint8_t pin, uint16_t value);

    // Pin configuration
    EmulatorError enablePullUp(uint8_t pin, bool enable);
    EmulatorError enablePullDown(uint8_t pin, bool enable);
    bool isPullUpEnabled(uint8_t pin) const;
    bool isPullDownEnabled(uint8_t pin) const;

    // Mass operations
    EmulatorError writePort(uint32_t mask, uint32_t value);
    uint32_t readPort() const;

    // Debug and inspection
    std::string getPinStatusString(uint8_t pin) const;
    std::vector<uint8_t> getConfiguredPins() const;

private:
    // Internal pin management
    void updatePinOutput(uint8_t pin);
    void checkPinInterrupt(uint8_t pin, bool old_level, bool new_level);
    void triggerPinInterrupt(uint8_t pin);

    // Register handling
    EmulatorError handleDirectionRegister(uint32_t value);
    EmulatorError handleOutputRegister(uint32_t value);
    EmulatorError handleInterruptEnable(uint32_t value);
    EmulatorError handleInterruptType(uint32_t value);

    // Pin state storage
    std::array<PinState, GPIO_PIN_COUNT> pin_states_;

    // Register shadows
    uint32_t direction_register_ = 0;
    uint32_t output_register_ = 0;
    uint32_t interrupt_enable_ = 0;
    uint32_t interrupt_type_ = 0;
    uint32_t interrupt_status_ = 0;
    uint32_t pullup_enable_ = 0;
    uint32_t pulldown_enable_ = 0;

    // PWM simulation
    ClockCycle last_pwm_update_ = 0;
    static constexpr uint32_t PWM_FREQUENCY = 1000; // 1kHz PWM

    // Statistics
    struct GPIOStatistics {
        uint64_t pin_reads = 0;
        uint64_t pin_writes = 0;
        uint64_t interrupts_generated = 0;
        uint64_t pwm_updates = 0;
        uint64_t adc_reads = 0;
    };

    GPIOStatistics stats_;

    // Interrupt ID
    static constexpr uint32_t GPIO_INTERRUPT_ID = 8;
};

} // namespace m5tab5::emulator