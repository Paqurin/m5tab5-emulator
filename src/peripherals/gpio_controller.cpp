#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("GpioController");

GpioController::GpioController()
    : initialized_(false),
      interrupt_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("GpioController created");
}

GpioController::~GpioController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("GpioController destroyed");
}

Result<void> GpioController::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "GPIO controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing GPIO controller with {} pins", GPIO_PIN_COUNT);
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize all GPIO pins
    for (u8 pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        pins_[pin] = std::make_unique<GpioPin>(pin);
        pins_[pin]->reset();
    }
    
    // Set up default pin configurations for ESP32-P4
    RETURN_IF_ERROR(setup_default_pin_functions());
    
    // Initialize MMIO registers
    RETURN_IF_ERROR(setup_registers());
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("GPIO controller initialized successfully");
    
    return {};
}

Result<void> GpioController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down GPIO controller");
    
    // Reset all pins
    for (auto& pin : pins_) {
        if (pin) {
            pin->reset();
            pin.reset();
        }
    }
    
    // Clear registers
    registers_.clear();
    interrupt_pins_.clear();
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("GPIO controller shutdown completed");
    return {};
}

Result<void> GpioController::configure_pin(u8 pin_number, const GpioPinConfig& config) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    auto& pin = pins_[pin_number];
    
    // Validate configuration
    RETURN_IF_ERROR(validate_pin_config(pin_number, config));
    
    // Apply configuration
    pin->set_mode(config.mode);
    pin->set_pull_mode(config.pull_mode);
    pin->set_drive_strength(config.drive_strength);
    pin->set_slew_rate(config.slew_rate);
    pin->set_function(config.function);
    
    // Configure interrupt if specified
    if (config.interrupt_type != GpioInterruptType::NONE) {
        pin->set_interrupt_type(config.interrupt_type);
        pin->enable_interrupt(true);
        
        // Add to interrupt monitoring list
        if (std::find(interrupt_pins_.begin(), interrupt_pins_.end(), pin_number) == interrupt_pins_.end()) {
            interrupt_pins_.push_back(pin_number);
        }
    } else {
        pin->enable_interrupt(false);
        
        // Remove from interrupt monitoring list
        interrupt_pins_.erase(
            std::remove(interrupt_pins_.begin(), interrupt_pins_.end(), pin_number),
            interrupt_pins_.end());
    }
    
    COMPONENT_LOG_DEBUG("Configured GPIO pin {}: mode={} pull={} function={}",
                       pin_number, static_cast<int>(config.mode),
                       static_cast<int>(config.pull_mode), static_cast<int>(config.function));
    
    return {};
}

Result<void> GpioController::set_pin_level(u8 pin_number, bool high) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    auto& pin = pins_[pin_number];
    
    // Check if pin is configured as output
    if (pin->get_mode() != GpioMode::OUTPUT && pin->get_mode() != GpioMode::OUTPUT_OPEN_DRAIN) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Pin " + std::to_string(pin_number) + " is not configured as output"));
    }
    
    bool previous_level = pin->get_level();
    pin->set_level(high);
    
    // Check for interrupt conditions on connected pins
    RETURN_IF_ERROR(check_pin_interrupts(pin_number, previous_level, high));
    
    statistics_.pin_writes++;
    
    COMPONENT_LOG_TRACE("GPIO pin {} set to {}", pin_number, high ? "HIGH" : "LOW");
    
    return {};
}

Result<bool> GpioController::get_pin_level(u8 pin_number) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    const auto& pin = pins_[pin_number];
    bool level = pin->get_level();
    
    // Apply pull-up/pull-down for input pins
    if (pin->get_mode() == GpioMode::INPUT || 
        pin->get_mode() == GpioMode::INPUT_PULLUP || 
        pin->get_mode() == GpioMode::INPUT_PULLDOWN) {
        
        switch (pin->get_pull_mode()) {
            case GpioPullMode::PULLUP:
                if (!pin->is_externally_driven()) {
                    level = true;
                }
                break;
            case GpioPullMode::PULLDOWN:
                if (!pin->is_externally_driven()) {
                    level = false;
                }
                break;
            case GpioPullMode::NONE:
                // Floating input - return current level
                break;
        }
    }
    
    statistics_.pin_reads++;
    return level;
}

Result<void> GpioController::set_pin_function(u8 pin_number, GpioPinFunction function) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    // Validate function for this pin
    if (!is_function_valid_for_pin(pin_number, function)) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Function not available for pin " + std::to_string(pin_number)));
    }
    
    auto& pin = pins_[pin_number];
    pin->set_function(function);
    
    COMPONENT_LOG_DEBUG("GPIO pin {} function set to {}", pin_number, static_cast<int>(function));
    
    return {};
}

Result<GpioPinFunction> GpioController::get_pin_function(u8 pin_number) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    return pins_[pin_number]->get_function();
}

Result<void> GpioController::enable_pin_interrupt(u8 pin_number, GpioInterruptType type) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    auto& pin = pins_[pin_number];
    pin->set_interrupt_type(type);
    pin->enable_interrupt(true);
    
    // Add to interrupt monitoring list
    if (std::find(interrupt_pins_.begin(), interrupt_pins_.end(), pin_number) == interrupt_pins_.end()) {
        interrupt_pins_.push_back(pin_number);
    }
    
    COMPONENT_LOG_DEBUG("Enabled interrupt on GPIO pin {} (type={})",
                       pin_number, static_cast<int>(type));
    
    return {};
}

Result<void> GpioController::disable_pin_interrupt(u8 pin_number) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    if (pin_number >= GPIO_PIN_COUNT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid GPIO pin number: " + std::to_string(pin_number)));
    }
    
    auto& pin = pins_[pin_number];
    pin->enable_interrupt(false);
    
    // Remove from interrupt monitoring list
    interrupt_pins_.erase(
        std::remove(interrupt_pins_.begin(), interrupt_pins_.end(), pin_number),
        interrupt_pins_.end());
    
    COMPONENT_LOG_DEBUG("Disabled interrupt on GPIO pin {}", pin_number);
    
    return {};
}

Result<void> GpioController::update() {
    if (!initialized_) {
        return {};
    }
    
    // Process any pending interrupt conditions
    for (u8 pin_number : interrupt_pins_) {
        auto& pin = pins_[pin_number];
        if (pin->has_pending_interrupt()) {
            RETURN_IF_ERROR(trigger_pin_interrupt(pin_number));
            pin->clear_pending_interrupt();
            statistics_.interrupts_generated++;
        }
    }
    
    return {};
}

const GpioStatistics& GpioController::get_statistics() const {
    return statistics_;
}

void GpioController::clear_statistics() {
    statistics_ = {};
}

Result<u32> GpioController::handle_mmio_read(Address address) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    u32 offset = address - GPIO_CONTROLLER_BASE_ADDR;
    
    switch (offset) {
        case GPIO_REG_INPUT_STATUS: {
            u32 status = 0;
            for (u8 pin = 0; pin < std::min(static_cast<u8>(32), GPIO_PIN_COUNT); ++pin) {
                auto level_result = get_pin_level(pin);
                if (level_result && level_result.value()) {
                    status |= (1U << pin);
                }
            }
            return status;
        }
        
        case GPIO_REG_INTERRUPT_STATUS: {
            u32 status = 0;
            for (u8 pin = 0; pin < std::min(static_cast<u8>(32), GPIO_PIN_COUNT); ++pin) {
                if (pins_[pin]->has_pending_interrupt()) {
                    status |= (1U << pin);
                }
            }
            return status;
        }
        
        default:
            return read_register(offset);
    }
}

Result<void> GpioController::handle_mmio_write(Address address, u32 value) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "GPIO controller not initialized"));
    }
    
    u32 offset = address - GPIO_CONTROLLER_BASE_ADDR;
    
    switch (offset) {
        case GPIO_REG_OUTPUT_SET: {
            // Set specified pins high
            for (u8 pin = 0; pin < std::min(static_cast<u8>(32), GPIO_PIN_COUNT); ++pin) {
                if (value & (1U << pin)) {
                    auto result = set_pin_level(pin, true);
                    if (!result) {
                        COMPONENT_LOG_WARN("Failed to set GPIO pin {}: {}", 
                                          pin, result.error().to_string());
                    }
                }
            }
            return {};
        }
        
        case GPIO_REG_OUTPUT_CLEAR: {
            // Set specified pins low
            for (u8 pin = 0; pin < std::min(static_cast<u8>(32), GPIO_PIN_COUNT); ++pin) {
                if (value & (1U << pin)) {
                    auto result = set_pin_level(pin, false);
                    if (!result) {
                        COMPONENT_LOG_WARN("Failed to clear GPIO pin {}: {}", 
                                          pin, result.error().to_string());
                    }
                }
            }
            return {};
        }
        
        case GPIO_REG_INTERRUPT_CLEAR: {
            // Clear interrupt flags for specified pins
            for (u8 pin = 0; pin < std::min(static_cast<u8>(32), GPIO_PIN_COUNT); ++pin) {
                if (value & (1U << pin)) {
                    pins_[pin]->clear_pending_interrupt();
                }
            }
            return {};
        }
        
        default:
            write_register(offset, value);
            return {};
    }
}

Result<void> GpioController::setup_default_pin_functions() {
    COMPONENT_LOG_DEBUG("Setting up default GPIO pin functions for ESP32-P4");
    
    // ESP32-P4 specific pin configurations
    // GPIO0-15: Touch sensing capable
    for (u8 pin = 0; pin <= 15; ++pin) {
        pins_[pin]->add_available_function(GpioPinFunction::GPIO);
        pins_[pin]->add_available_function(GpioPinFunction::TOUCH);
    }
    
    // GPIO16-23: ADC1 channels
    for (u8 pin = 16; pin <= 23; ++pin) {
        pins_[pin]->add_available_function(GpioPinFunction::GPIO);
        pins_[pin]->add_available_function(GpioPinFunction::ADC);
    }
    
    // Common peripheral pins
    for (u8 pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        pins_[pin]->add_available_function(GpioPinFunction::GPIO);
        
        // Most pins can be used for SPI/I2C/UART
        if (pin < 50) { // Reserve some pins for special functions
            pins_[pin]->add_available_function(GpioPinFunction::SPI);
            pins_[pin]->add_available_function(GpioPinFunction::I2C);
            pins_[pin]->add_available_function(GpioPinFunction::UART);
            pins_[pin]->add_available_function(GpioPinFunction::PWM);
        }
    }
    
    // USB pins (GPIO24-25)
    pins_[24]->add_available_function(GpioPinFunction::USB);
    pins_[25]->add_available_function(GpioPinFunction::USB);
    
    // MIPI CSI pins (camera interface)
    for (u8 pin = 40; pin <= 47; ++pin) {
        if (pin < GPIO_PIN_COUNT) {
            pins_[pin]->add_available_function(GpioPinFunction::MIPI_CSI);
        }
    }
    
    // MIPI DSI pins (display interface)
    for (u8 pin = 48; pin <= 54; ++pin) {
        if (pin < GPIO_PIN_COUNT) {
            pins_[pin]->add_available_function(GpioPinFunction::MIPI_DSI);
        }
    }
    
    return {};
}

Result<void> GpioController::setup_registers() {
    // Initialize GPIO control registers
    registers_[GPIO_REG_INPUT_STATUS] = 0;
    registers_[GPIO_REG_OUTPUT_SET] = 0;
    registers_[GPIO_REG_OUTPUT_CLEAR] = 0;
    registers_[GPIO_REG_INTERRUPT_STATUS] = 0;
    registers_[GPIO_REG_INTERRUPT_CLEAR] = 0;
    registers_[GPIO_REG_INTERRUPT_ENABLE] = 0;
    
    // Pin configuration registers (one per pin)
    for (u8 pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        u32 config_reg = GPIO_REG_PIN_CONFIG_BASE + (pin * 4);
        registers_[config_reg] = 0; // Default configuration
    }
    
    return {};
}

Result<void> GpioController::validate_pin_config(u8 pin_number, const GpioPinConfig& config) const {
    // Check if the requested function is available for this pin
    if (!is_function_valid_for_pin(pin_number, config.function)) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Function not available for pin " + std::to_string(pin_number)));
    }
    
    // Validate mode and pull configuration
    if (config.mode == GpioMode::INPUT_PULLUP && config.pull_mode != GpioPullMode::PULLUP) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "INPUT_PULLUP mode requires PULLUP pull mode"));
    }
    
    if (config.mode == GpioMode::INPUT_PULLDOWN && config.pull_mode != GpioPullMode::PULLDOWN) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "INPUT_PULLDOWN mode requires PULLDOWN pull mode"));
    }
    
    return {};
}

bool GpioController::is_function_valid_for_pin(u8 pin_number, GpioPinFunction function) const {
    if (pin_number >= GPIO_PIN_COUNT) {
        return false;
    }
    
    return pins_[pin_number]->is_function_available(function);
}

Result<void> GpioController::check_pin_interrupts(u8 pin_number, bool previous_level, bool current_level) {
    auto& pin = pins_[pin_number];
    
    if (!pin->is_interrupt_enabled()) {
        return {};
    }
    
    bool trigger_interrupt = false;
    
    switch (pin->get_interrupt_type()) {
        case GpioInterruptType::RISING_EDGE:
            trigger_interrupt = (!previous_level && current_level);
            break;
            
        case GpioInterruptType::FALLING_EDGE:
            trigger_interrupt = (previous_level && !current_level);
            break;
            
        case GpioInterruptType::BOTH_EDGES:
            trigger_interrupt = (previous_level != current_level);
            break;
            
        case GpioInterruptType::LOW_LEVEL:
            trigger_interrupt = !current_level;
            break;
            
        case GpioInterruptType::HIGH_LEVEL:
            trigger_interrupt = current_level;
            break;
            
        default:
            break;
    }
    
    if (trigger_interrupt) {
        pin->set_pending_interrupt(true);
    }
    
    return {};
}

Result<void> GpioController::trigger_pin_interrupt(u8 pin_number) {
    if (interrupt_controller_) {
        auto result = interrupt_controller_->trigger_interrupt(
            CoreId::CORE_0, InterruptType::GPIO, pin_number);
        if (!result) {
            return std::unexpected(result.error());
        }
        
        COMPONENT_LOG_TRACE("Triggered GPIO interrupt for pin {}", pin_number);
    }
    
    return {};
}

void GpioController::write_register(u32 offset, u32 value) {
    registers_[offset] = value;
}

u32 GpioController::read_register(u32 offset) const {
    auto it = registers_.find(offset);
    return (it != registers_.end()) ? it->second : 0;
}

void GpioController::dump_pin_status() const {
    COMPONENT_LOG_INFO("=== GPIO Controller Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Pin reads: {}", statistics_.pin_reads);
        COMPONENT_LOG_INFO("  Pin writes: {}", statistics_.pin_writes);
        COMPONENT_LOG_INFO("  Interrupts generated: {}", statistics_.interrupts_generated);
        
        COMPONENT_LOG_INFO("Pin Configuration Summary:");
        for (u8 pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
            const auto& gpio_pin = pins_[pin];
            if (gpio_pin->get_function() != GpioPinFunction::GPIO || 
                gpio_pin->get_mode() != GpioMode::INPUT) {
                
                COMPONENT_LOG_INFO("  Pin {}: mode={} function={} level={} interrupt={}",
                                  pin,
                                  static_cast<int>(gpio_pin->get_mode()),
                                  static_cast<int>(gpio_pin->get_function()),
                                  gpio_pin->get_level() ? "HIGH" : "LOW",
                                  gpio_pin->is_interrupt_enabled() ? "EN" : "DIS");
            }
        }
        
        if (!interrupt_pins_.empty()) {
            COMPONENT_LOG_INFO("Interrupt-enabled pins: [{}]",
                              [this]() {
                                  std::string pins;
                                  for (size_t i = 0; i < interrupt_pins_.size(); ++i) {
                                      if (i > 0) pins += ", ";
                                      pins += std::to_string(interrupt_pins_[i]);
                                  }
                                  return pins;
                              }());
        }
    }
}

// GpioPin implementation

GpioPin::GpioPin(u8 pin_number)
    : pin_number_(pin_number),
      mode_(GpioMode::INPUT),
      pull_mode_(GpioPullMode::NONE),
      drive_strength_(GpioDriveStrength::MEDIUM),
      slew_rate_(GpioSlewRate::MEDIUM),
      function_(GpioPinFunction::GPIO),
      level_(false),
      externally_driven_(false),
      interrupt_enabled_(false),
      interrupt_type_(GpioInterruptType::NONE),
      pending_interrupt_(false) {
}

void GpioPin::reset() {
    mode_ = GpioMode::INPUT;
    pull_mode_ = GpioPullMode::NONE;
    drive_strength_ = GpioDriveStrength::MEDIUM;
    slew_rate_ = GpioSlewRate::MEDIUM;
    function_ = GpioPinFunction::GPIO;
    level_ = false;
    externally_driven_ = false;
    interrupt_enabled_ = false;
    interrupt_type_ = GpioInterruptType::NONE;
    pending_interrupt_ = false;
    available_functions_.clear();
    available_functions_.insert(GpioPinFunction::GPIO);
}

void GpioPin::add_available_function(GpioPinFunction function) {
    available_functions_.insert(function);
}

bool GpioPin::is_function_available(GpioPinFunction function) const {
    return available_functions_.find(function) != available_functions_.end();
}

}  // namespace m5tab5::emulator