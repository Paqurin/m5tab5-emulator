/**
 * @file gpio_api.cpp
 * @brief ESP-IDF GPIO driver API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible GPIO functions that map to the
 * emulated GPIO controller, providing seamless compatibility for ESP-IDF
 * applications running on the emulator.
 */

#include "emulator/esp_idf/driver/gpio.h"
#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"

namespace {
    using namespace m5tab5::emulator;
    using GPIOController = peripherals::GPIOController;
    
    /**
     * @brief Get GPIO controller instance from emulator core
     */
    GPIOController* get_gpio_controller() {
        static GPIOController* gpio_controller = nullptr;
        
        if (!gpio_controller) {
            // Get emulator core instance (this would be set up by the emulator)
            // For now, we'll use a placeholder - this needs integration with EmulatorCore
            LOG_DEBUG("Getting GPIO controller instance from EmulatorCore");
            
            // TODO: Implement proper EmulatorCore integration
            // auto emulator = EmulatorCore::get_instance();
            // if (emulator) {
            //     auto result = emulator->get_component<GPIOController>();
            //     if (result.has_value()) {
            //         gpio_controller = result.value();
            //     }
            // }
        }
        
        return gpio_controller;
    }
    
    /**
     * @brief Convert ESP-IDF GPIO mode to emulator GPIO mode
     */
    GPIOController::Mode convert_gpio_mode(gpio_mode_t esp_mode) {
        switch (esp_mode) {
            case GPIO_MODE_INPUT:
                return GPIOController::Mode::INPUT;
            case GPIO_MODE_OUTPUT:
                return GPIOController::Mode::OUTPUT;
            case GPIO_MODE_OUTPUT_OD:
                return GPIOController::Mode::OUTPUT_OPEN_DRAIN;
            case GPIO_MODE_INPUT_OUTPUT:
                return GPIOController::Mode::INPUT_OUTPUT;
            case GPIO_MODE_INPUT_OUTPUT_OD:
                return GPIOController::Mode::INPUT_OUTPUT_OPEN_DRAIN;
            case GPIO_MODE_DISABLE:
            default:
                return GPIOController::Mode::DISABLED;
        }
    }
    
    /**
     * @brief Convert ESP-IDF interrupt type to emulator interrupt type
     */
    GPIOController::InterruptType convert_interrupt_type(gpio_int_type_t esp_intr) {
        switch (esp_intr) {
            case GPIO_INTR_POSEDGE:
                return GPIOController::InterruptType::RISING_EDGE;
            case GPIO_INTR_NEGEDGE:
                return GPIOController::InterruptType::FALLING_EDGE;
            case GPIO_INTR_ANYEDGE:
                return GPIOController::InterruptType::ANY_EDGE;
            case GPIO_INTR_LOW_LEVEL:
                return GPIOController::InterruptType::LOW_LEVEL;
            case GPIO_INTR_HIGH_LEVEL:
                return GPIOController::InterruptType::HIGH_LEVEL;
            case GPIO_INTR_DISABLE:
            default:
                return GPIOController::InterruptType::NONE;
        }
    }
    
    /**
     * @brief Validate GPIO pin number
     */
    bool is_valid_gpio(gpio_num_t gpio_num) {
        return (gpio_num >= 0 && gpio_num <= GPIO_NUM_MAX);
    }
}

// ============================================================================
// ESP-IDF GPIO API Implementation
// ============================================================================

extern "C" {

int gpio_config(const gpio_config_t* pGPIOConfig) {
    if (!pGPIOConfig) {
        LOG_ERROR("gpio_config: null configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_config: GPIO controller not available");
        return ESP_FAIL;
    }
    
    LOG_DEBUG("gpio_config: pin_bit_mask=0x{:016x}, mode={}, pull_up={}, pull_down={}, intr_type={}", 
              pGPIOConfig->pin_bit_mask, 
              static_cast<int>(pGPIOConfig->mode),
              static_cast<int>(pGPIOConfig->pull_up_en),
              static_cast<int>(pGPIOConfig->pull_down_en),
              static_cast<int>(pGPIOConfig->intr_type));
    
    // Configure each GPIO pin in the bit mask
    for (int pin = 0; pin <= GPIO_NUM_MAX; pin++) {
        if (pGPIOConfig->pin_bit_mask & (1ULL << pin)) {
            if (!is_valid_gpio(pin)) {
                LOG_ERROR("gpio_config: invalid GPIO pin {}", pin);
                return ESP_ERR_INVALID_ARG;
            }
            
            // Set pin mode
            auto mode = convert_gpio_mode(pGPIOConfig->mode);
            auto result = controller->configure_pin(pin, mode);
            if (!result.has_value()) {
                LOG_ERROR("gpio_config: failed to configure pin {}", pin);
                return ESP_FAIL;
            }
            
            // Configure pull resistors
            if (pGPIOConfig->pull_up_en == GPIO_PULLUP_ENABLE) {
                controller->set_pull_mode(pin, GPIOController::PullMode::PULL_UP);
            } else if (pGPIOConfig->pull_down_en == GPIO_PULLDOWN_ENABLE) {
                controller->set_pull_mode(pin, GPIOController::PullMode::PULL_DOWN);
            } else {
                controller->set_pull_mode(pin, GPIOController::PullMode::FLOATING);
            }
            
            // Configure interrupt if needed
            if (pGPIOConfig->intr_type != GPIO_INTR_DISABLE) {
                auto intr_type = convert_interrupt_type(pGPIOConfig->intr_type);
                controller->set_interrupt_type(pin, intr_type);
            }
        }
    }
    
    return ESP_OK;
}

int gpio_reset_pin(gpio_num_t gpio_num) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_reset_pin: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_reset_pin: GPIO controller not available");
        return ESP_FAIL;
    }
    
    LOG_DEBUG("gpio_reset_pin: resetting GPIO {}", gpio_num);
    
    // Reset pin to default state (input, no pull, no interrupt)
    auto result = controller->configure_pin(gpio_num, GPIOController::Mode::INPUT);
    if (!result.has_value()) {
        LOG_ERROR("gpio_reset_pin: failed to reset pin {}", gpio_num);
        return ESP_FAIL;
    }
    
    controller->set_pull_mode(gpio_num, GPIOController::PullMode::FLOATING);
    controller->set_interrupt_type(gpio_num, GPIOController::InterruptType::NONE);
    
    return ESP_OK;
}

int gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_set_level: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_set_level: GPIO controller not available");
        return ESP_FAIL;
    }
    
    LOG_DEBUG("gpio_set_level: GPIO {} = {}", gpio_num, level ? "HIGH" : "LOW");
    
    auto result = controller->digital_write(gpio_num, level != 0);
    if (!result.has_value()) {
        LOG_ERROR("gpio_set_level: failed to set GPIO {} level", gpio_num);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_get_level: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_get_level: GPIO controller not available");
        return ESP_FAIL;
    }
    
    auto result = controller->digital_read(gpio_num);
    if (!result.has_value()) {
        LOG_ERROR("gpio_get_level: failed to read GPIO {} level", gpio_num);
        return ESP_FAIL;
    }
    
    bool level = result.value();
    LOG_DEBUG("gpio_get_level: GPIO {} = {}", gpio_num, level ? "HIGH" : "LOW");
    
    return level ? 1 : 0;
}

int gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_set_direction: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_set_direction: GPIO controller not available");
        return ESP_FAIL;
    }
    
    LOG_DEBUG("gpio_set_direction: GPIO {} mode {}", gpio_num, static_cast<int>(mode));
    
    auto gpio_mode = convert_gpio_mode(mode);
    auto result = controller->configure_pin(gpio_num, gpio_mode);
    if (!result.has_value()) {
        LOG_ERROR("gpio_set_direction: failed to set GPIO {} direction", gpio_num);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

int gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_set_pull_mode: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_set_pull_mode: GPIO controller not available");
        return ESP_FAIL;
    }
    
    LOG_DEBUG("gpio_set_pull_mode: GPIO {} pull mode {}", gpio_num, static_cast<int>(pull));
    
    GPIOController::PullMode pull_mode;
    switch (pull) {
        case GPIO_PULLUP_ONLY:
            pull_mode = GPIOController::PullMode::PULL_UP;
            break;
        case GPIO_PULLDOWN_ONLY:
            pull_mode = GPIOController::PullMode::PULL_DOWN;
            break;
        case GPIO_PULLUP_PULLDOWN:
            // Note: Most hardware doesn't support simultaneous pull-up/down
            // Use pull-up as default
            pull_mode = GPIOController::PullMode::PULL_UP;
            LOG_WARN("gpio_set_pull_mode: simultaneous pull-up/down not supported, using pull-up");
            break;
        case GPIO_FLOATING:
        default:
            pull_mode = GPIOController::PullMode::FLOATING;
            break;
    }
    
    controller->set_pull_mode(gpio_num, pull_mode);
    return ESP_OK;
}

int gpio_pullup_en(gpio_num_t gpio_num) {
    return gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
}

int gpio_pullup_dis(gpio_num_t gpio_num) {
    return gpio_set_pull_mode(gpio_num, GPIO_FLOATING);
}

int gpio_pulldown_en(gpio_num_t gpio_num) {
    return gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
}

int gpio_pulldown_dis(gpio_num_t gpio_num) {
    return gpio_set_pull_mode(gpio_num, GPIO_FLOATING);
}

// ============================================================================
// GPIO Interrupt Functions (Simplified Implementation)
// ============================================================================

int gpio_install_isr_service(int intr_alloc_flags) {
    LOG_DEBUG("gpio_install_isr_service: installing GPIO ISR service");
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_install_isr_service: GPIO controller not available");
        return ESP_FAIL;
    }
    
    // TODO: Implement interrupt service installation
    // For now, just log that ISR service is "installed"
    LOG_INFO("gpio_install_isr_service: ISR service installed (emulated)");
    return ESP_OK;
}

void gpio_uninstall_isr_service(void) {
    LOG_DEBUG("gpio_uninstall_isr_service: uninstalling GPIO ISR service");
    // TODO: Implement ISR service cleanup
    LOG_INFO("gpio_uninstall_isr_service: ISR service uninstalled (emulated)");
}

int gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void* args) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_isr_handler_add: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!isr_handler) {
        LOG_ERROR("gpio_isr_handler_add: null ISR handler");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("gpio_isr_handler_add: adding ISR handler for GPIO {}", gpio_num);
    
    // TODO: Implement ISR handler registration with GPIO controller
    // This would need to integrate with the interrupt controller
    LOG_INFO("gpio_isr_handler_add: ISR handler added for GPIO {} (emulated)", gpio_num);
    return ESP_OK;
}

int gpio_isr_handler_remove(gpio_num_t gpio_num) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_isr_handler_remove: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("gpio_isr_handler_remove: removing ISR handler for GPIO {}", gpio_num);
    
    // TODO: Implement ISR handler removal
    LOG_INFO("gpio_isr_handler_remove: ISR handler removed for GPIO {} (emulated)", gpio_num);
    return ESP_OK;
}

int gpio_intr_enable(gpio_num_t gpio_num) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_intr_enable: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("gpio_intr_enable: enabling interrupt for GPIO {}", gpio_num);
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_intr_enable: GPIO controller not available");
        return ESP_FAIL;
    }
    
    // TODO: Enable interrupt for the specific pin
    // This requires integration with interrupt controller
    LOG_INFO("gpio_intr_enable: interrupt enabled for GPIO {} (emulated)", gpio_num);
    return ESP_OK;
}

int gpio_intr_disable(gpio_num_t gpio_num) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_intr_disable: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("gpio_intr_disable: disabling interrupt for GPIO {}", gpio_num);
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_intr_disable: GPIO controller not available");
        return ESP_FAIL;
    }
    
    controller->set_interrupt_type(gpio_num, GPIOController::InterruptType::NONE);
    LOG_INFO("gpio_intr_disable: interrupt disabled for GPIO {}", gpio_num);
    return ESP_OK;
}

int gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t intr_type) {
    if (!is_valid_gpio(gpio_num)) {
        LOG_ERROR("gpio_set_intr_type: invalid GPIO pin {}", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("gpio_set_intr_type: setting interrupt type {} for GPIO {}", 
              static_cast<int>(intr_type), gpio_num);
    
    GPIOController* controller = get_gpio_controller();
    if (!controller) {
        LOG_ERROR("gpio_set_intr_type: GPIO controller not available");
        return ESP_FAIL;
    }
    
    auto interrupt_type = convert_interrupt_type(intr_type);
    controller->set_interrupt_type(gpio_num, interrupt_type);
    
    return ESP_OK;
}

} // extern "C"