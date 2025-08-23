/**
 * @file gpio.h
 * @brief ESP-IDF GPIO driver API emulation for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible GPIO APIs that map to the emulated
 * GPIO controller. Applications using standard ESP-IDF GPIO functions will
 * work seamlessly with the emulator.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP-IDF GPIO error codes
 */
#define ESP_OK                 (0)     ///< No error
#define ESP_FAIL              (-1)     ///< Generic error
#define ESP_ERR_INVALID_ARG   (0x102)  ///< Invalid argument
#define ESP_ERR_INVALID_STATE (0x103)  ///< Invalid state
#define ESP_ERR_NOT_FOUND     (0x105)  ///< Component not found

/**
 * @brief GPIO number definitions for ESP32-P4
 * 
 * M5Stack Tab5 specific GPIO mappings:
 * - GPIO0-GPIO54: Available GPIO pins
 * - Some pins reserved for internal peripherals
 */
typedef int gpio_num_t;

#define GPIO_NUM_0    ((gpio_num_t)0)     ///< GPIO0, input only
#define GPIO_NUM_1    ((gpio_num_t)1)     ///< GPIO1
#define GPIO_NUM_2    ((gpio_num_t)2)     ///< GPIO2, Tab5 Button A
#define GPIO_NUM_3    ((gpio_num_t)3)     ///< GPIO3, Tab5 Button B  
#define GPIO_NUM_4    ((gpio_num_t)4)     ///< GPIO4, Tab5 Button C
#define GPIO_NUM_5    ((gpio_num_t)5)     ///< GPIO5
#define GPIO_NUM_6    ((gpio_num_t)6)     ///< GPIO6
#define GPIO_NUM_7    ((gpio_num_t)7)     ///< GPIO7
#define GPIO_NUM_8    ((gpio_num_t)8)     ///< GPIO8
#define GPIO_NUM_9    ((gpio_num_t)9)     ///< GPIO9
#define GPIO_NUM_10   ((gpio_num_t)10)    ///< GPIO10
#define GPIO_NUM_11   ((gpio_num_t)11)    ///< GPIO11
#define GPIO_NUM_12   ((gpio_num_t)12)    ///< GPIO12
#define GPIO_NUM_13   ((gpio_num_t)13)    ///< GPIO13
#define GPIO_NUM_14   ((gpio_num_t)14)    ///< GPIO14
#define GPIO_NUM_15   ((gpio_num_t)15)    ///< GPIO15
#define GPIO_NUM_16   ((gpio_num_t)16)    ///< GPIO16
#define GPIO_NUM_17   ((gpio_num_t)17)    ///< GPIO17
#define GPIO_NUM_18   ((gpio_num_t)18)    ///< GPIO18
#define GPIO_NUM_19   ((gpio_num_t)19)    ///< GPIO19
#define GPIO_NUM_20   ((gpio_num_t)20)    ///< GPIO20
#define GPIO_NUM_21   ((gpio_num_t)21)    ///< GPIO21, Tab5 I2C SDA
#define GPIO_NUM_22   ((gpio_num_t)22)    ///< GPIO22, Tab5 I2C SCL
#define GPIO_NUM_MAX  ((gpio_num_t)54)    ///< Maximum GPIO number

#define GPIO_NUM_NC   (-1)                ///< Not connected

/**
 * @brief GPIO direction modes
 */
typedef enum {
    GPIO_MODE_DISABLE = 0,                ///< GPIO disabled
    GPIO_MODE_INPUT = 1,                  ///< GPIO input
    GPIO_MODE_OUTPUT = 2,                 ///< GPIO output
    GPIO_MODE_OUTPUT_OD = 6,              ///< GPIO open drain output
    GPIO_MODE_INPUT_OUTPUT_OD = 7,        ///< GPIO input/output open drain
    GPIO_MODE_INPUT_OUTPUT = 3            ///< GPIO input/output
} gpio_mode_t;

/**
 * @brief GPIO pull modes
 */
typedef enum {
    GPIO_PULLUP_DISABLE = 0x0,            ///< Disable GPIO pull-up resistor
    GPIO_PULLUP_ENABLE = 0x1              ///< Enable GPIO pull-up resistor
} gpio_pullup_t;

typedef enum {
    GPIO_PULLDOWN_DISABLE = 0x0,          ///< Disable GPIO pull-down resistor
    GPIO_PULLDOWN_ENABLE = 0x1            ///< Enable GPIO pull-down resistor
} gpio_pulldown_t;

/**
 * @brief GPIO interrupt types
 */
typedef enum {
    GPIO_INTR_DISABLE = 0,                ///< Disable GPIO interrupt
    GPIO_INTR_POSEDGE = 1,                ///< Rising edge interrupt
    GPIO_INTR_NEGEDGE = 2,                ///< Falling edge interrupt
    GPIO_INTR_ANYEDGE = 3,                ///< Any edge interrupt
    GPIO_INTR_LOW_LEVEL = 4,              ///< Low level interrupt
    GPIO_INTR_HIGH_LEVEL = 5              ///< High level interrupt
} gpio_int_type_t;

/**
 * @brief GPIO configuration structure
 */
typedef struct {
    uint64_t pin_bit_mask;                ///< GPIO pin: set with bit mask, each bit maps to a GPIO
    gpio_mode_t mode;                     ///< GPIO mode: set input/output mode
    gpio_pullup_t pull_up_en;             ///< GPIO pull-up
    gpio_pulldown_t pull_down_en;         ///< GPIO pull-down
    gpio_int_type_t intr_type;            ///< GPIO interrupt type
} gpio_config_t;

/**
 * @brief GPIO ISR handler function type
 */
typedef void (*gpio_isr_t)(void* arg);

// ============================================================================
// ESP-IDF GPIO API Functions
// ============================================================================

/**
 * @brief Configure GPIO settings
 * 
 * @param pGPIOConfig Pointer to GPIO configuration structure
 * @return ESP_OK on success
 */
int gpio_config(const gpio_config_t* pGPIOConfig);

/**
 * @brief Reset GPIO settings to default state
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_reset_pin(gpio_num_t gpio_num);

/**
 * @brief Set GPIO output level
 * 
 * @param gpio_num GPIO number
 * @param level Output level (0 for LOW, non-zero for HIGH)
 * @return ESP_OK on success
 */
int gpio_set_level(gpio_num_t gpio_num, uint32_t level);

/**
 * @brief Get GPIO input level
 * 
 * @param gpio_num GPIO number
 * @return GPIO level (0 for LOW, 1 for HIGH, -1 for error)
 */
int gpio_get_level(gpio_num_t gpio_num);

/**
 * @brief Set GPIO direction
 * 
 * @param gpio_num GPIO number
 * @param mode Direction mode
 * @return ESP_OK on success
 */
int gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);

/**
 * @brief Set GPIO pull mode
 * 
 * @param gpio_num GPIO number
 * @param pull Pull mode
 * @return ESP_OK on success
 */
int gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);

/**
 * @brief Enable GPIO pull-up resistor
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_pullup_en(gpio_num_t gpio_num);

/**
 * @brief Disable GPIO pull-up resistor
 * 
 * @param gpio_num GPIO number  
 * @return ESP_OK on success
 */
int gpio_pullup_dis(gpio_num_t gpio_num);

/**
 * @brief Enable GPIO pull-down resistor
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_pulldown_en(gpio_num_t gpio_num);

/**
 * @brief Disable GPIO pull-down resistor
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_pulldown_dis(gpio_num_t gpio_num);

/**
 * @brief Install GPIO interrupt service routine
 * 
 * @param intr_alloc_flags Interrupt allocation flags
 * @return ESP_OK on success
 */
int gpio_install_isr_service(int intr_alloc_flags);

/**
 * @brief Uninstall GPIO interrupt service routine
 */
void gpio_uninstall_isr_service(void);

/**
 * @brief Add ISR handler for specific GPIO pin
 * 
 * @param gpio_num GPIO number
 * @param isr_handler ISR handler function
 * @param args Arguments passed to handler
 * @return ESP_OK on success
 */
int gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void* args);

/**
 * @brief Remove ISR handler for specific GPIO pin
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_isr_handler_remove(gpio_num_t gpio_num);

/**
 * @brief Enable GPIO interrupt
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_intr_enable(gpio_num_t gpio_num);

/**
 * @brief Disable GPIO interrupt
 * 
 * @param gpio_num GPIO number
 * @return ESP_OK on success
 */
int gpio_intr_disable(gpio_num_t gpio_num);

/**
 * @brief Set GPIO interrupt type
 * 
 * @param gpio_num GPIO number
 * @param intr_type Interrupt type
 * @return ESP_OK on success
 */
int gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t intr_type);

// Missing typedef for gpio_pull_mode_t - let me add it
typedef enum {
    GPIO_PULLUP_ONLY,                     ///< Pad pull up
    GPIO_PULLDOWN_ONLY,                   ///< Pad pull down
    GPIO_PULLUP_PULLDOWN,                 ///< Pad pull up + pull down
    GPIO_FLOATING                         ///< Pad floating
} gpio_pull_mode_t;

#ifdef __cplusplus
}
#endif