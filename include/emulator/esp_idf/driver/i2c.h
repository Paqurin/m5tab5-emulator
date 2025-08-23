/**
 * @file i2c.h
 * @brief ESP-IDF I2C driver API emulation for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible I2C APIs that map to the emulated
 * I2C controller. Applications using standard ESP-IDF I2C functions will
 * work seamlessly with the emulator.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP-IDF error codes (shared with GPIO)
 */
#ifndef ESP_OK
#define ESP_OK                 (0)     ///< No error
#define ESP_FAIL              (-1)     ///< Generic error
#define ESP_ERR_INVALID_ARG   (0x102)  ///< Invalid argument
#define ESP_ERR_INVALID_STATE (0x103)  ///< Invalid state
#define ESP_ERR_NOT_FOUND     (0x105)  ///< Component not found
#define ESP_ERR_TIMEOUT       (0x107)  ///< Timeout occurred
#define ESP_ERR_INVALID_SIZE  (0x104)  ///< Invalid data size
#define ESP_ERR_NOT_SUPPORTED (0x106)  ///< Operation not supported
#endif

/**
 * @brief I2C port numbers for ESP32-P4
 * 
 * M5Stack Tab5 specific I2C configuration:
 * - I2C_NUM_0: Main I2C bus (GPIO21=SDA, GPIO22=SCL) for BMI270, ES8388, etc.
 * - I2C_NUM_1: Secondary I2C bus (configurable pins)
 */
typedef enum {
    I2C_NUM_0 = 0,                    ///< I2C port 0
    I2C_NUM_1 = 1,                    ///< I2C port 1
    I2C_NUM_MAX = 2                   ///< Maximum I2C port number
} i2c_port_t;

/**
 * @brief I2C mode definitions
 */
typedef enum {
    I2C_MODE_SLAVE = 0,               ///< I2C slave mode
    I2C_MODE_MASTER = 1               ///< I2C master mode
} i2c_mode_t;

/**
 * @brief I2C acknowledgment control
 */
typedef enum {
    I2C_MASTER_WRITE = 0,             ///< I2C write operation
    I2C_MASTER_READ = 1               ///< I2C read operation
} i2c_rw_t;

/**
 * @brief I2C acknowledgment levels
 */
typedef enum {
    I2C_MASTER_ACK = 0,               ///< ACK signal
    I2C_MASTER_NACK = 1,              ///< NACK signal  
    I2C_MASTER_LAST_NACK = 2          ///< NACK signal for last byte
} i2c_ack_type_t;

/**
 * @brief I2C address mode
 */
typedef enum {
    I2C_ADDR_BIT_LEN_7 = 0,          ///< I2C 7-bit address mode
    I2C_ADDR_BIT_LEN_10 = 1          ///< I2C 10-bit address mode
} i2c_addr_mode_t;

/**
 * @brief I2C configuration structure
 */
typedef struct {
    i2c_mode_t mode;                  ///< I2C mode (master or slave)
    int sda_io_num;                   ///< GPIO number for SDA signal
    int scl_io_num;                   ///< GPIO number for SCL signal
    bool sda_pullup_en;               ///< Internal SDA pull-up enable
    bool scl_pullup_en;               ///< Internal SCL pull-up enable
    
    union {
        struct {
            uint32_t clk_speed;       ///< I2C clock frequency for master mode
        } master;                     ///< I2C master configuration
        
        struct {
            uint8_t addr_10bit_en;    ///< I2C 10-bit address mode enable for slave
            uint16_t slave_addr;      ///< I2C address of slave
            uint32_t maximum_speed;   ///< I2C expected clock speed from master
        } slave;                      ///< I2C slave configuration
    };
    
    uint32_t clk_flags;              ///< Bitwise flags for I2C clock choice
} i2c_config_t;

/**
 * @brief I2C command handle type
 */
typedef void* i2c_cmd_handle_t;

// ============================================================================
// ESP-IDF I2C Driver API Functions
// ============================================================================

/**
 * @brief Install and initialize I2C driver
 * 
 * @param i2c_num I2C port number
 * @param mode I2C mode (master or slave)
 * @param slv_rx_buf_len Buffer length for slave RX (0 if master mode)
 * @param slv_tx_buf_len Buffer length for slave TX (0 if master mode)  
 * @param intr_alloc_flags Interrupt allocation flags
 * @return ESP_OK on success
 */
int i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, 
                       size_t slv_tx_buf_len, int intr_alloc_flags);

/**
 * @brief Delete I2C driver
 * 
 * @param i2c_num I2C port number
 * @return ESP_OK on success
 */
int i2c_driver_delete(i2c_port_t i2c_num);

/**
 * @brief Configure I2C parameters
 * 
 * @param i2c_num I2C port number
 * @param i2c_conf Pointer to I2C configuration structure
 * @return ESP_OK on success
 */
int i2c_param_config(i2c_port_t i2c_num, const i2c_config_t* i2c_conf);

/**
 * @brief Create I2C command link
 * 
 * @return I2C command handle
 */
i2c_cmd_handle_t i2c_cmd_link_create(void);

/**
 * @brief Delete I2C command link
 * 
 * @param cmd_handle I2C command handle
 */
void i2c_cmd_link_delete(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue I2C start condition
 * 
 * @param cmd_handle I2C command handle
 * @return ESP_OK on success
 */
int i2c_master_start(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue I2C stop condition
 * 
 * @param cmd_handle I2C command handle
 * @return ESP_OK on success
 */
int i2c_master_stop(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue I2C write operation
 * 
 * @param cmd_handle I2C command handle
 * @param data Pointer to data to write
 * @param data_len Length of data
 * @param ack_en Enable acknowledge check
 * @return ESP_OK on success
 */
int i2c_master_write(i2c_cmd_handle_t cmd_handle, const uint8_t* data, 
                     size_t data_len, bool ack_en);

/**
 * @brief Queue I2C write single byte
 * 
 * @param cmd_handle I2C command handle
 * @param data Byte to write
 * @param ack_en Enable acknowledge check
 * @return ESP_OK on success
 */
int i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en);

/**
 * @brief Queue I2C read operation
 * 
 * @param cmd_handle I2C command handle
 * @param data Pointer to buffer for read data
 * @param data_len Length of data to read
 * @param ack Acknowledge type for last byte
 * @return ESP_OK on success
 */
int i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t* data, 
                    size_t data_len, i2c_ack_type_t ack);

/**
 * @brief Queue I2C read single byte
 * 
 * @param cmd_handle I2C command handle
 * @param data Pointer to byte buffer
 * @param ack Acknowledge type
 * @return ESP_OK on success
 */
int i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t* data, i2c_ack_type_t ack);

/**
 * @brief Execute I2C command sequence
 * 
 * @param i2c_num I2C port number
 * @param cmd_handle I2C command handle
 * @param ticks_to_wait Timeout in FreeRTOS ticks
 * @return ESP_OK on success
 */
int i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, uint32_t ticks_to_wait);

/**
 * @brief Reset I2C bus (for error recovery)
 * 
 * @param i2c_num I2C port number
 * @return ESP_OK on success
 */
int i2c_reset_tx_fifo(i2c_port_t i2c_num);

/**
 * @brief Reset I2C receive FIFO
 * 
 * @param i2c_num I2C port number
 * @return ESP_OK on success
 */
int i2c_reset_rx_fifo(i2c_port_t i2c_num);

/**
 * @brief Set I2C clock source
 * 
 * @param i2c_num I2C port number
 * @param clk_src Clock source selection
 * @return ESP_OK on success
 */
int i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, 
                bool sda_pullup_en, bool scl_pullup_en, i2c_mode_t mode);

// ============================================================================
// Convenience Functions for Common Operations
// ============================================================================

/**
 * @brief Convenience function to read from I2C device
 * 
 * @param i2c_num I2C port number
 * @param device_addr I2C device address
 * @param data_addr Register address to read from
 * @param data Pointer to buffer for read data
 * @param data_len Length of data to read
 * @return ESP_OK on success
 */
int i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_addr,
                                uint8_t data_addr, uint8_t* data, size_t data_len);

/**
 * @brief Convenience function to write to I2C device
 * 
 * @param i2c_num I2C port number
 * @param device_addr I2C device address  
 * @param data_addr Register address to write to
 * @param data Pointer to data to write
 * @param data_len Length of data to write
 * @return ESP_OK on success
 */
int i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_addr,
                               uint8_t data_addr, const uint8_t* data, size_t data_len);

// ============================================================================
// M5Stack Tab5 Specific I2C Definitions
// ============================================================================

/**
 * @brief M5Stack Tab5 I2C device addresses
 */
#define M5TAB5_BMI270_ADDR      (0x68)    ///< BMI270 IMU I2C address
#define M5TAB5_ES8388_ADDR      (0x10)    ///< ES8388 Audio Codec I2C address  
#define M5TAB5_RX8130CE_ADDR    (0x64)    ///< RX8130CE RTC I2C address
#define M5TAB5_GT911_ADDR       (0x5D)    ///< GT911 Touch Controller I2C address

/**
 * @brief M5Stack Tab5 default I2C configuration
 */
#define M5TAB5_I2C_SDA_PIN      (21)      ///< Default SDA pin for M5Stack Tab5
#define M5TAB5_I2C_SCL_PIN      (22)      ///< Default SCL pin for M5Stack Tab5  
#define M5TAB5_I2C_FREQ_HZ      (400000)  ///< Default I2C frequency (400kHz)

#ifdef __cplusplus
}
#endif