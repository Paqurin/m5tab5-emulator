/**
 * @file uart.h
 * @brief ESP-IDF UART driver API emulation for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible UART APIs that map to the emulated
 * UART controller, providing seamless compatibility for ESP-IDF applications
 * running on the emulator.
 * 
 * M5Stack Tab5 UART Configuration:
 * - UART_NUM_0: USB-Serial (CP2102N) - Debug/Programming interface
 * - UART_NUM_1: RS-485 interface (MAX485) - Industrial communication  
 * - UART_NUM_2: General purpose GPIO UART - Sensor/device communication
 */

#pragma once

#include "../esp_types.h"
#include "../freertos/FreeRTOS.h"
#include "../freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART port number definitions
 */
typedef enum {
    UART_NUM_0 = 0,  ///< UART port 0 - USB-Serial (CP2102N)
    UART_NUM_1 = 1,  ///< UART port 1 - RS-485 (MAX485) 
    UART_NUM_2 = 2,  ///< UART port 2 - General purpose
    UART_NUM_MAX     ///< Maximum UART port number
} uart_port_t;

/**
 * @brief UART data bits configuration
 */
typedef enum {
    UART_DATA_5_BITS = 0x0,  ///< 5 data bits
    UART_DATA_6_BITS = 0x1,  ///< 6 data bits  
    UART_DATA_7_BITS = 0x2,  ///< 7 data bits
    UART_DATA_8_BITS = 0x3,  ///< 8 data bits
} uart_word_length_t;

/**
 * @brief UART stop bits configuration
 */
typedef enum {
    UART_STOP_BITS_1   = 0x1,  ///< 1 stop bit
    UART_STOP_BITS_1_5 = 0x2,  ///< 1.5 stop bits
    UART_STOP_BITS_2   = 0x3,  ///< 2 stop bits
} uart_stop_bits_t;

/**
 * @brief UART parity configuration
 */
typedef enum {
    UART_PARITY_DISABLE = 0x0,  ///< No parity
    UART_PARITY_EVEN    = 0x2,  ///< Even parity
    UART_PARITY_ODD     = 0x3   ///< Odd parity
} uart_parity_t;

/**
 * @brief UART hardware flow control configuration
 */
typedef enum {
    UART_HW_FLOWCTRL_DISABLE = 0x0,  ///< No flow control
    UART_HW_FLOWCTRL_RTS     = 0x1,  ///< RTS flow control
    UART_HW_FLOWCTRL_CTS     = 0x2,  ///< CTS flow control
    UART_HW_FLOWCTRL_CTS_RTS = 0x3,  ///< Both RTS and CTS
} uart_hw_flowcontrol_t;

/**
 * @brief UART communication mode  
 */
typedef enum {
    UART_MODE_UART = 0x00,  ///< Standard UART mode
    UART_MODE_RS485_HALF_DUPLEX = 0x01,  ///< RS-485 half duplex mode
    UART_MODE_IRDA = 0x02,  ///< IrDA mode (not supported)
    UART_MODE_RS485_COLLISION_DETECT = 0x03,  ///< RS-485 with collision detection
    UART_MODE_RS485_APP_CTRL = 0x04,  ///< RS-485 application controlled
} uart_mode_t;

/**
 * @brief UART signal inversion configuration
 */
typedef enum {
    UART_SIGNAL_INV_DISABLE  = 0,  ///< No signal inversion
    UART_SIGNAL_INV_TXD = (0x1 << 0),  ///< Invert TXD signal
    UART_SIGNAL_INV_RXD = (0x1 << 1),  ///< Invert RXD signal  
    UART_SIGNAL_INV_RTS = (0x1 << 2),  ///< Invert RTS signal
    UART_SIGNAL_INV_CTS = (0x1 << 3),  ///< Invert CTS signal
} uart_signal_inv_t;

/**
 * @brief UART event types for event queue
 */
typedef enum {
    UART_DATA,              ///< UART data event
    UART_BREAK,             ///< UART break event
    UART_BUFFER_FULL,       ///< UART RX buffer full
    UART_FIFO_OVF,          ///< UART FIFO overflow
    UART_FRAME_ERR,         ///< UART RX frame error
    UART_PARITY_ERR,        ///< UART RX parity error
    UART_DATA_BREAK,        ///< UART data followed by break
    UART_PATTERN_DET,       ///< UART pattern detected
    UART_EVENT_MAX,         ///< UART event max index
} uart_event_type_t;

/**
 * @brief UART event structure for event queue
 */
typedef struct {
    uart_event_type_t type;  ///< UART event type
    size_t size;             ///< UART data size for UART_DATA event
    bool timeout_flag;       ///< UART data read timeout flag
} uart_event_t;

/**
 * @brief UART configuration structure
 */
typedef struct {
    int baud_rate;                      ///< UART baud rate
    uart_word_length_t data_bits;       ///< UART data bits
    uart_parity_t parity;               ///< UART parity mode
    uart_stop_bits_t stop_bits;         ///< UART stop bits
    uart_hw_flowcontrol_t flow_ctrl;    ///< UART HW flow control mode
    uint8_t rx_flow_ctrl_thresh;        ///< UART HW RTS threshold
    union {
        uart_signal_inv_t source_clk;   ///< UART source clock selection (legacy)
    };
} uart_config_t;

/**
 * @brief UART interrupt configuration structure
 */
typedef struct {
    uint32_t intr_enable_mask;          ///< UART interrupt enable mask
    uint8_t  rx_timeout_thresh;         ///< UART timeout interrupt threshold
    uint8_t  txfifo_empty_intr_thresh;  ///< UART TX empty interrupt threshold
    uint8_t  rxfifo_full_thresh;        ///< UART RX full interrupt threshold
} uart_intr_config_t;

// ============================================================================
// ESP-IDF UART Driver API Functions
// ============================================================================

/**
 * @brief Install UART driver and set UART to specified configuration.
 *
 * UART ISR handler will be attached to the same CPU core that this function is running on.
 *
 * @note  Rx_buffer_size should be greater than UART_HW_FIFO_LEN(uart_num). Tx_buffer_size should be either zero or greater than UART_HW_FIFO_LEN(uart_num).
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param rx_buffer_size UART RX ring buffer size.
 * @param tx_buffer_size UART TX ring buffer size.
 *        If set to zero, driver will not use TX buffer, TX function will block task until all data have been sent out.
 * @param queue_size UART event queue size/depth.
 * @param uart_queue UART event queue handle (out param). On success, a new queue handle is written here to provide
 *        access to UART events. If set to NULL, driver will not use an event queue.
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred)
 *        ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info. Do not set ESP_INTR_FLAG_IRAM here
 *        (the driver's ISR handler is not located in IRAM)
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, 
                              int queue_size, QueueHandle_t* uart_queue, int intr_alloc_flags);

/**
 * @brief Uninstall UART driver.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_driver_delete(uart_port_t uart_num);

/**
 * @brief Set UART data communication format
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param uart_config UART configure parameters
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t* uart_config);

/**
 * @brief Configure UART interrupts.
 *
 * @param uart_num  UART port number, the max port number is (UART_NUM_MAX -1).
 * @param intr_conf UART interrupt settings
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_intr_config(uart_port_t uart_num, const uart_intr_config_t* intr_conf);

/**
 * @brief Set UART pin number
 *
 * @note Internal signal can be output to multiple GPIO pads. Only one GPIO pad can connect with input signal.
 *
 * @param uart_num   UART port number, the max port number is (UART_NUM_MAX -1).
 * @param tx_io_num  UART TX pin GPIO number.
 * @param rx_io_num  UART RX pin GPIO number.
 * @param rts_io_num UART RTS pin GPIO number.
 * @param cts_io_num UART CTS pin GPIO number.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);

/**
 * @brief UART set RTS level (before inverse)
 *        UART rx hardware flow control should not be set.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param level    1: RTS output high, 0: RTS output low
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_rts(uart_port_t uart_num, int level);

/**
 * @brief UART set DTR level (before inverse)
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param level    1: DTR output high, 0: DTR output low
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_dtr(uart_port_t uart_num, int level);

/**
 * @brief Set UART baud rate.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param baudrate UART baud rate.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate);

/**
 * @brief Get UART baud rate.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param baudrate Pointer to store the UART baud rate
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_baudrate(uart_port_t uart_num, uint32_t* baudrate);

/**
 * @brief UART write bytes to the buffer and then send data.
 *
 * This function will not wait for enough space in TX FIFO. It will just fill the available TX buffer.
 * If no TX buffer is installed, this function will return after copying all the data to hardware TX FIFO.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param src   data buffer address
 * @param size  data length to send
 *
 * @return
 *     - (-1)  Parameter error
 *     - Others (>=0) The number of bytes pushed to the TX FIFO
 */
int uart_write_bytes(uart_port_t uart_num, const void* src, size_t size);

/**
 * @brief UART write bytes to the buffer and then send data with timeout.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param src   data buffer address
 * @param size  data length to send  
 * @param ticks_to_wait Timeout in RTOS ticks
 *
 * @return
 *     - (-1)  Parameter error
 *     - Others (>=0) The number of bytes pushed to the TX FIFO
 */
int uart_write_bytes_with_break(uart_port_t uart_num, const void* src, size_t size, int brk_len);

/**
 * @brief UART read bytes from UART buffer
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param buf    pointer to the buffer.
 * @param length data length
 * @param ticks_to_wait sTimeout, count in RTOS ticks
 *
 * @return
 *     - (-1) Error
 *     - Others (>=0) The number of bytes read from UART FIFO
 */
int uart_read_bytes(uart_port_t uart_num, void* buf, uint32_t length, TickType_t ticks_to_wait);

/**
 * @brief Alias of uart_flush_input.
 *        UART ring buffer flush. This will discard all data in the UART RX buffer.
 * @note  Instead of waiting the data sent out, this function will clear UART rx buffer.
 *        In order to send all the data in tx FIFO, we can use uart_wait_tx_done function.
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_flush(uart_port_t uart_num);

/**
 * @brief Clear input buffer, discard all the data is in the ring-buffer.
 * @note  In order to send all the data in tx FIFO, we can use uart_wait_tx_done function.
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_flush_input(uart_port_t uart_num);

/**
 * @brief UART get RX ring buffer cached data length
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param size Pointer of size_t to accept cached data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t* size);

/**
 * @brief UART disable pattern detect function.
 *        Designed for applications like 'AT commands'.
 *        When the hardware detect a series of one same character, the interrupt will be triggered.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_disable_pattern_det_intr(uart_port_t uart_num);

/**
 * @brief UART enable pattern detect function.
 *        Designed for applications like 'AT commands'.
 *        When the hardware detect a series of one same character, the interrupt will be triggered.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param pattern_chr character of the pattern.
 * @param chr_num number of the character, 8bit value.
 * @param chr_tout timeout of the interval between each pattern characters, 16bit value, unit is the baud-rate cycle.
 *        When the duration is more than this value, it will not take this data as at_cmd char.
 * @param post_idle idle time after the last pattern character, 16bit value, unit is the baud-rate cycle.
 *        When the duration is less than this value, it will not take the previous data as the last at_cmd char
 * @param pre_idle idle time before the first pattern character, 16bit value, unit is the baud-rate cycle.
 *        When the duration is less than this value, it will not take this data as the first at_cmd char.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_enable_pattern_det_baud_intr(uart_port_t uart_num, char pattern_chr, uint8_t chr_num, 
                                            int chr_tout, int post_idle, int pre_idle);

/**
 * @brief Return the nearest detected pattern position in buffer.
 *        The positions of the detected pattern are saved in a queue,
 *        this function will dequeue the first pattern position and move the position pointer to next pattern position.
 * @note  If the RX buffer is full and flow control is not enabled,
 *        the detected pattern may not be found in the rx buffer due to overflow.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - (-1) No pattern found for current index or parameter error
 *     - others the pattern position in rx buffer.
 */
int uart_pattern_pop_pos(uart_port_t uart_num);

/**
 * @brief Return the nearest detected pattern position in buffer.
 *        The positions of the detected pattern are saved in a queue,
 *        This function do nothing to the queue.
 * @note  If the RX buffer is full and flow control is not enabled,
 *        the detected pattern may not be found in the rx buffer due to overflow.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - (-1) No pattern found for current index or parameter error
 *     - others the pattern position in rx buffer.
 */
int uart_pattern_get_pos(uart_port_t uart_num);

/**
 * @brief Allocate a new memory with the given length to save record the detected pattern position in rx buffer.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param queue_length Max queue length for the detected pattern.
 *        If the queue length is not large enough, some pattern positions might be lost.
 *        Set this value to the maximum number of patterns that could be saved in data buffer at the same time.
 * @return
 *     - ESP_ERR_NO_MEM No enough memory to allocate the pattern queue.
 *     - ESP_ERR_INVALID_STATE Driver not installed.
 *     - ESP_ERR_INVALID_ARG Parameter error.
 *     - ESP_OK Success.
 */
esp_err_t uart_pattern_queue_reset(uart_port_t uart_num, int queue_length);

/**
 * @brief UART set communication mode
 * @note  This function must be executed after uart_driver_install(), when the driver object is initialized.
 * @param uart_num     Uart number to configure, the max port number is (UART_NUM_MAX -1).
 * @param mode UART    UART mode to set
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t uart_set_mode(uart_port_t uart_num, uart_mode_t mode);

/**
 * @brief Set uart threshold value for RX fifo full
 * @note If application is using higher baudrate and it is observed that bytes in hardware RX fifo are overwritten then this threshold can be reduced
 *
 * @param uart_num UART_NUM_0, UART_NUM_1 or UART_NUM_2
 * @param threshold Threshold value above which RX fifo full interrupt is generated.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE Driver is not installed
 */
esp_err_t uart_set_rx_full_threshold(uart_port_t uart_num, int threshold);

/**
 * @brief Set uart threshold values for TX fifo empty
 *
 * @param uart_num UART_NUM_0, UART_NUM_1 or UART_NUM_2
 * @param threshold Threshold value below which TX fifo empty interrupt is generated.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE Driver is not installed
 */
esp_err_t uart_set_tx_empty_threshold(uart_port_t uart_num, int threshold);

/**
 * @brief UART set threshold timeout for TOUT feature
 *
 * @param uart_num     Uart number to configure, the max port number is (UART_NUM_MAX -1).
 * @param tout_thresh  This parameter defines timeout threshold in uart symbol periods. The maximum value of threshold is 127.
 *        For example, if we want a timeout of 1ms and the symbol period is 8.68us (115200 8-N-1), then the timeout threshold is 1000/8.68 = 115.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE Driver is not installed
 */
esp_err_t uart_set_rx_timeout(uart_port_t uart_num, const uint8_t tout_thresh);

/**
 * @brief Returns collision detection flag for RS485 mode
 *        Function returns the collision detection flag into variable pointed by collision_flag.
 *        *collision_flag = true, if collision detected else it is equal to false.
 *        This function should be executed when actual transmission is completed over RS485 bus.
 *        The collision flag is cleared automatically when UART is configured for RS485 mode and new transmission is started.
 *
 * @param uart_num  Uart number to configure the max port number is (UART_NUM_MAX -1).
 * @param collision_flag Pointer to variable for collision flag.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t uart_get_collision_flag(uart_port_t uart_num, bool* collision_flag);

/**
 * @brief Set the number of RX timeout interrupt threshold, and config RX timeout feature.
 *
 * @param uart_num     Uart number to configure, the max port number is (UART_NUM_MAX -1).
 * @param tout_thresh  This parameter defines timeout threshold in uart symbol periods. The maximum value of threshold is 127.
 *        If tout_thresh == 0, the timeout feature will be disabled.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE Driver is not installed
 */
esp_err_t uart_set_wakeup_threshold(uart_port_t uart_num, int wakeup_threshold);

/**
 * @brief Get the number of RX timeout interrupt threshold.
 *
 * @param uart_num     Uart number to configure, the max port number is (UART_NUM_MAX -1).
 * @param out_thresh   Pointer to accept the value of out_thresh
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t uart_get_wakeup_threshold(uart_port_t uart_num, int* out_thresh);

/**
 * @brief Wait until UART TX FIFO is empty.
 *
 * @param uart_num      UART port number, the max port number is (UART_NUM_MAX -1).
 * @param ticks_to_wait Timeout, count in RTOS ticks
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 *     - ESP_ERR_TIMEOUT Timeout
 */
esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait);

/**
 * @brief Send data to the UART port from a given buffer and length,
 *
 * This function will not wait for enough space in TX FIFO. It will just fill the available TX buffer.
 * If no TX buffer is installed, this function will return after copying all the data to hardware TX FIFO.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param buffer data buffer address
 * @param len    data length to send
 * @param brk_len break signal length (unit: the time it takes to send one data bit at current baudrate)
 *
 * @return
 *     - (-1) Parameter error
 *     - Others (>=0) The number of bytes pushed to the TX FIFO
 */
int uart_tx_chars(uart_port_t uart_num, const char* buffer, uint32_t len);

/**
 * @brief Send data to the UART port from a given buffer and length,
 *        and add a break signal at the end.
 *
 * This function will not wait for enough space in TX FIFO. It will just fill the available TX buffer.
 * If no TX buffer is installed, this function will return after copying all the data to hardware TX FIFO.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param buffer data buffer address
 * @param len    data length to send
 * @param brk_len break signal length (unit: the time it takes to send one data bit at current baudrate)
 *
 * @return
 *     - (-1) Parameter error
 *     - Others (>=0) The number of bytes pushed to the TX FIFO
 */
int uart_tx_chars_with_break(uart_port_t uart_num, const char* buffer, uint32_t len, int brk_len);

#ifdef __cplusplus
}
#endif