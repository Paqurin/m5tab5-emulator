/**
 * @file spi_master.h
 * @brief ESP-IDF SPI Master driver API for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible SPI master driver functions that map to the
 * emulated SPI controller, providing seamless compatibility for ESP-IDF applications
 * running on the emulator.
 */

#pragma once

#include "../esp_types.h"
#include "../esp_err.h"
#include "gpio.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of SPI peripherals available
 */
#define SOC_SPI_PERIPH_NUM 4

/**
 * @brief SPI peripheral definitions for ESP32-P4
 */
typedef enum {
    SPI1_HOST = 0,  ///< SPI1 (flash)
    SPI2_HOST = 1,  ///< SPI2 (general purpose)
    SPI3_HOST = 2,  ///< SPI3 (general purpose)
    SPI_HOST_MAX,   ///< Number of SPI hosts
} spi_host_device_t;

/**
 * @brief SPI transaction flags
 */
#define SPI_TRANS_USE_RXDATA      (1<<0)   ///< Receive into rx_data member instead of memory pointed by rx_buffer
#define SPI_TRANS_USE_TXDATA      (1<<1)   ///< Transmit tx_data member instead of memory pointed by tx_buffer
#define SPI_TRANS_MODE_DIO        (1<<2)   ///< Transmit/receive data in 2-bit mode
#define SPI_TRANS_MODE_QIO        (1<<3)   ///< Transmit/receive data in 4-bit mode
#define SPI_TRANS_USE_ADDR        (1<<4)   ///< Use 'addr' member of transaction struct
#define SPI_TRANS_MULTILINE_CMD   (1<<5)   ///< The command is in multiple bit mode (DIO/QIO)
#define SPI_TRANS_MULTILINE_ADDR  (1<<6)   ///< The address is in multiple bit mode
#define SPI_TRANS_VARIABLE_CMD    (1<<7)   ///< Use variable command length defined in transaction
#define SPI_TRANS_VARIABLE_ADDR   (1<<8)   ///< Use variable address length defined in transaction
#define SPI_TRANS_VARIABLE_DUMMY  (1<<9)   ///< Use variable dummy cycle length defined in transaction
#define SPI_TRANS_VARIABLE_DUMMY_BITS  (1<<10)  ///< Use variable dummy bits length defined in transaction

/**
 * @brief Forward declarations
 */
typedef struct spi_transaction_t spi_transaction_t;
typedef void (*transaction_cb_t)(spi_transaction_t *trans);

/**
 * @brief SPI device interface configuration structure
 */
typedef struct {
    uint8_t command_bits;           ///< Default command field bits, used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored
    uint8_t address_bits;           ///< Default address field bits, used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored
    uint8_t dummy_bits;             ///< Default dummy field bits, used when ``SPI_TRANS_VARIABLE_DUMMY`` is not used, otherwise ignored
    uint8_t mode;                   ///< SPI mode (0-3)
    uint8_t duty_cycle_pos;         ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty)
    uint8_t cs_ena_pretrans;        ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16)
    uint8_t cs_ena_posttrans;       ///< Amount of SPI bit-cycles the cs should be kept activated after the transmission (0-16)
    int clock_speed_hz;             ///< Clock out frequency, divisors of 80MHz, in Hz
    int input_delay_ns;             ///< Maximum data valid time of slave
    int spics_io_num;               ///< CS GPIO pin for this device, or -1 if not used
    uint32_t flags;                 ///< Bitwise OR of SPI_DEVICE_* flags
    int queue_size;                 ///< Transaction queue size. This sets how many transactions can be 'in the air'
    transaction_cb_t pre_cb;        ///< Callback to be called before a transmission is started
    transaction_cb_t post_cb;       ///< Callback to be called after a transmission has completed
} spi_device_interface_config_t;

/**
 * @brief SPI bus configuration structure
 */
typedef struct {
    int mosi_io_num;                ///< GPIO pin for Master Out Slave In (MOSI) signal, or -1 if not used
    int miso_io_num;                ///< GPIO pin for Master In Slave Out (MISO) signal, or -1 if not used
    int sclk_io_num;                ///< GPIO pin for Spi ClocK signal, or -1 if not used
    int quadwp_io_num;              ///< GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used
    int quadhd_io_num;              ///< GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used
    int data4_io_num;               ///< GPIO pin for spi data4 signal, or -1 if not used
    int data5_io_num;               ///< GPIO pin for spi data5 signal, or -1 if not used
    int data6_io_num;               ///< GPIO pin for spi data6 signal, or -1 if not used
    int data7_io_num;               ///< GPIO pin for spi data7 signal, or -1 if not used
    int max_transfer_sz;            ///< Maximum transfer size, in bytes. Defaults to 4094 if 0
    uint32_t flags;                 ///< Abilities of bus to be checked by the driver
    int intr_flags;                 ///< Interrupt flag for the bus interrupt
} spi_bus_config_t;

/**
 * @brief SPI device handle type
 */
typedef struct spi_device_t* spi_device_handle_t;


/**
 * @brief SPI transaction structure
 */
struct spi_transaction_t {
    uint32_t flags;                 ///< Bitwise OR of SPI_TRANS_* flags
    uint16_t cmd;                   ///< Command data, of which the length is set in the ``command_bits`` of spi_device_interface_config_t
    uint64_t addr;                  ///< Address data, of which the length is set in the ``address_bits`` of spi_device_interface_config_t
    size_t length;                  ///< Total data length, in bits
    size_t rxlength;                ///< Total data length received, should be not greater than ``length`` in full-duplex mode (0 defaults this to the value of ``length``)
    void *user;                     ///< User-defined variable. Can be used to store e.g. transaction ID
    union {
        const void *tx_buffer;      ///< Pointer to transmit buffer, or NULL for no MOSI phase
        uint8_t tx_data[4];         ///< If SPI_TRANS_USE_TXDATA is set, data set here is sent directly from this variable
    };
    union {
        void *rx_buffer;            ///< Pointer to receive buffer, or NULL for no MISO phase
        uint8_t rx_data[4];         ///< If SPI_TRANS_USE_RXDATA is set, data is received directly to this variable
    };
};

/**
 * @brief SPI device flags
 */
#define SPI_DEVICE_TXBIT_LSBFIRST   (1<<0)  ///< Transmit LSB first
#define SPI_DEVICE_RXBIT_LSBFIRST   (1<<1)  ///< Receive LSB first
#define SPI_DEVICE_BIT_LSBFIRST     (SPI_DEVICE_TXBIT_LSBFIRST|SPI_DEVICE_RXBIT_LSBFIRST) ///< Transmit and receive LSB first
#define SPI_DEVICE_3WIRE            (1<<2)  ///< Use spiq for both sending and receiving data
#define SPI_DEVICE_POSITIVE_CS      (1<<3)  ///< Make CS positive during a transaction instead of negative
#define SPI_DEVICE_HALFDUPLEX       (1<<4)  ///< Transmit data before receiving it, instead of simultaneously
#define SPI_DEVICE_CLK_AS_CS        (1<<5)  ///< Output clock on CS line if CS is active
#define SPI_DEVICE_NO_DUMMY         (1<<6)  ///< Don't insert dummy bits after address phase

/**
 * @brief SPI bus flags
 */
#define SPICOMMON_BUSFLAG_SLAVE     (1<<0)  ///< Initialize I/O in slave mode
#define SPICOMMON_BUSFLAG_MASTER    (1<<1)  ///< Initialize I/O in master mode
#define SPICOMMON_BUSFLAG_IOMUX_PINS (1<<2)  ///< Check using iomux pins. Or indicate the pins are configured through iomux
#define SPICOMMON_BUSFLAG_SCLK      (1<<3)  ///< Check existing of SCLK pin. Or indicate SCLK pin is configured
#define SPICOMMON_BUSFLAG_MISO      (1<<4)  ///< Check existing of MISO pin. Or indicate MISO pin is configured
#define SPICOMMON_BUSFLAG_MOSI      (1<<5)  ///< Check existing of MOSI pin. Or indicate MOSI pin is configured
#define SPICOMMON_BUSFLAG_DUAL      (1<<6)  ///< Check dual mode. Or indicate bus is in dual mode
#define SPICOMMON_BUSFLAG_WPHD      (1<<7)  ///< Check existing of WP and HD pins. Or indicate WP & HD pins are configured
#define SPICOMMON_BUSFLAG_QUAD      (SPICOMMON_BUSFLAG_DUAL|SPICOMMON_BUSFLAG_WPHD)  ///< Check quad mode. Or indicate bus is in quad mode
#define SPICOMMON_BUSFLAG_OCTAL     (1<<8)  ///< Check octal mode. Or indicate bus is in octal mode

/**
 * @brief Initialize a SPI bus
 *
 * @warning SPI0/1 is not supported
 *
 * @param host_id       SPI peripheral to use (SPI2_HOST or SPI3_HOST)
 * @param bus_config    Pointer to a spi_bus_config_t struct specifying how the host should be initialized
 * @param dma_chan      Either channel 1 or 2, or SPI_DMA_DISABLED
 *
 * @return
 *         - ESP_ERR_INVALID_ARG   if configuration is invalid
 *         - ESP_ERR_INVALID_STATE if host already is in use
 *         - ESP_ERR_NO_MEM        if out of memory
 *         - ESP_OK                on success
 */
esp_err_t spi_bus_initialize(spi_host_device_t host_id, const spi_bus_config_t *bus_config, int dma_chan);

/**
 * @brief Free a SPI bus
 *
 * @warning In order for this to succeed, all devices have to be removed first.
 *
 * @param host_id SPI peripheral to free
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_ERR_INVALID_STATE if bus hasn't been initialized before, or not all devices on the bus are freed
 *         - ESP_OK                on success
 */
esp_err_t spi_bus_free(spi_host_device_t host_id);

/**
 * @brief Add a device to a SPI bus
 *
 * This initializes the internal structures for a device, plus allocates a CS pin on the indicated SPI master
 * peripheral and routes it to the indicated GPIO. All SPI master devices have three CS pins and can thus control
 * up to three devices.
 *
 * @param host_id       SPI peripheral to allocate device on
 * @param dev_config    SPI interface protocol config for the device
 * @param handle        Pointer to variable to store the device handle
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_ERR_NOT_FOUND     if host doesn't have any free CS slots
 *         - ESP_ERR_NO_MEM        if out of memory
 *         - ESP_OK                on success
 */
esp_err_t spi_bus_add_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);

/**
 * @brief Remove a device from a SPI bus
 *
 * @param handle Device handle to free
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_ERR_INVALID_STATE if device already is freed
 *         - ESP_OK                on success
 */
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);

/**
 * @brief Queue a SPI transaction for execution
 *
 * @param handle Device handle obtained using spi_host_add_dev
 * @param trans_desc Description of transaction to execute
 * @param ticks_to_wait Ticks to wait until there's room in the queue; use portMAX_DELAY to
 *                      never time out.
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *trans_desc, TickType_t ticks_to_wait);

/**
 * @brief Get the result of a SPI transaction queued earlier
 *
 * This routine will wait until a transaction to the given device (queued earlier with
 * spi_device_queue_trans) has succesfully completed. It will then return the description of the
 * completed transaction so software can inspect the result and e.g. free the memory or
 * re-use the buffers.
 *
 * @param handle Device handle obtained using spi_host_add_dev
 * @param trans_desc Pointer to variable able to contain a pointer to the description of the transaction
 *        that is executed
 * @param ticks_to_wait Ticks to wait until there's a returned item; use portMAX_DELAY to never time
 *                      out.
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spi_device_get_trans_result(spi_device_handle_t handle, spi_transaction_t **trans_desc, TickType_t ticks_to_wait);

/**
 * @brief Do a SPI transaction
 *
 * Essentially does the same as spi_device_queue_trans followed by spi_device_get_trans_result.
 * Do not use this when there is still a transaction separately queued (started) from
 * spi_device_queue_trans) or the function will block until all queued transactions are finished.
 *
 * @param handle Device handle obtained using spi_host_add_dev
 * @param trans_desc Description of transaction to execute
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc);

/**
 * @brief Immediately start a polling transaction.
 *
 * @note Normally a transaction consists of acquire bus, send transaction, release bus. This function is faster as
 * it doesn't need to wait for lock of the bus.
 * @param handle Device handle obtained using spi_host_add_dev
 * @param trans_desc Description of transaction to execute
 * @param ticks_to_wait Ticks to wait until there's room in the queue; currently only portMAX_DELAY is supported.
 *
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid. This can happen if SPI_TRANS_CS_KEEP_ACTIVE flag is specified.
 *         - ESP_ERR_TIMEOUT       if the device cannot get control of the bus before ``ticks_to_wait`` expired
 *         - ESP_ERR_INVALID_STATE if a transaction is in flight
 *         - ESP_OK                on success
 */
esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc, TickType_t ticks_to_wait);

/**
 * @brief Occupy the SPI bus for a device to do continuous transactions.
 *
 * Transactions to all other devices will be put on hold until ``spi_device_release_bus`` is called.
 *
 * @note The function will set the amount of command, address, dummy cycles to 0.
 *
 * @param device The device to occupy the bus.
 * @param wait Time to wait before the the bus is occupied by the device. Currently MUST set to portMAX_DELAY.
 *
 * @return
 *      - ESP_ERR_INVALID_ARG    if parameter is invalid
 *      - ESP_OK                 on success
 */
esp_err_t spi_device_acquire_bus(spi_device_handle_t device, TickType_t wait);

/**
 * @brief Release the SPI bus occupied by the device.
 *
 * All other devices will be able to start sending transactions.
 *
 * @param device The device to release the bus.
 */
void spi_device_release_bus(spi_device_handle_t device);

/**
 * @brief Calculate the working frequency that is most close to desired frequency.
 *
 * @param fapb The frequency of apb clock, should be ``APB_CLK_FREQ``.
 * @param hz Desired working frequency
 * @param duty_cycle Duty cycle of the spi clock
 *
 * @return Actual working frequency that most fit.
 */
int spi_get_actual_clock(int fapb, int hz, int duty_cycle);

/**
 * @brief Calculate the timing settings of specified frequency and settings.
 *
 * @param gpio_is_used True if using GPIO matrix, or False if iomux pins are used.
 * @param input_delay_ns Input delay from SCLK launch edge to MISO data valid
 * @param eff_clk Effective clock frequency (in Hz) from spi_get_actual_clock.
 * @param dummy_o Address of dummy bits used output. Set to NULL if not needed.
 * @param cycles_remain_o Address of cycles remaining (after dummy bits are used) output.
 *         - -1 if too many cycles remaining, suggest to use lower mode to improve efficiency.
 * @return
 *      - ESP_ERR_INVALID_ARG   if parameter is invalid
 *      - ESP_OK                on success
 */
esp_err_t spi_get_timing(bool gpio_is_used, int input_delay_ns, int eff_clk, int *dummy_o, int *cycles_remain_o);

/**
 * @brief Get the frequency limit of current configurations.
 *         SPI master working at this limit ensures a balance between the speed and the success rate.
 *
 * @param gpio_is_used True if using GPIO matrix, or False if dedicated iomux pins are used.
 * @param input_delay_ns Input delay from SCLK launch edge to MISO data valid.
 * @return Frequency limit of current configurations.
 */
int spi_get_freq_limit(bool gpio_is_used, int input_delay_ns);

/**
 * @brief Get the SPI bus which is in use by the device.
 *
 * @param device SPI device handle.
 * @return The bus ID this device is attached to.
 */
spi_host_device_t spi_device_get_bus(spi_device_handle_t device);

/**
 * @brief Get the actual frequency a SPI device is working at.
 *
 * @param handle SPI device handle
 * @return the actual frequency
 */
int spi_device_get_actual_freq(spi_device_handle_t handle);

#ifdef __cplusplus
}
#endif