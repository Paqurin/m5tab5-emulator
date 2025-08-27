/**
 * @file uart_api.cpp
 * @brief ESP-IDF UART driver API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible UART functions that map to the
 * emulated UART controller, providing seamless compatibility for ESP-IDF
 * applications running on the emulator.
 * 
 * M5Stack Tab5 UART Configuration:
 * - UART_NUM_0: USB-Serial (CP2102N) - Debug/Programming interface
 * - UART_NUM_1: RS-485 interface (MAX485) - Industrial communication
 * - UART_NUM_2: General purpose GPIO UART - Sensor/device communication
 */

#include "emulator/esp_idf/driver/uart.h"
#include "emulator/peripherals/uart_controller.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/esp_idf/esp_idf.h"
#include <vector>
#include <map>
#include <chrono>
#include <thread>

namespace {
    using namespace m5tab5::emulator;
    using UARTController = m5tab5::emulator::UARTController;
    
    DECLARE_LOGGER("UART_API");
    
    // UART driver instance tracking
    struct UARTDriverInstance {
        bool installed = false;
        int rx_buffer_size = 0;
        int tx_buffer_size = 0;
        QueueHandle_t event_queue = nullptr;
        UARTController* controller = nullptr;
        uart_config_t config = {};
        uart_mode_t mode = UART_MODE_UART;
        
        // Pattern detection
        bool pattern_detection_enabled = false;
        char pattern_char = 0;
        uint8_t pattern_count = 0;
        std::vector<int> pattern_positions;
        
        // Buffer management
        std::vector<uint8_t> rx_buffer;
        std::vector<uint8_t> tx_buffer;
        size_t rx_head = 0;
        size_t rx_tail = 0;
        size_t tx_head = 0;
        size_t tx_tail = 0;
        
        // Flow control and status
        bool collision_detected = false;
        uint32_t baudrate_cached = 115200;
    };
    
    static std::map<uart_port_t, UARTDriverInstance> uart_instances;
    
    /**
     * @brief Get UART controller instance from emulator core
     */
    UARTController* get_uart_controller(uart_port_t uart_num) {
        static UARTController* uart_controllers[UART_NUM_MAX] = {nullptr};
        
        if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
            return nullptr;
        }
        
        if (!uart_controllers[uart_num]) {
            // Get emulator core instance from ESP-IDF context
            auto emulator = esp_idf_get_emulator_core();
            if (emulator) {
                LOG_DEBUG("Getting UART controller {} instance from EmulatorCore", uart_num);
                
                // Get UART controller component from emulator core
                // For now, we'll use a simplified approach - the emulator should have
                // multiple UART controllers indexed by number
                auto uart_shared = emulator->getComponent<UARTController>();
                if (uart_shared) {
                    uart_controllers[uart_num] = uart_shared.get();
                    LOG_DEBUG("Successfully retrieved UART controller {} from EmulatorCore", uart_num);
                } else {
                    LOG_WARN("UART controller {} not available from EmulatorCore", uart_num);
                }
            } else {
                LOG_WARN("EmulatorCore not available - ESP-IDF not properly initialized");
            }
        }
        
        return uart_controllers[uart_num];
    }
    
    /**
     * @brief Convert ESP-IDF data bits to emulator data bits
     */
    UARTDataBits convert_data_bits(uart_word_length_t esp_data_bits) {
        switch (esp_data_bits) {
            case UART_DATA_5_BITS: return UARTDataBits::BITS_5;
            case UART_DATA_6_BITS: return UARTDataBits::BITS_6;
            case UART_DATA_7_BITS: return UARTDataBits::BITS_7;
            case UART_DATA_8_BITS: return UARTDataBits::BITS_8;
            default: return UARTDataBits::BITS_8;
        }
    }
    
    /**
     * @brief Convert ESP-IDF stop bits to emulator stop bits
     */
    UARTStopBits convert_stop_bits(uart_stop_bits_t esp_stop_bits) {
        switch (esp_stop_bits) {
            case UART_STOP_BITS_1: return UARTStopBits::STOP_1;
            case UART_STOP_BITS_1_5: return UARTStopBits::STOP_1_5;
            case UART_STOP_BITS_2: return UARTStopBits::STOP_2;
            default: return UARTStopBits::STOP_1;
        }
    }
    
    /**
     * @brief Convert ESP-IDF parity to emulator parity
     */
    UARTParity convert_parity(uart_parity_t esp_parity) {
        switch (esp_parity) {
            case UART_PARITY_DISABLE: return UARTParity::NONE;
            case UART_PARITY_EVEN: return UARTParity::EVEN;
            case UART_PARITY_ODD: return UARTParity::ODD;
            default: return UARTParity::NONE;
        }
    }
    
    /**
     * @brief Convert ESP-IDF flow control to emulator flow control
     */
    UARTFlowControl convert_flow_control(uart_hw_flowcontrol_t esp_flow_ctrl) {
        switch (esp_flow_ctrl) {
            case UART_HW_FLOWCTRL_DISABLE: return UARTFlowControl::NONE;
            case UART_HW_FLOWCTRL_RTS:
            case UART_HW_FLOWCTRL_CTS:
            case UART_HW_FLOWCTRL_CTS_RTS: return UARTFlowControl::RTS_CTS;
            default: return UARTFlowControl::NONE;
        }
    }
    
    /**
     * @brief Validate UART port number
     */
    bool is_valid_uart_num(uart_port_t uart_num) {
        return (uart_num >= 0 && uart_num < UART_NUM_MAX);
    }
    
    /**
     * @brief Get or create UART driver instance
     */
    UARTDriverInstance* get_uart_instance(uart_port_t uart_num) {
        if (!is_valid_uart_num(uart_num)) {
            return nullptr;
        }
        
        return &uart_instances[uart_num];
    }
    
    /**
     * @brief Check if pattern is detected in buffer
     */
    void check_pattern_detection(uart_port_t uart_num, const uint8_t* data, size_t len) {
        auto instance = get_uart_instance(uart_num);
        if (!instance || !instance->pattern_detection_enabled) {
            return;
        }
        
        // Simple pattern detection - look for consecutive pattern characters
        int consecutive_count = 0;
        for (size_t i = 0; i < len; i++) {
            if (data[i] == static_cast<uint8_t>(instance->pattern_char)) {
                consecutive_count++;
                if (consecutive_count >= instance->pattern_count) {
                    // Pattern detected
                    int position = instance->rx_buffer.size() + i - instance->pattern_count + 1;
                    instance->pattern_positions.push_back(position);
                    LOG_DEBUG("UART {} pattern detected at position {}", uart_num, position);
                    consecutive_count = 0; // Reset for next pattern
                }
            } else {
                consecutive_count = 0;
            }
        }
    }
    
    /**
     * @brief Simulate buffer operations with ring buffer (for future use)
     */
    [[maybe_unused]] size_t ring_buffer_available_read(const UARTDriverInstance* instance) {
        if (instance->rx_head >= instance->rx_tail) {
            return instance->rx_head - instance->rx_tail;
        } else {
            return instance->rx_buffer_size - instance->rx_tail + instance->rx_head;
        }
    }
    
    [[maybe_unused]] size_t ring_buffer_available_write(const UARTDriverInstance* instance) {
        return instance->tx_buffer_size - ((instance->tx_head - instance->tx_tail + instance->tx_buffer_size) % instance->tx_buffer_size) - 1;
    }
}

// ============================================================================
// ESP-IDF UART Driver API Implementation
// ============================================================================

extern "C" {

esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, 
                              int queue_size, QueueHandle_t* uart_queue, int intr_alloc_flags) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_driver_install: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (rx_buffer_size <= 0 || tx_buffer_size < 0) {
        LOG_ERROR("uart_driver_install: invalid buffer sizes rx={} tx={}", rx_buffer_size, tx_buffer_size);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_driver_install: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (instance->installed) {
        LOG_ERROR("uart_driver_install: UART {} driver already installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_INFO("uart_driver_install: installing UART {} driver (rx_buf={}, tx_buf={}, queue={})", 
             uart_num, rx_buffer_size, tx_buffer_size, queue_size);
    
    // Initialize instance
    instance->installed = true;
    instance->rx_buffer_size = rx_buffer_size;
    instance->tx_buffer_size = tx_buffer_size;
    instance->controller = controller;
    instance->rx_buffer.resize(rx_buffer_size);
    instance->tx_buffer.resize(tx_buffer_size > 0 ? tx_buffer_size : 0);
    instance->rx_head = instance->rx_tail = 0;
    instance->tx_head = instance->tx_tail = 0;
    
    // Create event queue if requested
    if (uart_queue && queue_size > 0) {
        // In a real implementation, we would create a FreeRTOS queue
        // For emulation, we'll just set a placeholder
        instance->event_queue = reinterpret_cast<QueueHandle_t>(0x12345600 + uart_num);
        *uart_queue = instance->event_queue;
        LOG_DEBUG("uart_driver_install: created event queue for UART {}", uart_num);
    }
    
    // Set default configuration
    instance->config.baud_rate = 115200;
    instance->config.data_bits = UART_DATA_8_BITS;
    instance->config.parity = UART_PARITY_DISABLE;
    instance->config.stop_bits = UART_STOP_BITS_1;
    instance->config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    instance->config.rx_flow_ctrl_thresh = 122;
    
    LOG_INFO("uart_driver_install: UART {} driver installed successfully", uart_num);
    return ESP_OK;
}

esp_err_t uart_driver_delete(uart_port_t uart_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_driver_delete: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_driver_delete: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_INFO("uart_driver_delete: deleting UART {} driver", uart_num);
    
    // Clean up instance
    instance->installed = false;
    instance->controller = nullptr;
    instance->event_queue = nullptr;
    instance->rx_buffer.clear();
    instance->tx_buffer.clear();
    instance->pattern_positions.clear();
    instance->rx_head = instance->rx_tail = 0;
    instance->tx_head = instance->tx_tail = 0;
    
    LOG_INFO("uart_driver_delete: UART {} driver deleted successfully", uart_num);
    return ESP_OK;
}

esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t* uart_config) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_param_config: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!uart_config) {
        LOG_ERROR("uart_param_config: null configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_param_config: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    
    LOG_INFO("uart_param_config: configuring UART {} - baud={}, data={}, parity={}, stop={}, flow={}", 
             uart_num, uart_config->baud_rate, static_cast<int>(uart_config->data_bits),
             static_cast<int>(uart_config->parity), static_cast<int>(uart_config->stop_bits),
             static_cast<int>(uart_config->flow_ctrl));
    
    // Convert ESP-IDF parameters to emulator parameters
    auto data_bits = convert_data_bits(uart_config->data_bits);
    auto stop_bits = convert_stop_bits(uart_config->stop_bits);
    auto parity = convert_parity(uart_config->parity);
    auto flow_ctrl = convert_flow_control(uart_config->flow_ctrl);
    
    // Configure the UART controller
    auto result = controller->configure(uart_config->baud_rate, data_bits, stop_bits, parity, flow_ctrl);
    if (!result.has_value()) {
        LOG_ERROR("uart_param_config: failed to configure UART {}", uart_num);
        return ESP_FAIL;
    }
    
    // Cache configuration
    instance->config = *uart_config;
    instance->baudrate_cached = uart_config->baud_rate;
    
    return ESP_OK;
}

esp_err_t uart_intr_config(uart_port_t uart_num, const uart_intr_config_t* intr_conf) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_intr_config: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!intr_conf) {
        LOG_ERROR("uart_intr_config: null interrupt configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("uart_intr_config: configuring UART {} interrupts (mask=0x{:x})", uart_num, intr_conf->intr_enable_mask);
    
    // In a real implementation, this would configure hardware interrupts
    // For emulation, we just log the configuration
    return ESP_OK;
}

esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_pin: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("uart_set_pin: UART {} pins - TX={}, RX={}, RTS={}, CTS={}", 
              uart_num, tx_io_num, rx_io_num, rts_io_num, cts_io_num);
    
    // In a real implementation, this would configure GPIO pins for UART
    // For M5Stack Tab5, UART pins are typically fixed:
    // UART0: USB-Serial (CP2102N) - internal connections
    // UART1: RS-485 - typically GPIO pins for industrial communication
    // UART2: General purpose - configurable GPIO pins
    
    return ESP_OK;
}

esp_err_t uart_set_rts(uart_port_t uart_num, int level) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_rts: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_set_rts: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    LOG_DEBUG("uart_set_rts: UART {} RTS = {}", uart_num, level);
    
    auto result = controller->set_rts(level != 0);
    if (!result.has_value()) {
        LOG_ERROR("uart_set_rts: failed to set RTS for UART {}", uart_num);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t uart_set_dtr(uart_port_t uart_num, int level) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_dtr: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_set_dtr: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    LOG_DEBUG("uart_set_dtr: UART {} DTR = {}", uart_num, level);
    
    auto result = controller->set_dtr(level != 0);
    if (!result.has_value()) {
        LOG_ERROR("uart_set_dtr: failed to set DTR for UART {}", uart_num);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_baudrate: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_set_baudrate: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    
    LOG_DEBUG("uart_set_baudrate: UART {} baud rate = {}", uart_num, baudrate);
    
    // Reconfigure with new baud rate
    auto data_bits = convert_data_bits(instance->config.data_bits);
    auto stop_bits = convert_stop_bits(instance->config.stop_bits);
    auto parity = convert_parity(instance->config.parity);
    auto flow_ctrl = convert_flow_control(instance->config.flow_ctrl);
    
    auto result = controller->configure(baudrate, data_bits, stop_bits, parity, flow_ctrl);
    if (!result.has_value()) {
        LOG_ERROR("uart_set_baudrate: failed to set baud rate for UART {}", uart_num);
        return ESP_FAIL;
    }
    
    // Update cached values
    instance->config.baud_rate = baudrate;
    instance->baudrate_cached = baudrate;
    
    return ESP_OK;
}

esp_err_t uart_get_baudrate(uart_port_t uart_num, uint32_t* baudrate) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_get_baudrate: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!baudrate) {
        LOG_ERROR("uart_get_baudrate: null baudrate pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    *baudrate = instance->baudrate_cached;
    
    LOG_DEBUG("uart_get_baudrate: UART {} baud rate = {}", uart_num, *baudrate);
    return ESP_OK;
}

int uart_write_bytes(uart_port_t uart_num, const void* src, size_t size) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_write_bytes: invalid UART port {}", uart_num);
        return -1;
    }
    
    if (!src || size == 0) {
        return 0;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_write_bytes: UART controller {} not available", uart_num);
        return -1;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_write_bytes: UART {} driver not installed", uart_num);
        return -1;
    }
    
    LOG_DEBUG("uart_write_bytes: UART {} writing {} bytes", uart_num, size);
    
    // Convert to vector for controller
    const uint8_t* data = static_cast<const uint8_t*>(src);
    std::vector<uint8_t> buffer(data, data + size);
    
    auto result = controller->send_data(buffer);
    if (!result.has_value()) {
        LOG_ERROR("uart_write_bytes: failed to send data on UART {}", uart_num);
        return -1;
    }
    
    return static_cast<int>(size);
}

int uart_write_bytes_with_break(uart_port_t uart_num, const void* src, size_t size, int brk_len) {
    // First write the data
    int bytes_written = uart_write_bytes(uart_num, src, size);
    if (bytes_written < 0) {
        return bytes_written;
    }
    
    // Then send break signal
    UARTController* controller = get_uart_controller(uart_num);
    if (controller && brk_len > 0) {
        LOG_DEBUG("uart_write_bytes_with_break: UART {} sending break for {} bit times", uart_num, brk_len);
        // Convert bit times to milliseconds (approximate)
        auto instance = get_uart_instance(uart_num);
        uint32_t break_duration_ms = (brk_len * 1000) / instance->baudrate_cached;
        if (break_duration_ms == 0) break_duration_ms = 1;
        
        controller->send_break(break_duration_ms);
    }
    
    return bytes_written;
}

int uart_read_bytes(uart_port_t uart_num, void* buf, uint32_t length, TickType_t ticks_to_wait) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_read_bytes: invalid UART port {}", uart_num);
        return -1;
    }
    
    if (!buf || length == 0) {
        return 0;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_read_bytes: UART controller {} not available", uart_num);
        return -1;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_read_bytes: UART {} driver not installed", uart_num);
        return -1;
    }
    
    LOG_DEBUG("uart_read_bytes: UART {} reading up to {} bytes (timeout={})", uart_num, length, ticks_to_wait);
    
    // For simplicity, we'll do a blocking read with timeout
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_ms = std::chrono::milliseconds(ticks_to_wait); // Simplified tick conversion
    
    uint8_t* output_buf = static_cast<uint8_t*>(buf);
    uint32_t total_read = 0;
    
    while (total_read < length) {
        auto data_result = controller->receive_data(length - total_read);
        if (data_result.has_value() && !data_result.value().empty()) {
            auto data = data_result.value();
            size_t copy_size = std::min(static_cast<size_t>(length - total_read), data.size());
            std::memcpy(output_buf + total_read, data.data(), copy_size);
            total_read += copy_size;
            
            // Check for pattern detection
            check_pattern_detection(uart_num, data.data(), copy_size);
            
            if (total_read >= length) {
                break; // Got enough data
            }
        }
        
        // Check timeout
        if (ticks_to_wait != portMAX_DELAY) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout_ms) {
                break; // Timeout
            }
            
            // Small delay to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    LOG_DEBUG("uart_read_bytes: UART {} read {} bytes", uart_num, total_read);
    return static_cast<int>(total_read);
}

esp_err_t uart_flush(uart_port_t uart_num) {
    return uart_flush_input(uart_num);
}

esp_err_t uart_flush_input(uart_port_t uart_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_flush_input: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_flush_input: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_flush_input: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_flush_input: flushing UART {} input", uart_num);
    
    // Clear any pending data
    controller->receive_data(0); // Read all available data to clear
    
    // Reset ring buffer pointers
    instance->rx_head = instance->rx_tail = 0;
    
    return ESP_OK;
}

esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t* size) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_get_buffered_data_len: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!size) {
        LOG_ERROR("uart_get_buffered_data_len: null size pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_get_buffered_data_len: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_get_buffered_data_len: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get buffered data length from controller
    *size = controller->get_rx_data_count();
    
    LOG_DEBUG("uart_get_buffered_data_len: UART {} has {} bytes buffered", uart_num, *size);
    return ESP_OK;
}

esp_err_t uart_disable_pattern_det_intr(uart_port_t uart_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_disable_pattern_det_intr: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_disable_pattern_det_intr: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_disable_pattern_det_intr: disabling pattern detection for UART {}", uart_num);
    
    instance->pattern_detection_enabled = false;
    instance->pattern_positions.clear();
    
    return ESP_OK;
}

esp_err_t uart_enable_pattern_det_baud_intr(uart_port_t uart_num, char pattern_chr, uint8_t chr_num, 
                                           int chr_tout, int post_idle, int pre_idle) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_enable_pattern_det_baud_intr: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_enable_pattern_det_baud_intr: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_enable_pattern_det_baud_intr: enabling pattern detection for UART {} (char='{}', count={})", 
              uart_num, pattern_chr, chr_num);
    
    instance->pattern_detection_enabled = true;
    instance->pattern_char = pattern_chr;
    instance->pattern_count = chr_num;
    instance->pattern_positions.clear();
    
    return ESP_OK;
}

int uart_pattern_pop_pos(uart_port_t uart_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_pattern_pop_pos: invalid UART port {}", uart_num);
        return -1;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_pattern_pop_pos: UART {} driver not installed", uart_num);
        return -1;
    }
    
    if (instance->pattern_positions.empty()) {
        return -1; // No pattern found
    }
    
    int pos = instance->pattern_positions.front();
    instance->pattern_positions.erase(instance->pattern_positions.begin());
    
    LOG_DEBUG("uart_pattern_pop_pos: UART {} popped pattern position {}", uart_num, pos);
    return pos;
}

int uart_pattern_get_pos(uart_port_t uart_num) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_pattern_get_pos: invalid UART port {}", uart_num);
        return -1;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_pattern_get_pos: UART {} driver not installed", uart_num);
        return -1;
    }
    
    if (instance->pattern_positions.empty()) {
        return -1; // No pattern found
    }
    
    int pos = instance->pattern_positions.front();
    LOG_DEBUG("uart_pattern_get_pos: UART {} pattern position {}", uart_num, pos);
    return pos;
}

esp_err_t uart_pattern_queue_reset(uart_port_t uart_num, int queue_length) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_pattern_queue_reset: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_pattern_queue_reset: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_pattern_queue_reset: resetting pattern queue for UART {} (length={})", uart_num, queue_length);
    
    instance->pattern_positions.clear();
    instance->pattern_positions.reserve(queue_length);
    
    return ESP_OK;
}

esp_err_t uart_set_mode(uart_port_t uart_num, uart_mode_t mode) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_mode: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_set_mode: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_set_mode: setting UART {} mode to {}", uart_num, static_cast<int>(mode));
    
    instance->mode = mode;
    
    // For M5Stack Tab5, UART1 supports RS-485 mode
    if (uart_num == UART_NUM_1 && (mode == UART_MODE_RS485_HALF_DUPLEX || mode == UART_MODE_RS485_COLLISION_DETECT)) {
        LOG_INFO("uart_set_mode: UART1 configured for RS-485 mode");
    }
    
    return ESP_OK;
}

esp_err_t uart_set_rx_full_threshold(uart_port_t uart_num, int threshold) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_rx_full_threshold: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_set_rx_full_threshold: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_set_rx_full_threshold: setting UART {} RX threshold to {}", uart_num, threshold);
    // In emulation, this is just cached
    
    return ESP_OK;
}

esp_err_t uart_set_tx_empty_threshold(uart_port_t uart_num, int threshold) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_tx_empty_threshold: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_set_tx_empty_threshold: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_set_tx_empty_threshold: setting UART {} TX threshold to {}", uart_num, threshold);
    // In emulation, this is just cached
    
    return ESP_OK;
}

esp_err_t uart_set_rx_timeout(uart_port_t uart_num, const uint8_t tout_thresh) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_rx_timeout: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_set_rx_timeout: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_set_rx_timeout: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_set_rx_timeout: setting UART {} RX timeout to {}", uart_num, tout_thresh);
    
    // Convert timeout threshold to milliseconds (simplified)
    uint32_t timeout_ms = (tout_thresh * 1000) / (instance->baudrate_cached / 8);
    controller->set_timeout(timeout_ms);
    
    return ESP_OK;
}

esp_err_t uart_get_collision_flag(uart_port_t uart_num, bool* collision_flag) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_get_collision_flag: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!collision_flag) {
        LOG_ERROR("uart_get_collision_flag: null collision flag pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_get_collision_flag: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    *collision_flag = instance->collision_detected;
    LOG_DEBUG("uart_get_collision_flag: UART {} collision flag = {}", uart_num, *collision_flag);
    
    // Clear collision flag after reading
    instance->collision_detected = false;
    
    return ESP_OK;
}

esp_err_t uart_set_wakeup_threshold(uart_port_t uart_num, int wakeup_threshold) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_set_wakeup_threshold: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_set_wakeup_threshold: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_set_wakeup_threshold: setting UART {} wakeup threshold to {}", uart_num, wakeup_threshold);
    // In emulation, this is just logged
    
    return ESP_OK;
}

esp_err_t uart_get_wakeup_threshold(uart_port_t uart_num, int* out_thresh) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_get_wakeup_threshold: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!out_thresh) {
        LOG_ERROR("uart_get_wakeup_threshold: null threshold pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_get_wakeup_threshold: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    *out_thresh = 0; // Default in emulation
    LOG_DEBUG("uart_get_wakeup_threshold: UART {} wakeup threshold = {}", uart_num, *out_thresh);
    
    return ESP_OK;
}

esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait) {
    if (!is_valid_uart_num(uart_num)) {
        LOG_ERROR("uart_wait_tx_done: invalid UART port {}", uart_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    UARTController* controller = get_uart_controller(uart_num);
    if (!controller) {
        LOG_ERROR("uart_wait_tx_done: UART controller {} not available", uart_num);
        return ESP_FAIL;
    }
    
    auto instance = get_uart_instance(uart_num);
    if (!instance->installed) {
        LOG_ERROR("uart_wait_tx_done: UART {} driver not installed", uart_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("uart_wait_tx_done: waiting for UART {} transmission complete (timeout={})", uart_num, ticks_to_wait);
    
    // Wait for transmission to complete
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_ms = std::chrono::milliseconds(ticks_to_wait);
    
    while (controller->is_tx_busy()) {
        if (ticks_to_wait != portMAX_DELAY) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout_ms) {
                LOG_DEBUG("uart_wait_tx_done: UART {} transmission timeout", uart_num);
                return ESP_ERR_TIMEOUT;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    LOG_DEBUG("uart_wait_tx_done: UART {} transmission complete", uart_num);
    return ESP_OK;
}

int uart_tx_chars(uart_port_t uart_num, const char* buffer, uint32_t len) {
    return uart_write_bytes(uart_num, buffer, len);
}

int uart_tx_chars_with_break(uart_port_t uart_num, const char* buffer, uint32_t len, int brk_len) {
    return uart_write_bytes_with_break(uart_num, buffer, len, brk_len);
}

} // extern "C"