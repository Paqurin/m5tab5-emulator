/**
 * @file test_esp_idf_uart.cpp
 * @brief Comprehensive tests for ESP-IDF UART driver API implementation
 * 
 * Tests the complete ESP-IDF UART driver API for M5Stack Tab5 emulator,
 * covering all major functionality including driver installation, configuration,
 * data transfer, flow control, pattern detection, and M5Stack specific features.
 */

#include <gtest/gtest.h>
#include "emulator/esp_idf/driver/uart.h"
#include "emulator/esp_idf/esp_types.h"
#include <string>
#include <vector>
#include <thread>
#include <chrono>

class ESPIDFUARTTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Clean up any existing driver instances
        for (int i = 0; i < UART_NUM_MAX; i++) {
            uart_driver_delete(static_cast<uart_port_t>(i));
        }
    }
    
    void TearDown() override {
        // Clean up after each test
        for (int i = 0; i < UART_NUM_MAX; i++) {
            uart_driver_delete(static_cast<uart_port_t>(i));
        }
    }
};

/**
 * @brief Test UART driver installation and deletion
 */
TEST_F(ESPIDFUARTTest, DriverInstallationAndDeletion) {
    uart_port_t uart_num = UART_NUM_0;
    int rx_buffer_size = 1024;
    int tx_buffer_size = 1024;
    int queue_size = 20;
    QueueHandle_t uart_queue = nullptr;
    
    // Test successful installation
    esp_err_t result = uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 
                                          queue_size, &uart_queue, 0);
    EXPECT_EQ(result, ESP_OK);
    EXPECT_NE(uart_queue, nullptr);
    
    // Test duplicate installation fails
    result = uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 
                                queue_size, nullptr, 0);
    EXPECT_EQ(result, ESP_ERR_INVALID_STATE);
    
    // Test successful deletion
    result = uart_driver_delete(uart_num);
    EXPECT_EQ(result, ESP_OK);
    
    // Test deletion of non-installed driver
    result = uart_driver_delete(uart_num);
    EXPECT_EQ(result, ESP_ERR_INVALID_STATE);
}

/**
 * @brief Test UART parameter configuration
 */
TEST_F(ESPIDFUARTTest, ParameterConfiguration) {
    uart_port_t uart_num = UART_NUM_1;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test valid configuration
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    
    result = uart_param_config(uart_num, &uart_config);
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid UART number
    result = uart_param_config(static_cast<uart_port_t>(-1), &uart_config);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    // Test null configuration
    result = uart_param_config(uart_num, nullptr);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    // Test different configurations
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_7_BITS;
    uart_config.parity = UART_PARITY_EVEN;
    uart_config.stop_bits = UART_STOP_BITS_2;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
    
    result = uart_param_config(uart_num, &uart_config);
    EXPECT_EQ(result, ESP_OK);
}

/**
 * @brief Test UART pin configuration
 */
TEST_F(ESPIDFUARTTest, PinConfiguration) {
    uart_port_t uart_num = UART_NUM_2;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test pin configuration - M5Stack Tab5 typical pins
    result = uart_set_pin(uart_num, 43, 44, -1, -1); // TX=43, RX=44, no flow control
    EXPECT_EQ(result, ESP_OK);
    
    // Test with flow control pins
    result = uart_set_pin(uart_num, 43, 44, 45, 46); // TX=43, RX=44, RTS=45, CTS=46
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid UART number
    result = uart_set_pin(static_cast<uart_port_t>(-1), 43, 44, -1, -1);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}

/**
 * @brief Test UART baud rate configuration
 */
TEST_F(ESPIDFUARTTest, BaudRateConfiguration) {
    uart_port_t uart_num = UART_NUM_0;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test setting different baud rates
    std::vector<uint32_t> baud_rates = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    
    for (uint32_t baud : baud_rates) {
        result = uart_set_baudrate(uart_num, baud);
        EXPECT_EQ(result, ESP_OK);
        
        uint32_t read_baud = 0;
        result = uart_get_baudrate(uart_num, &read_baud);
        EXPECT_EQ(result, ESP_OK);
        EXPECT_EQ(read_baud, baud);
    }
    
    // Test invalid parameters
    result = uart_set_baudrate(static_cast<uart_port_t>(-1), 115200);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    result = uart_get_baudrate(uart_num, nullptr);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}

/**
 * @brief Test UART data transmission
 */
TEST_F(ESPIDFUARTTest, DataTransmission) {
    uart_port_t uart_num = UART_NUM_0;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    
    result = uart_param_config(uart_num, &uart_config);
    EXPECT_EQ(result, ESP_OK);
    
    // Test writing data
    const char* test_data = "Hello, M5Stack Tab5 UART!";
    int bytes_written = uart_write_bytes(uart_num, test_data, strlen(test_data));
    EXPECT_EQ(bytes_written, static_cast<int>(strlen(test_data)));
    
    // Test writing with break
    bytes_written = uart_write_bytes_with_break(uart_num, test_data, strlen(test_data), 100);
    EXPECT_EQ(bytes_written, static_cast<int>(strlen(test_data)));
    
    // Test wait for transmission done
    result = uart_wait_tx_done(uart_num, pdMS_TO_TICKS(1000));
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid parameters
    bytes_written = uart_write_bytes(static_cast<uart_port_t>(-1), test_data, strlen(test_data));
    EXPECT_EQ(bytes_written, -1);
    
    bytes_written = uart_write_bytes(uart_num, nullptr, 10);
    EXPECT_EQ(bytes_written, 0);
}

/**
 * @brief Test UART data reception
 */
TEST_F(ESPIDFUARTTest, DataReception) {
    uart_port_t uart_num = UART_NUM_1;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    
    result = uart_param_config(uart_num, &uart_config);
    EXPECT_EQ(result, ESP_OK);
    
    // Test reading with timeout (should timeout with no data)
    char read_buffer[100];
    int bytes_read = uart_read_bytes(uart_num, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(10));
    EXPECT_GE(bytes_read, 0); // May be 0 if no data available
    
    // Test buffered data length
    size_t buffered_len = 0;
    result = uart_get_buffered_data_len(uart_num, &buffered_len);
    EXPECT_EQ(result, ESP_OK);
    
    // Test flush operations
    result = uart_flush_input(uart_num);
    EXPECT_EQ(result, ESP_OK);
    
    result = uart_flush(uart_num);
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid parameters
    bytes_read = uart_read_bytes(static_cast<uart_port_t>(-1), read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(10));
    EXPECT_EQ(bytes_read, -1);
    
    bytes_read = uart_read_bytes(uart_num, nullptr, 10, pdMS_TO_TICKS(10));
    EXPECT_EQ(bytes_read, 0);
}

/**
 * @brief Test UART flow control signals
 */
TEST_F(ESPIDFUARTTest, FlowControlSignals) {
    uart_port_t uart_num = UART_NUM_2;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test RTS control
    result = uart_set_rts(uart_num, 1);
    EXPECT_EQ(result, ESP_OK);
    
    result = uart_set_rts(uart_num, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test DTR control
    result = uart_set_dtr(uart_num, 1);
    EXPECT_EQ(result, ESP_OK);
    
    result = uart_set_dtr(uart_num, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid UART number
    result = uart_set_rts(static_cast<uart_port_t>(-1), 1);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    result = uart_set_dtr(static_cast<uart_port_t>(-1), 1);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}

/**
 * @brief Test UART pattern detection
 */
TEST_F(ESPIDFUARTTest, PatternDetection) {
    uart_port_t uart_num = UART_NUM_0;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test pattern queue reset
    result = uart_pattern_queue_reset(uart_num, 10);
    EXPECT_EQ(result, ESP_OK);
    
    // Test enable pattern detection
    result = uart_enable_pattern_det_baud_intr(uart_num, '+', 3, 10, 10, 10);
    EXPECT_EQ(result, ESP_OK);
    
    // Test pattern position functions (should return -1 with no patterns)
    int pos = uart_pattern_get_pos(uart_num);
    EXPECT_EQ(pos, -1);
    
    pos = uart_pattern_pop_pos(uart_num);
    EXPECT_EQ(pos, -1);
    
    // Test disable pattern detection
    result = uart_disable_pattern_det_intr(uart_num);
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid parameters
    result = uart_enable_pattern_det_baud_intr(static_cast<uart_port_t>(-1), '+', 3, 10, 10, 10);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    pos = uart_pattern_get_pos(static_cast<uart_port_t>(-1));
    EXPECT_EQ(pos, -1);
}

/**
 * @brief Test UART modes including RS-485
 */
TEST_F(ESPIDFUARTTest, UARTModes) {
    uart_port_t uart_num = UART_NUM_1; // UART1 supports RS-485 on M5Stack Tab5
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test standard UART mode
    result = uart_set_mode(uart_num, UART_MODE_UART);
    EXPECT_EQ(result, ESP_OK);
    
    // Test RS-485 half duplex mode
    result = uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
    EXPECT_EQ(result, ESP_OK);
    
    // Test RS-485 collision detection mode
    result = uart_set_mode(uart_num, UART_MODE_RS485_COLLISION_DETECT);
    EXPECT_EQ(result, ESP_OK);
    
    // Test collision flag
    bool collision_flag = false;
    result = uart_get_collision_flag(uart_num, &collision_flag);
    EXPECT_EQ(result, ESP_OK);
    EXPECT_FALSE(collision_flag); // Should be false initially
    
    // Test invalid parameters
    result = uart_set_mode(static_cast<uart_port_t>(-1), UART_MODE_UART);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    result = uart_get_collision_flag(uart_num, nullptr);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}

/**
 * @brief Test UART interrupt and threshold configuration
 */
TEST_F(ESPIDFUARTTest, InterruptAndThresholdConfiguration) {
    uart_port_t uart_num = UART_NUM_0;
    
    // Install driver first
    esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
    EXPECT_EQ(result, ESP_OK);
    
    // Test interrupt configuration
    uart_intr_config_t intr_conf = {
        .intr_enable_mask = 0xFF,
        .rx_timeout_thresh = 10,
        .txfifo_empty_intr_thresh = 10,
        .rxfifo_full_thresh = 112
    };
    
    result = uart_intr_config(uart_num, &intr_conf);
    EXPECT_EQ(result, ESP_OK);
    
    // Test threshold configuration
    result = uart_set_rx_full_threshold(uart_num, 100);
    EXPECT_EQ(result, ESP_OK);
    
    result = uart_set_tx_empty_threshold(uart_num, 10);
    EXPECT_EQ(result, ESP_OK);
    
    result = uart_set_rx_timeout(uart_num, 10);
    EXPECT_EQ(result, ESP_OK);
    
    // Test wakeup threshold
    result = uart_set_wakeup_threshold(uart_num, 5);
    EXPECT_EQ(result, ESP_OK);
    
    int wakeup_thresh = 0;
    result = uart_get_wakeup_threshold(uart_num, &wakeup_thresh);
    EXPECT_EQ(result, ESP_OK);
    
    // Test invalid parameters
    result = uart_intr_config(uart_num, nullptr);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    result = uart_get_wakeup_threshold(uart_num, nullptr);
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}

/**
 * @brief Test all UART ports (M5Stack Tab5 specific)
 */
TEST_F(ESPIDFUARTTest, AllUARTPorts) {
    // Test all available UART ports
    for (int i = 0; i < UART_NUM_MAX; i++) {
        uart_port_t uart_num = static_cast<uart_port_t>(i);
        
        // Install driver
        esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
        EXPECT_EQ(result, ESP_OK);
        
        // Configure basic parameters
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122
        };
        
        result = uart_param_config(uart_num, &uart_config);
        EXPECT_EQ(result, ESP_OK);
        
        // Test basic write operation
        const char* test_msg = "Test message";
        int bytes_written = uart_write_bytes(uart_num, test_msg, strlen(test_msg));
        EXPECT_EQ(bytes_written, static_cast<int>(strlen(test_msg)));
        
        // Clean up
        result = uart_driver_delete(uart_num);
        EXPECT_EQ(result, ESP_OK);
    }
}

/**
 * @brief Test error conditions and edge cases
 */
TEST_F(ESPIDFUARTTest, ErrorConditionsAndEdgeCases) {
    // Test operations on non-installed driver
    uart_port_t uart_num = UART_NUM_0;
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    
    // These should fail without driver installed
    esp_err_t result = uart_param_config(uart_num, &uart_config);
    EXPECT_NE(result, ESP_OK);
    
    int bytes_written = uart_write_bytes(uart_num, "test", 4);
    EXPECT_EQ(bytes_written, -1);
    
    char buffer[10];
    int bytes_read = uart_read_bytes(uart_num, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
    EXPECT_EQ(bytes_read, -1);
    
    // Test invalid buffer sizes during installation
    result = uart_driver_install(uart_num, 0, -1, 10, nullptr, 0); // Invalid sizes
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
    
    result = uart_driver_install(uart_num, -1, 1024, 10, nullptr, 0); // Invalid sizes
    EXPECT_EQ(result, ESP_ERR_INVALID_ARG);
}