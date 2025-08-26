/**
 * @file uart_industrial_demo.cpp
 * @brief M5Stack Tab5 Industrial UART Communication Demo
 * 
 * This example demonstrates the complete UART functionality of the M5Stack Tab5
 * emulator, showcasing industrial IoT communication patterns including:
 * 
 * - USB-Serial debugging (UART0)
 * - RS-485 industrial communication (UART1) 
 * - General purpose sensor communication (UART2)
 * 
 * Features demonstrated:
 * - Multi-UART configuration and operation
 * - RS-485 collision detection and flow control
 * - Pattern detection for AT commands
 * - Industrial protocol handling
 * - Error handling and recovery
 */

#include "emulator/esp_idf/esp_idf.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>

// Industrial communication protocols
namespace Industrial {
    
    /**
     * @brief Modbus RTU frame structure
     */
    struct ModbusFrame {
        uint8_t slave_addr;
        uint8_t function_code;
        std::vector<uint8_t> data;
        uint16_t crc;
        
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> frame;
            frame.push_back(slave_addr);
            frame.push_back(function_code);
            frame.insert(frame.end(), data.begin(), data.end());
            frame.push_back(crc & 0xFF);
            frame.push_back((crc >> 8) & 0xFF);
            return frame;
        }
    };
    
    /**
     * @brief Calculate Modbus CRC16
     */
    uint16_t calculate_crc16(const std::vector<uint8_t>& data) {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }
}

/**
 * @brief Debug output helper
 */
class Logger {
public:
    static void info(const std::string& message) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "] INFO: " << message << std::endl;
    }
    
    static void error(const std::string& message) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "] ERROR: " << message << std::endl;
    }
    
    static void hex_dump(const std::string& label, const std::vector<uint8_t>& data) {
        std::cout << label << ": ";
        for (uint8_t byte : data) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
    }
};

/**
 * @brief USB-Serial Debug Console (UART0)
 */
class DebugConsole {
private:
    uart_port_t uart_num = UART_NUM_0;
    bool initialized = false;
    
public:
    esp_err_t initialize() {
        Logger::info("Initializing USB-Serial debug console (UART0)");
        
        // Install UART driver
        esp_err_t result = uart_driver_install(uart_num, 2048, 2048, 20, nullptr, 0);
        if (result != ESP_OK) {
            Logger::error("Failed to install UART0 driver");
            return result;
        }
        
        // Configure UART parameters for debug console
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122
        };
        
        result = uart_param_config(uart_num, &uart_config);
        if (result != ESP_OK) {
            Logger::error("Failed to configure UART0 parameters");
            return result;
        }
        
        // Set pins (CP2102N USB-Serial bridge)
        result = uart_set_pin(uart_num, 1, 3, -1, -1); // TX=1, RX=3
        if (result != ESP_OK) {
            Logger::error("Failed to set UART0 pins");
            return result;
        }
        
        initialized = true;
        Logger::info("USB-Serial debug console initialized successfully");
        return ESP_OK;
    }
    
    void print(const std::string& message) {
        if (!initialized) return;
        
        std::string msg_with_newline = message + "\\r\\n";
        uart_write_bytes(uart_num, msg_with_newline.c_str(), msg_with_newline.length());
    }
    
    void shutdown() {
        if (initialized) {
            uart_driver_delete(uart_num);
            initialized = false;
        }
    }
};

/**
 * @brief RS-485 Industrial Communication (UART1)
 */
class RS485Interface {
private:
    uart_port_t uart_num = UART_NUM_1;
    bool initialized = false;
    QueueHandle_t uart_queue = nullptr;
    
public:
    esp_err_t initialize() {
        Logger::info("Initializing RS-485 industrial interface (UART1)");
        
        // Install UART driver with event queue for industrial communication
        esp_err_t result = uart_driver_install(uart_num, 2048, 2048, 20, &uart_queue, 0);
        if (result != ESP_OK) {
            Logger::error("Failed to install UART1 driver");
            return result;
        }
        
        // Configure UART parameters for RS-485
        uart_config_t uart_config = {
            .baud_rate = 9600,    // Common industrial baud rate
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_EVEN,  // Even parity for industrial protocols
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // RS-485 uses half-duplex
            .rx_flow_ctrl_thresh = 122
        };
        
        result = uart_param_config(uart_num, &uart_config);
        if (result != ESP_OK) {
            Logger::error("Failed to configure UART1 parameters");
            return result;
        }
        
        // Set RS-485 mode with collision detection
        result = uart_set_mode(uart_num, UART_MODE_RS485_COLLISION_DETECT);
        if (result != ESP_OK) {
            Logger::error("Failed to set RS-485 mode");
            return result;
        }
        
        // Set pins for RS-485 (MAX485 transceiver)
        result = uart_set_pin(uart_num, 18, 8, -1, -1); // TX=18, RX=8
        if (result != ESP_OK) {
            Logger::error("Failed to set UART1 pins");
            return result;
        }
        
        initialized = true;
        Logger::info("RS-485 industrial interface initialized successfully");
        return ESP_OK;
    }
    
    esp_err_t send_modbus_frame(const Industrial::ModbusFrame& frame) {
        if (!initialized) return ESP_ERR_INVALID_STATE;
        
        auto data = frame.serialize();
        Logger::hex_dump("Sending Modbus frame", data);
        
        int bytes_written = uart_write_bytes(uart_num, data.data(), data.size());
        if (bytes_written != static_cast<int>(data.size())) {
            Logger::error("Failed to send complete Modbus frame");
            return ESP_FAIL;
        }
        
        // Wait for transmission to complete
        esp_err_t result = uart_wait_tx_done(uart_num, pdMS_TO_TICKS(1000));
        if (result != ESP_OK) {
            Logger::error("Timeout waiting for transmission");
            return result;
        }
        
        // Check for collision detection
        bool collision_detected = false;
        uart_get_collision_flag(uart_num, &collision_detected);
        if (collision_detected) {
            Logger::error("RS-485 collision detected during transmission");
            return ESP_ERR_INVALID_RESPONSE;
        }
        
        Logger::info("Modbus frame sent successfully");
        return ESP_OK;
    }
    
    std::vector<uint8_t> receive_response(uint32_t timeout_ms = 1000) {
        if (!initialized) return {};
        
        std::vector<uint8_t> response;
        uint8_t buffer[256];
        
        int bytes_read = uart_read_bytes(uart_num, buffer, sizeof(buffer), pdMS_TO_TICKS(timeout_ms));
        if (bytes_read > 0) {
            response.assign(buffer, buffer + bytes_read);
            Logger::hex_dump("Received response", response);
        } else {
            Logger::error("No response received within timeout");
        }
        
        return response;
    }
    
    void shutdown() {
        if (initialized) {
            uart_driver_delete(uart_num);
            initialized = false;
        }
    }
};

/**
 * @brief General Purpose Sensor Communication (UART2)
 */
class SensorInterface {
private:
    uart_port_t uart_num = UART_NUM_2;
    bool initialized = false;
    
public:
    esp_err_t initialize() {
        Logger::info("Initializing sensor communication interface (UART2)");
        
        // Install UART driver
        esp_err_t result = uart_driver_install(uart_num, 1024, 1024, 10, nullptr, 0);
        if (result != ESP_OK) {
            Logger::error("Failed to install UART2 driver");
            return result;
        }
        
        // Configure for GPS/sensor communication
        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122
        };
        
        result = uart_param_config(uart_num, &uart_config);
        if (result != ESP_OK) {
            Logger::error("Failed to configure UART2 parameters");
            return result;
        }
        
        // Set pins for general purpose UART
        result = uart_set_pin(uart_num, 43, 44, -1, -1); // TX=43, RX=44
        if (result != ESP_OK) {
            Logger::error("Failed to set UART2 pins");
            return result;
        }
        
        // Enable pattern detection for NMEA sentences
        result = uart_enable_pattern_det_baud_intr(uart_num, '\\n', 1, 10, 10, 10);
        if (result != ESP_OK) {
            Logger::error("Failed to enable pattern detection");
            return result;
        }
        
        initialized = true;
        Logger::info("Sensor communication interface initialized successfully");
        return ESP_OK;
    }
    
    esp_err_t send_at_command(const std::string& command) {
        if (!initialized) return ESP_ERR_INVALID_STATE;
        
        std::string cmd_with_crlf = command + "\\r\\n";
        Logger::info("Sending AT command: " + command);
        
        int bytes_written = uart_write_bytes(uart_num, cmd_with_crlf.c_str(), cmd_with_crlf.length());
        if (bytes_written != static_cast<int>(cmd_with_crlf.length())) {
            Logger::error("Failed to send AT command");
            return ESP_FAIL;
        }
        
        return ESP_OK;
    }
    
    std::string receive_nmea_sentence(uint32_t timeout_ms = 5000) {
        if (!initialized) return "";
        
        char buffer[256];
        int bytes_read = uart_read_bytes(uart_num, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(timeout_ms));
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\\0';
            std::string sentence(buffer);
            Logger::info("Received NMEA: " + sentence.substr(0, sentence.find('\\n')));
            return sentence;
        }
        
        Logger::error("No NMEA sentence received within timeout");
        return "";
    }
    
    void shutdown() {
        if (initialized) {
            uart_driver_delete(uart_num);
            initialized = false;
        }
    }
};

/**
 * @brief Main demonstration function
 */
int main() {
    Logger::info("=== M5Stack Tab5 Industrial UART Communication Demo ===");
    
    // Initialize ESP-IDF emulation layer
    esp_err_t result = esp_idf_init_all();
    if (result != ESP_OK) {
        Logger::error("Failed to initialize ESP-IDF emulation");
        return -1;
    }
    
    Logger::info("ESP-IDF emulation initialized successfully");
    
    // Initialize communication interfaces
    DebugConsole debug_console;
    RS485Interface rs485;
    SensorInterface sensor;
    
    // Initialize all interfaces
    if (debug_console.initialize() != ESP_OK) {
        Logger::error("Failed to initialize debug console");
        return -1;
    }
    
    if (rs485.initialize() != ESP_OK) {
        Logger::error("Failed to initialize RS-485 interface");
        return -1;
    }
    
    if (sensor.initialize() != ESP_OK) {
        Logger::error("Failed to initialize sensor interface");
        return -1;
    }
    
    Logger::info("All UART interfaces initialized successfully");
    
    // Demonstrate debug console
    debug_console.print("M5Stack Tab5 Industrial Demo Started");
    debug_console.print("All UART interfaces operational");
    
    // Demonstrate RS-485 Modbus communication
    Logger::info("=== Demonstrating RS-485 Modbus Communication ===");
    
    // Create a Modbus Read Holding Registers request
    Industrial::ModbusFrame modbus_frame;
    modbus_frame.slave_addr = 0x01;        // Slave address 1
    modbus_frame.function_code = 0x03;      // Read Holding Registers
    modbus_frame.data = {0x00, 0x00,        // Starting register address (0)
                        0x00, 0x02};        // Number of registers (2)
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {modbus_frame.slave_addr, modbus_frame.function_code};
    crc_data.insert(crc_data.end(), modbus_frame.data.begin(), modbus_frame.data.end());
    modbus_frame.crc = Industrial::calculate_crc16(crc_data);
    
    // Send Modbus frame
    result = rs485.send_modbus_frame(modbus_frame);
    if (result == ESP_OK) {
        debug_console.print("Modbus request sent successfully");
        
        // Wait for response
        auto response = rs485.receive_response(2000);
        if (!response.empty()) {
            debug_console.print("Modbus response received");
        }
    }
    
    // Demonstrate sensor communication
    Logger::info("=== Demonstrating Sensor Communication ===");
    
    // Send AT command to cellular/WiFi module
    result = sensor.send_at_command("AT+GMR"); // Get version
    if (result == ESP_OK) {
        debug_console.print("AT command sent to sensor module");
    }
    
    // Simulate GPS NMEA sentence reception
    std::thread gps_simulator([&sensor, &debug_console]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Simulate GPS sending NMEA sentence
        std::string nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        Logger::info("Simulating GPS NMEA reception");
        
        auto received = sensor.receive_nmea_sentence(1000);
        if (!received.empty()) {
            debug_console.print("GPS NMEA sentence processed");
        }
    });
    
    gps_simulator.join();
    
    // Demonstrate pattern detection
    Logger::info("=== Demonstrating Pattern Detection ===");
    
    // Pattern detection is useful for protocols like AT commands
    // The UART driver will detect patterns and notify when found
    
    // Run communication loop for a few seconds
    Logger::info("=== Running Communication Loop ===");
    
    for (int i = 0; i < 10; i++) {
        debug_console.print("Loop iteration: " + std::to_string(i + 1));
        
        // Periodic status check
        if (i % 3 == 0) {
            Logger::info("Checking UART status...");
            
            // Check buffered data lengths
            size_t uart0_buffered = 0, uart1_buffered = 0, uart2_buffered = 0;
            uart_get_buffered_data_len(UART_NUM_0, &uart0_buffered);
            uart_get_buffered_data_len(UART_NUM_1, &uart1_buffered);
            uart_get_buffered_data_len(UART_NUM_2, &uart2_buffered);
            
            Logger::info("Buffered data - UART0: " + std::to_string(uart0_buffered) + 
                        " UART1: " + std::to_string(uart1_buffered) + 
                        " UART2: " + std::to_string(uart2_buffered));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Cleanup
    Logger::info("=== Cleaning Up ===");
    
    debug_console.print("Demo completed successfully");
    
    sensor.shutdown();
    rs485.shutdown();
    debug_console.shutdown();
    
    // Shutdown ESP-IDF emulation
    esp_idf_deinit_all();
    
    Logger::info("=== M5Stack Tab5 Industrial UART Demo Completed ===");
    
    return 0;
}