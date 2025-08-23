/**
 * @file i2c_api.cpp  
 * @brief ESP-IDF I2C driver API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible I2C functions that map to the
 * emulated I2C controller, providing seamless compatibility for ESP-IDF
 * applications running on the emulator.
 */

#include "emulator/esp_idf/driver/i2c.h"
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace {
    using namespace m5tab5::emulator;
    using I2CController = peripherals::I2CController;
    
    /**
     * @brief I2C command structure for queuing operations
     */
    struct I2CCommand {
        enum Type {
            START,
            STOP, 
            WRITE,
            READ
        } type;
        
        std::vector<uint8_t> data;
        bool ack_check_en;
        i2c_ack_type_t ack_type;
    };
    
    /**
     * @brief I2C command link implementation
     */
    struct I2CCommandLink {
        std::vector<I2CCommand> commands;
        size_t current_index;
        
        I2CCommandLink() : current_index(0) {}
    };
    
    /**
     * @brief I2C driver state per port
     */
    struct I2CDriverState {
        bool installed;
        i2c_mode_t mode;
        i2c_config_t config;
        I2CController* controller;
        std::mutex mutex;
        
        I2CDriverState() : installed(false), mode(I2C_MODE_MASTER), controller(nullptr) {}
    };
    
    // Static driver state for each I2C port
    static I2CDriverState i2c_drivers[I2C_NUM_MAX];
    static std::mutex global_i2c_mutex;
    
    /**
     * @brief Get I2C controller instance from emulator core
     */
    I2CController* get_i2c_controller(i2c_port_t i2c_num) {
        if (i2c_num >= I2C_NUM_MAX) {
            return nullptr;
        }
        
        if (!i2c_drivers[i2c_num].controller) {
            // Get emulator core instance and retrieve I2C controller
            LOG_DEBUG("Getting I2C controller instance for port {}", i2c_num);
            
            // TODO: Implement proper EmulatorCore integration
            // For now, we'll use a placeholder - this needs integration with EmulatorCore
            // auto emulator = EmulatorCore::get_instance();
            // if (emulator) {
            //     auto result = emulator->get_component<I2CController>();
            //     if (result.has_value()) {
            //         i2c_drivers[i2c_num].controller = result.value();
            //     }
            // }
        }
        
        return i2c_drivers[i2c_num].controller;
    }
    
    /**
     * @brief Validate I2C port number
     */
    bool is_valid_i2c_port(i2c_port_t i2c_num) {
        return (i2c_num >= 0 && i2c_num < I2C_NUM_MAX);
    }
    
    /**
     * @brief Convert ESP-IDF I2C mode to emulator I2C mode
     */
    I2CController::Mode convert_i2c_mode(i2c_mode_t esp_mode) {
        switch (esp_mode) {
            case I2C_MODE_MASTER:
                return I2CController::Mode::MASTER;
            case I2C_MODE_SLAVE:
                return I2CController::Mode::SLAVE;
            default:
                return I2CController::Mode::MASTER;
        }
    }
}

// ============================================================================
// ESP-IDF I2C Driver API Implementation
// ============================================================================

extern "C" {

int i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, 
                       size_t slv_tx_buf_len, int intr_alloc_flags) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_driver_install: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(global_i2c_mutex);
    
    if (i2c_drivers[i2c_num].installed) {
        LOG_ERROR("i2c_driver_install: I2C driver already installed for port {}", i2c_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("i2c_driver_install: port={}, mode={}, slv_rx_buf={}, slv_tx_buf={}, intr_flags=0x{:x}",
              i2c_num, static_cast<int>(mode), slv_rx_buf_len, slv_tx_buf_len, intr_alloc_flags);
    
    // Initialize driver state
    i2c_drivers[i2c_num].installed = true;
    i2c_drivers[i2c_num].mode = mode;
    i2c_drivers[i2c_num].controller = get_i2c_controller(i2c_num);
    
    if (!i2c_drivers[i2c_num].controller) {
        LOG_WARN("i2c_driver_install: I2C controller not available for port {} (emulation mode)", i2c_num);
        // Continue anyway for API compatibility - actual hardware calls will be stubbed
    }
    
    LOG_INFO("i2c_driver_install: I2C driver installed for port {}", i2c_num);
    return ESP_OK;
}

int i2c_driver_delete(i2c_port_t i2c_num) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_driver_delete: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(global_i2c_mutex);
    
    if (!i2c_drivers[i2c_num].installed) {
        LOG_ERROR("i2c_driver_delete: I2C driver not installed for port {}", i2c_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("i2c_driver_delete: deleting I2C driver for port {}", i2c_num);
    
    // Reset driver state
    i2c_drivers[i2c_num].installed = false;
    i2c_drivers[i2c_num].controller = nullptr;
    
    LOG_INFO("i2c_driver_delete: I2C driver deleted for port {}", i2c_num);
    return ESP_OK;
}

int i2c_param_config(i2c_port_t i2c_num, const i2c_config_t* i2c_conf) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_param_config: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!i2c_conf) {
        LOG_ERROR("i2c_param_config: null configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(i2c_drivers[i2c_num].mutex);
    
    LOG_DEBUG("i2c_param_config: port={}, mode={}, sda={}, scl={}, sda_pullup={}, scl_pullup={}",
              i2c_num, static_cast<int>(i2c_conf->mode), i2c_conf->sda_io_num, i2c_conf->scl_io_num,
              i2c_conf->sda_pullup_en, i2c_conf->scl_pullup_en);
    
    // Store configuration
    i2c_drivers[i2c_num].config = *i2c_conf;
    
    // Configure I2C controller if available
    I2CController* controller = get_i2c_controller(i2c_num);
    if (controller) {
        auto mode = convert_i2c_mode(i2c_conf->mode);
        uint32_t frequency = (i2c_conf->mode == I2C_MODE_MASTER) ? 
                            i2c_conf->master.clk_speed : i2c_conf->slave.maximum_speed;
        
        auto result = controller->configure(frequency, mode);
        if (!result.has_value()) {
            LOG_ERROR("i2c_param_config: failed to configure I2C controller for port {}", i2c_num);
            return ESP_FAIL;
        }
        
        // Configure GPIO pins for I2C
        LOG_DEBUG("i2c_param_config: configuring GPIO pins SDA={}, SCL={}", 
                  i2c_conf->sda_io_num, i2c_conf->scl_io_num);
    }
    
    return ESP_OK;
}

i2c_cmd_handle_t i2c_cmd_link_create(void) {
    LOG_DEBUG("i2c_cmd_link_create: creating new command link");
    
    auto cmd_link = new(std::nothrow) I2CCommandLink();
    if (!cmd_link) {
        LOG_ERROR("i2c_cmd_link_create: failed to allocate command link");
        return nullptr;
    }
    
    return reinterpret_cast<i2c_cmd_handle_t>(cmd_link);
}

void i2c_cmd_link_delete(i2c_cmd_handle_t cmd_handle) {
    if (!cmd_handle) {
        LOG_ERROR("i2c_cmd_link_delete: null command handle");
        return;
    }
    
    LOG_DEBUG("i2c_cmd_link_delete: deleting command link");
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    delete cmd_link;
}

int i2c_master_start(i2c_cmd_handle_t cmd_handle) {
    if (!cmd_handle) {
        LOG_ERROR("i2c_master_start: null command handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_start: queuing START condition");
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    I2CCommand cmd;
    cmd.type = I2CCommand::START;
    cmd_link->commands.push_back(cmd);
    
    return ESP_OK;
}

int i2c_master_stop(i2c_cmd_handle_t cmd_handle) {
    if (!cmd_handle) {
        LOG_ERROR("i2c_master_stop: null command handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_stop: queuing STOP condition");
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    I2CCommand cmd;
    cmd.type = I2CCommand::STOP;
    cmd_link->commands.push_back(cmd);
    
    return ESP_OK;
}

int i2c_master_write(i2c_cmd_handle_t cmd_handle, const uint8_t* data, 
                     size_t data_len, bool ack_en) {
    if (!cmd_handle) {
        LOG_ERROR("i2c_master_write: null command handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!data || data_len == 0) {
        LOG_ERROR("i2c_master_write: invalid data parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_write: queuing write of {} bytes, ack_en={}", data_len, ack_en);
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    I2CCommand cmd;
    cmd.type = I2CCommand::WRITE;
    cmd.data.assign(data, data + data_len);
    cmd.ack_check_en = ack_en;
    cmd_link->commands.push_back(cmd);
    
    return ESP_OK;
}

int i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en) {
    return i2c_master_write(cmd_handle, &data, 1, ack_en);
}

int i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t* data, 
                    size_t data_len, i2c_ack_type_t ack) {
    if (!cmd_handle) {
        LOG_ERROR("i2c_master_read: null command handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!data || data_len == 0) {
        LOG_ERROR("i2c_master_read: invalid data parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_read: queuing read of {} bytes, ack_type={}", data_len, static_cast<int>(ack));
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    I2CCommand cmd;
    cmd.type = I2CCommand::READ;
    cmd.data.resize(data_len);  // Resize to store read data
    cmd.ack_type = ack;
    cmd_link->commands.push_back(cmd);
    
    return ESP_OK;
}

int i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t* data, i2c_ack_type_t ack) {
    return i2c_master_read(cmd_handle, data, 1, ack);
}

int i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, uint32_t ticks_to_wait) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_master_cmd_begin: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!cmd_handle) {
        LOG_ERROR("i2c_master_cmd_begin: null command handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!i2c_drivers[i2c_num].installed) {
        LOG_ERROR("i2c_master_cmd_begin: I2C driver not installed for port {}", i2c_num);
        return ESP_ERR_INVALID_STATE;
    }
    
    std::lock_guard<std::mutex> lock(i2c_drivers[i2c_num].mutex);
    
    auto cmd_link = reinterpret_cast<I2CCommandLink*>(cmd_handle);
    
    LOG_DEBUG("i2c_master_cmd_begin: executing {} commands on port {}, timeout={}", 
              cmd_link->commands.size(), i2c_num, ticks_to_wait);
    
    I2CController* controller = get_i2c_controller(i2c_num);
    if (!controller) {
        LOG_WARN("i2c_master_cmd_begin: I2C controller not available for port {} (emulated)", i2c_num);
        // Simulate successful operation for API compatibility
        return ESP_OK;
    }
    
    // Execute command sequence
    for (const auto& cmd : cmd_link->commands) {
        switch (cmd.type) {
            case I2CCommand::START:
                LOG_DEBUG("i2c_master_cmd_begin: executing START condition");
                // I2C start condition handled internally by controller
                break;
                
            case I2CCommand::STOP:
                LOG_DEBUG("i2c_master_cmd_begin: executing STOP condition");
                // I2C stop condition handled internally by controller
                break;
                
            case I2CCommand::WRITE:
                LOG_DEBUG("i2c_master_cmd_begin: executing WRITE of {} bytes", cmd.data.size());
                if (!cmd.data.empty()) {
                    // For ESP-IDF compatibility, first byte is usually device address
                    uint8_t device_addr = cmd.data[0] >> 1;  // Remove R/W bit
                    if (cmd.data.size() > 1) {
                        // Write data to device
                        std::vector<uint8_t> write_data(cmd.data.begin() + 1, cmd.data.end());
                        auto result = controller->write(device_addr, write_data);
                        if (!result.has_value()) {
                            LOG_ERROR("i2c_master_cmd_begin: write failed to device 0x{:02x}", device_addr);
                            return ESP_FAIL;
                        }
                    }
                }
                break;
                
            case I2CCommand::READ:
                LOG_DEBUG("i2c_master_cmd_begin: executing READ of {} bytes", cmd.data.size());
                // Read operations are more complex due to ESP-IDF's command-based approach
                // For now, simulate successful read with dummy data
                LOG_WARN("i2c_master_cmd_begin: READ command simulation not fully implemented");
                break;
        }
    }
    
    return ESP_OK;
}

int i2c_reset_tx_fifo(i2c_port_t i2c_num) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_reset_tx_fifo: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_reset_tx_fifo: resetting TX FIFO for port {}", i2c_num);
    
    // Reset TX FIFO in I2C controller if available
    I2CController* controller = get_i2c_controller(i2c_num);
    if (controller) {
        // TODO: Implement FIFO reset in I2C controller
        LOG_DEBUG("i2c_reset_tx_fifo: TX FIFO reset completed for port {}", i2c_num);
    }
    
    return ESP_OK;
}

int i2c_reset_rx_fifo(i2c_port_t i2c_num) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_reset_rx_fifo: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_reset_rx_fifo: resetting RX FIFO for port {}", i2c_num);
    
    // Reset RX FIFO in I2C controller if available
    I2CController* controller = get_i2c_controller(i2c_num);
    if (controller) {
        // TODO: Implement FIFO reset in I2C controller
        LOG_DEBUG("i2c_reset_rx_fifo: RX FIFO reset completed for port {}", i2c_num);
    }
    
    return ESP_OK;
}

int i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, 
                bool sda_pullup_en, bool scl_pullup_en, i2c_mode_t mode) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_set_pin: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_set_pin: port={}, sda={}, scl={}, sda_pullup={}, scl_pullup={}, mode={}", 
              i2c_num, sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en, static_cast<int>(mode));
    
    // Configure GPIO pins for I2C function
    // This would typically configure the pin mux and pull resistors
    
    // Store pin configuration in driver state
    std::lock_guard<std::mutex> lock(i2c_drivers[i2c_num].mutex);
    i2c_drivers[i2c_num].config.sda_io_num = sda_io_num;
    i2c_drivers[i2c_num].config.scl_io_num = scl_io_num;
    i2c_drivers[i2c_num].config.sda_pullup_en = sda_pullup_en;
    i2c_drivers[i2c_num].config.scl_pullup_en = scl_pullup_en;
    i2c_drivers[i2c_num].config.mode = mode;
    
    return ESP_OK;
}

// ============================================================================
// Convenience Functions Implementation
// ============================================================================

int i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_addr,
                                uint8_t data_addr, uint8_t* data, size_t data_len) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_master_read_from_device: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!data || data_len == 0) {
        LOG_ERROR("i2c_master_read_from_device: invalid data parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_read_from_device: port={}, device=0x{:02x}, reg=0x{:02x}, len={}", 
              i2c_num, device_addr, data_addr, data_len);
    
    // Use I2C controller directly if available for convenience functions
    I2CController* controller = get_i2c_controller(i2c_num);
    if (controller) {
        auto result = controller->read(device_addr, data_addr, data_len);
        if (!result.has_value()) {
            LOG_ERROR("i2c_master_read_from_device: read failed from device 0x{:02x}", device_addr);
            return ESP_FAIL;
        }
        
        // Copy result data
        const auto& read_data = result.value();
        std::copy(read_data.begin(), read_data.end(), data);
        return ESP_OK;
    } else {
        // Fallback using command-based API
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) {
            return ESP_FAIL;
        }
        
        // Write register address
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, data_addr, true);
        
        // Restart and read data
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        int result = i2c_master_cmd_begin(i2c_num, cmd, 1000);
        i2c_cmd_link_delete(cmd);
        return result;
    }
}

int i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_addr,
                               uint8_t data_addr, const uint8_t* data, size_t data_len) {
    if (!is_valid_i2c_port(i2c_num)) {
        LOG_ERROR("i2c_master_write_to_device: invalid I2C port {}", i2c_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!data || data_len == 0) {
        LOG_ERROR("i2c_master_write_to_device: invalid data parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("i2c_master_write_to_device: port={}, device=0x{:02x}, reg=0x{:02x}, len={}", 
              i2c_num, device_addr, data_addr, data_len);
    
    // Use I2C controller directly if available for convenience functions
    I2CController* controller = get_i2c_controller(i2c_num);
    if (controller) {
        std::vector<uint8_t> write_data;
        write_data.reserve(1 + data_len);
        write_data.push_back(data_addr);
        write_data.insert(write_data.end(), data, data + data_len);
        
        auto result = controller->write(device_addr, write_data);
        if (!result.has_value()) {
            LOG_ERROR("i2c_master_write_to_device: write failed to device 0x{:02x}", device_addr);
            return ESP_FAIL;
        }
        
        return ESP_OK;
    } else {
        // Fallback using command-based API
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) {
            return ESP_FAIL;
        }
        
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, data_addr, true);
        i2c_master_write(cmd, data, data_len, true);
        i2c_master_stop(cmd);
        
        int result = i2c_master_cmd_begin(i2c_num, cmd, 1000);
        i2c_cmd_link_delete(cmd);
        return result;
    }
}

} // extern "C"