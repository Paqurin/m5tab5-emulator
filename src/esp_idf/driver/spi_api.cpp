/**
 * @file spi_api.cpp  
 * @brief ESP-IDF SPI Master driver API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible SPI functions that map to the
 * emulated SPI controller, providing seamless compatibility for ESP-IDF
 * applications running on the emulator.
 */

#include "emulator/esp_idf/driver/spi_master.h"
#include "emulator/peripherals/spi_controller.hpp"
#include "emulator/peripherals/pin_mux_controller.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/esp_idf/esp_idf.h"
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <queue>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>

namespace {
    using namespace m5tab5::emulator;
    using SPIController = m5tab5::emulator::SPIController;
    using PinMuxController = m5tab5::emulator::PinMuxController;
    
    /**
     * @brief SPI device instance structure
     */
    struct SPIDevice {
        spi_host_device_t host_id;
        spi_device_interface_config_t config;
        int cs_id;  // Chip select ID (0-2)
        bool is_active;
        std::mutex device_mutex;
        std::queue<spi_transaction_t*> pending_transactions;
        std::queue<spi_transaction_t*> completed_transactions;
        std::condition_variable transaction_cv;
        
        SPIDevice() : host_id(SPI_HOST_MAX), cs_id(-1), is_active(false) {}
    };
    
    /**
     * @brief SPI bus state per host
     */
    struct SPIBusState {
        bool initialized;
        spi_bus_config_t config;
        SPIController* controller;
        PinMuxController* pin_mux;
        int dma_channel;
        std::mutex bus_mutex;
        std::vector<std::unique_ptr<SPIDevice>> devices;
        SPIDevice* bus_owner;  // Device that has acquired the bus
        
        SPIBusState() : initialized(false), controller(nullptr), pin_mux(nullptr), 
                       dma_channel(-1), bus_owner(nullptr) {
            devices.resize(3);  // Max 3 devices per bus
        }
    };
    
    // Static bus state for each SPI host
    static SPIBusState spi_buses[SPI_HOST_MAX];
    static std::mutex global_spi_mutex;
    
    /**
     * @brief Get SPI controller instance from emulator core
     */
    SPIController* get_spi_controller(spi_host_device_t host_id) {
        if (host_id >= SPI_HOST_MAX) {
            return nullptr;
        }
        
        if (!spi_buses[host_id].controller) {
            auto emulator = esp_idf_get_emulator_core();
            if (emulator) {
                LOG_DEBUG("Getting SPI controller instance for host {}", static_cast<int>(host_id));
                
                // M5Stack Tab5 SPI mapping: SPI1->SPI0, SPI2->SPI1, SPI3->SPI2
                uint8_t controller_id = static_cast<uint8_t>(host_id);
                
                // Get SPI controller component from emulator core
                auto spi_shared = emulator->getComponent<SPIController>();
                if (spi_shared) {
                    spi_buses[host_id].controller = spi_shared.get();
                    LOG_DEBUG("Successfully retrieved SPI controller from EmulatorCore");
                } else {
                    LOG_WARN("SPI controller not available from EmulatorCore");
                }
            } else {
                LOG_WARN("EmulatorCore not available - ESP-IDF not properly initialized");
            }
        }
        
        return spi_buses[host_id].controller;
    }
    
    /**
     * @brief Get Pin Mux controller instance from emulator core
     */
    PinMuxController* get_pin_mux_controller() {
        static PinMuxController* pin_mux = nullptr;
        
        if (!pin_mux) {
            auto emulator = esp_idf_get_emulator_core();
            if (emulator) {
                auto pin_mux_shared = emulator->getComponent<PinMuxController>();
                if (pin_mux_shared) {
                    pin_mux = pin_mux_shared.get();
                }
            }
        }
        
        return pin_mux;
    }
    
    
    /**
     * @brief Validate SPI host
     */
    bool is_valid_spi_host(spi_host_device_t host_id) {
        return (host_id > SPI1_HOST && host_id < SPI_HOST_MAX);
    }
    
    /**
     * @brief Convert ESP-IDF SPI mode to emulator SPI configuration
     */
    m5tab5::emulator::SPIClockPolarity get_clock_polarity(uint8_t mode) {
        return (mode & 0x02) ? m5tab5::emulator::SPIClockPolarity::IDLE_HIGH : 
                              m5tab5::emulator::SPIClockPolarity::IDLE_LOW;
    }
    
    m5tab5::emulator::SPIClockPhase get_clock_phase(uint8_t mode) {
        return (mode & 0x01) ? m5tab5::emulator::SPIClockPhase::SECOND_EDGE : 
                              m5tab5::emulator::SPIClockPhase::FIRST_EDGE;
    }
    
    /**
     * @brief Configure GPIO pins for SPI
     */
    esp_err_t configure_spi_pins(spi_host_device_t host_id, const spi_bus_config_t* config) {
        auto pin_mux = get_pin_mux_controller();
        if (!pin_mux) {
            LOG_WARN("Pin mux controller not available, skipping pin configuration");
            return ESP_OK;  // Continue in emulation mode
        }
        
        // Configure MOSI pin
        if (config->mosi_io_num >= 0) {
            auto result = pin_mux->configure_pin(config->mosi_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_MOSI);
            if (!result.has_value()) {
                LOG_ERROR("Failed to configure MOSI pin {}", config->mosi_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
        
        // Configure MISO pin
        if (config->miso_io_num >= 0) {
            auto result = pin_mux->configure_pin(config->miso_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_MISO);
            if (!result.has_value()) {
                LOG_ERROR("Failed to configure MISO pin {}", config->miso_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
        
        // Configure SCLK pin
        if (config->sclk_io_num >= 0) {
            auto result = pin_mux->configure_pin(config->sclk_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_CLK);
            if (!result.has_value()) {
                LOG_ERROR("Failed to configure SCLK pin {}", config->sclk_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
        
        // Configure Quad SPI pins if used
        if (config->quadwp_io_num >= 0) {
            auto result = pin_mux->configure_pin(config->quadwp_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_MOSI);
            if (!result.has_value()) {
                LOG_ERROR("Failed to configure QUADWP pin {}", config->quadwp_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
        
        if (config->quadhd_io_num >= 0) {
            auto result = pin_mux->configure_pin(config->quadhd_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_MISO);
            if (!result.has_value()) {
                LOG_ERROR("Failed to configure QUADHD pin {}", config->quadhd_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
        
        LOG_DEBUG("SPI pins configured for host {}: MOSI={}, MISO={}, SCLK={}", 
                  static_cast<int>(host_id), config->mosi_io_num, 
                  config->miso_io_num, config->sclk_io_num);
        
        return ESP_OK;
    }
    
    /**
     * @brief Execute SPI transaction on hardware
     */
    esp_err_t execute_transaction(SPIDevice* device, spi_transaction_t* trans) {
        if (!device || !trans) {
            return ESP_ERR_INVALID_ARG;
        }
        
        auto& bus = spi_buses[device->host_id];
        SPIController* controller = bus.controller;
        
        if (!controller) {
            LOG_WARN("SPI controller not available, simulating transaction");
            
            // Simulate transaction delay
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            
            // Fill receive buffer with dummy data if needed
            if (trans->rx_buffer && trans->rxlength > 0) {
                memset(trans->rx_buffer, 0xAA, (trans->rxlength + 7) / 8);
            } else if (trans->flags & SPI_TRANS_USE_RXDATA) {
                memset(trans->rx_data, 0xAA, sizeof(trans->rx_data));
            }
            
            return ESP_OK;
        }
        
        // Configure SPI controller for this transaction
        uint32_t clock_rate = device->config.clock_speed_hz;
        auto cpol = get_clock_polarity(device->config.mode);
        auto cpha = get_clock_phase(device->config.mode);
        
        auto config_result = controller->configure(
            m5tab5::emulator::SPIMode::MASTER, 
            clock_rate, cpol, cpha, 
            m5tab5::emulator::SPIBitOrder::MSB_FIRST,
            m5tab5::emulator::SPIDataSize::BITS_8
        );
        
        if (!config_result.has_value()) {
            LOG_ERROR("Failed to configure SPI controller for transaction");
            return ESP_FAIL;
        }
        
        // Activate chip select
        auto cs_result = controller->set_chip_select(device->cs_id, true);
        if (!cs_result.has_value()) {
            LOG_ERROR("Failed to activate chip select {}", device->cs_id);
            return ESP_FAIL;
        }
        
        // Prepare transmission data
        std::vector<uint8_t> tx_data;
        
        // Add command if present
        if (trans->flags & SPI_TRANS_VARIABLE_CMD) {
            // Variable command length - not commonly used, skip for now
        } else if (device->config.command_bits > 0) {
            uint8_t cmd_bytes = (device->config.command_bits + 7) / 8;
            for (int i = cmd_bytes - 1; i >= 0; i--) {
                tx_data.push_back((trans->cmd >> (i * 8)) & 0xFF);
            }
        }
        
        // Add address if present
        if (trans->flags & SPI_TRANS_USE_ADDR) {
            uint8_t addr_bytes = (device->config.address_bits + 7) / 8;
            for (int i = addr_bytes - 1; i >= 0; i--) {
                tx_data.push_back((trans->addr >> (i * 8)) & 0xFF);
            }
        }
        
        // Add dummy bits if needed
        uint8_t dummy_bytes = (device->config.dummy_bits + 7) / 8;
        for (uint8_t i = 0; i < dummy_bytes; i++) {
            tx_data.push_back(0x00);
        }
        
        // Add data payload
        size_t data_length = (trans->length + 7) / 8;
        if (trans->flags & SPI_TRANS_USE_TXDATA) {
            for (size_t i = 0; i < std::min(data_length, sizeof(trans->tx_data)); i++) {
                tx_data.push_back(trans->tx_data[i]);
            }
        } else if (trans->tx_buffer && data_length > 0) {
            const uint8_t* tx_buf = static_cast<const uint8_t*>(trans->tx_buffer);
            tx_data.insert(tx_data.end(), tx_buf, tx_buf + data_length);
        } else {
            // Read-only transaction - send dummy bytes
            for (size_t i = 0; i < data_length; i++) {
                tx_data.push_back(0x00);
            }
        }
        
        // Execute transaction
        auto transfer_result = controller->start_transfer(tx_data);
        if (!transfer_result.has_value()) {
            controller->set_chip_select(device->cs_id, false);
            LOG_ERROR("Failed to start SPI transfer");
            return ESP_FAIL;
        }
        
        // Wait for transfer completion (polling mode for now)
        const int max_wait_ms = 1000;
        int wait_count = 0;
        while (controller->is_busy() && wait_count < max_wait_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            controller->update();
            wait_count++;
        }
        
        if (controller->is_busy()) {
            LOG_ERROR("SPI transaction timeout");
            controller->abort_transfer();
            controller->set_chip_select(device->cs_id, false);
            return ESP_ERR_TIMEOUT;
        }
        
        // Get received data
        auto rx_result = controller->get_received_data();
        if (rx_result.has_value()) {
            const auto& rx_data = rx_result.value();
            size_t rx_length = (trans->rxlength > 0) ? (trans->rxlength + 7) / 8 : data_length;
            size_t copy_length = std::min(rx_length, rx_data.size());
            
            // Skip command, address, and dummy bytes in received data
            size_t skip_bytes = tx_data.size() - data_length;
            
            if (trans->flags & SPI_TRANS_USE_RXDATA) {
                memset(trans->rx_data, 0, sizeof(trans->rx_data));
                if (rx_data.size() > skip_bytes) {
                    size_t available = std::min(sizeof(trans->rx_data), rx_data.size() - skip_bytes);
                    memcpy(trans->rx_data, rx_data.data() + skip_bytes, available);
                }
            } else if (trans->rx_buffer && copy_length > 0) {
                if (rx_data.size() > skip_bytes) {
                    size_t available = std::min(copy_length, rx_data.size() - skip_bytes);
                    memcpy(trans->rx_buffer, rx_data.data() + skip_bytes, available);
                }
            }
        }
        
        // Deactivate chip select
        controller->set_chip_select(device->cs_id, false);
        
        return ESP_OK;
    }
    
    /**
     * @brief Find available chip select ID for device
     */
    int find_available_cs(spi_host_device_t host_id) {
        auto& bus = spi_buses[host_id];
        for (int cs = 0; cs < 3; cs++) {
            if (!bus.devices[cs] || !bus.devices[cs]->is_active) {
                return cs;
            }
        }
        return -1;  // No available CS
    }
}

// ============================================================================
// ESP-IDF SPI Master API Implementation
// ============================================================================

extern "C" {

esp_err_t spi_bus_initialize(spi_host_device_t host_id, const spi_bus_config_t *bus_config, int dma_chan) {
    if (!is_valid_spi_host(host_id)) {
        LOG_ERROR("spi_bus_initialize: invalid SPI host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!bus_config) {
        LOG_ERROR("spi_bus_initialize: null bus configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    if (spi_buses[host_id].initialized) {
        LOG_ERROR("spi_bus_initialize: SPI bus already initialized for host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("spi_bus_initialize: host={}, MOSI={}, MISO={}, SCLK={}, max_transfer_sz={}, dma_chan={}", 
              static_cast<int>(host_id), bus_config->mosi_io_num, bus_config->miso_io_num, 
              bus_config->sclk_io_num, bus_config->max_transfer_sz, dma_chan);
    
    auto& bus = spi_buses[host_id];
    
    // Store bus configuration
    bus.config = *bus_config;
    bus.dma_channel = dma_chan;
    
    // Configure GPIO pins
    esp_err_t pin_result = configure_spi_pins(host_id, bus_config);
    if (pin_result != ESP_OK) {
        LOG_ERROR("spi_bus_initialize: failed to configure SPI pins");
        return pin_result;
    }
    
    // Get controller instances
    bus.controller = get_spi_controller(host_id);
    bus.pin_mux = get_pin_mux_controller();
    
    // Initialize SPI controller if available
    if (bus.controller) {
        // Use a default configuration - devices will reconfigure as needed
        m5tab5::emulator::Configuration config;
        auto init_result = bus.controller->initialize(config, nullptr);
        if (!init_result.has_value()) {
            LOG_ERROR("spi_bus_initialize: failed to initialize SPI controller");
            return ESP_FAIL;
        }
        
        LOG_DEBUG("spi_bus_initialize: SPI controller initialized successfully");
    } else {
        LOG_WARN("spi_bus_initialize: SPI controller not available for host {} (emulation mode)", 
                 static_cast<int>(host_id));
    }
    
    bus.initialized = true;
    LOG_INFO("spi_bus_initialize: SPI bus initialized for host {}", static_cast<int>(host_id));
    
    return ESP_OK;
}

esp_err_t spi_bus_free(spi_host_device_t host_id) {
    if (!is_valid_spi_host(host_id)) {
        LOG_ERROR("spi_bus_free: invalid SPI host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    if (!spi_buses[host_id].initialized) {
        LOG_ERROR("spi_bus_free: SPI bus not initialized for host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_STATE;
    }
    
    auto& bus = spi_buses[host_id];
    
    // Check if any devices are still attached
    for (const auto& device : bus.devices) {
        if (device && device->is_active) {
            LOG_ERROR("spi_bus_free: cannot free bus with active devices");
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    LOG_DEBUG("spi_bus_free: freeing SPI bus for host {}", static_cast<int>(host_id));
    
    // Shutdown SPI controller
    if (bus.controller) {
        bus.controller->shutdown();
    }
    
    // Clear bus state
    bus.initialized = false;
    bus.controller = nullptr;
    bus.pin_mux = nullptr;
    bus.bus_owner = nullptr;
    
    LOG_INFO("spi_bus_free: SPI bus freed for host {}", static_cast<int>(host_id));
    return ESP_OK;
}

esp_err_t spi_bus_add_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, 
                             spi_device_handle_t *handle) {
    if (!is_valid_spi_host(host_id)) {
        LOG_ERROR("spi_bus_add_device: invalid SPI host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!dev_config || !handle) {
        LOG_ERROR("spi_bus_add_device: null parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    if (!spi_buses[host_id].initialized) {
        LOG_ERROR("spi_bus_add_device: SPI bus not initialized for host {}", static_cast<int>(host_id));
        return ESP_ERR_INVALID_STATE;
    }
    
    // Find available chip select
    int cs_id = find_available_cs(host_id);
    if (cs_id < 0) {
        LOG_ERROR("spi_bus_add_device: no available chip select slots");
        return ESP_ERR_NOT_FOUND;
    }
    
    LOG_DEBUG("spi_bus_add_device: host={}, CS={}, mode={}, clock={} Hz, CS_pin={}, queue_size={}", 
              static_cast<int>(host_id), cs_id, dev_config->mode, dev_config->clock_speed_hz, 
              dev_config->spics_io_num, dev_config->queue_size);
    
    // Create device instance
    auto device = std::make_unique<SPIDevice>();
    device->host_id = host_id;
    device->config = *dev_config;
    device->cs_id = cs_id;
    device->is_active = true;
    
    // Configure CS pin if specified
    if (dev_config->spics_io_num >= 0) {
        auto pin_mux = get_pin_mux_controller();
        if (pin_mux) {
            auto result = pin_mux->configure_pin(dev_config->spics_io_num, 
                                                m5tab5::emulator::PinFunction::SPI_CS);
            if (!result.has_value()) {
                LOG_ERROR("spi_bus_add_device: failed to configure CS pin {}", dev_config->spics_io_num);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    
    // Store device and return handle
    SPIDevice* device_ptr = device.get();
    spi_buses[host_id].devices[cs_id] = std::move(device);
    *handle = reinterpret_cast<spi_device_handle_t>(device_ptr);
    
    LOG_INFO("spi_bus_add_device: device added to host {}, CS {}", static_cast<int>(host_id), cs_id);
    return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t handle) {
    if (!handle) {
        LOG_ERROR("spi_bus_remove_device: null device handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    if (!device->is_active) {
        LOG_ERROR("spi_bus_remove_device: device already freed");
        return ESP_ERR_INVALID_STATE;
    }
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    LOG_DEBUG("spi_bus_remove_device: removing device from host {}, CS {}", 
              static_cast<int>(device->host_id), device->cs_id);
    
    // Wait for pending transactions to complete
    std::unique_lock<std::mutex> device_lock(device->device_mutex);
    while (!device->pending_transactions.empty()) {
        device->transaction_cv.wait_for(device_lock, std::chrono::milliseconds(100));
    }
    
    // Mark device as inactive
    device->is_active = false;
    
    // Clear device from bus
    spi_buses[device->host_id].devices[device->cs_id].reset();
    
    LOG_INFO("spi_bus_remove_device: device removed from host {}, CS {}", 
             static_cast<int>(device->host_id), device->cs_id);
    
    return ESP_OK;
}

esp_err_t spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *trans_desc, TickType_t ticks_to_wait) {
    if (!handle || !trans_desc) {
        LOG_ERROR("spi_device_queue_trans: null parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    if (!device->is_active) {
        LOG_ERROR("spi_device_queue_trans: device not active");
        return ESP_ERR_INVALID_STATE;
    }
    
    std::unique_lock<std::mutex> lock(device->device_mutex);
    
    // Check queue size
    if (device->pending_transactions.size() >= static_cast<size_t>(device->config.queue_size)) {
        if (ticks_to_wait == 0) {
            return ESP_ERR_TIMEOUT;
        }
        
        // Wait for space in queue
        auto timeout = std::chrono::milliseconds(ticks_to_wait * 10);  // Approximate tick conversion
        if (!device->transaction_cv.wait_for(lock, timeout, 
                [device] { return device->pending_transactions.size() < static_cast<size_t>(device->config.queue_size); })) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Add transaction to queue
    device->pending_transactions.push(trans_desc);
    
    LOG_DEBUG("spi_device_queue_trans: transaction queued for host {}, CS {}, length={} bits", 
              static_cast<int>(device->host_id), device->cs_id, trans_desc->length);
    
    // Process transaction immediately (blocking mode for simplicity)
    lock.unlock();
    esp_err_t result = execute_transaction(device, trans_desc);
    lock.lock();
    
    // Move from pending to completed
    if (!device->pending_transactions.empty()) {
        device->pending_transactions.pop();
    }
    device->completed_transactions.push(trans_desc);
    device->transaction_cv.notify_all();
    
    return result;
}

esp_err_t spi_device_get_trans_result(spi_device_handle_t handle, spi_transaction_t **trans_desc, TickType_t ticks_to_wait) {
    if (!handle || !trans_desc) {
        LOG_ERROR("spi_device_get_trans_result: null parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    if (!device->is_active) {
        LOG_ERROR("spi_device_get_trans_result: device not active");
        return ESP_ERR_INVALID_STATE;
    }
    
    std::unique_lock<std::mutex> lock(device->device_mutex);
    
    // Wait for completed transaction
    if (device->completed_transactions.empty()) {
        if (ticks_to_wait == 0) {
            return ESP_ERR_TIMEOUT;
        }
        
        auto timeout = std::chrono::milliseconds(ticks_to_wait * 10);  // Approximate tick conversion
        if (!device->transaction_cv.wait_for(lock, timeout, 
                [device] { return !device->completed_transactions.empty(); })) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Get completed transaction
    *trans_desc = device->completed_transactions.front();
    device->completed_transactions.pop();
    
    LOG_DEBUG("spi_device_get_trans_result: transaction result retrieved for host {}, CS {}", 
              static_cast<int>(device->host_id), device->cs_id);
    
    return ESP_OK;
}

esp_err_t spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc) {
    if (!handle || !trans_desc) {
        LOG_ERROR("spi_device_transmit: null parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    if (!device->is_active) {
        LOG_ERROR("spi_device_transmit: device not active");
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("spi_device_transmit: executing transaction for host {}, CS {}, length={} bits", 
              static_cast<int>(device->host_id), device->cs_id, trans_desc->length);
    
    // Execute transaction directly (blocking)
    return execute_transaction(device, trans_desc);
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc, TickType_t ticks_to_wait) {
    // For emulation, polling transmit is the same as regular transmit
    (void)ticks_to_wait;  // Unused parameter
    return spi_device_transmit(handle, trans_desc);
}

esp_err_t spi_device_acquire_bus(spi_device_handle_t handle, TickType_t wait) {
    if (!handle) {
        LOG_ERROR("spi_device_acquire_bus: null device handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    if (!device->is_active) {
        LOG_ERROR("spi_device_acquire_bus: device not active");
        return ESP_ERR_INVALID_STATE;
    }
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    auto& bus = spi_buses[device->host_id];
    
    if (bus.bus_owner && bus.bus_owner != device) {
        LOG_ERROR("spi_device_acquire_bus: bus already owned by another device");
        return ESP_ERR_TIMEOUT;
    }
    
    bus.bus_owner = device;
    
    LOG_DEBUG("spi_device_acquire_bus: bus acquired by device on host {}, CS {}", 
              static_cast<int>(device->host_id), device->cs_id);
    
    return ESP_OK;
}

void spi_device_release_bus(spi_device_handle_t handle) {
    if (!handle) {
        LOG_ERROR("spi_device_release_bus: null device handle");
        return;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    std::lock_guard<std::mutex> lock(global_spi_mutex);
    
    auto& bus = spi_buses[device->host_id];
    
    if (bus.bus_owner == device) {
        bus.bus_owner = nullptr;
        LOG_DEBUG("spi_device_release_bus: bus released by device on host {}, CS {}", 
                  static_cast<int>(device->host_id), device->cs_id);
    }
}

int spi_get_actual_clock(int fapb, int hz, int duty_cycle) {
    // Simple frequency calculation for emulation
    (void)duty_cycle;  // Unused parameter
    
    int divisor = (fapb + hz - 1) / hz;  // Round up
    if (divisor < 1) divisor = 1;
    if (divisor > 65536) divisor = 65536;
    
    int actual_freq = fapb / divisor;
    
    LOG_DEBUG("spi_get_actual_clock: requested={} Hz, actual={} Hz, divisor={}", hz, actual_freq, divisor);
    
    return actual_freq;
}

esp_err_t spi_get_timing(bool gpio_is_used, int input_delay_ns, int eff_clk, int *dummy_o, int *cycles_remain_o) {
    // Simplified timing calculation for emulation
    (void)gpio_is_used;
    (void)input_delay_ns;
    
    if (dummy_o) {
        *dummy_o = 0;  // No dummy cycles needed in emulation
    }
    
    if (cycles_remain_o) {
        *cycles_remain_o = 0;  // No remaining cycles
    }
    
    LOG_DEBUG("spi_get_timing: eff_clk={} Hz, dummy={}, remaining={}", 
              eff_clk, dummy_o ? *dummy_o : 0, cycles_remain_o ? *cycles_remain_o : 0);
    
    return ESP_OK;
}

int spi_get_freq_limit(bool gpio_is_used, int input_delay_ns) {
    // Conservative frequency limit for emulation
    (void)gpio_is_used;
    (void)input_delay_ns;
    
    const int max_freq = 40000000;  // 40 MHz limit
    
    LOG_DEBUG("spi_get_freq_limit: returning {} Hz", max_freq);
    
    return max_freq;
}

spi_host_device_t spi_device_get_bus(spi_device_handle_t handle) {
    if (!handle) {
        LOG_ERROR("spi_device_get_bus: null device handle");
        return SPI_HOST_MAX;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    return device->host_id;
}

int spi_device_get_actual_freq(spi_device_handle_t handle) {
    if (!handle) {
        LOG_ERROR("spi_device_get_actual_freq: null device handle");
        return 0;
    }
    
    SPIDevice* device = reinterpret_cast<SPIDevice*>(handle);
    
    // Calculate actual frequency based on device configuration
    const int apb_freq = 80000000;  // 80 MHz APB clock
    int actual_freq = spi_get_actual_clock(apb_freq, device->config.clock_speed_hz, 128);
    
    LOG_DEBUG("spi_device_get_actual_freq: device on host {}, CS {}, actual_freq={} Hz", 
              static_cast<int>(device->host_id), device->cs_id, actual_freq);
    
    return actual_freq;
}

} // extern "C"