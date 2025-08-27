/**
 * @file test_esp_idf_spi.cpp
 * @brief Tests for ESP-IDF SPI driver API emulation
 * 
 * This file tests the ESP-IDF compatible SPI driver functions to ensure
 * they work correctly with the M5Stack Tab5 emulator.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <thread>
#include <chrono>

extern "C" {
    #include "emulator/esp_idf/driver/spi_master.h"
    #include "emulator/esp_idf/esp_idf.h"
}

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"

class ESPIDFSPITest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create emulator configuration
        auto config_result = m5tab5::emulator::config::Configuration::load_default();
        ASSERT_TRUE(config_result.has_value()) << "Failed to load default configuration";
        config_ = std::move(config_result.value());
        
        // Create emulator core
        auto emulator_result = m5tab5::emulator::EmulatorCore::create(config_);
        ASSERT_TRUE(emulator_result.has_value()) << "Failed to create emulator core";
        emulator_ = std::move(emulator_result.value());
        
        // Initialize emulator
        auto init_result = emulator_->initialize();
        ASSERT_TRUE(init_result.has_value()) << "Failed to initialize emulator";
        
        // Initialize ESP-IDF with emulator core
        esp_err_t esp_result = esp_idf_init_all_with_core(emulator_.get());
        ASSERT_EQ(ESP_OK, esp_result) << "Failed to initialize ESP-IDF layer";
    }
    
    void TearDown() override {
        // Clean up ESP-IDF
        esp_idf_deinit_all();
        
        // Shutdown emulator
        if (emulator_) {
            emulator_->shutdown();
        }
    }

    std::unique_ptr<m5tab5::emulator::config::Configuration> config_;
    std::unique_ptr<m5tab5::emulator::EmulatorCore> emulator_;
};

TEST_F(ESPIDFSPITest, SPIBusInitializationAndCleanup) {
    // Test SPI bus initialization
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    EXPECT_EQ(ESP_OK, result) << "SPI bus initialization failed";
    
    // Test double initialization (should fail)
    result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    EXPECT_EQ(ESP_ERR_INVALID_STATE, result) << "Double initialization should fail";
    
    // Test bus cleanup
    result = spi_bus_free(SPI2_HOST);
    EXPECT_EQ(ESP_OK, result) << "SPI bus cleanup failed";
    
    // Test cleanup of uninitialized bus (should fail)
    result = spi_bus_free(SPI2_HOST);
    EXPECT_EQ(ESP_ERR_INVALID_STATE, result) << "Cleanup of uninitialized bus should fail";
}

TEST_F(ESPIDFSPITest, SPIBusInvalidParameters) {
    // Test invalid host
    spi_bus_config_t bus_config = {};
    esp_err_t result = spi_bus_initialize(SPI1_HOST, &bus_config, 1);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "SPI1_HOST should not be allowed";
    
    result = spi_bus_initialize(SPI_HOST_MAX, &bus_config, 1);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Invalid host should be rejected";
    
    // Test null configuration
    result = spi_bus_initialize(SPI2_HOST, nullptr, 1);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null configuration should be rejected";
}

TEST_F(ESPIDFSPITest, SPIDeviceManagement) {
    // Initialize SPI bus first
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result) << "SPI bus initialization failed";
    
    // Add device to bus
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;  // 1 MHz
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 7;
    dev_config.command_bits = 8;
    dev_config.address_bits = 24;
    dev_config.dummy_bits = 0;
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    EXPECT_EQ(ESP_OK, result) << "Device addition failed";
    EXPECT_NE(nullptr, device_handle) << "Device handle should not be null";
    
    // Test device properties
    spi_host_device_t bus_id = spi_device_get_bus(device_handle);
    EXPECT_EQ(SPI2_HOST, bus_id) << "Device should belong to SPI2_HOST";
    
    int actual_freq = spi_device_get_actual_freq(device_handle);
    EXPECT_GT(actual_freq, 0) << "Actual frequency should be positive";
    
    // Remove device from bus
    result = spi_bus_remove_device(device_handle);
    EXPECT_EQ(ESP_OK, result) << "Device removal failed";
    
    // Cleanup bus
    result = spi_bus_free(SPI2_HOST);
    EXPECT_EQ(ESP_OK, result) << "SPI bus cleanup failed";
}

TEST_F(ESPIDFSPITest, SPIDeviceInvalidParameters) {
    // Test device operations without initialized bus
    spi_device_interface_config_t dev_config = {};
    spi_device_handle_t device_handle;
    
    esp_err_t result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    EXPECT_EQ(ESP_ERR_INVALID_STATE, result) << "Device addition without bus should fail";
    
    // Test null parameters
    result = spi_bus_add_device(SPI2_HOST, nullptr, &device_handle);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null device config should be rejected";
    
    result = spi_bus_add_device(SPI2_HOST, &dev_config, nullptr);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null handle pointer should be rejected";
}

TEST_F(ESPIDFSPITest, SPITransactionBasic) {
    // Initialize SPI bus and device
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 7;
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Test basic transmission using tx_data/rx_data
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans.length = 8;  // 1 byte
    trans.tx_data[0] = 0xA5;
    trans.tx_data[1] = 0x5A;
    
    result = spi_device_transmit(device_handle, &trans);
    EXPECT_EQ(ESP_OK, result) << "Basic SPI transmission failed";
    
    // Check that some response was received (emulation fills with 0xAA)
    // Note: In emulation mode, we expect dummy data
    EXPECT_NE(0x00, trans.rx_data[0]) << "Should receive some response data";
    
    // Cleanup
    spi_bus_remove_device(device_handle);
    spi_bus_free(SPI2_HOST);
}

TEST_F(ESPIDFSPITest, SPITransactionWithBuffer) {
    // Initialize SPI bus and device
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 7;
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Test transmission with buffers
    uint8_t tx_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t rx_buffer[5] = {0};
    
    spi_transaction_t trans = {};
    trans.length = 5 * 8;  // 5 bytes
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;
    
    result = spi_device_transmit(device_handle, &trans);
    EXPECT_EQ(ESP_OK, result) << "Buffer-based SPI transmission failed";
    
    // Check that receive buffer was modified (emulation should fill with dummy data)
    bool rx_modified = false;
    for (int i = 0; i < 5; i++) {
        if (rx_buffer[i] != 0x00) {
            rx_modified = true;
            break;
        }
    }
    EXPECT_TRUE(rx_modified) << "Receive buffer should be modified by emulation";
    
    // Cleanup
    spi_bus_remove_device(device_handle);
    spi_bus_free(SPI2_HOST);
}

TEST_F(ESPIDFSPITest, SPITransactionQueue) {
    // Initialize SPI bus and device
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 3;  // Small queue for testing
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Queue multiple transactions
    spi_transaction_t trans1 = {};
    trans1.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans1.length = 8;
    trans1.tx_data[0] = 0x11;
    
    spi_transaction_t trans2 = {};
    trans2.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans2.length = 8;
    trans2.tx_data[0] = 0x22;
    
    // Queue first transaction
    result = spi_device_queue_trans(device_handle, &trans1, 1000);
    EXPECT_EQ(ESP_OK, result) << "First transaction queuing failed";
    
    // Queue second transaction
    result = spi_device_queue_trans(device_handle, &trans2, 1000);
    EXPECT_EQ(ESP_OK, result) << "Second transaction queuing failed";
    
    // Get results
    spi_transaction_t* completed_trans;
    result = spi_device_get_trans_result(device_handle, &completed_trans, 1000);
    EXPECT_EQ(ESP_OK, result) << "Getting first transaction result failed";
    EXPECT_EQ(&trans1, completed_trans) << "Should get first transaction back";
    
    result = spi_device_get_trans_result(device_handle, &completed_trans, 1000);
    EXPECT_EQ(ESP_OK, result) << "Getting second transaction result failed";
    EXPECT_EQ(&trans2, completed_trans) << "Should get second transaction back";
    
    // Cleanup
    spi_bus_remove_device(device_handle);
    spi_bus_free(SPI2_HOST);
}

TEST_F(ESPIDFSPITest, SPIBusAcquisition) {
    // Initialize SPI bus and device
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 7;
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Test bus acquisition
    result = spi_device_acquire_bus(device_handle, portMAX_DELAY);
    EXPECT_EQ(ESP_OK, result) << "Bus acquisition failed";
    
    // Test double acquisition by same device (should succeed)
    result = spi_device_acquire_bus(device_handle, portMAX_DELAY);
    EXPECT_EQ(ESP_OK, result) << "Double acquisition by same device should succeed";
    
    // Test bus release
    spi_device_release_bus(device_handle);
    
    // Test release of already released bus (should be safe)
    spi_device_release_bus(device_handle);
    
    // Cleanup
    spi_bus_remove_device(device_handle);
    spi_bus_free(SPI2_HOST);
}

TEST_F(ESPIDFSPITest, SPIFrequencyCalculations) {
    const int apb_freq = 80000000;  // 80 MHz
    
    // Test frequency calculations
    int actual_1mhz = spi_get_actual_clock(apb_freq, 1000000, 128);
    EXPECT_GT(actual_1mhz, 0) << "1 MHz calculation should return positive value";
    EXPECT_LE(actual_1mhz, 1000000) << "Actual frequency should not exceed requested";
    
    int actual_10mhz = spi_get_actual_clock(apb_freq, 10000000, 128);
    EXPECT_GT(actual_10mhz, actual_1mhz) << "10 MHz should be higher than 1 MHz";
    
    // Test frequency limit
    int freq_limit = spi_get_freq_limit(false, 0);
    EXPECT_GT(freq_limit, 0) << "Frequency limit should be positive";
    EXPECT_LE(freq_limit, 80000000) << "Frequency limit should be reasonable";
    
    // Test timing calculation
    int dummy_bits, cycles_remain;
    esp_err_t result = spi_get_timing(false, 0, actual_1mhz, &dummy_bits, &cycles_remain);
    EXPECT_EQ(ESP_OK, result) << "Timing calculation should succeed";
    EXPECT_GE(dummy_bits, 0) << "Dummy bits should be non-negative";
    EXPECT_GE(cycles_remain, 0) << "Remaining cycles should be non-negative";
}

TEST_F(ESPIDFSPITest, SPITransactionInvalidParameters) {
    // Initialize SPI bus and device
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    spi_device_interface_config_t dev_config = {};
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.queue_size = 7;
    
    spi_device_handle_t device_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config, &device_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Test null parameters
    result = spi_device_transmit(nullptr, nullptr);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null device handle should be rejected";
    
    spi_transaction_t trans = {};
    result = spi_device_transmit(device_handle, nullptr);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null transaction should be rejected";
    
    result = spi_device_transmit(nullptr, &trans);
    EXPECT_EQ(ESP_ERR_INVALID_ARG, result) << "Null device with valid transaction should be rejected";
    
    // Cleanup
    spi_bus_remove_device(device_handle);
    spi_bus_free(SPI2_HOST);
}

// Test multiple devices on same bus
TEST_F(ESPIDFSPITest, MultipleDevicesOnBus) {
    // Initialize SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = 23;
    bus_config.miso_io_num = 25;
    bus_config.sclk_io_num = 19;
    bus_config.max_transfer_sz = 4096;
    
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config, 1);
    ASSERT_EQ(ESP_OK, result);
    
    // Add first device
    spi_device_interface_config_t dev_config1 = {};
    dev_config1.mode = 0;
    dev_config1.clock_speed_hz = 1000000;
    dev_config1.spics_io_num = 5;
    dev_config1.queue_size = 7;
    
    spi_device_handle_t device1_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config1, &device1_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Add second device
    spi_device_interface_config_t dev_config2 = {};
    dev_config2.mode = 1;  // Different mode
    dev_config2.clock_speed_hz = 2000000;  // Different frequency
    dev_config2.spics_io_num = 15;  // Different CS pin
    dev_config2.queue_size = 5;
    
    spi_device_handle_t device2_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config2, &device2_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Add third device
    spi_device_interface_config_t dev_config3 = {};
    dev_config3.mode = 2;
    dev_config3.clock_speed_hz = 500000;
    dev_config3.spics_io_num = 18;
    dev_config3.queue_size = 3;
    
    spi_device_handle_t device3_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config3, &device3_handle);
    ASSERT_EQ(ESP_OK, result);
    
    // Try to add fourth device (should fail - only 3 CS available)
    spi_device_interface_config_t dev_config4 = {};
    spi_device_handle_t device4_handle;
    result = spi_bus_add_device(SPI2_HOST, &dev_config4, &device4_handle);
    EXPECT_EQ(ESP_ERR_NOT_FOUND, result) << "Fourth device should fail (no CS available)";
    
    // Test transactions on different devices
    spi_transaction_t trans1 = {};
    trans1.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans1.length = 8;
    trans1.tx_data[0] = 0x11;
    
    spi_transaction_t trans2 = {};
    trans2.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans2.length = 8;
    trans2.tx_data[0] = 0x22;
    
    result = spi_device_transmit(device1_handle, &trans1);
    EXPECT_EQ(ESP_OK, result) << "Device 1 transaction failed";
    
    result = spi_device_transmit(device2_handle, &trans2);
    EXPECT_EQ(ESP_OK, result) << "Device 2 transaction failed";
    
    // Cleanup all devices
    spi_bus_remove_device(device1_handle);
    spi_bus_remove_device(device2_handle);
    spi_bus_remove_device(device3_handle);
    spi_bus_free(SPI2_HOST);
}