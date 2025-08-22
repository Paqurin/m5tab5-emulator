#include <gtest/gtest.h>
#include "emulator/peripherals/i2c_controller.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

namespace emulator::peripherals::test {

class I2CControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<memory::MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value());
        
        auto i2c_result = I2CController::create(0, memory_controller);
        ASSERT_TRUE(i2c_result.has_value()) << "Failed to create I2C controller";
        i2c_controller = std::move(i2c_result.value());
        
        ASSERT_TRUE(i2c_controller->initialize().has_value()) 
            << "Failed to initialize I2C controller";
    }
    
    void TearDown() override {
        if (i2c_controller) {
            i2c_controller->shutdown();
        }
    }
    
    std::shared_ptr<memory::MemoryController> memory_controller;
    std::unique_ptr<I2CController> i2c_controller;
};

TEST_F(I2CControllerTest, BasicConfiguration) {
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    
    auto config = i2c_controller->get_configuration();
    ASSERT_TRUE(config.has_value());
    EXPECT_EQ(config.value().clock_frequency, 400000);
    EXPECT_EQ(config.value().mode, I2CController::Mode::MASTER);
}

TEST_F(I2CControllerTest, ClockFrequencyValidation) {
    EXPECT_TRUE(i2c_controller->configure(100000, I2CController::Mode::MASTER).has_value());
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    EXPECT_TRUE(i2c_controller->configure(1000000, I2CController::Mode::MASTER).has_value());
    
    auto invalid_freq = i2c_controller->configure(5000000, I2CController::Mode::MASTER);
    EXPECT_FALSE(invalid_freq.has_value());
    EXPECT_EQ(invalid_freq.error(), utils::ErrorCode::INVALID_PARAMETER);
}

TEST_F(I2CControllerTest, DeviceDetection) {
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    
    i2c_controller->add_virtual_device(0x68);
    i2c_controller->add_virtual_device(0x1D);
    
    auto scan_result = i2c_controller->scan_devices();
    ASSERT_TRUE(scan_result.has_value());
    
    std::vector<uint8_t> expected_devices = {0x68, 0x1D};
    EXPECT_EQ(scan_result.value(), expected_devices);
}

TEST_F(I2CControllerTest, SingleByteWrite) {
    uint8_t device_addr = 0x68;
    uint8_t register_addr = 0x6B;
    uint8_t data = 0x80;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto write_result = i2c_controller->write_register(device_addr, register_addr, data);
    ASSERT_TRUE(write_result.has_value()) << "Failed to write single byte";
    
    auto read_result = i2c_controller->read_register(device_addr, register_addr);
    ASSERT_TRUE(read_result.has_value()) << "Failed to read single byte";
    EXPECT_EQ(read_result.value(), data);
}

TEST_F(I2CControllerTest, MultiByteWrite) {
    uint8_t device_addr = 0x1D;
    uint8_t register_addr = 0x20;
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto write_result = i2c_controller->write_registers(device_addr, register_addr, data);
    ASSERT_TRUE(write_result.has_value()) << "Failed to write multiple bytes";
    
    auto read_result = i2c_controller->read_registers(device_addr, register_addr, data.size());
    ASSERT_TRUE(read_result.has_value()) << "Failed to read multiple bytes";
    EXPECT_EQ(read_result.value(), data);
}

TEST_F(I2CControllerTest, NonExistentDeviceHandling) {
    uint8_t non_existent_addr = 0x77;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    
    auto write_result = i2c_controller->write_register(non_existent_addr, 0x00, 0xFF);
    EXPECT_FALSE(write_result.has_value());
    EXPECT_EQ(write_result.error(), utils::ErrorCode::DEVICE_NOT_FOUND);
    
    auto read_result = i2c_controller->read_register(non_existent_addr, 0x00);
    EXPECT_FALSE(read_result.has_value());
    EXPECT_EQ(read_result.error(), utils::ErrorCode::DEVICE_NOT_FOUND);
}

TEST_F(I2CControllerTest, TransactionIntegrity) {
    uint8_t device_addr = 0x48;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto transaction = i2c_controller->begin_transaction(device_addr);
    ASSERT_TRUE(transaction.has_value()) << "Failed to begin transaction";
    
    EXPECT_TRUE(i2c_controller->write_byte(0x10).has_value());
    EXPECT_TRUE(i2c_controller->write_byte(0xAB).has_value());
    EXPECT_TRUE(i2c_controller->write_byte(0xCD).has_value());
    
    EXPECT_TRUE(i2c_controller->end_transaction().has_value());
    
    auto read_result = i2c_controller->read_registers(device_addr, 0x10, 2);
    ASSERT_TRUE(read_result.has_value());
    std::vector<uint8_t> expected = {0xAB, 0xCD};
    EXPECT_EQ(read_result.value(), expected);
}

TEST_F(I2CControllerTest, RepeatedStart) {
    uint8_t device_addr = 0x3C;
    uint8_t register_addr = 0x00;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto transaction = i2c_controller->begin_transaction(device_addr);
    ASSERT_TRUE(transaction.has_value());
    
    EXPECT_TRUE(i2c_controller->write_byte(register_addr).has_value());
    
    EXPECT_TRUE(i2c_controller->repeated_start(device_addr | 0x01).has_value());
    
    auto read_result = i2c_controller->read_byte();
    ASSERT_TRUE(read_result.has_value());
    
    EXPECT_TRUE(i2c_controller->end_transaction().has_value());
}

TEST_F(I2CControllerTest, ClockStretching) {
    uint8_t device_addr = 0x50;
    
    EXPECT_TRUE(i2c_controller->configure(100000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    i2c_controller->enable_clock_stretching(device_addr, true);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto write_result = i2c_controller->write_register(device_addr, 0x00, 0xFF);
    ASSERT_TRUE(write_result.has_value());
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    EXPECT_GT(duration.count(), 100);
}

TEST_F(I2CControllerTest, ErrorRecovery) {
    uint8_t device_addr = 0x24;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto transaction = i2c_controller->begin_transaction(device_addr);
    ASSERT_TRUE(transaction.has_value());
    
    i2c_controller->simulate_bus_error();
    
    auto write_result = i2c_controller->write_byte(0xFF);
    EXPECT_FALSE(write_result.has_value());
    EXPECT_EQ(write_result.error(), utils::ErrorCode::COMMUNICATION_ERROR);
    
    EXPECT_TRUE(i2c_controller->reset_bus().has_value());
    
    auto recovery_transaction = i2c_controller->begin_transaction(device_addr);
    EXPECT_TRUE(recovery_transaction.has_value());
    EXPECT_TRUE(i2c_controller->write_byte(0xFF).has_value());
    EXPECT_TRUE(i2c_controller->end_transaction().has_value());
}

TEST_F(I2CControllerTest, MasterSlaveMode) {
    auto slave_controller = I2CController::create(1, memory_controller);
    ASSERT_TRUE(slave_controller.has_value());
    ASSERT_TRUE(slave_controller.value()->initialize().has_value());
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    EXPECT_TRUE(slave_controller.value()->configure(400000, I2CController::Mode::SLAVE, 0x42).has_value());
    
    std::vector<uint8_t> received_data;
    auto callback = [&received_data](const std::vector<uint8_t>& data) {
        received_data = data;
    };
    EXPECT_TRUE(slave_controller.value()->set_receive_callback(callback).has_value());
    
    std::vector<uint8_t> test_data = {0xDE, 0xAD, 0xBE, 0xEF};
    auto write_result = i2c_controller->write_bulk(0x42, test_data);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_EQ(received_data, test_data);
    
    slave_controller.value()->shutdown();
}

TEST_F(I2CControllerTest, PerformanceBenchmark) {
    uint8_t device_addr = 0x60;
    const int num_operations = 1000;
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    i2c_controller->add_virtual_device(device_addr);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_operations; ++i) {
        uint8_t register_addr = i % 256;
        uint8_t data = (i * 7) % 256;
        
        ASSERT_TRUE(i2c_controller->write_register(device_addr, register_addr, data).has_value());
        
        auto read_result = i2c_controller->read_register(device_addr, register_addr);
        ASSERT_TRUE(read_result.has_value());
        EXPECT_EQ(read_result.value(), data);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    double ops_per_second = (2.0 * num_operations * 1000.0) / duration.count();
    EXPECT_GT(ops_per_second, 1000.0);
}

TEST_F(I2CControllerTest, ConcurrentAccess) {
    const int num_threads = 3;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};
    
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    
    for (int i = 0; i < num_threads; ++i) {
        uint8_t device_addr = 0x20 + i;
        i2c_controller->add_virtual_device(device_addr);
        
        threads.emplace_back([&, device_addr]() {
            for (int j = 0; j < 100; ++j) {
                uint8_t register_addr = j % 256;
                uint8_t data = (j * 13) % 256;
                
                if (i2c_controller->write_register(device_addr, register_addr, data).has_value()) {
                    auto read_result = i2c_controller->read_register(device_addr, register_addr);
                    if (read_result.has_value() && read_result.value() == data) {
                        success_count++;
                    }
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    EXPECT_EQ(success_count.load(), num_threads * 100);
}

TEST_F(I2CControllerTest, AddressValidation) {
    EXPECT_TRUE(i2c_controller->configure(400000, I2CController::Mode::MASTER).has_value());
    
    auto result_7bit = i2c_controller->write_register(0x7F, 0x00, 0xFF);
    EXPECT_TRUE(result_7bit.has_value());
    
    auto result_invalid = i2c_controller->write_register(0x80, 0x00, 0xFF);
    EXPECT_FALSE(result_invalid.has_value());
    EXPECT_EQ(result_invalid.error(), utils::ErrorCode::INVALID_ADDRESS);
    
    auto result_reserved = i2c_controller->write_register(0x00, 0x00, 0xFF);
    EXPECT_FALSE(result_reserved.has_value());
    EXPECT_EQ(result_reserved.error(), utils::ErrorCode::INVALID_ADDRESS);
}

} // namespace emulator::peripherals::test