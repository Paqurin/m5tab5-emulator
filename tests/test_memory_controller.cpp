#include <gtest/gtest.h>
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <thread>
#include <vector>
#include <random>

namespace emulator::memory::test {

class MemoryControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value()) 
            << "Failed to initialize memory controller";
    }
    
    void TearDown() override {
        if (memory_controller) {
            memory_controller->shutdown();
        }
    }
    
    std::shared_ptr<MemoryController> memory_controller;
};

TEST_F(MemoryControllerTest, BasicReadWrite) {
    uint32_t test_address = 0x20000000;
    uint32_t test_value = 0xDEADBEEF;
    
    auto write_result = memory_controller->write32(test_address, test_value);
    ASSERT_TRUE(write_result.has_value()) << "Write operation failed";
    
    auto read_result = memory_controller->read32(test_address);
    ASSERT_TRUE(read_result.has_value()) << "Read operation failed";
    EXPECT_EQ(read_result.value(), test_value);
}

TEST_F(MemoryControllerTest, DifferentDataSizes) {
    uint32_t base_address = 0x20000000;
    
    uint8_t byte_value = 0xAB;
    EXPECT_TRUE(memory_controller->write8(base_address, byte_value).has_value());
    auto byte_result = memory_controller->read8(base_address);
    ASSERT_TRUE(byte_result.has_value());
    EXPECT_EQ(byte_result.value(), byte_value);
    
    uint16_t halfword_value = 0x1234;
    EXPECT_TRUE(memory_controller->write16(base_address + 4, halfword_value).has_value());
    auto halfword_result = memory_controller->read16(base_address + 4);
    ASSERT_TRUE(halfword_result.has_value());
    EXPECT_EQ(halfword_result.value(), halfword_value);
    
    uint32_t word_value = 0x12345678;
    EXPECT_TRUE(memory_controller->write32(base_address + 8, word_value).has_value());
    auto word_result = memory_controller->read32(base_address + 8);
    ASSERT_TRUE(word_result.has_value());
    EXPECT_EQ(word_result.value(), word_value);
}

TEST_F(MemoryControllerTest, MemoryRegionValidation) {
    auto valid_write = memory_controller->write32(PSRAM_BASE, 0x12345678);
    EXPECT_TRUE(valid_write.has_value());
    
    auto invalid_write = memory_controller->write32(0xFFFFFFFF, 0x12345678);
    EXPECT_FALSE(invalid_write.has_value());
    EXPECT_EQ(invalid_write.error(), utils::ErrorCode::INVALID_ADDRESS);
}

TEST_F(MemoryControllerTest, BulkOperations) {
    uint32_t base_address = 0x20000000;
    std::vector<uint8_t> test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
    
    auto write_result = memory_controller->write_bulk(base_address, test_data);
    ASSERT_TRUE(write_result.has_value()) << "Bulk write failed";
    
    auto read_result = memory_controller->read_bulk(base_address, test_data.size());
    ASSERT_TRUE(read_result.has_value()) << "Bulk read failed";
    EXPECT_EQ(read_result.value(), test_data);
}

TEST_F(MemoryControllerTest, AlignmentChecking) {
    uint32_t unaligned_address = 0x20000001;
    
    auto aligned_write = memory_controller->write32(0x20000000, 0x12345678);
    EXPECT_TRUE(aligned_write.has_value());
    
    auto unaligned_write = memory_controller->write32(unaligned_address, 0x12345678);
    EXPECT_FALSE(unaligned_write.has_value());
    EXPECT_EQ(unaligned_write.error(), utils::ErrorCode::ALIGNMENT_ERROR);
}

TEST_F(MemoryControllerTest, CacheCoherency) {
    uint32_t test_address = 0x20000000;
    uint32_t original_value = 0x11111111;
    uint32_t modified_value = 0x22222222;
    
    EXPECT_TRUE(memory_controller->write32(test_address, original_value).has_value());
    
    auto read1 = memory_controller->read32(test_address);
    ASSERT_TRUE(read1.has_value());
    EXPECT_EQ(read1.value(), original_value);
    
    EXPECT_TRUE(memory_controller->write32(test_address, modified_value).has_value());
    
    auto read2 = memory_controller->read32(test_address);
    ASSERT_TRUE(read2.has_value());
    EXPECT_EQ(read2.value(), modified_value);
}

TEST_F(MemoryControllerTest, DMAOperations) {
    uint32_t src_address = 0x20000000;
    uint32_t dst_address = 0x20001000;
    size_t transfer_size = 256;
    
    std::vector<uint8_t> test_data(transfer_size);
    std::iota(test_data.begin(), test_data.end(), 0);
    
    EXPECT_TRUE(memory_controller->write_bulk(src_address, test_data).has_value());
    
    auto dma_result = memory_controller->dma_transfer(src_address, dst_address, transfer_size);
    ASSERT_TRUE(dma_result.has_value()) << "DMA transfer failed";
    
    auto verify_data = memory_controller->read_bulk(dst_address, transfer_size);
    ASSERT_TRUE(verify_data.has_value());
    EXPECT_EQ(verify_data.value(), test_data);
}

TEST_F(MemoryControllerTest, MemoryProtection) {
    uint32_t protected_address = ROM_BASE;
    
    auto read_result = memory_controller->read32(protected_address);
    EXPECT_TRUE(read_result.has_value());
    
    auto write_result = memory_controller->write32(protected_address, 0x12345678);
    EXPECT_FALSE(write_result.has_value());
    EXPECT_EQ(write_result.error(), utils::ErrorCode::ACCESS_VIOLATION);
}

TEST_F(MemoryControllerTest, ConcurrentAccess) {
    const int num_threads = 4;
    const int operations_per_thread = 100;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            uint32_t base_address = 0x20000000 + (i * 0x1000);
            
            for (int j = 0; j < operations_per_thread; ++j) {
                uint32_t address = base_address + (j * 4);
                uint32_t value = (i << 16) | j;
                
                if (memory_controller->write32(address, value).has_value()) {
                    auto read_result = memory_controller->read32(address);
                    if (read_result.has_value() && read_result.value() == value) {
                        success_count++;
                    }
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    EXPECT_EQ(success_count.load(), num_threads * operations_per_thread);
}

TEST_F(MemoryControllerTest, PerformanceMetrics) {
    auto initial_stats = memory_controller->get_statistics();
    ASSERT_TRUE(initial_stats.has_value());
    
    uint32_t test_address = 0x20000000;
    
    for (int i = 0; i < 100; ++i) {
        EXPECT_TRUE(memory_controller->write32(test_address + (i * 4), i).has_value());
        EXPECT_TRUE(memory_controller->read32(test_address + (i * 4)).has_value());
    }
    
    auto final_stats = memory_controller->get_statistics();
    ASSERT_TRUE(final_stats.has_value());
    
    EXPECT_GT(final_stats.value().total_reads, initial_stats.value().total_reads);
    EXPECT_GT(final_stats.value().total_writes, initial_stats.value().total_writes);
}

TEST_F(MemoryControllerTest, MemoryMapping) {
    uint32_t virtual_addr = 0x40000000;
    uint32_t physical_addr = 0x20000000;
    size_t mapping_size = 4096;
    
    auto mapping_result = memory_controller->map_region(virtual_addr, physical_addr, mapping_size);
    ASSERT_TRUE(mapping_result.has_value()) << "Memory mapping failed";
    
    uint32_t test_value = 0x87654321;
    EXPECT_TRUE(memory_controller->write32(virtual_addr, test_value).has_value());
    
    auto read_virtual = memory_controller->read32(virtual_addr);
    auto read_physical = memory_controller->read32(physical_addr);
    
    ASSERT_TRUE(read_virtual.has_value());
    ASSERT_TRUE(read_physical.has_value());
    EXPECT_EQ(read_virtual.value(), read_physical.value());
    EXPECT_EQ(read_virtual.value(), test_value);
}

TEST_F(MemoryControllerTest, StressTest) {
    const size_t num_iterations = 1000;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> addr_dist(PSRAM_BASE, PSRAM_BASE + 0x10000);
    std::uniform_int_distribution<uint32_t> value_dist(0, 0xFFFFFFFF);
    
    std::unordered_map<uint32_t, uint32_t> written_values;
    
    for (size_t i = 0; i < num_iterations; ++i) {
        uint32_t address = addr_dist(gen) & ~3;
        uint32_t value = value_dist(gen);
        
        if (memory_controller->write32(address, value).has_value()) {
            written_values[address] = value;
        }
    }
    
    for (const auto& [address, expected_value] : written_values) {
        auto read_result = memory_controller->read32(address);
        ASSERT_TRUE(read_result.has_value()) 
            << "Failed to read address 0x" << std::hex << address;
        EXPECT_EQ(read_result.value(), expected_value)
            << "Value mismatch at address 0x" << std::hex << address;
    }
}

} // namespace emulator::memory::test