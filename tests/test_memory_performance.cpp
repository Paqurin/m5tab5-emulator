#include <gtest/gtest.h>
#include "emulator/memory/memory_controller.hpp"
#include <memory>
#include <chrono>
#include <vector>
#include <thread>
#include <random>

namespace emulator::memory::performance::test {

class MemoryPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value());
    }
    
    void TearDown() override {
        if (memory_controller) {
            memory_controller->shutdown();
        }
    }
    
    std::shared_ptr<MemoryController> memory_controller;
};

TEST_F(MemoryPerformanceTest, SequentialAccessThroughput) {
    const size_t buffer_size = 1024 * 1024;  // 1MB
    const uint32_t base_address = PSRAM_BASE;
    
    std::vector<uint8_t> test_data(buffer_size);
    std::iota(test_data.begin(), test_data.end(), 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    ASSERT_TRUE(memory_controller->write_bulk(base_address, test_data).has_value());
    
    auto write_end_time = std::chrono::high_resolution_clock::now();
    
    auto read_result = memory_controller->read_bulk(base_address, buffer_size);
    ASSERT_TRUE(read_result.has_value());
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(write_end_time - start_time);
    auto read_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - write_end_time);
    
    double write_throughput_mbps = (buffer_size / 1024.0 / 1024.0) / (write_duration.count() / 1e6);
    double read_throughput_mbps = (buffer_size / 1024.0 / 1024.0) / (read_duration.count() / 1e6);
    
    std::cout << "Sequential Memory Performance:" << std::endl;
    std::cout << "  Write throughput: " << write_throughput_mbps << " MB/s" << std::endl;
    std::cout << "  Read throughput: " << read_throughput_mbps << " MB/s" << std::endl;
    
    EXPECT_GT(write_throughput_mbps, 10.0);
    EXPECT_GT(read_throughput_mbps, 10.0);
    EXPECT_EQ(read_result.value(), test_data);
}

TEST_F(MemoryPerformanceTest, RandomAccessLatency) {
    const int num_accesses = 10000;
    const uint32_t base_address = PSRAM_BASE;
    const size_t region_size = 64 * 1024;  // 64KB region
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> addr_dist(0, region_size - 4);
    
    std::vector<uint32_t> addresses;
    std::vector<uint32_t> values;
    
    for (int i = 0; i < num_accesses; ++i) {
        addresses.push_back(base_address + (addr_dist(gen) & ~3));  // Align to 4 bytes
        values.push_back(gen());
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_accesses; ++i) {
        ASSERT_TRUE(memory_controller->write32(addresses[i], values[i]).has_value());
    }
    
    auto write_end_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_accesses; ++i) {
        auto read_result = memory_controller->read32(addresses[i]);
        ASSERT_TRUE(read_result.has_value());
        EXPECT_EQ(read_result.value(), values[i]);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto write_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(write_end_time - start_time);
    auto read_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - write_end_time);
    
    double avg_write_latency = write_duration.count() / double(num_accesses);
    double avg_read_latency = read_duration.count() / double(num_accesses);
    
    std::cout << "Random Access Performance:" << std::endl;
    std::cout << "  Average write latency: " << avg_write_latency << " ns" << std::endl;
    std::cout << "  Average read latency: " << avg_read_latency << " ns" << std::endl;
    
    EXPECT_LT(avg_write_latency, 10000);  // Less than 10μs
    EXPECT_LT(avg_read_latency, 5000);    // Less than 5μs
}

TEST_F(MemoryPerformanceTest, ConcurrentAccessScalability) {
    const std::vector<int> thread_counts = {1, 2, 4, 8};
    const int operations_per_thread = 5000;
    
    for (int num_threads : thread_counts) {
        std::vector<std::thread> threads;
        std::atomic<int> successful_operations{0};
        std::atomic<int> failed_operations{0};
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < num_threads; ++i) {
            threads.emplace_back([&, i]() {
                uint32_t base_addr = PSRAM_BASE + (i * 0x10000);  // 64KB per thread
                
                for (int j = 0; j < operations_per_thread; ++j) {
                    uint32_t address = base_addr + (j * 4);
                    uint32_t value = (i << 16) | j;
                    
                    if (memory_controller->write32(address, value).has_value()) {
                        auto read_result = memory_controller->read32(address);
                        if (read_result.has_value() && read_result.value() == value) {
                            successful_operations++;
                        } else {
                            failed_operations++;
                        }
                    } else {
                        failed_operations++;
                    }
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        double operations_per_second = (successful_operations.load() * 1000.0) / duration.count();
        double success_rate = double(successful_operations.load()) / (num_threads * operations_per_thread);
        
        std::cout << "Concurrent Performance (" << num_threads << " threads):" << std::endl;
        std::cout << "  Operations per second: " << operations_per_second << std::endl;
        std::cout << "  Success rate: " << (success_rate * 100) << "%" << std::endl;
        std::cout << "  Failed operations: " << failed_operations.load() << std::endl;
        
        EXPECT_GT(success_rate, 0.95);
        EXPECT_GT(operations_per_second, 1000);
    }
}

TEST_F(MemoryPerformanceTest, DMATransferPerformance) {
    const std::vector<size_t> transfer_sizes = {1024, 4096, 16384, 65536, 262144};  // 1KB to 256KB
    
    for (size_t size : transfer_sizes) {
        uint32_t src_address = PSRAM_BASE;
        uint32_t dst_address = PSRAM_BASE + 0x100000;  // 1MB offset
        
        std::vector<uint8_t> test_data(size);
        std::iota(test_data.begin(), test_data.end(), 0);
        
        ASSERT_TRUE(memory_controller->write_bulk(src_address, test_data).has_value());
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        ASSERT_TRUE(memory_controller->dma_transfer(src_address, dst_address, size).has_value());
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        double throughput_mbps = (size / 1024.0 / 1024.0) / (duration.count() / 1e6);
        
        auto verify_data = memory_controller->read_bulk(dst_address, size);
        ASSERT_TRUE(verify_data.has_value());
        EXPECT_EQ(verify_data.value(), test_data);
        
        std::cout << "DMA Transfer (" << (size / 1024) << "KB): " << throughput_mbps << " MB/s" << std::endl;
        
        EXPECT_GT(throughput_mbps, 50.0);  // Expect at least 50 MB/s
    }
}

TEST_F(MemoryPerformanceTest, CacheCoherencyPerformance) {
    const int num_operations = 10000;
    const uint32_t test_address = PSRAM_BASE;
    
    std::vector<std::thread> threads;
    std::atomic<int> coherency_violations{0};
    std::atomic<int> successful_operations{0};
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Writer thread
    threads.emplace_back([&]() {
        for (int i = 0; i < num_operations; ++i) {
            uint32_t value = i;
            if (memory_controller->write32(test_address, value).has_value()) {
                successful_operations++;
            }
            std::this_thread::sleep_for(std::chrono::nanoseconds(100));
        }
    });
    
    // Reader threads
    for (int t = 0; t < 3; ++t) {
        threads.emplace_back([&]() {
            uint32_t last_value = 0;
            for (int i = 0; i < num_operations; ++i) {
                auto read_result = memory_controller->read32(test_address);
                if (read_result.has_value()) {
                    uint32_t current_value = read_result.value();
                    if (current_value < last_value) {
                        coherency_violations++;
                    }
                    last_value = current_value;
                    successful_operations++;
                }
                std::this_thread::sleep_for(std::chrono::nanoseconds(150));
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    double operations_per_second = (successful_operations.load() * 1000.0) / duration.count();
    double coherency_success_rate = 1.0 - (double(coherency_violations.load()) / successful_operations.load());
    
    std::cout << "Cache Coherency Performance:" << std::endl;
    std::cout << "  Operations per second: " << operations_per_second << std::endl;
    std::cout << "  Coherency success rate: " << (coherency_success_rate * 100) << "%" << std::endl;
    std::cout << "  Coherency violations: " << coherency_violations.load() << std::endl;
    
    EXPECT_GT(coherency_success_rate, 0.99);
    EXPECT_GT(operations_per_second, 5000);
}

TEST_F(MemoryPerformanceTest, MemoryFragmentationImpact) {
    const int num_allocations = 1000;
    const size_t allocation_size = 1024;  // 1KB allocations
    
    std::vector<uint32_t> allocated_addresses;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Allocate many small blocks
    for (int i = 0; i < num_allocations; ++i) {
        uint32_t address = PSRAM_BASE + (i * allocation_size * 2);  // Leave gaps
        allocated_addresses.push_back(address);
        
        std::vector<uint8_t> data(allocation_size, i % 256);
        ASSERT_TRUE(memory_controller->write_bulk(address, data).has_value());
    }
    
    auto allocation_end_time = std::chrono::high_resolution_clock::now();
    
    // Random access to fragmented memory
    std::shuffle(allocated_addresses.begin(), allocated_addresses.end(), gen);
    
    for (uint32_t address : allocated_addresses) {
        auto read_result = memory_controller->read_bulk(address, allocation_size);
        ASSERT_TRUE(read_result.has_value());
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto allocation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(allocation_end_time - start_time);
    auto access_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - allocation_end_time);
    
    double allocation_rate = (num_allocations * 1000.0) / allocation_duration.count();
    double access_rate = (num_allocations * 1000.0) / access_duration.count();
    
    std::cout << "Memory Fragmentation Performance:" << std::endl;
    std::cout << "  Allocation rate: " << allocation_rate << " allocs/sec" << std::endl;
    std::cout << "  Fragmented access rate: " << access_rate << " accesses/sec" << std::endl;
    
    EXPECT_GT(allocation_rate, 100);
    EXPECT_GT(access_rate, 500);
}

TEST_F(MemoryPerformanceTest, MemoryBandwidthSaturation) {
    const size_t buffer_size = 4 * 1024 * 1024;  // 4MB
    const int num_threads = 8;
    
    std::vector<std::thread> threads;
    std::atomic<double> total_bandwidth{0.0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            uint32_t base_address = PSRAM_BASE + (i * buffer_size);
            std::vector<uint8_t> data(buffer_size / num_threads);
            std::iota(data.begin(), data.end(), i);
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            for (int j = 0; j < 10; ++j) {  // 10 iterations per thread
                ASSERT_TRUE(memory_controller->write_bulk(base_address, data).has_value());
                auto read_result = memory_controller->read_bulk(base_address, data.size());
                ASSERT_TRUE(read_result.has_value());
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            double bytes_transferred = data.size() * 2 * 10;  // Read + Write, 10 iterations
            double bandwidth_mbps = (bytes_transferred / 1024.0 / 1024.0) / (duration.count() / 1e6);
            
            total_bandwidth.fetch_add(bandwidth_mbps);
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    std::cout << "Memory Bandwidth Saturation:" << std::endl;
    std::cout << "  Total bandwidth: " << total_bandwidth.load() << " MB/s" << std::endl;
    std::cout << "  Average per thread: " << (total_bandwidth.load() / num_threads) << " MB/s" << std::endl;
    
    EXPECT_GT(total_bandwidth.load(), 100.0);
}

} // namespace emulator::memory::performance::test