#include <gtest/gtest.h>
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include <memory>
#include <chrono>
#include <vector>
#include <thread>
#include <random>

namespace emulator::cpu::performance::test {

class CPUPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<memory::MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value());
        
        auto cpu_result = CPUCore::create(0, memory_controller);
        ASSERT_TRUE(cpu_result.has_value());
        cpu_core = std::move(cpu_result.value());
        ASSERT_TRUE(cpu_core->initialize().has_value());
    }
    
    void TearDown() override {
        if (cpu_core) {
            cpu_core->shutdown();
        }
    }
    
    std::shared_ptr<memory::MemoryController> memory_controller;
    std::unique_ptr<CPUCore> cpu_core;
    
    struct PerformanceResult {
        double operations_per_second;
        double average_latency_ns;
        double peak_memory_usage_mb;
        size_t cache_hits;
        size_t cache_misses;
    };
    
    void load_test_program(uint32_t base_address, const std::vector<uint32_t>& instructions) {
        for (size_t i = 0; i < instructions.size(); ++i) {
            ASSERT_TRUE(memory_controller->write32(base_address + (i * 4), instructions[i]).has_value());
        }
    }
};

TEST_F(CPUPerformanceTest, InstructionExecutionThroughput) {
    const uint32_t base_address = 0x1000;
    const int num_iterations = 100000;
    
    std::vector<uint32_t> test_program = {
        0x00100093,  // addi x1, x0, 1
        0x00200113,  // addi x2, x0, 2
        0x002081B3,  // add x3, x1, x2
        0x0030A223,  // sw x3, 4(x1)
        0x00000013,  // nop
        0xFF5FF06F   // jal x0, -12 (loop back)
    };
    
    load_test_program(base_address, test_program);
    EXPECT_TRUE(cpu_core->set_pc(base_address).has_value());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_iterations; ++i) {
        ASSERT_TRUE(cpu_core->step().has_value());
        if (cpu_core->get_pc() >= base_address + (test_program.size() * 4)) {
            cpu_core->set_pc(base_address);
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    
    double instructions_per_second = (num_iterations * 1e9) / duration.count();
    double average_instruction_time = duration.count() / double(num_iterations);
    
    std::cout << "Instruction Execution Performance:" << std::endl;
    std::cout << "  Instructions per second: " << instructions_per_second << std::endl;
    std::cout << "  Average instruction time: " << average_instruction_time << " ns" << std::endl;
    
    EXPECT_GT(instructions_per_second, 10000);
    EXPECT_LT(average_instruction_time, 100000);
}

TEST_F(CPUPerformanceTest, MemoryAccessPerformance) {
    const int num_operations = 50000;
    const uint32_t base_address = 0x20000000;
    
    std::vector<uint32_t> test_program = {
        0x00100093,  // addi x1, x0, 1
        0x14000113,  // addi x2, x0, 320 (base_address >> 16)
        0x00011117,  // auipc x2, base_address[31:12]
        0x00010113,  // addi x2, x2, base_address[11:0]
        0x0000A003,  // lw x0, 0(x1)
        0x00112023,  // sw x1, 0(x2)
        0x00408093,  // addi x1, x1, 4
        0xFF9FF06F   // jal x0, -8 (loop back)
    };
    
    load_test_program(0x1000, test_program);
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_TRUE(cpu_core->set_register(1, base_address).has_value());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_operations; ++i) {
        ASSERT_TRUE(cpu_core->step().has_value());
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    
    double operations_per_second = (num_operations * 1e9) / duration.count();
    double average_access_time = duration.count() / double(num_operations);
    
    std::cout << "Memory Access Performance:" << std::endl;
    std::cout << "  Memory operations per second: " << operations_per_second << std::endl;
    std::cout << "  Average memory access time: " << average_access_time << " ns" << std::endl;
    
    EXPECT_GT(operations_per_second, 5000);
}

TEST_F(CPUPerformanceTest, BranchPredictionPerformance) {
    const int num_branches = 20000;
    const uint32_t base_address = 0x1000;
    
    std::vector<uint32_t> test_program = {
        0x00100093,  // addi x1, x0, 1
        0x00000113,  // addi x2, x0, 0
        0x00208133,  // add x2, x1, x2
        0x00208163,  // beq x1, x2, 8  (taken branch)
        0x00100093,  // addi x1, x0, 1  (not taken path)
        0x00000013,  // nop
        0x00108093,  // addi x1, x1, 1  (taken path)
        0xFF1FF06F   // jal x0, -16 (loop back)
    };
    
    load_test_program(base_address, test_program);
    EXPECT_TRUE(cpu_core->set_pc(base_address).has_value());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_branches; ++i) {
        while (true) {
            ASSERT_TRUE(cpu_core->step().has_value());
            
            if (cpu_core->get_pc() == base_address) {
                break;
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    
    double branches_per_second = (num_branches * 1e9) / duration.count();
    double average_branch_time = duration.count() / double(num_branches);
    
    std::cout << "Branch Performance:" << std::endl;
    std::cout << "  Branches per second: " << branches_per_second << std::endl;
    std::cout << "  Average branch time: " << average_branch_time << " ns" << std::endl;
    
    EXPECT_GT(branches_per_second, 1000);
}

TEST_F(CPUPerformanceTest, MultiCorePerformance) {
    std::vector<std::unique_ptr<CPUCore>> cores;
    const int num_cores = 4;
    const int operations_per_core = 10000;
    
    for (int i = 0; i < num_cores; ++i) {
        auto core_result = CPUCore::create(i, memory_controller);
        ASSERT_TRUE(core_result.has_value());
        ASSERT_TRUE(core_result.value()->initialize().has_value());
        cores.push_back(std::move(core_result.value()));
    }
    
    std::vector<uint32_t> test_program = {
        0x00100093,  // addi x1, x0, 1
        0x00200113,  // addi x2, x0, 2
        0x002081B3,  // add x3, x1, x2
        0x00108093,  // addi x1, x1, 1
        0xFFDFF06F   // jal x0, -4 (loop back)
    };
    
    load_test_program(0x1000, test_program);
    
    std::vector<std::thread> threads;
    std::atomic<int> total_operations{0};
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_cores; ++i) {
        threads.emplace_back([&, i]() {
            cores[i]->set_pc(0x1000);
            
            for (int j = 0; j < operations_per_core; ++j) {
                if (cores[i]->step().has_value()) {
                    total_operations++;
                }
                
                if (cores[i]->get_pc() >= 0x1014) {
                    cores[i]->set_pc(0x1000);
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    
    double total_ops_per_second = (total_operations.load() * 1e9) / duration.count();
    double scalability = total_ops_per_second / num_cores;
    
    std::cout << "Multi-Core Performance:" << std::endl;
    std::cout << "  Total operations per second: " << total_ops_per_second << std::endl;
    std::cout << "  Operations per core per second: " << scalability << std::endl;
    std::cout << "  Completed operations: " << total_operations.load() << std::endl;
    
    EXPECT_GT(total_ops_per_second, 1000);
    EXPECT_EQ(total_operations.load(), num_cores * operations_per_core);
    
    for (auto& core : cores) {
        core->shutdown();
    }
}

TEST_F(CPUPerformanceTest, InterruptLatencyBenchmark) {
    const int num_interrupts = 1000;
    const uint32_t base_address = 0x1000;
    
    std::vector<uint32_t> idle_program = {
        0x00000013,  // nop
        0x00000013,  // nop
        0x00000013,  // nop
        0x00000013,  // nop
        0xFFCFF06F   // jal x0, -4 (loop back)
    };
    
    load_test_program(base_address, idle_program);
    EXPECT_TRUE(cpu_core->set_pc(base_address).has_value());
    EXPECT_TRUE(cpu_core->start().has_value());
    
    std::vector<double> interrupt_latencies;
    
    for (int i = 0; i < num_interrupts; ++i) {
        auto interrupt_time = std::chrono::high_resolution_clock::now();
        
        ASSERT_TRUE(cpu_core->trigger_interrupt(1).has_value());
        
        while (true) {
            auto pending = cpu_core->get_pending_interrupts();
            if (!pending.has_value() || (pending.value() & (1 << 1)) == 0) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::nanoseconds(100));
        }
        
        auto handled_time = std::chrono::high_resolution_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::nanoseconds>(handled_time - interrupt_time);
        interrupt_latencies.push_back(latency.count());
    }
    
    EXPECT_TRUE(cpu_core->stop().has_value());
    
    double average_latency = 0;
    double max_latency = 0;
    double min_latency = std::numeric_limits<double>::max();
    
    for (double latency : interrupt_latencies) {
        average_latency += latency;
        max_latency = std::max(max_latency, latency);
        min_latency = std::min(min_latency, latency);
    }
    average_latency /= interrupt_latencies.size();
    
    std::cout << "Interrupt Performance:" << std::endl;
    std::cout << "  Average interrupt latency: " << average_latency << " ns" << std::endl;
    std::cout << "  Maximum interrupt latency: " << max_latency << " ns" << std::endl;
    std::cout << "  Minimum interrupt latency: " << min_latency << " ns" << std::endl;
    
    EXPECT_LT(average_latency, 50000);
    EXPECT_LT(max_latency, 100000);
}

TEST_F(CPUPerformanceTest, CachePerformanceBenchmark) {
    const int num_accesses = 10000;
    const uint32_t base_address = 0x20000000;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> addr_dist(0, 1024);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int cache_hits = 0;
    for (int i = 0; i < num_accesses; ++i) {
        uint32_t offset = addr_dist(gen) * 4;
        uint32_t address = base_address + offset;
        
        auto read_result = memory_controller->read32(address);
        if (read_result.has_value()) {
            cache_hits++;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    
    double accesses_per_second = (num_accesses * 1e9) / duration.count();
    double average_access_time = duration.count() / double(num_accesses);
    double cache_hit_rate = double(cache_hits) / num_accesses;
    
    std::cout << "Cache Performance:" << std::endl;
    std::cout << "  Memory accesses per second: " << accesses_per_second << std::endl;
    std::cout << "  Average access time: " << average_access_time << " ns" << std::endl;
    std::cout << "  Cache hit rate: " << (cache_hit_rate * 100) << "%" << std::endl;
    
    EXPECT_GT(accesses_per_second, 10000);
    EXPECT_GT(cache_hit_rate, 0.8);
}

TEST_F(CPUPerformanceTest, ScalabilityTest) {
    std::vector<int> thread_counts = {1, 2, 4, 8};
    std::vector<double> throughput_results;
    
    for (int num_threads : thread_counts) {
        const int operations_per_thread = 5000;
        
        std::vector<std::thread> threads;
        std::atomic<int> completed_operations{0};
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < num_threads; ++i) {
            threads.emplace_back([&, i]() {
                for (int j = 0; j < operations_per_thread; ++j) {
                    uint32_t register_id = (j % 31) + 1;
                    uint32_t value = i * 1000 + j;
                    
                    if (cpu_core->set_register(register_id, value).has_value()) {
                        auto read_result = cpu_core->get_register(register_id);
                        if (read_result.has_value() && read_result.value() == value) {
                            completed_operations++;
                        }
                    }
                }
            });
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        
        double throughput = (completed_operations.load() * 1e9) / duration.count();
        throughput_results.push_back(throughput);
        
        std::cout << "Threads: " << num_threads << ", Throughput: " << throughput << " ops/sec" << std::endl;
    }
    
    for (size_t i = 1; i < throughput_results.size(); ++i) {
        double scaling_factor = throughput_results[i] / throughput_results[0];
        double expected_scaling = thread_counts[i];
        double efficiency = scaling_factor / expected_scaling;
        
        std::cout << "Scaling efficiency (" << thread_counts[i] << " threads): " 
                  << (efficiency * 100) << "%" << std::endl;
        
        EXPECT_GT(efficiency, 0.5);
    }
}

} // namespace emulator::cpu::performance::test