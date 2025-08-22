#include <gtest/gtest.h>
#include "emulator/cpu/cpu_core.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <thread>
#include <chrono>

namespace emulator::cpu::test {

class CPUCoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<memory::MemoryController>();
        
        auto cpu_result = CPUCore::create(0, memory_controller);
        ASSERT_TRUE(cpu_result.has_value()) << "Failed to create CPU core: " 
            << static_cast<int>(cpu_result.error());
        cpu_core = std::move(cpu_result.value());
        
        ASSERT_TRUE(cpu_core->initialize().has_value()) << "Failed to initialize CPU core";
    }
    
    void TearDown() override {
        if (cpu_core) {
            cpu_core->shutdown();
        }
    }
    
    std::shared_ptr<memory::MemoryController> memory_controller;
    std::unique_ptr<CPUCore> cpu_core;
};

TEST_F(CPUCoreTest, BasicInitialization) {
    EXPECT_EQ(cpu_core->get_core_id(), 0);
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::IDLE);
    EXPECT_EQ(cpu_core->get_pc(), 0);
}

TEST_F(CPUCoreTest, RegisterOperations) {
    EXPECT_TRUE(cpu_core->set_register(1, 0x12345678).has_value());
    
    auto reg_result = cpu_core->get_register(1);
    ASSERT_TRUE(reg_result.has_value());
    EXPECT_EQ(reg_result.value(), 0x12345678);
    
    auto invalid_reg = cpu_core->get_register(32);
    EXPECT_FALSE(invalid_reg.has_value());
}

TEST_F(CPUCoreTest, PCOperations) {
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_EQ(cpu_core->get_pc(), 0x1000);
    
    EXPECT_TRUE(cpu_core->increment_pc().has_value());
    EXPECT_EQ(cpu_core->get_pc(), 0x1004);
}

TEST_F(CPUCoreTest, InstructionExecution) {
    uint32_t add_instruction = 0x00208093;
    
    EXPECT_TRUE(memory_controller->write32(0x1000, add_instruction).has_value());
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_TRUE(cpu_core->set_register(1, 10).has_value());
    EXPECT_TRUE(cpu_core->set_register(2, 20).has_value());
    
    EXPECT_TRUE(cpu_core->step().has_value());
    
    auto result = cpu_core->get_register(1);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 30);
}

TEST_F(CPUCoreTest, StateTransitions) {
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::IDLE);
    
    EXPECT_TRUE(cpu_core->start().has_value());
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::RUNNING);
    
    EXPECT_TRUE(cpu_core->pause().has_value());
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::PAUSED);
    
    EXPECT_TRUE(cpu_core->resume().has_value());
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::RUNNING);
    
    EXPECT_TRUE(cpu_core->stop().has_value());
    EXPECT_EQ(cpu_core->get_state(), CPUCore::State::STOPPED);
}

TEST_F(CPUCoreTest, ExceptionHandling) {
    auto invalid_addr = cpu_core->set_pc(0xFFFFFFFF);
    EXPECT_FALSE(invalid_addr.has_value());
    EXPECT_EQ(invalid_addr.error(), utils::ErrorCode::INVALID_ADDRESS);
}

TEST_F(CPUCoreTest, InterruptHandling) {
    EXPECT_TRUE(cpu_core->trigger_interrupt(1).has_value());
    
    auto pending_interrupts = cpu_core->get_pending_interrupts();
    ASSERT_TRUE(pending_interrupts.has_value());
    EXPECT_TRUE(pending_interrupts.value() & (1 << 1));
}

TEST_F(CPUCoreTest, PerformanceCounters) {
    auto initial_cycles = cpu_core->get_cycle_count();
    
    uint32_t nop_instruction = 0x00000013;
    EXPECT_TRUE(memory_controller->write32(0x1000, nop_instruction).has_value());
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(cpu_core->step().has_value());
        EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    }
    
    auto final_cycles = cpu_core->get_cycle_count();
    EXPECT_GT(final_cycles, initial_cycles);
}

TEST_F(CPUCoreTest, ConcurrentExecution) {
    std::atomic<bool> running{true};
    std::atomic<int> step_count{0};
    
    uint32_t nop_instruction = 0x00000013;
    EXPECT_TRUE(memory_controller->write32(0x1000, nop_instruction).has_value());
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_TRUE(cpu_core->start().has_value());
    
    std::thread execution_thread([&]() {
        while (running) {
            if (cpu_core->step().has_value()) {
                step_count++;
                cpu_core->set_pc(0x1000);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    running = false;
    execution_thread.join();
    
    EXPECT_GT(step_count.load(), 0);
    EXPECT_TRUE(cpu_core->stop().has_value());
}

TEST_F(CPUCoreTest, BranchInstructions) {
    uint32_t beq_instruction = 0x00208063;
    
    EXPECT_TRUE(memory_controller->write32(0x1000, beq_instruction).has_value());
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_TRUE(cpu_core->set_register(1, 42).has_value());
    EXPECT_TRUE(cpu_core->set_register(2, 42).has_value());
    
    uint32_t original_pc = cpu_core->get_pc();
    EXPECT_TRUE(cpu_core->step().has_value());
    
    EXPECT_NE(cpu_core->get_pc(), original_pc + 4);
}

TEST_F(CPUCoreTest, LoadStoreInstructions) {
    uint32_t test_value = 0xDEADBEEF;
    uint32_t test_addr = 0x2000;
    
    uint32_t sw_instruction = 0x0020A023;
    uint32_t lw_instruction = 0x00002083;
    
    EXPECT_TRUE(memory_controller->write32(0x1000, sw_instruction).has_value());
    EXPECT_TRUE(memory_controller->write32(0x1004, lw_instruction).has_value());
    
    EXPECT_TRUE(cpu_core->set_pc(0x1000).has_value());
    EXPECT_TRUE(cpu_core->set_register(1, test_addr).has_value());
    EXPECT_TRUE(cpu_core->set_register(2, test_value).has_value());
    
    EXPECT_TRUE(cpu_core->step().has_value());
    
    EXPECT_TRUE(cpu_core->step().has_value());
    
    auto loaded_value = cpu_core->get_register(1);
    ASSERT_TRUE(loaded_value.has_value());
    EXPECT_EQ(loaded_value.value(), test_value);
}

} // namespace emulator::cpu::test