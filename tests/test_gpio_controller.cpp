#include <gtest/gtest.h>
#include "emulator/peripherals/gpio_controller.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <thread>
#include <chrono>

namespace emulator::peripherals::test {

class GPIOControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        memory_controller = std::make_shared<memory::MemoryController>();
        ASSERT_TRUE(memory_controller->initialize().has_value());
        
        auto gpio_result = GPIOController::create(memory_controller);
        ASSERT_TRUE(gpio_result.has_value()) << "Failed to create GPIO controller";
        gpio_controller = std::move(gpio_result.value());
        
        ASSERT_TRUE(gpio_controller->initialize().has_value()) 
            << "Failed to initialize GPIO controller";
    }
    
    void TearDown() override {
        if (gpio_controller) {
            gpio_controller->shutdown();
        }
    }
    
    std::shared_ptr<memory::MemoryController> memory_controller;
    std::unique_ptr<GPIOController> gpio_controller;
};

TEST_F(GPIOControllerTest, PinConfiguration) {
    uint8_t test_pin = 2;
    
    auto config_result = gpio_controller->configure_pin(test_pin, GPIOController::Mode::OUTPUT);
    ASSERT_TRUE(config_result.has_value()) << "Failed to configure pin as output";
    
    auto mode_result = gpio_controller->get_pin_mode(test_pin);
    ASSERT_TRUE(mode_result.has_value());
    EXPECT_EQ(mode_result.value(), GPIOController::Mode::OUTPUT);
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    mode_result = gpio_controller->get_pin_mode(test_pin);
    ASSERT_TRUE(mode_result.has_value());
    EXPECT_EQ(mode_result.value(), GPIOController::Mode::INPUT);
}

TEST_F(GPIOControllerTest, DigitalWrite) {
    uint8_t test_pin = 5;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::OUTPUT).has_value());
    
    EXPECT_TRUE(gpio_controller->digital_write(test_pin, true).has_value());
    auto state_result = gpio_controller->digital_read(test_pin);
    ASSERT_TRUE(state_result.has_value());
    EXPECT_TRUE(state_result.value());
    
    EXPECT_TRUE(gpio_controller->digital_write(test_pin, false).has_value());
    state_result = gpio_controller->digital_read(test_pin);
    ASSERT_TRUE(state_result.has_value());
    EXPECT_FALSE(state_result.value());
}

TEST_F(GPIOControllerTest, DigitalRead) {
    uint8_t test_pin = 10;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    gpio_controller->simulate_external_input(test_pin, true);
    auto read_result = gpio_controller->digital_read(test_pin);
    ASSERT_TRUE(read_result.has_value());
    EXPECT_TRUE(read_result.value());
    
    gpio_controller->simulate_external_input(test_pin, false);
    read_result = gpio_controller->digital_read(test_pin);
    ASSERT_TRUE(read_result.has_value());
    EXPECT_FALSE(read_result.value());
}

TEST_F(GPIOControllerTest, PullResistors) {
    uint8_t test_pin = 15;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    EXPECT_TRUE(gpio_controller->set_pull_mode(test_pin, GPIOController::Pull::UP).has_value());
    auto pull_result = gpio_controller->get_pull_mode(test_pin);
    ASSERT_TRUE(pull_result.has_value());
    EXPECT_EQ(pull_result.value(), GPIOController::Pull::UP);
    
    EXPECT_TRUE(gpio_controller->set_pull_mode(test_pin, GPIOController::Pull::DOWN).has_value());
    pull_result = gpio_controller->get_pull_mode(test_pin);
    ASSERT_TRUE(pull_result.has_value());
    EXPECT_EQ(pull_result.value(), GPIOController::Pull::DOWN);
    
    EXPECT_TRUE(gpio_controller->set_pull_mode(test_pin, GPIOController::Pull::NONE).has_value());
    pull_result = gpio_controller->get_pull_mode(test_pin);
    ASSERT_TRUE(pull_result.has_value());
    EXPECT_EQ(pull_result.value(), GPIOController::Pull::NONE);
}

TEST_F(GPIOControllerTest, InterruptConfiguration) {
    uint8_t test_pin = 20;
    bool interrupt_triggered = false;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    auto callback = [&interrupt_triggered](uint8_t pin, bool state) {
        interrupt_triggered = true;
    };
    
    EXPECT_TRUE(gpio_controller->attach_interrupt(test_pin, 
        GPIOController::InterruptTrigger::RISING_EDGE, callback).has_value());
    
    gpio_controller->simulate_external_input(test_pin, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_FALSE(interrupt_triggered);
    
    gpio_controller->simulate_external_input(test_pin, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(interrupt_triggered);
}

TEST_F(GPIOControllerTest, InterruptTypes) {
    uint8_t test_pin = 25;
    int interrupt_count = 0;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    auto callback = [&interrupt_count](uint8_t pin, bool state) {
        interrupt_count++;
    };
    
    EXPECT_TRUE(gpio_controller->attach_interrupt(test_pin, 
        GPIOController::InterruptTrigger::BOTH_EDGES, callback).has_value());
    
    gpio_controller->simulate_external_input(test_pin, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    gpio_controller->simulate_external_input(test_pin, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    gpio_controller->simulate_external_input(test_pin, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    EXPECT_EQ(interrupt_count, 3);
}

TEST_F(GPIOControllerTest, InvalidPinHandling) {
    uint8_t invalid_pin = 100;
    
    auto config_result = gpio_controller->configure_pin(invalid_pin, GPIOController::Mode::OUTPUT);
    EXPECT_FALSE(config_result.has_value());
    EXPECT_EQ(config_result.error(), utils::ErrorCode::INVALID_PIN);
    
    auto write_result = gpio_controller->digital_write(invalid_pin, true);
    EXPECT_FALSE(write_result.has_value());
    EXPECT_EQ(write_result.error(), utils::ErrorCode::INVALID_PIN);
    
    auto read_result = gpio_controller->digital_read(invalid_pin);
    EXPECT_FALSE(read_result.has_value());
    EXPECT_EQ(read_result.error(), utils::ErrorCode::INVALID_PIN);
}

TEST_F(GPIOControllerTest, PinStateValidation) {
    uint8_t test_pin = 30;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    auto write_result = gpio_controller->digital_write(test_pin, true);
    EXPECT_FALSE(write_result.has_value());
    EXPECT_EQ(write_result.error(), utils::ErrorCode::INVALID_OPERATION);
}

TEST_F(GPIOControllerTest, MultipleInterrupts) {
    std::vector<uint8_t> test_pins = {1, 3, 5, 7};
    std::atomic<int> total_interrupts{0};
    
    for (auto pin : test_pins) {
        EXPECT_TRUE(gpio_controller->configure_pin(pin, GPIOController::Mode::INPUT).has_value());
        
        auto callback = [&total_interrupts](uint8_t pin, bool state) {
            total_interrupts++;
        };
        
        EXPECT_TRUE(gpio_controller->attach_interrupt(pin, 
            GPIOController::InterruptTrigger::RISING_EDGE, callback).has_value());
    }
    
    for (auto pin : test_pins) {
        gpio_controller->simulate_external_input(pin, true);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(total_interrupts.load(), test_pins.size());
}

TEST_F(GPIOControllerTest, InterruptDetach) {
    uint8_t test_pin = 35;
    bool interrupt_triggered = false;
    
    EXPECT_TRUE(gpio_controller->configure_pin(test_pin, GPIOController::Mode::INPUT).has_value());
    
    auto callback = [&interrupt_triggered](uint8_t pin, bool state) {
        interrupt_triggered = true;
    };
    
    EXPECT_TRUE(gpio_controller->attach_interrupt(test_pin, 
        GPIOController::InterruptTrigger::RISING_EDGE, callback).has_value());
    
    EXPECT_TRUE(gpio_controller->detach_interrupt(test_pin).has_value());
    
    gpio_controller->simulate_external_input(test_pin, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_FALSE(interrupt_triggered);
}

TEST_F(GPIOControllerTest, AllPinsConfiguration) {
    for (uint8_t pin = 0; pin < 55; ++pin) {
        if (gpio_controller->is_pin_available(pin)) {
            EXPECT_TRUE(gpio_controller->configure_pin(pin, GPIOController::Mode::OUTPUT).has_value())
                << "Failed to configure pin " << static_cast<int>(pin);
            
            EXPECT_TRUE(gpio_controller->digital_write(pin, true).has_value())
                << "Failed to write to pin " << static_cast<int>(pin);
            
            auto read_result = gpio_controller->digital_read(pin);
            ASSERT_TRUE(read_result.has_value()) 
                << "Failed to read pin " << static_cast<int>(pin);
            EXPECT_TRUE(read_result.value()) 
                << "Pin " << static_cast<int>(pin) << " state mismatch";
        }
    }
}

TEST_F(GPIOControllerTest, ConcurrentAccess) {
    const int num_threads = 4;
    const int pins_per_thread = 10;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            for (int j = 0; j < pins_per_thread; ++j) {
                uint8_t pin = i * pins_per_thread + j;
                if (pin < 55 && gpio_controller->is_pin_available(pin)) {
                    if (gpio_controller->configure_pin(pin, GPIOController::Mode::OUTPUT).has_value()) {
                        if (gpio_controller->digital_write(pin, true).has_value()) {
                            auto read_result = gpio_controller->digital_read(pin);
                            if (read_result.has_value() && read_result.value()) {
                                success_count++;
                            }
                        }
                    }
                }
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    EXPECT_GT(success_count.load(), 0);
}

} // namespace emulator::peripherals::test