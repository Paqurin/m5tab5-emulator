/**
 * @file gpio_control.cpp
 * @brief GPIO control example demonstrating pin configuration, digital I/O, and interrupts
 * 
 * This example demonstrates:
 * - GPIO pin configuration (input/output modes)
 * - Digital read/write operations
 * - Interrupt handling and callbacks
 * - Pull-up/pull-down resistor configuration
 * - LED blinking and button input
 * 
 * Hardware simulation:
 * - Pin 2: LED output (active high)
 * - Pin 0: Button input (active low with pull-up)
 * - Pin 4: Another LED for interrupt demo
 * 
 * Usage:
 *   ./gpio_control
 * 
 * @author M5Stack Tab5 Emulator Project
 * @date 2024
 */

#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/gpio_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

// Global flag for graceful shutdown
std::atomic<bool> running{true};
std::atomic<bool> led_state{false};
std::atomic<int> button_press_count{0};

// Signal handler for graceful shutdown
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived shutdown signal, stopping GPIO example..." << std::endl;
        running = false;
    }
}

// Interrupt callback for button press
void button_interrupt_callback(uint8_t pin, bool state) {
    if (pin == 0 && !state) {  // Button pressed (active low)
        button_press_count++;
        std::cout << "🔘 Button pressed! (Count: " << button_press_count.load() << ")" << std::endl;
        
        // Toggle LED on button press
        led_state = !led_state.load();
        std::cout << "💡 LED " << (led_state.load() ? "ON" : "OFF") << " (triggered by button)" << std::endl;
    }
}

// Demonstrate basic GPIO operations
bool demonstrate_basic_gpio(std::shared_ptr<peripherals::GPIOController> gpio) {
    std::cout << "\n=== Basic GPIO Operations ===" << std::endl;
    
    // Configure pins
    std::cout << "Configuring pins..." << std::endl;
    
    // Configure LED pin as output
    auto result = gpio->configure_pin(2, peripherals::GPIOController::Mode::OUTPUT);
    if (!result.has_value()) {
        std::cerr << "❌ Failed to configure pin 2 as output" << std::endl;
        return false;
    }
    std::cout << "✅ Pin 2 configured as OUTPUT (LED)" << std::endl;
    
    // Configure button pin as input with pull-up
    result = gpio->configure_pin(0, peripherals::GPIOController::Mode::INPUT);
    if (!result.has_value()) {
        std::cerr << "❌ Failed to configure pin 0 as input" << std::endl;
        return false;
    }
    
    result = gpio->set_pull_mode(0, peripherals::GPIOController::Pull::UP);
    if (!result.has_value()) {
        std::cerr << "❌ Failed to set pull-up on pin 0" << std::endl;
        return false;
    }
    std::cout << "✅ Pin 0 configured as INPUT with PULL-UP (Button)" << std::endl;
    
    // Configure another LED pin for interrupt demo
    result = gpio->configure_pin(4, peripherals::GPIOController::Mode::OUTPUT);
    if (!result.has_value()) {
        std::cerr << "❌ Failed to configure pin 4 as output" << std::endl;
        return false;
    }
    std::cout << "✅ Pin 4 configured as OUTPUT (Interrupt LED)" << std::endl;
    
    return true;
}

// Demonstrate digital I/O operations
void demonstrate_digital_io(std::shared_ptr<peripherals::GPIOController> gpio) {
    std::cout << "\n=== Digital I/O Operations ===" << std::endl;
    
    // LED blinking sequence
    std::cout << "Starting LED blink sequence on pin 2..." << std::endl;
    
    for (int i = 0; i < 10 && running; ++i) {
        // Turn LED on
        auto write_result = gpio->digital_write(2, true);
        if (write_result.has_value()) {
            std::cout << "💡 LED ON (Pin 2: HIGH)" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (!running) break;
        
        // Turn LED off
        write_result = gpio->digital_write(2, false);
        if (write_result.has_value()) {
            std::cout << "💡 LED OFF (Pin 2: LOW)" << std::endl;
        }
        
        // Read button state while LED is off
        auto read_result = gpio->digital_read(0);
        if (read_result.has_value()) {
            if (!read_result.value()) {  // Button pressed (active low)
                std::cout << "🔘 Button is pressed (Pin 0: LOW)" << std::endl;
            } else {
                std::cout << "🔘 Button is released (Pin 0: HIGH)" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// Demonstrate interrupt handling
bool demonstrate_interrupts(std::shared_ptr<peripherals::GPIOController> gpio) {
    std::cout << "\n=== Interrupt Handling ===" << std::endl;
    
    // Attach interrupt to button pin
    auto interrupt_result = gpio->attach_interrupt(
        0,  // Pin 0 (button)
        peripherals::GPIOController::InterruptTrigger::FALLING_EDGE,  // Trigger on press
        button_interrupt_callback
    );
    
    if (!interrupt_result.has_value()) {
        std::cerr << "❌ Failed to attach interrupt to pin 0" << std::endl;
        return false;
    }
    
    std::cout << "✅ Interrupt attached to pin 0 (button)" << std::endl;
    std::cout << "Press the button (simulate by changing pin state) to trigger interrupt" << std::endl;
    
    // Simulate button presses and demonstrate interrupt-driven LED control
    std::cout << "\nSimulating button presses..." << std::endl;
    
    for (int i = 0; i < 5 && running; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        if (!running) break;
        
        std::cout << "📍 Simulating button press " << (i + 1) << "..." << std::endl;
        
        // Simulate button press (this would normally come from hardware)
        gpio->simulate_pin_change(0, false);  // Press (active low)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Update the interrupt LED based on button state
        auto led_write_result = gpio->digital_write(4, led_state.load());
        if (!led_write_result.has_value()) {
            std::cerr << "⚠️  Failed to update interrupt LED" << std::endl;
        }
        
        gpio->simulate_pin_change(0, true);   // Release
    }
    
    return true;
}

// Demonstrate advanced GPIO features
void demonstrate_advanced_features(std::shared_ptr<peripherals::GPIOController> gpio) {
    std::cout << "\n=== Advanced GPIO Features ===" << std::endl;
    
    // Demonstrate pin state queries
    std::cout << "Querying pin configurations..." << std::endl;
    
    for (uint8_t pin : {0, 2, 4}) {
        auto mode_result = gpio->get_pin_mode(pin);
        if (mode_result.has_value()) {
            std::string mode_str;
            switch (mode_result.value()) {
                case peripherals::GPIOController::Mode::INPUT:
                    mode_str = "INPUT";
                    break;
                case peripherals::GPIOController::Mode::OUTPUT:
                    mode_str = "OUTPUT";
                    break;
                case peripherals::GPIOController::Mode::INPUT_PULLUP:
                    mode_str = "INPUT_PULLUP";
                    break;
                case peripherals::GPIOController::Mode::INPUT_PULLDOWN:
                    mode_str = "INPUT_PULLDOWN";
                    break;
            }
            std::cout << "📌 Pin " << (int)pin << ": " << mode_str << std::endl;
        }
    }
    
    // Demonstrate multiple interrupt types
    std::cout << "\nDemonstrating different interrupt types..." << std::endl;
    
    // Create a callback that shows the interrupt type
    auto advanced_callback = [](uint8_t pin, bool state) {
        std::cout << "🔔 Advanced interrupt on pin " << (int)pin 
                  << ", state: " << (state ? "HIGH" : "LOW") << std::endl;
    };
    
    // Configure pin 1 for change interrupt
    gpio->configure_pin(1, peripherals::GPIOController::Mode::INPUT);
    gpio->attach_interrupt(1, peripherals::GPIOController::InterruptTrigger::CHANGE, advanced_callback);
    
    // Simulate various state changes
    std::cout << "Simulating state changes on pin 1..." << std::endl;
    gpio->simulate_pin_change(1, true);   // Rising edge
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gpio->simulate_pin_change(1, false);  // Falling edge
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gpio->simulate_pin_change(1, true);   // Rising edge again
}

int main() {
    std::cout << "=== M5Stack Tab5 Emulator - GPIO Control Example ===" << std::endl;
    std::cout << "Demonstrating GPIO operations, digital I/O, and interrupts" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    
    // Set up signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
        // Load configuration
        auto config = config::Configuration::load("config/default.json");
        if (!config.has_value()) {
            std::cerr << "❌ Failed to load configuration" << std::endl;
            return -1;
        }
        
        // Create and initialize emulator
        auto emulator = EmulatorCore::create(config.value());
        if (!emulator.has_value()) {
            std::cerr << "❌ Failed to create emulator" << std::endl;
            return -1;
        }
        
        if (!emulator.value()->initialize().has_value()) {
            std::cerr << "❌ Failed to initialize emulator" << std::endl;
            return -1;
        }
        
        // Get GPIO controller
        auto gpio = emulator.value()->get_component<peripherals::GPIOController>();
        if (!gpio.has_value()) {
            std::cerr << "❌ Failed to get GPIO controller" << std::endl;
            return -1;
        }
        
        std::cout << "✅ GPIO controller obtained successfully" << std::endl;
        
        // Demonstrate basic GPIO operations
        if (!demonstrate_basic_gpio(gpio.value())) {
            return -1;
        }
        
        // Demonstrate digital I/O
        if (running) {
            demonstrate_digital_io(gpio.value());
        }
        
        // Demonstrate interrupt handling
        if (running && !demonstrate_interrupts(gpio.value())) {
            return -1;
        }
        
        // Demonstrate advanced features
        if (running) {
            demonstrate_advanced_features(gpio.value());
        }
        
        // Keep running until interrupted
        if (running) {
            std::cout << "\n⏳ GPIO example complete. Press Ctrl+C to exit..." << std::endl;
            while (running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        // Cleanup
        std::cout << "\n🧹 Cleaning up..." << std::endl;
        
        // Detach interrupts
        gpio.value()->detach_interrupt(0);
        gpio.value()->detach_interrupt(1);
        
        // Turn off all LEDs
        gpio.value()->digital_write(2, false);
        gpio.value()->digital_write(4, false);
        
        emulator.value()->shutdown();
        
        std::cout << "✅ GPIO control example completed successfully!" << std::endl;
        std::cout << "📊 Total button presses detected: " << button_press_count.load() << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "💥 Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

/*
 * Example Output:
 * 
 * === M5Stack Tab5 Emulator - GPIO Control Example ===
 * Demonstrating GPIO operations, digital I/O, and interrupts
 * Press Ctrl+C to exit
 * ✅ GPIO controller obtained successfully
 * 
 * === Basic GPIO Operations ===
 * Configuring pins...
 * ✅ Pin 2 configured as OUTPUT (LED)
 * ✅ Pin 0 configured as INPUT with PULL-UP (Button)
 * ✅ Pin 4 configured as OUTPUT (Interrupt LED)
 * 
 * === Digital I/O Operations ===
 * Starting LED blink sequence on pin 2...
 * 💡 LED ON (Pin 2: HIGH)
 * 💡 LED OFF (Pin 2: LOW)
 * 🔘 Button is released (Pin 0: HIGH)
 * 💡 LED ON (Pin 2: HIGH)
 * 💡 LED OFF (Pin 2: LOW)
 * 🔘 Button is released (Pin 0: HIGH)
 * ...
 * 
 * === Interrupt Handling ===
 * ✅ Interrupt attached to pin 0 (button)
 * Press the button (simulate by changing pin state) to trigger interrupt
 * 
 * Simulating button presses...
 * 📍 Simulating button press 1...
 * 🔘 Button pressed! (Count: 1)
 * 💡 LED ON (triggered by button)
 * 📍 Simulating button press 2...
 * 🔘 Button pressed! (Count: 2)
 * 💡 LED OFF (triggered by button)
 * ...
 * 
 * === Advanced GPIO Features ===
 * Querying pin configurations...
 * 📌 Pin 0: INPUT
 * 📌 Pin 2: OUTPUT
 * 📌 Pin 4: OUTPUT
 * 
 * Demonstrating different interrupt types...
 * Simulating state changes on pin 1...
 * 🔔 Advanced interrupt on pin 1, state: HIGH
 * 🔔 Advanced interrupt on pin 1, state: LOW
 * 🔔 Advanced interrupt on pin 1, state: HIGH
 * 
 * ⏳ GPIO example complete. Press Ctrl+C to exit...
 * ^C
 * Received shutdown signal, stopping GPIO example...
 * 
 * 🧹 Cleaning up...
 * ✅ GPIO control example completed successfully!
 * 📊 Total button presses detected: 5
 */