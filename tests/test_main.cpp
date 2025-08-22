#include <gtest/gtest.h>
#include <iostream>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "==============================================" << std::endl;
    std::cout << "M5Stack Tab5 Emulator - Unit Test Suite" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << "Testing comprehensive emulator functionality:" << std::endl;
    std::cout << "• CPU core and dual-core management" << std::endl;
    std::cout << "• Memory controller and cache coherency" << std::endl;
    std::cout << "• GPIO system (55 pins) with interrupts" << std::endl;
    std::cout << "• Communication interfaces (I2C/SPI/UART)" << std::endl;
    std::cout << "• Sensor fusion and IMU simulation" << std::endl;
    std::cout << "• Connectivity (WiFi/Bluetooth/USB/RS485)" << std::endl;
    std::cout << "• Error handling and configuration systems" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << std::endl;
    
    return RUN_ALL_TESTS();
}