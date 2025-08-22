#include <gtest/gtest.h>
#include <iostream>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "Running M5Stack Tab5 Emulator Performance Tests..." << std::endl;
    std::cout << "These tests measure system performance under various loads." << std::endl;
    
    return RUN_ALL_TESTS();
}