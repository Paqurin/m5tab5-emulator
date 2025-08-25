# Simple test firmware for M5Stack Tab5 Emulator
cmake_minimum_required(VERSION 3.16)

# Set the target to ESP32-P4 for M5Stack Tab5
set(IDF_TARGET esp32p4)

project(m5tab5_test_firmware)

# Add the main source file
add_executable(m5tab5_test_firmware
    m5tab5_test_firmware.cpp
)

# If building standalone, create a simple binary
if(NOT DEFINED IDF_PATH)
    message(STATUS "Building standalone test firmware for emulator")
    
    # Create a simple binary output for emulator testing
    set_target_properties(m5tab5_test_firmware PROPERTIES
        OUTPUT_NAME "m5tab5_test_firmware.elf"
    )
    
    # Add basic includes for compatibility
    target_include_directories(m5tab5_test_firmware PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()