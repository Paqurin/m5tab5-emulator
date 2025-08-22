# M5Stack Tab5 Emulator - Examples CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# Only build examples if requested
if(NOT BUILD_EXAMPLES)
    return()
endif()

message(STATUS "Building examples...")

# Find required dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${SDL2_INCLUDE_DIRS})

# Compiler flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Common libraries for all examples
set(EXAMPLE_LIBRARIES
    emulator_core
    emulator_config
    emulator_peripherals
    emulator_graphics
    emulator_connectivity
    emulator_audio
    emulator_sensors
    emulator_debug
    emulator_utils
    ${SDL2_LIBRARIES}
    pthread
)

# Basic Emulator Example
add_executable(basic_emulator basic_emulator.cpp)
target_link_libraries(basic_emulator ${EXAMPLE_LIBRARIES})
target_compile_definitions(basic_emulator PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# GPIO Control Example  
add_executable(gpio_control gpio_control.cpp)
target_link_libraries(gpio_control ${EXAMPLE_LIBRARIES})
target_compile_definitions(gpio_control PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# I2C Sensor Communication Example
add_executable(i2c_sensor i2c_sensor.cpp)
target_link_libraries(i2c_sensor ${EXAMPLE_LIBRARIES})
target_compile_definitions(i2c_sensor PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Touch Interface Example
add_executable(touch_interface touch_interface.cpp)
target_link_libraries(touch_interface ${EXAMPLE_LIBRARIES})
target_compile_definitions(touch_interface PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Audio Processing Example
add_executable(audio_processing audio_processing.cpp)
target_link_libraries(audio_processing ${EXAMPLE_LIBRARIES})
target_compile_definitions(audio_processing PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Wi-Fi Connectivity Example
add_executable(wifi_connectivity wifi_connectivity.cpp)
target_link_libraries(wifi_connectivity ${EXAMPLE_LIBRARIES})
target_compile_definitions(wifi_connectivity PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Sensor Fusion Example
add_executable(sensor_fusion sensor_fusion.cpp)
target_link_libraries(sensor_fusion ${EXAMPLE_LIBRARIES})
target_compile_definitions(sensor_fusion PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Performance Profiling Example
add_executable(performance_profiling performance_profiling.cpp)
target_link_libraries(performance_profiling ${EXAMPLE_LIBRARIES})
target_compile_definitions(performance_profiling PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Complete Application Example
add_executable(complete_application complete_application.cpp)
target_link_libraries(complete_application ${EXAMPLE_LIBRARIES})
target_compile_definitions(complete_application PRIVATE 
    ${SDL2_CFLAGS_OTHER}
)

# Set output directory for examples
set_target_properties(
    basic_emulator
    gpio_control
    i2c_sensor
    touch_interface
    audio_processing
    wifi_connectivity
    sensor_fusion
    performance_profiling
    complete_application
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
)

# Copy configuration files to examples directory
configure_file(
    ${CMAKE_SOURCE_DIR}/config/default.json
    ${CMAKE_BINARY_DIR}/examples/config/default.json
    COPYONLY
)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/development.json
    ${CMAKE_BINARY_DIR}/examples/config/development.json
    COPYONLY
)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/performance.json
    ${CMAKE_BINARY_DIR}/examples/config/performance.json
    COPYONLY
)

# Create a custom target to build all examples
add_custom_target(examples
    DEPENDS 
        basic_emulator
        gpio_control
        i2c_sensor
        touch_interface
        audio_processing
        wifi_connectivity
        sensor_fusion
        performance_profiling
        complete_application
)

# Install examples (optional)
if(INSTALL_EXAMPLES)
    install(TARGETS
        basic_emulator
        gpio_control
        i2c_sensor
        touch_interface
        audio_processing
        wifi_connectivity
        sensor_fusion
        performance_profiling
        complete_application
        RUNTIME DESTINATION bin/examples
    )
    
    install(FILES
        ${CMAKE_SOURCE_DIR}/config/default.json
        ${CMAKE_SOURCE_DIR}/config/development.json
        ${CMAKE_SOURCE_DIR}/config/performance.json
        DESTINATION bin/examples/config
    )
    
    install(FILES
        README.md
        DESTINATION bin/examples
    )
endif()

message(STATUS "Examples configured successfully")
message(STATUS "Use 'make examples' to build all examples")
message(STATUS "Examples will be built in: ${CMAKE_BINARY_DIR}/examples/")

# Print available examples
message(STATUS "Available examples:")
message(STATUS "  - basic_emulator: Basic emulator setup and lifecycle")
message(STATUS "  - gpio_control: GPIO operations and interrupts") 
message(STATUS "  - i2c_sensor: I2C sensor communication")
message(STATUS "  - touch_interface: Touch input handling")
message(STATUS "  - audio_processing: Audio playback and recording")
message(STATUS "  - wifi_connectivity: Wi-Fi operations") 
message(STATUS "  - sensor_fusion: IMU sensor fusion")
message(STATUS "  - performance_profiling: Performance measurement")
message(STATUS "  - complete_application: Full-featured demo app")