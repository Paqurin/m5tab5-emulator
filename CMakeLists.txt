cmake_minimum_required(VERSION 3.16)

project(m5tab5-emulator VERSION 1.0.0 LANGUAGES CXX C)

# C++ standard and compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g")

# Fix fmt v9/spdlog dangling reference warnings in GCC 13+ for entire project
# This suppresses known false positives in fmt library template instantiation when using spdlog
# See: https://github.com/fmtlib/fmt/issues/3415
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dangling-reference")
    message(STATUS "Applied -Wno-dangling-reference fix for fmt v9/spdlog usage with GCC ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# Fix spdlog/fmt deprecation warnings being treated as errors
# Suppress deprecation warnings from external libraries (spdlog, fmt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
message(STATUS "Applied -Wno-deprecated-declarations fix for spdlog/fmt library usage")

# Platform-specific settings
if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# Find required packages
find_package(Threads REQUIRED)

# SQLite3 for NVS storage backend
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SQLITE3 QUIET sqlite3)
endif()

if(NOT SQLITE3_FOUND)
    find_library(SQLITE3_LIBRARIES sqlite3)
    find_path(SQLITE3_INCLUDE_DIRS sqlite3.h)
    if(SQLITE3_LIBRARIES AND SQLITE3_INCLUDE_DIRS)
        set(SQLITE3_FOUND TRUE)
    else()
        message(STATUS "SQLite3 not found. Building with in-memory NVS storage.")
        set(SQLITE3_LIBRARIES "")
        set(SQLITE3_INCLUDE_DIRS "")
    endif()
endif()

# Add SQLite3 compile definition if found
if(SQLITE3_FOUND)
    add_compile_definitions(HAVE_SQLITE3=1)
    message(STATUS "SQLite3 found - building with persistent NVS storage")
else()
    message(STATUS "SQLite3 not found - building with in-memory NVS storage")
endif()

# Try to find PkgConfig (optional)

# SDL2 for graphics and input (enhanced detection with manual fallback)
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND AND PkgConfig_FOUND)
    pkg_check_modules(SDL2 QUIET sdl2)
endif()

# Manual SDL2 detection for runtime-only installations
if(NOT SDL2_FOUND)
    # Force search for SDL2 library with specific names and paths
    find_library(SDL2_LIBRARY_MANUAL 
        NAMES SDL2 libSDL2 SDL2-2.0 libSDL2-2.0
        PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib
        NO_DEFAULT_PATH)
    
    find_path(SDL2_INCLUDE_MANUAL SDL.h 
        PATHS /usr/include/SDL2 /usr/local/include/SDL2 /usr/include
        NO_DEFAULT_PATH)
    
    # Check for library by specific filename pattern
    if(NOT SDL2_LIBRARY_MANUAL)
        file(GLOB SDL2_LIB_FILES "/usr/lib/x86_64-linux-gnu/libSDL2*.so*")
        if(SDL2_LIB_FILES)
            list(GET SDL2_LIB_FILES 0 SDL2_LIBRARY_MANUAL)
        endif()
    endif()
    
    if(SDL2_LIBRARY_MANUAL AND SDL2_INCLUDE_MANUAL)
        message(STATUS "SDL2 found manually with headers: ${SDL2_LIBRARY_MANUAL}")
        set(SDL2_LIBRARIES ${SDL2_LIBRARY_MANUAL})
        set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_MANUAL})
        set(SDL2_FOUND TRUE)
    elseif(SDL2_LIBRARY_MANUAL)
        # Library found but no headers - use internal declarations
        message(STATUS "SDL2 library found: ${SDL2_LIBRARY_MANUAL}")
        message(STATUS "SDL2 headers not found - using internal declarations")
        set(SDL2_LIBRARIES ${SDL2_LIBRARY_MANUAL})
        set(SDL2_INCLUDE_DIRS "")
        set(SDL2_FOUND TRUE)
        # Enable internal SDL2 headers
        add_compile_definitions(INTERNAL_SDL2_HEADERS=1)
        message(STATUS "Building with internal SDL2 header support")
    else()
        message(WARNING "SDL2 library not found. Building without graphics support.")
        add_compile_definitions(NO_GRAPHICS=1)
        set(SDL2_LIBRARIES "")
        set(SDL2_INCLUDE_DIRS "")
    endif()
endif()

# JSON library for configuration
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# spdlog for logging
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.12.0
    )
    FetchContent_MakeAvailable(spdlog)
    
    # Fix fmt v9/spdlog dangling reference warnings in GCC 13+
    # This suppresses known false positives in fmt library template instantiation
    # See: https://github.com/fmtlib/fmt/issues/3415
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")
        target_compile_options(spdlog PRIVATE -Wno-dangling-reference)
        message(STATUS "Applied -Wno-dangling-reference fix for spdlog/fmt v9 with GCC ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
endif()

# Include directories
include_directories(include)
include_directories(${SDL2_INCLUDE_DIRS})

# Source files organization
file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp")
file(GLOB_RECURSE CPU_SOURCES "src/cpu/*.cpp")
file(GLOB_RECURSE MEMORY_SOURCES "src/memory/*.cpp")
file(GLOB_RECURSE PERIPHERAL_SOURCES "src/peripherals/*.cpp")
file(GLOB_RECURSE GRAPHICS_SOURCES "src/graphics/*.cpp")
file(GLOB_RECURSE AUDIO_SOURCES "src/audio/*.cpp")
file(GLOB_RECURSE CONFIG_SOURCES "src/config/*.cpp")
file(GLOB_RECURSE DEBUG_SOURCES "src/debug/*.cpp")
file(GLOB_RECURSE PLUGIN_SOURCES "src/plugin/*.cpp")
file(GLOB_RECURSE UTILS_SOURCES "src/utils/*.cpp")
file(GLOB_RECURSE FREERTOS_SOURCES "src/freertos/*.cpp")
# Temporarily exclude ESP-IDF driver sources due to header conflicts
file(GLOB_RECURSE ESP_IDF_SOURCES "src/esp_idf/*.cpp")
# Remove problematic driver files for now
list(FILTER ESP_IDF_SOURCES EXCLUDE REGEX "src/esp_idf/driver/.*\\.cpp$")
message(STATUS "ESP-IDF sources (excluding drivers): ${ESP_IDF_SOURCES}")
file(GLOB_RECURSE FIRMWARE_SOURCES "src/firmware/*.cpp")
file(GLOB_RECURSE STORAGE_SOURCES "src/storage/*.cpp")
file(GLOB_RECURSE GUI_SOURCES "src/gui/*.cpp")

set(ALL_SOURCES
    ${CORE_SOURCES}
    ${CPU_SOURCES}
    ${MEMORY_SOURCES}
    ${PERIPHERAL_SOURCES}
    ${GRAPHICS_SOURCES}
    ${AUDIO_SOURCES}
    ${CONFIG_SOURCES}
    ${DEBUG_SOURCES}
    ${PLUGIN_SOURCES}
    ${UTILS_SOURCES}
    ${FREERTOS_SOURCES}
    ${ESP_IDF_SOURCES}
    ${FIRMWARE_SOURCES}
    ${STORAGE_SOURCES}
)

# GUI sources (separate for optional GUI builds)
set(GUI_LIBRARY_SOURCES
    ${GUI_SOURCES}
)

# Create static library for core emulator
add_library(m5tab5-emulator-core STATIC ${ALL_SOURCES})

# Link libraries to core
target_link_libraries(m5tab5-emulator-core
    PUBLIC
        Threads::Threads
        ${SDL2_LIBRARIES}
        nlohmann_json::nlohmann_json
        spdlog::spdlog
        ${CMAKE_DL_LIBS}  # For plugin loading
)

# Conditionally link SQLite3 if available
if(SQLITE3_FOUND)
    target_link_libraries(m5tab5-emulator-core PUBLIC ${SQLITE3_LIBRARIES})
endif()

# Set target properties
target_include_directories(m5tab5-emulator-core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${SDL2_INCLUDE_DIRS}
)

# Conditionally add SQLite3 include directories
if(SQLITE3_FOUND)
    target_include_directories(m5tab5-emulator-core PUBLIC ${SQLITE3_INCLUDE_DIRS})
endif()

target_compile_definitions(m5tab5-emulator-core
    PUBLIC
        M5TAB5_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        M5TAB5_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        M5TAB5_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# Main executable (command-line version)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE m5tab5-emulator-core)

# 🏆 CHAMPIONSHIP FIX: GUI Library with proper dependency chain
if(GUI_LIBRARY_SOURCES)
    add_library(m5tab5-gui-lib STATIC ${GUI_LIBRARY_SOURCES})
    
    # CRITICAL: Link order fix for PersonalityManager symbols
    target_link_libraries(m5tab5-gui-lib 
        PUBLIC 
            m5tab5-emulator-core  # GUI lib needs core symbols
        PRIVATE 
            ${SDL2_LIBRARIES}     # SDL2 only for GUI implementation
    )
    
    target_include_directories(m5tab5-gui-lib
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            ${SDL2_INCLUDE_DIRS}
    )
    
    # Enable GUI-specific features
    target_compile_definitions(m5tab5-gui-lib
        PUBLIC
            ENABLE_GUI=1
    )
    
    message(STATUS "GUI library built with ${GUI_LIBRARY_SOURCES}")
    set(GUI_AVAILABLE TRUE)
else()
    message(STATUS "No GUI sources found - GUI library not built")
    set(GUI_AVAILABLE FALSE)
endif()

# 🔥 CRITICAL LINKING FIX: GUI executable with correct dependency order
add_executable(${PROJECT_NAME}-gui main_gui.cpp)
if(GUI_AVAILABLE)
    # CHAMPIONSHIP LINKING: GUI lib first, then core (proper dependency resolution)
    target_link_libraries(${PROJECT_NAME}-gui 
        PRIVATE 
            m5tab5-gui-lib          # Contains PersonalityManager symbols
            m5tab5-emulator-core    # Core provides base functionality
    )
    message(STATUS "GUI executable built with full GUI library support")
else()
    target_link_libraries(${PROJECT_NAME}-gui PRIVATE m5tab5-emulator-core)
    message(STATUS "GUI executable built with basic GUI support only")
endif()
target_compile_definitions(${PROJECT_NAME}-gui PRIVATE ENABLE_GUI=1)

if(SDL2_FOUND)
    message(STATUS "Building GUI version with SDL2 support: ${PROJECT_NAME}-gui")
else()
    message(STATUS "Building GUI version without graphics (NO_GRAPHICS): ${PROJECT_NAME}-gui")
endif()

# Plugin support library (temporarily disabled - build success priority)
# Note: Main emulator works without plugin system
# add_library(m5tab5-plugin-support SHARED
#     src/plugin/plugin_loader.cpp
#     src/plugin/plugin_host.cpp
# )
# target_link_libraries(m5tab5-plugin-support
#     PUBLIC m5tab5-emulator-core
#     PRIVATE ${CMAKE_DL_LIBS}
# )

# Example plugins
add_subdirectory(plugins EXCLUDE_FROM_ALL)

# Tools and utilities
option(BUILD_TOOLS "Build emulator tools" ON)
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# Testing
enable_testing()
add_subdirectory(tests)

# Test executables
add_executable(test_display_connection
    test_display_connection.cpp
)

# NVS persistence test
add_executable(test_nvs_persistence
    test_nvs_persistence.cpp
)

# Simple NVS test
add_executable(test_nvs_simple
    test_nvs_simple.cpp
)

# ESP32-P4 Boot Sequence Test
add_executable(test_esp32p4_boot_sequence
    test_esp32p4_boot_sequence.cpp
)

# ESP32-P4 Boot Sequence Simple Test (without ESP-IDF dependencies)
add_executable(test_esp32p4_boot_simple
    test_esp32p4_boot_simple.cpp
)

# ELF Loader Test
add_executable(test_elf_loader
    test_elf_loader.cpp
)

# Comprehensive ELF Loader Test
add_executable(test_elf_loader_comprehensive
    test_elf_loader_comprehensive.cpp
)

# Link test with core library
target_link_libraries(test_display_connection
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link NVS test with core library
target_link_libraries(test_nvs_persistence
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link simple NVS test with core library
target_link_libraries(test_nvs_simple
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link ESP32-P4 boot sequence test with core library
target_link_libraries(test_esp32p4_boot_sequence
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link ESP32-P4 boot simple test with core library (excluding problematic ESP-IDF APIs)
target_link_libraries(test_esp32p4_boot_simple
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link ELF loader test with core library
target_link_libraries(test_elf_loader
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Link comprehensive ELF loader test with core library
target_link_libraries(test_elf_loader_comprehensive
    PRIVATE m5tab5-emulator-core
    PRIVATE Threads::Threads
)

# Set C++ standard for NVS tests
target_compile_features(test_nvs_persistence PRIVATE cxx_std_20)
target_compile_features(test_nvs_simple PRIVATE cxx_std_20)
target_compile_features(test_esp32p4_boot_sequence PRIVATE cxx_std_20)
target_compile_features(test_esp32p4_boot_simple PRIVATE cxx_std_20)
target_compile_features(test_elf_loader PRIVATE cxx_std_20)
target_compile_features(test_elf_loader_comprehensive PRIVATE cxx_std_20)

# Set C++ standard and properties for test
target_compile_features(test_display_connection PRIVATE cxx_std_20)
if(SDL2_LIBRARY_MANUAL)
    target_link_libraries(test_display_connection PRIVATE ${SDL2_LIBRARY_MANUAL})
    if(SDL2_INCLUDE_MANUAL)
        target_include_directories(test_display_connection PRIVATE ${SDL2_INCLUDE_MANUAL})
    endif()
    target_compile_definitions(test_display_connection PRIVATE INTERNAL_SDL2_HEADERS)
    message(STATUS "Building test_display_connection with manual SDL2: ${SDL2_LIBRARY_MANUAL}")
else()
    target_compile_definitions(test_display_connection PRIVATE NO_GRAPHICS)
    message(STATUS "Building test_display_connection without SDL2 (NO_GRAPHICS mode)")
endif()

# Examples
option(BUILD_EXAMPLES "Build example applications" ON)
if(BUILD_EXAMPLES)
    # Simple FreeRTOS integration test example
    add_executable(simple-freertos-test examples/simple_freertos_test.cpp)
    target_link_libraries(simple-freertos-test m5tab5-emulator-core)
    message(STATUS "Building simple FreeRTOS test example")
    
    # ELF Loading demonstration example
    add_executable(elf-loading-demo examples/elf_loading_demo.cpp)
    target_link_libraries(elf-loading-demo m5tab5-emulator-core)
    target_compile_features(elf-loading-demo PRIVATE cxx_std_20)
    message(STATUS "Building ELF loading demo example")
endif()

# Documentation
option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        add_subdirectory(docs)
    endif()
endif()

# Installation
install(TARGETS ${PROJECT_NAME} m5tab5-emulator-core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Configuration files
install(FILES config/default.json config/development.json
    DESTINATION share/m5tab5-emulator/config
    OPTIONAL
)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/m5tab5-emulator-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/m5tab5-emulator-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/m5tab5-emulator-config.cmake"
    INSTALL_DESTINATION lib/cmake/m5tab5-emulator
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/m5tab5-emulator-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/m5tab5-emulator-config-version.cmake"
    DESTINATION lib/cmake/m5tab5-emulator
)