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

# Platform-specific settings
if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# Find required packages
find_package(Threads REQUIRED)

# Try to find PkgConfig (optional)
find_package(PkgConfig QUIET)

# SDL2 for graphics and input (try multiple approaches)
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND AND PkgConfig_FOUND)
    pkg_check_modules(SDL2 QUIET sdl2)
endif()

# If SDL2 is not found, provide a fallback or disable graphics
if(NOT SDL2_FOUND)
    message(WARNING "SDL2 not found. Building without graphics support.")
    add_compile_definitions(NO_GRAPHICS=1)
    set(SDL2_LIBRARIES "")
    set(SDL2_INCLUDE_DIRS "")
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

# Set target properties
target_include_directories(m5tab5-emulator-core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${SDL2_INCLUDE_DIRS}
)

target_compile_definitions(m5tab5-emulator-core
    PUBLIC
        M5TAB5_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        M5TAB5_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        M5TAB5_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# Main executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE m5tab5-emulator-core)

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