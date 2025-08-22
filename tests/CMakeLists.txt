find_package(GTest QUIET)

if(GTest_FOUND)
    set(TEST_SOURCES
        test_main.cpp
        test_cpu_core.cpp
        test_memory_controller.cpp
        test_gpio_controller.cpp
        test_i2c_controller.cpp
        test_spi_controller.cpp
        test_uart_controller.cpp
        test_bmi270_imu.cpp
        test_sensor_fusion.cpp
        test_wifi_controller.cpp
        test_bluetooth_controller.cpp
        test_usb_controller.cpp
        test_rs485_controller.cpp
        test_configuration.cpp
        test_error_handling.cpp
    )
    
    add_executable(unit_tests ${TEST_SOURCES})
    target_link_libraries(unit_tests 
        GTest::gtest 
        GTest::gtest_main
        ${SDL2_LIBRARIES}
        pthread
    )
    target_include_directories(unit_tests PRIVATE 
        ../include
        ${SDL2_INCLUDE_DIRS}
    )
    
    add_executable(integration_tests
        integration_test_main.cpp
        test_system_integration.cpp
        test_peripheral_coordination.cpp
        test_connectivity_stack.cpp
    )
    target_link_libraries(integration_tests 
        GTest::gtest 
        GTest::gtest_main
        ${SDL2_LIBRARIES}
        pthread
    )
    target_include_directories(integration_tests PRIVATE 
        ../include
        ${SDL2_INCLUDE_DIRS}
    )
    
    add_executable(performance_tests
        performance_test_main.cpp
        test_cpu_performance.cpp
        test_memory_performance.cpp
        test_graphics_performance.cpp
    )
    target_link_libraries(performance_tests 
        GTest::gtest 
        GTest::gtest_main
        ${SDL2_LIBRARIES}
        pthread
    )
    target_include_directories(performance_tests PRIVATE 
        ../include
        ${SDL2_INCLUDE_DIRS}
    )
    
    add_test(NAME UnitTests COMMAND unit_tests)
    add_test(NAME IntegrationTests COMMAND integration_tests)
    add_test(NAME PerformanceTests COMMAND performance_tests)
    
    enable_testing()
else()
    message(STATUS "Google Test not found. Skipping test targets.")
endif()