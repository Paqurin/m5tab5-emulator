find_package(GTest QUIET)

if(GTest_FOUND)
    add_executable(unit_tests test_main.cpp)
    target_link_libraries(unit_tests GTest::gtest GTest::gtest_main)
    target_include_directories(unit_tests PRIVATE ../include)
    
    add_test(NAME UnitTests COMMAND unit_tests)
else()
    message(STATUS "Google Test not found. Skipping test target.")
endif()