#include <gtest/gtest.h>
#include "emulator/utils/error.hpp"
#include <string>
#include <memory>

namespace emulator::utils::test {

TEST(ErrorHandlingTest, BasicErrorCreation) {
    auto error = Error::create(ErrorCode::INVALID_PARAMETER, "Test error message");
    
    EXPECT_EQ(error.code(), ErrorCode::INVALID_PARAMETER);
    EXPECT_EQ(error.message(), "Test error message");
    EXPECT_FALSE(error.location().file_name().empty());
    EXPECT_GT(error.location().line(), 0);
}

TEST(ErrorHandlingTest, ErrorWithContext) {
    auto error = Error::create(ErrorCode::DEVICE_NOT_FOUND, "Device not found", "TestDevice");
    
    EXPECT_EQ(error.code(), ErrorCode::DEVICE_NOT_FOUND);
    EXPECT_EQ(error.message(), "Device not found");
    EXPECT_EQ(error.context(), "TestDevice");
}

TEST(ErrorHandlingTest, ErrorChaining) {
    auto root_error = Error::create(ErrorCode::HARDWARE_ERROR, "Hardware failure");
    auto chained_error = Error::create(ErrorCode::COMMUNICATION_ERROR, "Communication failed")
                            .with_cause(std::make_shared<Error>(root_error));
    
    EXPECT_EQ(chained_error.code(), ErrorCode::COMMUNICATION_ERROR);
    EXPECT_TRUE(chained_error.cause() != nullptr);
    EXPECT_EQ(chained_error.cause()->code(), ErrorCode::HARDWARE_ERROR);
}

TEST(ErrorHandlingTest, ErrorSeverity) {
    auto warning_error = Error::create(ErrorCode::INVALID_PARAMETER, "Warning message")
                            .with_severity(Error::Severity::WARNING);
    auto critical_error = Error::create(ErrorCode::MEMORY_ERROR, "Critical error")
                            .with_severity(Error::Severity::CRITICAL);
    
    EXPECT_EQ(warning_error.severity(), Error::Severity::WARNING);
    EXPECT_EQ(critical_error.severity(), Error::Severity::CRITICAL);
}

TEST(ErrorHandlingTest, ErrorCategories) {
    EXPECT_EQ(get_error_category(ErrorCode::INVALID_PARAMETER), ErrorCategory::INVALID_INPUT);
    EXPECT_EQ(get_error_category(ErrorCode::MEMORY_ERROR), ErrorCategory::SYSTEM);
    EXPECT_EQ(get_error_category(ErrorCode::DEVICE_NOT_FOUND), ErrorCategory::HARDWARE);
    EXPECT_EQ(get_error_category(ErrorCode::AUTHENTICATION_FAILED), ErrorCategory::SECURITY);
}

TEST(ErrorHandlingTest, ErrorCodeToString) {
    EXPECT_EQ(error_code_to_string(ErrorCode::SUCCESS), "SUCCESS");
    EXPECT_EQ(error_code_to_string(ErrorCode::INVALID_PARAMETER), "INVALID_PARAMETER");
    EXPECT_EQ(error_code_to_string(ErrorCode::OUT_OF_MEMORY), "OUT_OF_MEMORY");
    EXPECT_EQ(error_code_to_string(ErrorCode::DEVICE_NOT_FOUND), "DEVICE_NOT_FOUND");
}

TEST(ErrorHandlingTest, ExpectedSuccess) {
    auto success_result = expected<int, ErrorCode>(42);
    
    EXPECT_TRUE(success_result.has_value());
    EXPECT_EQ(success_result.value(), 42);
    EXPECT_FALSE(success_result.has_error());
}

TEST(ErrorHandlingTest, ExpectedError) {
    auto error_result = expected<int, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
    
    EXPECT_FALSE(error_result.has_value());
    EXPECT_TRUE(error_result.has_error());
    EXPECT_EQ(error_result.error(), ErrorCode::INVALID_PARAMETER);
}

TEST(ErrorHandlingTest, ExpectedTransform) {
    auto original = expected<int, ErrorCode>(10);
    
    auto transformed = original.transform([](int x) { return x * 2; });
    EXPECT_TRUE(transformed.has_value());
    EXPECT_EQ(transformed.value(), 20);
    
    auto error_original = expected<int, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
    auto transformed_error = error_original.transform([](int x) { return x * 2; });
    
    EXPECT_FALSE(transformed_error.has_value());
    EXPECT_EQ(transformed_error.error(), ErrorCode::INVALID_PARAMETER);
}

TEST(ErrorHandlingTest, ExpectedAndThen) {
    auto original = expected<int, ErrorCode>(5);
    
    auto chained = original.and_then([](int x) -> expected<std::string, ErrorCode> {
        if (x > 0) {
            return expected<std::string, ErrorCode>(std::to_string(x));
        }
        return expected<std::string, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
    });
    
    EXPECT_TRUE(chained.has_value());
    EXPECT_EQ(chained.value(), "5");
}

TEST(ErrorHandlingTest, ExpectedOrElse) {
    auto error_result = expected<int, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
    
    auto recovered = error_result.or_else([](ErrorCode) -> expected<int, ErrorCode> {
        return expected<int, ErrorCode>(42);
    });
    
    EXPECT_TRUE(recovered.has_value());
    EXPECT_EQ(recovered.value(), 42);
}

TEST(ErrorHandlingTest, ResultMacroSuccess) {
    auto test_function = []() -> expected<int, ErrorCode> {
        auto result = expected<int, ErrorCode>(10);
        TRY(value, result);
        return expected<int, ErrorCode>(value * 2);
    };
    
    auto result = test_function();
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 20);
}

TEST(ErrorHandlingTest, ResultMacroError) {
    auto test_function = []() -> expected<int, ErrorCode> {
        auto result = expected<int, ErrorCode>::error(ErrorCode::INVALID_PARAMETER);
        TRY(value, result);
        return expected<int, ErrorCode>(value * 2);
    };
    
    auto result = test_function();
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ErrorCode::INVALID_PARAMETER);
}

TEST(ErrorHandlingTest, ErrorFormattingDetailed) {
    auto error = Error::create(ErrorCode::COMMUNICATION_ERROR, "I2C communication failed")
                    .with_context("Device 0x68")
                    .with_severity(Error::Severity::ERROR);
    
    auto formatted = format_error_detailed(error);
    
    EXPECT_FALSE(formatted.empty());
    EXPECT_NE(formatted.find("COMMUNICATION_ERROR"), std::string::npos);
    EXPECT_NE(formatted.find("I2C communication failed"), std::string::npos);
    EXPECT_NE(formatted.find("Device 0x68"), std::string::npos);
}

TEST(ErrorHandlingTest, ErrorRegistry) {
    ErrorRegistry registry;
    
    auto error1 = Error::create(ErrorCode::INVALID_PARAMETER, "First error");
    auto error2 = Error::create(ErrorCode::DEVICE_NOT_FOUND, "Second error");
    
    registry.register_error(error1);
    registry.register_error(error2);
    
    auto errors = registry.get_errors();
    EXPECT_EQ(errors.size(), 2);
    
    auto errors_by_code = registry.get_errors_by_code(ErrorCode::INVALID_PARAMETER);
    EXPECT_EQ(errors_by_code.size(), 1);
    EXPECT_EQ(errors_by_code[0].message(), "First error");
}

TEST(ErrorHandlingTest, ErrorThresholds) {
    ErrorRegistry registry;
    
    for (int i = 0; i < 5; ++i) {
        auto error = Error::create(ErrorCode::COMMUNICATION_ERROR, "Repeated error")
                        .with_severity(Error::Severity::WARNING);
        registry.register_error(error);
    }
    
    EXPECT_FALSE(registry.is_threshold_exceeded(ErrorCode::COMMUNICATION_ERROR, 10));
    EXPECT_TRUE(registry.is_threshold_exceeded(ErrorCode::COMMUNICATION_ERROR, 3));
}

TEST(ErrorHandlingTest, ErrorMetrics) {
    ErrorRegistry registry;
    
    registry.register_error(Error::create(ErrorCode::INVALID_PARAMETER, "Error 1"));
    registry.register_error(Error::create(ErrorCode::INVALID_PARAMETER, "Error 2"));
    registry.register_error(Error::create(ErrorCode::DEVICE_NOT_FOUND, "Error 3"));
    
    auto metrics = registry.get_metrics();
    
    EXPECT_EQ(metrics.total_errors, 3);
    EXPECT_EQ(metrics.errors_by_category[ErrorCategory::INVALID_INPUT], 2);
    EXPECT_EQ(metrics.errors_by_category[ErrorCategory::HARDWARE], 1);
}

TEST(ErrorHandlingTest, ThreadSafetyBasic) {
    ErrorRegistry registry;
    const int num_threads = 4;
    const int errors_per_thread = 100;
    
    std::vector<std::thread> threads;
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&registry, i, errors_per_thread]() {
            for (int j = 0; j < errors_per_thread; ++j) {
                auto error = Error::create(ErrorCode::INVALID_PARAMETER, 
                    "Thread " + std::to_string(i) + " Error " + std::to_string(j));
                registry.register_error(error);
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    auto errors = registry.get_errors();
    EXPECT_EQ(errors.size(), num_threads * errors_per_thread);
}

} // namespace emulator::utils::test