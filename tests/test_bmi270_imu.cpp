#include <gtest/gtest.h>
#include "emulator/peripherals/bmi270_imu.hpp"

namespace emulator::peripherals::test {

TEST(BMI270IMUTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}

} // namespace emulator::peripherals::test