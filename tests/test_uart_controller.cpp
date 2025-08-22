#include <gtest/gtest.h>
#include "emulator/peripherals/uart_controller.hpp"

namespace emulator::peripherals::test {

TEST(UARTControllerTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}

} // namespace emulator::peripherals::test