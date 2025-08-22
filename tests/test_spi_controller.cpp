#include <gtest/gtest.h>
#include "emulator/peripherals/spi_controller.hpp"

namespace emulator::peripherals::test {

TEST(SPIControllerTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}

} // namespace emulator::peripherals::test