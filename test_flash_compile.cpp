#include "emulator/storage/flash_controller.hpp"
#include "emulator/storage/partition_table.hpp"

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::storage;

int main() {
    FlashController::Config config;
    FlashController controller(config);
    return 0;
}