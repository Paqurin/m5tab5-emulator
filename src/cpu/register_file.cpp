#include "emulator/cpu/register_file.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("RegisterFile");

RegisterFile::RegisterFile() {
    reset();
    COMPONENT_LOG_DEBUG("RegisterFile created (stub)");
}

RegisterFile::~RegisterFile() {
    COMPONENT_LOG_DEBUG("RegisterFile destroyed");
}

void RegisterFile::reset() {
    // Stub implementation
    COMPONENT_LOG_DEBUG("RegisterFile reset completed");
}

u32 RegisterFile::read(u8 reg_num) const {
    // x0 always reads as 0
    if (reg_num == 0) {
        return 0;
    }
    
    // Stub implementation - return 0 for all registers
    return 0;
}

void RegisterFile::write(u8 reg_num, u32 value) {
    // x0 is hardwired to 0 and cannot be written
    if (reg_num == 0) {
        return;
    }
    
    // Stub implementation - ignore writes
    COMPONENT_LOG_TRACE("GP register x{} = 0x{:08X} (stub)", reg_num, value);
}

}  // namespace m5tab5::emulator