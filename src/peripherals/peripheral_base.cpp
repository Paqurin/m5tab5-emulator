#include "emulator/peripherals/peripheral_base.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

// Default implementations for optional virtual methods
EmulatorError PeripheralBase::suspend() {
    return EmulatorError::Success;
}

EmulatorError PeripheralBase::resume() {
    return EmulatorError::Success;
}

EmulatorError PeripheralBase::shutdown() {
    return EmulatorError::Success;
}

EmulatorError PeripheralBase::configure(const std::string& config_json) {
    return EmulatorError::Success;
}

std::string PeripheralBase::getConfiguration() const {
    return "{}";
}

EmulatorError PeripheralBase::readMemory(uint32_t address, uint32_t size, uint8_t* data) {
    // Default implementation that delegates to register-based access
    if (size == 4 && (address % 4) == 0) {
        uint32_t value;
        EmulatorError result = readRegister(address, value);
        if (result == EmulatorError::Success && data != nullptr) {
            *reinterpret_cast<uint32_t*>(data) = value;
        }
        return result;
    }
    return EmulatorError::InvalidOperation;
}

EmulatorError PeripheralBase::writeMemory(uint32_t address, uint32_t size, const uint8_t* data) {
    // Default implementation that delegates to register-based access
    if (size == 4 && (address % 4) == 0 && data != nullptr) {
        uint32_t value = *reinterpret_cast<const uint32_t*>(data);
        return writeRegister(address, value);
    }
    return EmulatorError::InvalidOperation;
}

std::vector<PeripheralBase::MemoryRegion> PeripheralBase::getMemoryRegions() const {
    // Default implementation creates regions based on register addresses
    std::vector<PeripheralBase::MemoryRegion> regions;
    auto addresses = getRegisterAddresses();
    
    if (!addresses.empty()) {
        // Find min and max addresses to create a single region
        Address min_addr = addresses[0];
        Address max_addr = addresses[0];
        
        for (Address addr : addresses) {
            if (addr < min_addr) min_addr = addr;
            if (addr > max_addr) max_addr = addr;
        }
        
        // Create a region that encompasses all registers
        PeripheralBase::MemoryRegion region;
        region.base_address = min_addr;
        region.size = (max_addr - min_addr) + 4; // +4 for the last register
        region.type = "peripheral_registers";
        region.readable = true;
        region.writable = true;
        
        regions.push_back(region);
    }
    
    return regions;
}

bool PeripheralBase::handlesInterrupt(uint32_t interrupt_id) const {
    auto interrupt_ids = getInterruptIds();
    return std::find(interrupt_ids.begin(), interrupt_ids.end(), interrupt_id) != interrupt_ids.end();
}

void PeripheralBase::processInterrupt(uint32_t interrupt_id, bool level) {
    // Default implementation does nothing
    // Subclasses should override this to handle specific interrupts
}

void PeripheralBase::step(uint64_t cycle_count) {
    // Default implementation delegates to tick method if available
    // Most peripherals will override this method directly
    if (cycle_count > 0) {
        tick(cycle_count);
    }
}

std::string PeripheralBase::getStatusString() const {
    return "Base peripheral - no status information";
}

void PeripheralBase::dumpRegisters() const {
    // Default implementation - subclasses should provide more detailed info
}

}  // namespace m5tab5::emulator