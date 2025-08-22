#pragma once

#include "emulator/peripherals/peripheral_base.hpp"

namespace m5tab5::emulator {

class CommunicationController : public PeripheralBase {
public:
    CommunicationController() = default;
    
    std::string getName() const override { return "communication"; }
    EmulatorError initialize() override { return EmulatorError::Success; }
    EmulatorError reset() override { return EmulatorError::Success; }
    EmulatorError tick(ClockCycle cycle) override { return EmulatorError::Success; }
    
    EmulatorError readRegister(Address address, uint32_t& value) override {
        value = 0;
        return EmulatorError::Success;
    }
    
    EmulatorError writeRegister(Address address, uint32_t value) override {
        return EmulatorError::Success;
    }
    
    std::vector<Address> getRegisterAddresses() const override { return {}; }
    std::vector<uint32_t> getInterruptIds() const override { return {}; }
};

} // namespace m5tab5::emulator