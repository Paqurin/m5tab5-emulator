#pragma once

#include "emulator/core/types.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace m5tab5::emulator {

/**
 * @brief Base class for all peripheral devices
 * 
 * Provides common interface and functionality for:
 * - Register-based memory-mapped I/O
 * - Interrupt generation
 * - Configuration and state management
 * - Debug and inspection capabilities
 */
class PeripheralBase {
public:
    virtual ~PeripheralBase() = default;

    // Basic peripheral interface
    virtual std::string getName() const = 0;
    virtual EmulatorError initialize() = 0;
    virtual EmulatorError reset() = 0;
    virtual EmulatorError tick(ClockCycle cycle) = 0;

    // Register access
    virtual EmulatorError readRegister(Address address, uint32_t& value) = 0;
    virtual EmulatorError writeRegister(Address address, uint32_t value) = 0;

    // Configuration
    virtual std::vector<Address> getRegisterAddresses() const = 0;
    virtual std::vector<uint32_t> getInterruptIds() const = 0;

    // Optional features (default implementations provided)
    virtual EmulatorError configure(const std::string& config_json) {
        return EmulatorError::Success;
    }

    virtual std::string getConfiguration() const {
        return "{}";
    }

    virtual EmulatorError suspend() {
        return EmulatorError::Success;
    }

    virtual EmulatorError resume() {
        return EmulatorError::Success;
    }

    // Debug and inspection
    virtual std::string getStatusString() const {
        return "OK";
    }

    virtual std::unordered_map<std::string, uint64_t> getStatistics() const {
        return {};
    }

    virtual void resetStatistics() {}

protected:
    // Helper for interrupt generation
    virtual void triggerInterrupt(uint32_t interrupt_id) {
        if (interrupt_callback_) {
            interrupt_callback_(interrupt_id, true);
        }
    }

    virtual void clearInterrupt(uint32_t interrupt_id) {
        if (interrupt_callback_) {
            interrupt_callback_(interrupt_id, false);
        }
    }

    // Interrupt callback (set by peripheral manager)
    std::function<void(uint32_t interrupt_id, bool level)> interrupt_callback_;

    friend class PeripheralManager;
};

} // namespace m5tab5::emulator