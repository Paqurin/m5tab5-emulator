#pragma once

#include "emulator/core/types.hpp"

namespace m5tab5::emulator {

/**
 * @brief Abstract interface for memory access operations
 * 
 * Provides a unified interface for memory operations that can be
 * implemented by different memory controllers, caches, or debug tools.
 */
class MemoryInterface {
public:
    virtual ~MemoryInterface() = default;

    // Basic read operations
    virtual EmulatorError read8(Address address, uint8_t& value) = 0;
    virtual EmulatorError read16(Address address, uint16_t& value) = 0;
    virtual EmulatorError read32(Address address, uint32_t& value) = 0;

    // Basic write operations
    virtual EmulatorError write8(Address address, uint8_t value) = 0;
    virtual EmulatorError write16(Address address, uint16_t value) = 0;
    virtual EmulatorError write32(Address address, uint32_t value) = 0;

    // Bulk operations (optional, can be implemented using basic operations)
    virtual EmulatorError readBlock(Address address, void* buffer, size_t size) {
        uint8_t* byte_buffer = static_cast<uint8_t*>(buffer);
        for (size_t i = 0; i < size; ++i) {
            auto result = read8(address + i, byte_buffer[i]);
            if (result != EmulatorError::Success) {
                return result;
            }
        }
        return EmulatorError::Success;
    }

    virtual EmulatorError writeBlock(Address address, const void* buffer, size_t size) {
        const uint8_t* byte_buffer = static_cast<const uint8_t*>(buffer);
        for (size_t i = 0; i < size; ++i) {
            auto result = write8(address + i, byte_buffer[i]);
            if (result != EmulatorError::Success) {
                return result;
            }
        }
        return EmulatorError::Success;
    }

    // Address validation
    virtual bool isValidAddress(Address address) const = 0;
    virtual bool isWritableAddress(Address address) const = 0;
    virtual bool isExecutableAddress(Address address) const = 0;
};

} // namespace m5tab5::emulator