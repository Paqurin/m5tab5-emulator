#include "emulator/peripherals/peripheral_manager.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/utils/error.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("PeripheralManager");

PeripheralManager::PeripheralManager(InterruptCallback interrupt_callback)
    : interrupt_callback_(interrupt_callback) {
    COMPONENT_LOG_DEBUG("PeripheralManager created");
}

PeripheralManager::~PeripheralManager() {
    if (initialized_) {
        auto result = shutdown();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to shutdown peripheral manager in destructor");
        }
    }
    COMPONENT_LOG_DEBUG("PeripheralManager destroyed");
}

EmulatorError PeripheralManager::initialize() {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (initialized_) {
        COMPONENT_LOG_WARN("Peripheral manager already initialized");
        return EmulatorError::ConfigurationError;
    }
    
    COMPONENT_LOG_INFO("Initializing peripheral manager");
    
    // Initialize core peripherals first
    auto result = initializeCorePeripherals();
    if (result != EmulatorError::Success) {
        COMPONENT_LOG_ERROR("Failed to initialize core peripherals: {}", static_cast<int>(result));
        return result;
    }
    
    // Initialize all registered peripherals
    for (auto& [name, peripheral] : peripherals_) {
        EmulatorError result = peripheral->initialize();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to initialize peripheral '{}': {}", 
                               name, static_cast<int>(result));
            return result;
        }
        COMPONENT_LOG_DEBUG("Initialized peripheral '{}'", name);
    }
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Peripheral manager initialized successfully");
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::reset() {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::ConfigurationError;
    }
    
    COMPONENT_LOG_INFO("Resetting peripheral manager");
    
    // Reset all peripherals
    for (auto& [name, peripheral] : peripherals_) {
        EmulatorError result = peripheral->reset();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to reset peripheral '{}': {}", 
                               name, static_cast<int>(result));
            // Continue with other peripherals
        }
    }
    
    // Clear interrupt states
    for (auto& state : interrupt_states_) {
        state.enabled = false;
        state.pending = false;
        state.source_peripheral.clear();
    }
    
    COMPONENT_LOG_INFO("Peripheral manager reset completed");
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::shutdown() {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::Success;
    }
    
    COMPONENT_LOG_INFO("Shutting down peripheral manager");
    
    // Shutdown all peripherals
    for (auto& [name, peripheral] : peripherals_) {
        EmulatorError result = peripheral->shutdown();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to shutdown peripheral '{}': {}", 
                               name, static_cast<int>(result));
        }
    }
    
    peripherals_.clear();
    address_map_.clear();
    
    // Clear interrupt states
    for (auto& state : interrupt_states_) {
        state.enabled = false;
        state.pending = false;
        state.source_peripheral.clear();
    }
    
    initialized_ = false;
    COMPONENT_LOG_INFO("Peripheral manager shutdown completed");
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::registerPeripheral(std::unique_ptr<PeripheralBase> peripheral) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!peripheral) {
        return EmulatorError::InvalidOperation;
    }
    
    std::string name = peripheral->getName();
    if (peripherals_.find(name) != peripherals_.end()) {
        COMPONENT_LOG_ERROR("Peripheral '{}' already registered", name);
        return EmulatorError::RESOURCE_BUSY;
    }
    
    // Register memory mappings
    auto memory_regions = peripheral->getMemoryRegions();
    for (const auto& region : memory_regions) {
        if (address_map_.find(region.base_address) != address_map_.end()) {
            COMPONENT_LOG_ERROR("Memory region 0x{:x} already mapped for peripheral '{}'", 
                               region.base_address, name);
            return EmulatorError::ResourceNotAvailable;
        }
        address_map_[region.base_address] = peripheral.get();
    }
    
    peripherals_[name] = std::move(peripheral);
    COMPONENT_LOG_INFO("Registered peripheral '{}'", name);
    
    // If already initialized, initialize the new peripheral
    if (initialized_) {
        EmulatorError result = peripherals_[name]->initialize();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to initialize newly registered peripheral '{}': {}", 
                               name, static_cast<int>(result));
            // Remove the failed peripheral
            peripherals_.erase(name);
            // Remove its memory mappings
            for (const auto& region : memory_regions) {
                address_map_.erase(region.base_address);
            }
            return result;
        }
    }
    
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::unregisterPeripheral(const std::string& name) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    auto it = peripherals_.find(name);
    if (it == peripherals_.end()) {
        COMPONENT_LOG_WARN("Peripheral '{}' not found for unregistration", name);
        return EmulatorError::ResourceNotAvailable;
    }
    
    // Shutdown the peripheral if initialized
    if (initialized_) {
        EmulatorError result = it->second->shutdown();
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Failed to shutdown peripheral '{}' during unregistration: {}", 
                               name, static_cast<int>(result));
        }
    }
    
    // Remove memory mappings
    auto memory_regions = it->second->getMemoryRegions();
    for (const auto& region : memory_regions) {
        address_map_.erase(region.base_address);
    }
    
    peripherals_.erase(it);
    COMPONENT_LOG_INFO("Unregistered peripheral '{}'", name);
    return EmulatorError::Success;
}

PeripheralBase* PeripheralManager::getPeripheral(const std::string& name) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    auto it = peripherals_.find(name);
    if (it != peripherals_.end()) {
        return it->second.get();
    }
    
    return nullptr;
}

EmulatorError PeripheralManager::readRegister(Address address, uint32_t& value) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::ConfigurationError;
    }
    
    // Find the peripheral that owns this address
    PeripheralBase* peripheral = findPeripheralByAddress(address);
    if (!peripheral) {
        COMPONENT_LOG_DEBUG("No peripheral found for register read at address 0x{:x}", address);
        return EmulatorError::InvalidAddress;
    }
    
    return peripheral->readRegister(address, value);
}

EmulatorError PeripheralManager::writeRegister(Address address, uint32_t value) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::ConfigurationError;
    }
    
    // Find the peripheral that owns this address
    PeripheralBase* peripheral = findPeripheralByAddress(address);
    if (!peripheral) {
        COMPONENT_LOG_DEBUG("No peripheral found for register write at address 0x{:x}", address);
        return EmulatorError::InvalidAddress;
    }
    
    return peripheral->writeRegister(address, value);
}

EmulatorError PeripheralManager::triggerInterrupt(uint32_t interrupt_id) {
    std::lock_guard<std::mutex> lock(interrupt_mutex_);
    
    if (!initialized_ || interrupt_id >= MAX_INTERRUPTS) {
        return EmulatorError::InvalidOperation;
    }
    
    interrupt_states_[interrupt_id].pending = true;
    
    // Notify the interrupt callback
    if (interrupt_callback_) {
        interrupt_callback_(interrupt_id, true);
    }
    
    COMPONENT_LOG_TRACE("Triggered interrupt {}", interrupt_id);
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::setInterruptEnabled(uint32_t interrupt_id, bool enabled) {
    std::lock_guard<std::mutex> lock(interrupt_mutex_);
    
    if (interrupt_id >= MAX_INTERRUPTS) {
        return EmulatorError::InvalidOperation;
    }
    
    interrupt_states_[interrupt_id].enabled = enabled;
    return EmulatorError::Success;
}

bool PeripheralManager::isInterruptPending(uint32_t interrupt_id) const {
    std::lock_guard<std::mutex> lock(interrupt_mutex_);
    
    if (interrupt_id >= MAX_INTERRUPTS) {
        return false;
    }
    
    return interrupt_states_[interrupt_id].pending && interrupt_states_[interrupt_id].enabled;
}

EmulatorError PeripheralManager::tick(ClockCycle cycle) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::ConfigurationError;
    }
    
    // Tick all peripherals
    for (auto& [name, peripheral] : peripherals_) {
        EmulatorError result = peripheral->tick(cycle);
        if (result != EmulatorError::Success) {
            COMPONENT_LOG_DEBUG("Peripheral '{}' tick returned error: {}", 
                               name, static_cast<int>(result));
        }
    }
    
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::syncPeripherals() {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (!initialized_) {
        return EmulatorError::ConfigurationError;
    }
    
    // Synchronization logic can be implemented here
    // For now, this is a placeholder
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::loadPlugin(const std::string& plugin_path) {
    // Plugin loading implementation
    // This is a placeholder for now
    COMPONENT_LOG_WARN("Plugin loading not implemented: {}", plugin_path);
    return EmulatorError::NotImplemented;
}

EmulatorError PeripheralManager::unloadPlugin(const std::string& plugin_name) {
    // Plugin unloading implementation
    // This is a placeholder for now
    COMPONENT_LOG_WARN("Plugin unloading not implemented: {}", plugin_name);
    return EmulatorError::NotImplemented;
}

std::vector<std::string> PeripheralManager::getLoadedPlugins() const {
    // Return empty vector for now
    return {};
}

EmulatorError PeripheralManager::configurePeripheral(const std::string& name, 
                                                   const std::string& config_json) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    auto it = peripherals_.find(name);
    if (it == peripherals_.end()) {
        return EmulatorError::ResourceNotAvailable;
    }
    
    return it->second->configure(config_json);
}

void PeripheralManager::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = PeripheralStatistics{};
}

EmulatorError PeripheralManager::sendMessage(const std::string& from_peripheral,
                                           const std::string& to_peripheral,
                                           const std::string& message_type,
                                           const void* data, size_t size) {
    std::lock_guard<std::mutex> lock(message_mutex_);
    
    // Create message
    Message msg;
    msg.from = from_peripheral;
    msg.to = to_peripheral;
    msg.type = message_type;
    if (data && size > 0) {
        msg.data.resize(size);
        std::memcpy(msg.data.data(), data, size);
    }
    
    message_queue_.push(msg);
    return EmulatorError::Success;
}

EmulatorError PeripheralManager::initializeCorePeripherals() {
    // Core peripherals initialization would go here
    // For now, this is a placeholder
    COMPONENT_LOG_DEBUG("Core peripherals initialization placeholder");
    return EmulatorError::Success;
}

PeripheralBase* PeripheralManager::findPeripheralByAddress(Address address) {
    // Performance optimization: check last accessed peripheral first
    if (last_peripheral_ && last_address_ == address) {
        auto regions = last_peripheral_->getMemoryRegions();
        for (const auto& region : regions) {
            if (address >= region.base_address && 
                address < (region.base_address + region.size)) {
                return last_peripheral_;
            }
        }
    }
    
    // Find the peripheral that owns this address range
    PeripheralBase* best_match = nullptr;
    Address best_base = 0;
    
    for (const auto& [base_address, peripheral] : address_map_) {
        if (address >= base_address && base_address >= best_base) {
            // Check if address is within this peripheral's range
            auto regions = peripheral->getMemoryRegions();
            for (const auto& region : regions) {
                if (address >= region.base_address && 
                    address < (region.base_address + region.size)) {
                    best_match = peripheral;
                    best_base = base_address;
                    break;
                }
            }
        }
    }
    
    // Cache the result for performance
    if (best_match) {
        last_address_ = address;
        last_peripheral_ = best_match;
    }
    
    return best_match;
}

}  // namespace m5tab5::emulator