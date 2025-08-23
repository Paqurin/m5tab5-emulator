#include "emulator/plugin/plugin_manager.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("PluginManager");

PluginManager::PluginManager() {
    COMPONENT_LOG_DEBUG("PluginManager created");
}

PluginManager::~PluginManager() {
    shutdown();
    COMPONENT_LOG_DEBUG("PluginManager destroyed");
}

Result<void> PluginManager::initialize(const Configuration& config) {
    COMPONENT_LOG_INFO("PluginManager initialized (stub implementation)");
    return {};
}

Result<void> PluginManager::shutdown() {
    COMPONENT_LOG_INFO("PluginManager shutdown");
    return {};
}

} // namespace m5tab5::emulator