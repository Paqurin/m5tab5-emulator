#include "emulator/memory/cache_coherency.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("CacheCoherency");

CacheCoherencyController::CacheCoherencyController()
    : initialized_(false),
      memory_controller_(nullptr),
      protocol_(CoherencyProtocol::MESI) {
    COMPONENT_LOG_DEBUG("CacheCoherencyController created");
}

CacheCoherencyController::~CacheCoherencyController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("CacheCoherencyController destroyed");
}

Result<void> CacheCoherencyController::initialize(const Configuration& config, 
                                                   MemoryController& memory_controller) {
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Cache coherency controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing cache coherency controller");
    
    memory_controller_ = &memory_controller;
    
    // Initialize coherency directories for each core
    for (int core_id = 0; core_id < MAX_CORES; ++core_id) {
        core_directories_[core_id] = std::make_unique<CoreDirectory>(static_cast<CoreId>(core_id));
    }
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Cache coherency controller initialized with {} protocol", 
                      get_protocol_name(protocol_));
    
    return {};
}

Result<void> CacheCoherencyController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down cache coherency controller");
    
    // Clear all directories
    for (auto& directory : core_directories_) {
        directory.reset();
    }
    
    memory_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Cache coherency controller shutdown completed");
    return {};
}

Result<void> CacheCoherencyController::register_cache_line(CoreId core_id, Address address, 
                                                           CacheLineState initial_state) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Cache coherency controller not initialized"));
    }
    
    int core_index = static_cast<int>(core_id);
    if (core_index >= MAX_CORES) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid core ID: " + std::to_string(core_index)));
    }
    
    Address cache_line_addr = align_to_cache_line(address);
    
    auto& directory = core_directories_[core_index];
    directory->add_cache_line(cache_line_addr, initial_state);
    
    COMPONENT_LOG_TRACE("Registered cache line 0x{:08X} for core {} in state {}",
                       cache_line_addr, core_index, static_cast<int>(initial_state));
    
    return {};
}

Result<void> CacheCoherencyController::invalidate_cache_line(CoreId core_id, Address address) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Cache coherency controller not initialized"));
    }
    
    Address cache_line_addr = align_to_cache_line(address);
    
    // Invalidate in all cores except the requesting one
    for (int i = 0; i < MAX_CORES; ++i) {
        if (i != static_cast<int>(core_id)) {
            auto& directory = core_directories_[i];
            if (directory->has_cache_line(cache_line_addr)) {
                directory->set_cache_line_state(cache_line_addr, CacheLineState::INVALID);
                statistics_.invalidations_sent++;
                
                COMPONENT_LOG_TRACE("Invalidated cache line 0x{:08X} in core {}",
                                   cache_line_addr, i);
            }
        }
    }
    
    return {};
}

Result<CacheCoherencyAction> CacheCoherencyController::handle_read_request(CoreId core_id, Address address) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Cache coherency controller not initialized"));
    }
    
    Address cache_line_addr = align_to_cache_line(address);
    int core_index = static_cast<int>(core_id);
    
    auto& requesting_directory = core_directories_[core_index];
    
    // Check current state of cache line in requesting core
    CacheLineState current_state = CacheLineState::INVALID;
    if (requesting_directory->has_cache_line(cache_line_addr)) {
        current_state = requesting_directory->get_cache_line_state(cache_line_addr);
    }
    
    CacheCoherencyAction action;
    action.address = cache_line_addr;
    action.requesting_core = core_id;
    
    switch (protocol_) {
        case CoherencyProtocol::MESI:
            action = handle_mesi_read(core_id, cache_line_addr, current_state);
            break;
        case CoherencyProtocol::MSI:
            action = handle_msi_read(core_id, cache_line_addr, current_state);
            break;
        default:
            return unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
                "Unsupported coherency protocol"));
    }
    
    statistics_.read_requests++;
    COMPONENT_LOG_TRACE("Read request from core {} for address 0x{:08X}: action={}",
                       core_index, cache_line_addr, static_cast<int>(action.action_type));
    
    return action;
}

Result<CacheCoherencyAction> CacheCoherencyController::handle_write_request(CoreId core_id, Address address) {
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Cache coherency controller not initialized"));
    }
    
    Address cache_line_addr = align_to_cache_line(address);
    int core_index = static_cast<int>(core_id);
    
    auto& requesting_directory = core_directories_[core_index];
    
    // Check current state of cache line in requesting core
    CacheLineState current_state = CacheLineState::INVALID;
    if (requesting_directory->has_cache_line(cache_line_addr)) {
        current_state = requesting_directory->get_cache_line_state(cache_line_addr);
    }
    
    CacheCoherencyAction action;
    action.address = cache_line_addr;
    action.requesting_core = core_id;
    
    switch (protocol_) {
        case CoherencyProtocol::MESI:
            action = handle_mesi_write(core_id, cache_line_addr, current_state);
            break;
        case CoherencyProtocol::MSI:
            action = handle_msi_write(core_id, cache_line_addr, current_state);
            break;
        default:
            return unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
                "Unsupported coherency protocol"));
    }
    
    statistics_.write_requests++;
    COMPONENT_LOG_TRACE("Write request from core {} for address 0x{:08X}: action={}",
                       core_index, cache_line_addr, static_cast<int>(action.action_type));
    
    return action;
}

Result<void> CacheCoherencyController::execute_coherency_action(const CacheCoherencyAction& action) {
    Address cache_line_addr = action.address;
    int requesting_core = static_cast<int>(action.requesting_core);
    
    switch (action.action_type) {
        case CoherencyActionType::CACHE_HIT:
            // No action needed
            break;
            
        case CoherencyActionType::CACHE_MISS_READ_SHARED:
            // Load from memory and mark as shared in requesting core
            core_directories_[requesting_core]->set_cache_line_state(cache_line_addr, CacheLineState::SHARED);
            statistics_.cache_misses++;
            break;
            
        case CoherencyActionType::CACHE_MISS_READ_EXCLUSIVE:
            // Load from memory and mark as exclusive in requesting core
            core_directories_[requesting_core]->set_cache_line_state(cache_line_addr, CacheLineState::EXCLUSIVE);
            statistics_.cache_misses++;
            break;
            
        case CoherencyActionType::CACHE_MISS_WRITE:
            // Load from memory, invalidate in other cores, mark as modified
            RETURN_IF_ERROR(invalidate_cache_line(action.requesting_core, cache_line_addr));
            core_directories_[requesting_core]->set_cache_line_state(cache_line_addr, CacheLineState::MODIFIED);
            statistics_.cache_misses++;
            break;
            
        case CoherencyActionType::INVALIDATE_OTHERS:
            // Invalidate cache line in all other cores
            RETURN_IF_ERROR(invalidate_cache_line(action.requesting_core, cache_line_addr));
            break;
            
        case CoherencyActionType::WRITE_BACK:
            // Write modified data back to memory (would be handled by cache implementation)
            statistics_.writebacks++;
            break;
            
        case CoherencyActionType::SHARED_TO_MODIFIED:
            // Invalidate in other cores and change to modified
            RETURN_IF_ERROR(invalidate_cache_line(action.requesting_core, cache_line_addr));
            core_directories_[requesting_core]->set_cache_line_state(cache_line_addr, CacheLineState::MODIFIED);
            break;
            
        case CoherencyActionType::EXCLUSIVE_TO_MODIFIED:
            // Simply change state to modified (no invalidation needed)
            core_directories_[requesting_core]->set_cache_line_state(cache_line_addr, CacheLineState::MODIFIED);
            break;
    }
    
    return {};
}

const CoherencyStatistics& CacheCoherencyController::get_statistics() const {
    return statistics_;
}

void CacheCoherencyController::clear_statistics() {
    statistics_ = {};
}

Result<std::vector<CoreId>> CacheCoherencyController::get_sharing_cores(Address address) const {
    Address cache_line_addr = align_to_cache_line(address);
    std::vector<CoreId> sharing_cores;
    
    for (int i = 0; i < MAX_CORES; ++i) {
        const auto& directory = core_directories_[i];
        if (directory->has_cache_line(cache_line_addr)) {
            CacheLineState state = directory->get_cache_line_state(cache_line_addr);
            if (state != CacheLineState::INVALID) {
                sharing_cores.push_back(static_cast<CoreId>(i));
            }
        }
    }
    
    return sharing_cores;
}

CacheCoherencyAction CacheCoherencyController::handle_mesi_read(CoreId core_id, Address address, 
                                                                CacheLineState current_state) {
    CacheCoherencyAction action;
    action.address = address;
    action.requesting_core = core_id;
    
    switch (current_state) {
        case CacheLineState::MODIFIED:
        case CacheLineState::EXCLUSIVE:
        case CacheLineState::SHARED:
            // Cache hit
            action.action_type = CoherencyActionType::CACHE_HIT;
            break;
            
        case CacheLineState::INVALID: {
            // Cache miss - check if other cores have the line
            auto sharing_cores_result = get_sharing_cores(address);
            if (!sharing_cores_result) {
                action.action_type = CoherencyActionType::CACHE_MISS_READ_EXCLUSIVE;
                break;
            }
            
            auto sharing_cores = sharing_cores_result.value();
            
            // Remove requesting core from list
            sharing_cores.erase(
                std::remove(sharing_cores.begin(), sharing_cores.end(), core_id),
                sharing_cores.end());
            
            if (sharing_cores.empty()) {
                // No other cores have the line - load as exclusive
                action.action_type = CoherencyActionType::CACHE_MISS_READ_EXCLUSIVE;
            } else {
                // Other cores have the line - load as shared
                action.action_type = CoherencyActionType::CACHE_MISS_READ_SHARED;
                
                // Mark sharing cores as shared too
                for (CoreId sharing_core : sharing_cores) {
                    int sharing_core_index = static_cast<int>(sharing_core);
                    auto& directory = core_directories_[sharing_core_index];
                    CacheLineState sharing_state = directory->get_cache_line_state(address);
                    
                    if (sharing_state == CacheLineState::EXCLUSIVE || sharing_state == CacheLineState::MODIFIED) {
                        if (sharing_state == CacheLineState::MODIFIED) {
                            // Need to write back modified data
                            action.additional_actions.push_back(CoherencyActionType::WRITE_BACK);
                        }
                        directory->set_cache_line_state(address, CacheLineState::SHARED);
                    }
                }
            }
            break;
        }
    }
    
    return action;
}

CacheCoherencyAction CacheCoherencyController::handle_mesi_write(CoreId core_id, Address address, 
                                                                 CacheLineState current_state) {
    CacheCoherencyAction action;
    action.address = address;
    action.requesting_core = core_id;
    
    switch (current_state) {
        case CacheLineState::MODIFIED:
            // Cache hit - already have exclusive write access
            action.action_type = CoherencyActionType::CACHE_HIT;
            break;
            
        case CacheLineState::EXCLUSIVE:
            // Upgrade to modified
            action.action_type = CoherencyActionType::EXCLUSIVE_TO_MODIFIED;
            break;
            
        case CacheLineState::SHARED:
            // Need to invalidate other copies and upgrade to modified
            action.action_type = CoherencyActionType::SHARED_TO_MODIFIED;
            break;
            
        case CacheLineState::INVALID:
            // Cache miss - need to load and invalidate others
            action.action_type = CoherencyActionType::CACHE_MISS_WRITE;
            break;
    }
    
    return action;
}

CacheCoherencyAction CacheCoherencyController::handle_msi_read(CoreId core_id, Address address, 
                                                               CacheLineState current_state) {
    // MSI protocol (simplified MESI without Exclusive state)
    CacheCoherencyAction action;
    action.address = address;
    action.requesting_core = core_id;
    
    switch (current_state) {
        case CacheLineState::MODIFIED:
        case CacheLineState::SHARED:
            action.action_type = CoherencyActionType::CACHE_HIT;
            break;
            
        case CacheLineState::INVALID:
        case CacheLineState::EXCLUSIVE:  // Treat as invalid in MSI
            action.action_type = CoherencyActionType::CACHE_MISS_READ_SHARED;
            break;
    }
    
    return action;
}

CacheCoherencyAction CacheCoherencyController::handle_msi_write(CoreId core_id, Address address, 
                                                                CacheLineState current_state) {
    CacheCoherencyAction action;
    action.address = address;
    action.requesting_core = core_id;
    
    switch (current_state) {
        case CacheLineState::MODIFIED:
            action.action_type = CoherencyActionType::CACHE_HIT;
            break;
            
        case CacheLineState::SHARED:
        case CacheLineState::EXCLUSIVE:  // Treat as shared in MSI
        case CacheLineState::INVALID:
            action.action_type = CoherencyActionType::CACHE_MISS_WRITE;
            break;
    }
    
    return action;
}

Address CacheCoherencyController::align_to_cache_line(Address address) const {
    return address & ~(CACHE_LINE_SIZE - 1);
}

const char* CacheCoherencyController::get_protocol_name(CoherencyProtocol protocol) {
    switch (protocol) {
        case CoherencyProtocol::MESI: return "MESI";
        case CoherencyProtocol::MSI: return "MSI";
        default: return "Unknown";
    }
}

// CoreDirectory implementation

CoreDirectory::CoreDirectory(CoreId core_id) 
    : core_id_(core_id) {
}

void CoreDirectory::add_cache_line(Address address, CacheLineState state) {
    cache_lines_[address] = state;
}

void CoreDirectory::remove_cache_line(Address address) {
    cache_lines_.erase(address);
}

bool CoreDirectory::has_cache_line(Address address) const {
    return cache_lines_.find(address) != cache_lines_.end();
}

CacheLineState CoreDirectory::get_cache_line_state(Address address) const {
    auto it = cache_lines_.find(address);
    return (it != cache_lines_.end()) ? it->second : CacheLineState::INVALID;
}

void CoreDirectory::set_cache_line_state(Address address, CacheLineState state) {
    if (state == CacheLineState::INVALID) {
        cache_lines_.erase(address);
    } else {
        cache_lines_[address] = state;
    }
}

void CoreDirectory::clear() {
    cache_lines_.clear();
}

size_t CoreDirectory::get_cache_line_count() const {
    return cache_lines_.size();
}

}  // namespace m5tab5::emulator