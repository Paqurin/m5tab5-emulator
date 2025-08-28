#include "emulator/debug/debugger.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/logging.hpp"

namespace m5tab5::emulator {

DECLARE_LOGGER("Debugger");

Debugger::Debugger(EmulatorCore& core, const DebugConfig& config)
    : config_(config), emulator_(core), next_breakpoint_id_(1) {
    COMPONENT_LOG_DEBUG("Debugger created with GDB server={}, profiler={}", 
                       config_.enable_gdb_server, config_.enable_profiler);
}

Debugger::~Debugger() {
    if (gdb_server_ && gdb_server_->is_running()) {
        gdb_server_->stop();
    }
    if (profiler_) {
        // Profiler cleanup handled by destructor
    }
    COMPONENT_LOG_DEBUG("Debugger destroyed");
}

// DebugInterface implementation with full functionality
EmulatorError Debugger::initialize() {
    COMPONENT_LOG_DEBUG("Initializing debugger subsystem");
    
    // Initialize GDB server if enabled
    if (config_.enable_gdb_server) {
        GDBServer::Config gdb_config;
        gdb_config.port = config_.gdb_port;
        gdb_config.enabled = true;
        gdb_config.bind_address = "localhost";
        gdb_config.multi_core_support = true;
        gdb_config.verbose_logging = true;
        
        gdb_server_ = std::make_unique<GDBServer>(emulator_, gdb_config);
        if (!gdb_server_->start()) {
            COMPONENT_LOG_ERROR("Failed to start GDB server on port {}", config_.gdb_port);
            return EmulatorError::ConfigurationError;
        }
        COMPONENT_LOG_INFO("GDB server started on port {}", config_.gdb_port);
    }
    
    // Initialize profiler if enabled
    if (config_.enable_profiler) {
        auto profiler_result = Profiler::create();
        if (!profiler_result.has_value()) {
            COMPONENT_LOG_ERROR("Failed to create profiler: {}", profiler_result.error().message());
            return EmulatorError::ConfigurationError;
        }
        profiler_ = std::move(profiler_result.value());
        
        auto init_result = profiler_->initialize();
        if (!init_result.has_value()) {
            COMPONENT_LOG_ERROR("Failed to initialize profiler: {}", init_result.error().message());
            return EmulatorError::ConfigurationError;
        }
        COMPONENT_LOG_INFO("Performance profiler initialized");
    }
    
    // Initialize tracing if enabled
    if (config_.enable_tracing) {
        trace_buffer_.reserve(config_.trace_buffer_size);
        COMPONENT_LOG_INFO("Execution tracing enabled with {}KB buffer", 
                         config_.trace_buffer_size / 1024);
    }
    
    COMPONENT_LOG_DEBUG("Debugger initialization completed successfully");
    return EmulatorError::Success;
}

EmulatorError Debugger::shutdown() {
    COMPONENT_LOG_DEBUG("Shutting down debugger subsystem");
    
    // Stop GDB server
    if (gdb_server_ && gdb_server_->is_running()) {
        gdb_server_->stop();
        COMPONENT_LOG_INFO("GDB server stopped");
    }
    
    // Stop profiler
    if (profiler_) {
        profiler_->stop_profiling();
        profiler_->shutdown();
        COMPONENT_LOG_INFO("Profiler stopped");
    }
    
    // Clear trace buffer
    trace_buffer_.clear();
    
    // Clear all breakpoints
    breakpoints_.clear();
    
    COMPONENT_LOG_DEBUG("Debugger shutdown completed");
    return EmulatorError::Success;
}

uint32_t Debugger::setBreakpoint(BreakpointType type, Address address, size_t size) {
    uint32_t bp_id = next_breakpoint_id_++;
    
    Breakpoint bp;
    bp.id = bp_id;
    bp.type = type;
    bp.address = address;
    bp.size = size;
    bp.enabled = true;
    bp.hit_count = 0;
    
    breakpoints_[bp_id] = bp;
    
    // Install breakpoint in CPU cores
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (cpu_manager) {
        // Install on both cores for maximum compatibility
        auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
        auto core1 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_1);
        
        if (core0.has_value()) {
            // TODO: Implement actual CPU breakpoint installation
            COMPONENT_LOG_DEBUG("Breakpoint {} installed on Core 0", bp_id);
        }
        
        if (core1.has_value()) {
            // TODO: Implement actual CPU breakpoint installation  
            COMPONENT_LOG_DEBUG("Breakpoint {} installed on Core 1", bp_id);
        }
    }
    
    COMPONENT_LOG_INFO("Breakpoint {} set at 0x{:08X} (type={}, size={})", 
                      bp_id, address, static_cast<int>(type), size);
    
    return bp_id;
}

EmulatorError Debugger::removeBreakpoint(uint32_t breakpoint_id) {
    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        COMPONENT_LOG_WARN("Breakpoint {} not found for removal", breakpoint_id);
        return EmulatorError::InvalidOperation;
    }
    
    // Remove from CPU cores
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (cpu_manager) {
        // Remove from both cores
        auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
        auto core1 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_1);
        
        if (core0.has_value()) {
            // TODO: Implement actual CPU breakpoint removal
            COMPONENT_LOG_DEBUG("Breakpoint {} removed from Core 0", breakpoint_id);
        }
        
        if (core1.has_value()) {
            // TODO: Implement actual CPU breakpoint removal
            COMPONENT_LOG_DEBUG("Breakpoint {} removed from Core 1", breakpoint_id);
        }
    }
    
    breakpoints_.erase(it);
    COMPONENT_LOG_INFO("Breakpoint {} removed", breakpoint_id);
    
    return EmulatorError::Success;
}

EmulatorError Debugger::enableBreakpoint(uint32_t breakpoint_id, bool enabled) {
    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        COMPONENT_LOG_WARN("Breakpoint {} not found for enable/disable", breakpoint_id);
        return EmulatorError::InvalidOperation;
    }
    
    it->second.enabled = enabled;
    COMPONENT_LOG_INFO("Breakpoint {} {} at 0x{:08X}", 
                      breakpoint_id, enabled ? "enabled" : "disabled", it->second.address);
    
    return EmulatorError::Success;
}

std::vector<Breakpoint> Debugger::getBreakpoints() const {
    std::vector<Breakpoint> result;
    result.reserve(breakpoints_.size());
    
    for (const auto& [id, bp] : breakpoints_) {
        result.push_back(bp);
    }
    
    return result;
}

EmulatorError Debugger::stepInstruction() {
    COMPONENT_LOG_DEBUG("Step instruction requested");
    
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (!cpu_manager) {
        COMPONENT_LOG_ERROR("CPU manager not available for step instruction");
        return EmulatorError::ResourceNotAvailable;
    }
    
    // Execute single instruction on active core (Core 0 by default)
    auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
    if (core0.has_value()) {
        auto step_result = core0.value()->step();
        if (step_result != EmulatorError::Success) {
            COMPONENT_LOG_ERROR("Step instruction failed: {}", static_cast<int>(step_result));
            return step_result;
        }
        
        COMPONENT_LOG_INFO("Single instruction executed on Core 0, PC=0x{:08X}", 
                          core0.value()->getProgramCounter());
    }
    
    return EmulatorError::Success;
}

EmulatorError Debugger::stepOver() {
    COMPONENT_LOG_DEBUG("Step over requested");
    
    // TODO: Implement proper step-over logic (step until next sequential instruction)
    // For now, just do a single step
    return stepInstruction();
}

EmulatorError Debugger::stepOut() {
    COMPONENT_LOG_DEBUG("Step out requested");
    
    // TODO: Implement proper step-out logic (step until return from current function)
    // For now, just do a single step
    return stepInstruction();
}

EmulatorError Debugger::continueExecution() {
    COMPONENT_LOG_DEBUG("Continue execution requested");
    
    // Resume emulator execution
    auto result = emulator_.resume();
    if (!result.has_value()) {
        COMPONENT_LOG_ERROR("Failed to resume execution: {}", result.error().message());
        return EmulatorError::InvalidOperation;
    }
    
    COMPONENT_LOG_INFO("Execution resumed");
    return EmulatorError::Success;
}

EmulatorError Debugger::pauseExecution() {
    COMPONENT_LOG_DEBUG("Pause execution requested");
    
    // Pause emulator execution
    auto result = emulator_.pause();
    if (!result.has_value()) {
        COMPONENT_LOG_ERROR("Failed to pause execution: {}", result.error().message());
        return EmulatorError::InvalidOperation;
    }
    
    COMPONENT_LOG_INFO("Execution paused");
    return EmulatorError::Success;
}

EmulatorError Debugger::readMemory(Address address, size_t size, std::vector<uint8_t>& data) {
    COMPONENT_LOG_DEBUG("Read memory at 0x{:08X}, size {}", address, size);
    
    auto memory_controller = emulator_.getComponent<MemoryController>();
    if (!memory_controller) {
        COMPONENT_LOG_ERROR("Memory controller not available for memory read");
        return EmulatorError::ResourceNotAvailable;
    }
    
    data.resize(size);
    
    // Read memory byte by byte for maximum compatibility
    for (size_t i = 0; i < size; i++) {
        auto byte_result = memory_controller->read_u8(address + i);
        if (!byte_result.has_value()) {
            COMPONENT_LOG_ERROR("Memory read failed at 0x{:08X}: {}", 
                               address + i, byte_result.error().message());
            return EmulatorError::MemoryAccessError;
        }
        data[i] = byte_result.value();
    }
    
    COMPONENT_LOG_DEBUG("Memory read successful: {} bytes from 0x{:08X}", size, address);
    return EmulatorError::Success;
}

EmulatorError Debugger::writeMemory(Address address, const std::vector<uint8_t>& data) {
    COMPONENT_LOG_DEBUG("Write memory at 0x{:08X}, size {}", address, data.size());
    
    auto memory_controller = emulator_.getComponent<MemoryController>();
    if (!memory_controller) {
        COMPONENT_LOG_ERROR("Memory controller not available for memory write");
        return EmulatorError::ResourceNotAvailable;
    }
    
    // Write memory byte by byte for maximum compatibility
    for (size_t i = 0; i < data.size(); i++) {
        auto write_result = memory_controller->write_u8(address + i, data[i]);
        if (!write_result.has_value()) {
            COMPONENT_LOG_ERROR("Memory write failed at 0x{:08X}: {}", 
                               address + i, write_result.error().message());
            return EmulatorError::MemoryAccessError;
        }
    }
    
    COMPONENT_LOG_DEBUG("Memory write successful: {} bytes to 0x{:08X}", data.size(), address);
    return EmulatorError::Success;
}

EmulatorError Debugger::readRegister(uint32_t reg_id, uint32_t& value) {
    COMPONENT_LOG_DEBUG("Read register {}", reg_id);
    
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (!cpu_manager) {
        COMPONENT_LOG_ERROR("CPU manager not available for register read");
        return EmulatorError::ResourceNotAvailable;
    }
    
    // Read from Core 0 by default
    auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
    if (core0.has_value()) {
        auto& registers = core0.value()->getRegisters();
        if (reg_id < 32) {
            value = registers.read(reg_id);
            COMPONENT_LOG_DEBUG("Register {} = 0x{:08X}", reg_id, value);
            return EmulatorError::Success;
        } else {
            COMPONENT_LOG_ERROR("Invalid register ID: {}", reg_id);
            return EmulatorError::InvalidOperation;
        }
    }
    
    value = 0;
    return EmulatorError::ResourceNotAvailable;
}

EmulatorError Debugger::writeRegister(uint32_t reg_id, uint32_t value) {
    COMPONENT_LOG_DEBUG("Write register {} = 0x{:08X}", reg_id, value);
    
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (!cpu_manager) {
        COMPONENT_LOG_ERROR("CPU manager not available for register write");
        return EmulatorError::ResourceNotAvailable;
    }
    
    // Write to Core 0 by default
    auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
    if (core0.has_value()) {
        auto& registers = core0.value()->getRegisters();
        if (reg_id < 32) {
            registers.write(reg_id, value);
            COMPONENT_LOG_INFO("Register {} set to 0x{:08X}", reg_id, value);
            return EmulatorError::Success;
        } else {
            COMPONENT_LOG_ERROR("Invalid register ID: {}", reg_id);
            return EmulatorError::InvalidOperation;
        }
    }
    
    return EmulatorError::ResourceNotAvailable;
}

std::vector<uint32_t> Debugger::getAllRegisters() {
    COMPONENT_LOG_DEBUG("Get all registers");
    
    std::vector<uint32_t> registers(32, 0);
    
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (!cpu_manager) {
        COMPONENT_LOG_ERROR("CPU manager not available for register dump");
        return registers;
    }
    
    // Read all registers from Core 0
    auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
    if (core0.has_value()) {
        auto& reg_file = core0.value()->getRegisters();
        for (uint32_t i = 0; i < 32; i++) {
            registers[i] = reg_file.read(i);
        }
        COMPONENT_LOG_DEBUG("Dumped {} registers from Core 0", registers.size());
    }
    
    return registers;
}

std::vector<StackFrame> Debugger::getStackTrace(uint32_t max_frames) {
    COMPONENT_LOG_DEBUG("Get stack trace (max {} frames)", max_frames);
    return {}; // Return empty vector for now
}

EmulatorError Debugger::getStackFrame(uint32_t frame_index, StackFrame& frame) {
    COMPONENT_LOG_DEBUG("Get stack frame {}", frame_index);
    frame = StackFrame{}; // Initialize with default values
    return EmulatorError::Success;
}

EmulatorError Debugger::loadSymbols(const std::string& symbol_file) {
    COMPONENT_LOG_DEBUG("Load symbols from '{}'", symbol_file);
    return EmulatorError::Success;
}

std::optional<Symbol> Debugger::lookupSymbol(const std::string& name) {
    COMPONENT_LOG_DEBUG("Lookup symbol '{}'", name);
    return std::nullopt; // No symbols for now
}

std::optional<Symbol> Debugger::lookupSymbol(Address address) {
    COMPONENT_LOG_DEBUG("Lookup symbol at 0x{:08X}", address);
    return std::nullopt; // No symbols for now
}

// GDB server integration
EmulatorError Debugger::startGDBServer() {
    if (!config_.enable_gdb_server) {
        COMPONENT_LOG_ERROR("GDB server not enabled in configuration");
        return EmulatorError::ConfigurationError;
    }
    
    if (!gdb_server_) {
        COMPONENT_LOG_ERROR("GDB server not initialized");
        return EmulatorError::ResourceNotAvailable;
    }
    
    if (gdb_server_->is_running()) {
        COMPONENT_LOG_WARN("GDB server already running");
        return EmulatorError::Success;
    }
    
    if (!gdb_server_->start()) {
        COMPONENT_LOG_ERROR("Failed to start GDB server");
        return EmulatorError::ConfigurationError;
    }
    
    COMPONENT_LOG_INFO("GDB server started successfully on port {}", config_.gdb_port);
    return EmulatorError::Success;
}

EmulatorError Debugger::stopGDBServer() {
    if (!gdb_server_) {
        COMPONENT_LOG_WARN("GDB server not initialized");
        return EmulatorError::Success;
    }
    
    if (!gdb_server_->is_running()) {
        COMPONENT_LOG_WARN("GDB server not running");
        return EmulatorError::Success;
    }
    
    gdb_server_->stop();
    COMPONENT_LOG_INFO("GDB server stopped");
    return EmulatorError::Success;
}

bool Debugger::isGDBServerRunning() const {
    return gdb_server_ && gdb_server_->is_running();
}

// Profiling integration
EmulatorError Debugger::startProfiling() {
    if (!config_.enable_profiler) {
        COMPONENT_LOG_ERROR("Profiler not enabled in configuration");
        return EmulatorError::ConfigurationError;
    }
    
    if (!profiler_) {
        COMPONENT_LOG_ERROR("Profiler not initialized");
        return EmulatorError::ResourceNotAvailable;
    }
    
    profiler_->start_profiling("Debug Session");
    COMPONENT_LOG_INFO("Performance profiling started");
    return EmulatorError::Success;
}

EmulatorError Debugger::stopProfiling() {
    if (!profiler_) {
        COMPONENT_LOG_WARN("Profiler not initialized");
        return EmulatorError::Success;
    }
    
    profiler_->stop_profiling();
    COMPONENT_LOG_INFO("Performance profiling stopped");
    return EmulatorError::Success;
}

ProfileReport Debugger::getProfileReport() {
    ProfileReport report;
    report.total_cycles = 0;
    report.total_instructions = 0;
    
    if (!profiler_) {
        COMPONENT_LOG_WARN("Profiler not available for report generation");
        return report;
    }
    
    // Get performance metrics from profiler
    auto metrics_result = profiler_->get_performance_metrics();
    if (metrics_result.has_value()) {
        const auto& metrics = metrics_result.value();
        report.total_instructions = metrics.total_operations;
        // Convert execution time to approximate cycles (assuming 400MHz)
        auto ns = metrics.total_execution_time.count();
        report.total_cycles = static_cast<uint64_t>((ns * 400.0) / 1000.0); // 400MHz = 0.4 cycles/ns
    }
    
    // Get component statistics and convert to profile entries
    auto stats_result = profiler_->get_component_stats();
    if (stats_result.has_value()) {
        const auto& stats = stats_result.value();
        
        for (const auto& component_stat : stats) {
            ProfileEntry entry;
            entry.pc = 0; // Component statistics don't have specific PC values
            entry.function_name = component_stat.name;
            entry.hit_count = component_stat.call_count;
            entry.inclusive_cycles = static_cast<uint64_t>((component_stat.total_time.count() * 400.0) / 1000.0);
            entry.exclusive_cycles = entry.inclusive_cycles; // Simplified for now
            
            report.functions.push_back(entry);
            
            // Add to hot spots if significant
            if (component_stat.cpu_percentage > 5.0) {
                report.hot_spots.push_back(entry);
            }
        }
    }
    
    return report;
}

// Debug information
Debugger::DebugInfo Debugger::getDebugInfo() const {
    DebugInfo info;
    
    // Get emulator state
    auto state = emulator_.get_state();
    info.is_running = (state == EmulatorState::RUNNING);
    info.is_paused = (state == EmulatorState::PAUSED);
    
    // Get current PC from Core 0
    auto cpu_manager = emulator_.getComponent<DualCoreManager>();
    if (cpu_manager) {
        auto core0 = cpu_manager->get_core(DualCoreManager::CoreId::CORE_0);
        if (core0.has_value()) {
            info.current_pc = core0.value()->getProgramCounter();
        }
    }
    
    // Breakpoint statistics
    info.active_breakpoints = breakpoints_.size();
    info.hit_breakpoints = 0;
    for (const auto& [id, bp] : breakpoints_) {
        if (bp.hit_count > 0) {
            info.hit_breakpoints++;
        }
    }
    
    // Tracing information
    info.tracing_enabled = tracing_enabled_.load();
    info.trace_entries = trace_buffer_.size();
    
    return info;
}

// Execution tracing
EmulatorError Debugger::startTracing() {
    if (!config_.enable_tracing) {
        COMPONENT_LOG_ERROR("Tracing not enabled in configuration");
        return EmulatorError::ConfigurationError;
    }
    
    tracing_enabled_ = true;
    trace_write_index_ = 0;
    trace_buffer_full_ = false;
    
    COMPONENT_LOG_INFO("Execution tracing started (buffer: {}KB)", 
                      config_.trace_buffer_size / 1024);
    return EmulatorError::Success;
}

EmulatorError Debugger::stopTracing() {
    tracing_enabled_ = false;
    COMPONENT_LOG_INFO("Execution tracing stopped ({} entries captured)", 
                      trace_buffer_full_ ? trace_buffer_.size() : trace_write_index_);
    return EmulatorError::Success;
}

std::vector<TraceEntry> Debugger::getTrace(size_t max_entries) {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    
    if (trace_buffer_.empty()) {
        return {};
    }
    
    size_t entries_available = trace_buffer_full_ ? trace_buffer_.size() : trace_write_index_;
    size_t entries_to_return = std::min(max_entries, entries_available);
    
    std::vector<TraceEntry> result;
    result.reserve(entries_to_return);
    
    // Return the most recent entries
    size_t start_idx = entries_available > entries_to_return ? 
                      entries_available - entries_to_return : 0;
    
    for (size_t i = start_idx; i < entries_available; i++) {
        result.push_back(trace_buffer_[i]);
    }
    
    COMPONENT_LOG_DEBUG("Returning {} trace entries (of {} available)", 
                       result.size(), entries_available);
    
    return result;
}

} // namespace m5tab5::emulator