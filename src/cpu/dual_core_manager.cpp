#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("DualCoreManager");

DualCoreManager::DualCoreManager() 
    : state_(CpuState::STOPPED),
      memory_controller_(nullptr),
      interrupt_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("DualCoreManager created");
}

DualCoreManager::~DualCoreManager() {
    if (state_ != CpuState::STOPPED) {
        stop();
    }
    COMPONENT_LOG_DEBUG("DualCoreManager destroyed");
}

Result<void> DualCoreManager::initialize(const Configuration& config, MemoryController& memory_controller) {
    if (state_ != CpuState::STOPPED) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Dual core manager already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing dual core manager");
    
    memory_controller_ = &memory_controller;
    
    // Initialize CPU cores
    cores_[CoreId::CORE_0] = std::make_unique<CpuCore>(CoreId::CORE_0);
    cores_[CoreId::CORE_1] = std::make_unique<CpuCore>(CoreId::CORE_1);
    cores_[CoreId::LP_CORE] = std::make_unique<CpuCore>(CoreId::LP_CORE);
    
    // Initialize each core
    for (auto& [core_id, core] : cores_) {
        RETURN_IF_ERROR(core->initialize(config, memory_controller));
        COMPONENT_LOG_DEBUG("Initialized CPU core {}", static_cast<int>(core_id));
    }
    
    // Initialize interrupt controller
    interrupt_controller_ = std::make_unique<InterruptController>();
    RETURN_IF_ERROR(interrupt_controller_->initialize(config));
    
    // Initialize timer controller
    timer_controller_ = std::make_unique<TimerController>();
    RETURN_IF_ERROR(timer_controller_->initialize(config));
    
    // Set up inter-core communication
    RETURN_IF_ERROR(setup_inter_core_communication());
    
    state_ = CpuState::HALTED;
    COMPONENT_LOG_INFO("Dual core manager initialized successfully");
    
    return {};
}

Result<void> DualCoreManager::start() {
    if (state_ != CpuState::HALTED) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Cores must be halted to start"));
    }
    
    COMPONENT_LOG_INFO("Starting all CPU cores");
    
    // Start all cores
    for (auto& [core_id, core] : cores_) {
        RETURN_IF_ERROR(core->start());
        COMPONENT_LOG_DEBUG("Started CPU core {}", static_cast<int>(core_id));
    }
    
    // Start interrupt controller
    if (interrupt_controller_) {
        RETURN_IF_ERROR(interrupt_controller_->start());
    }
    
    // Start timer controller
    if (timer_controller_) {
        RETURN_IF_ERROR(timer_controller_->start());
    }
    
    state_ = CpuState::RUNNING;
    COMPONENT_LOG_INFO("All CPU cores started successfully");
    
    return {};
}

Result<void> DualCoreManager::stop() {
    if (state_ == CpuState::STOPPED) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Stopping all CPU cores");
    
    // Stop timer controller
    if (timer_controller_) {
        timer_controller_->stop();
    }
    
    // Stop interrupt controller
    if (interrupt_controller_) {
        interrupt_controller_->stop();
    }
    
    // Stop all cores
    for (auto& [core_id, core] : cores_) {
        auto result = core->stop();
        if (!result) {
            COMPONENT_LOG_WARN("Failed to stop CPU core {}: {}", 
                              static_cast<int>(core_id), result.error().to_string());
        }
    }
    
    state_ = CpuState::STOPPED;
    COMPONENT_LOG_INFO("All CPU cores stopped");
    
    return {};
}

Result<void> DualCoreManager::pause() {
    if (state_ != CpuState::RUNNING) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Cores must be running to pause"));
    }
    
    COMPONENT_LOG_DEBUG("Pausing all CPU cores");
    
    // Pause all cores
    for (auto& [core_id, core] : cores_) {
        RETURN_IF_ERROR(core->pause());
    }
    
    state_ = CpuState::PAUSED;
    return {};
}

Result<void> DualCoreManager::resume() {
    if (state_ != CpuState::PAUSED) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Cores must be paused to resume"));
    }
    
    COMPONENT_LOG_DEBUG("Resuming all CPU cores");
    
    // Resume all cores
    for (auto& [core_id, core] : cores_) {
        RETURN_IF_ERROR(core->resume());
    }
    
    state_ = CpuState::RUNNING;
    return {};
}

Result<void> DualCoreManager::reset() {
    COMPONENT_LOG_INFO("Resetting all CPU cores");
    
    bool was_running = (state_ == CpuState::RUNNING);
    
    // Stop if running
    if (was_running) {
        RETURN_IF_ERROR(stop());
    }
    
    // Reset all cores
    for (auto& [core_id, core] : cores_) {
        RETURN_IF_ERROR(core->reset());
    }
    
    // Reset interrupt controller
    if (interrupt_controller_) {
        RETURN_IF_ERROR(interrupt_controller_->reset());
    }
    
    // Reset timer controller
    if (timer_controller_) {
        RETURN_IF_ERROR(timer_controller_->reset());
    }
    
    // Restart if it was running before
    if (was_running) {
        RETURN_IF_ERROR(start());
    } else {
        state_ = CpuState::HALTED;
    }
    
    COMPONENT_LOG_INFO("All CPU cores reset completed");
    return {};
}

Result<void> DualCoreManager::shutdown() {
    COMPONENT_LOG_INFO("Shutting down dual core manager");
    
    // Stop all cores
    RETURN_IF_ERROR(stop());
    
    // Shutdown components
    if (timer_controller_) {
        timer_controller_->shutdown();
        timer_controller_.reset();
    }
    
    if (interrupt_controller_) {
        interrupt_controller_->shutdown();
        interrupt_controller_.reset();
    }
    
    // Clear cores
    cores_.clear();
    
    memory_controller_ = nullptr;
    state_ = CpuState::STOPPED;
    
    COMPONENT_LOG_INFO("Dual core manager shutdown completed");
    return {};
}

Result<Cycles> DualCoreManager::execute_cycles(Cycles max_cycles) {
    if (state_ != CpuState::RUNNING) {
        return 0;
    }
    
    Cycles total_cycles = 0;
    
    // Calculate cycles per core (simple round-robin scheduling)
    Cycles cycles_per_core = max_cycles / cores_.size();
    Cycles remaining_cycles = max_cycles % cores_.size();
    
    // Execute cycles on each core
    for (auto& [core_id, core] : cores_) {
        if (core->get_state() == CpuState::RUNNING) {
            Cycles core_cycles = cycles_per_core;
            if (remaining_cycles > 0) {
                core_cycles++;
                remaining_cycles--;
            }
            
            auto result = core->execute_cycles(core_cycles);
            if (result) {
                total_cycles += result.value();
            } else {
                COMPONENT_LOG_ERROR("Core {} execution error: {}", 
                                   static_cast<int>(core_id), result.error().to_string());
            }
        }
    }
    
    // Update timer controller
    if (timer_controller_) {
        timer_controller_->update(total_cycles);
    }
    
    // Process interrupts
    if (interrupt_controller_) {
        RETURN_IF_ERROR(process_interrupts());
    }
    
    return total_cycles;
}

CpuState DualCoreManager::get_state() const {
    return state_;
}

Result<CpuCore*> DualCoreManager::get_core(CoreId core_id) {
    auto it = cores_.find(core_id);
    if (it == cores_.end()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid core ID: " + std::to_string(static_cast<int>(core_id))));
    }
    return it->second.get();
}

Result<const CpuCore*> DualCoreManager::get_core(CoreId core_id) const {
    auto it = cores_.find(core_id);
    if (it == cores_.end()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid core ID: " + std::to_string(static_cast<int>(core_id))));
    }
    return it->second.get();
}

Cycles DualCoreManager::get_total_cycles_executed() const {
    Cycles total = 0;
    for (const auto& [core_id, core] : cores_) {
        total += core->get_cycles_executed();
    }
    return total;
}

u64 DualCoreManager::get_total_instructions_executed() const {
    u64 total = 0;
    for (const auto& [core_id, core] : cores_) {
        total += core->get_instructions_executed();
    }
    return total;
}

InterruptController* DualCoreManager::get_interrupt_controller() const {
    return interrupt_controller_.get();
}

TimerController* DualCoreManager::get_timer_controller() const {
    return timer_controller_.get();
}

Result<void> DualCoreManager::send_inter_core_interrupt(CoreId source, CoreId target, u32 data) {
    auto target_core_result = get_core(target);
    if (!target_core_result) {
        return std::unexpected(target_core_result.error());
    }
    
    CpuCore* target_core = target_core_result.value();
    
    // Send software interrupt to target core
    if (interrupt_controller_) {
        RETURN_IF_ERROR(interrupt_controller_->trigger_interrupt(target, InterruptType::SOFTWARE, data));
    }
    
    COMPONENT_LOG_DEBUG("Inter-core interrupt sent from core {} to core {} with data 0x{:08X}",
                       static_cast<int>(source), static_cast<int>(target), data);
    
    return {};
}

Result<void> DualCoreManager::wait_for_event(CoreId core_id, u32 timeout_cycles) {
    auto core_result = get_core(core_id);
    if (!core_result) {
        return std::unexpected(core_result.error());
    }
    
    CpuCore* core = core_result.value();
    
    // Put core into wait state
    // In a real implementation, this would involve power management
    RETURN_IF_ERROR(core->pause());
    
    // Set up timeout if specified
    if (timeout_cycles > 0 && timer_controller_) {
        RETURN_IF_ERROR(timer_controller_->set_timeout(core_id, timeout_cycles));
    }
    
    COMPONENT_LOG_DEBUG("Core {} waiting for event (timeout: {} cycles)",
                       static_cast<int>(core_id), timeout_cycles);
    
    return {};
}

Result<void> DualCoreManager::signal_event(CoreId core_id) {
    auto core_result = get_core(core_id);
    if (!core_result) {
        return std::unexpected(core_result.error());
    }
    
    CpuCore* core = core_result.value();
    
    // Wake up core if it's waiting
    if (core->get_state() == CpuState::PAUSED) {
        RETURN_IF_ERROR(core->resume());
        COMPONENT_LOG_DEBUG("Signaled event to core {}", static_cast<int>(core_id));
    }
    
    return {};
}

void DualCoreManager::dump_all_cores_state() const {
    COMPONENT_LOG_INFO("=== Dual Core Manager State ===");
    COMPONENT_LOG_INFO("Overall state: {}", static_cast<int>(state_));
    COMPONENT_LOG_INFO("Total cycles executed: {}", get_total_cycles_executed());
    COMPONENT_LOG_INFO("Total instructions executed: {}", get_total_instructions_executed());
    
    for (const auto& [core_id, core] : cores_) {
        COMPONENT_LOG_INFO("--- Core {} ---", static_cast<int>(core_id));
        COMPONENT_LOG_INFO("  State: {}", static_cast<int>(core->get_state()));
        COMPONENT_LOG_INFO("  Frequency: {} Hz", core->get_frequency());
        COMPONENT_LOG_INFO("  Cycles executed: {}", core->get_cycles_executed());
        COMPONENT_LOG_INFO("  Instructions executed: {}", core->get_instructions_executed());
        
        // Dump register state for debugging
        core->get_register_file().dump_state();
    }
}

Result<void> DualCoreManager::setup_inter_core_communication() {
    COMPONENT_LOG_DEBUG("Setting up inter-core communication");
    
    // Set up shared memory regions for inter-core communication
    // In the ESP32-P4, cores share most memory but have separate instruction caches
    
    // TODO: Set up mailbox registers or shared memory for communication
    // TODO: Set up semaphores for synchronization
    
    return {};
}

Result<void> DualCoreManager::process_interrupts() {
    if (!interrupt_controller_) {
        return {};
    }
    
    // Process pending interrupts for each core
    for (auto& [core_id, core] : cores_) {
        if (core->get_state() == CpuState::RUNNING) {
            auto pending_interrupts = interrupt_controller_->get_pending_interrupts(core_id);
            
            for (const auto& interrupt : pending_interrupts) {
                // Deliver interrupt to core
                auto& register_file = core->get_register_file();
                
                // Save current PC to MEPC
                register_file.write_csr(CSR_MEPC, register_file.get_pc());
                
                // Set MCAUSE
                u32 mcause = static_cast<u32>(interrupt.type) | (interrupt.external ? 0x80000000 : 0);
                register_file.write_csr(CSR_MCAUSE, mcause);
                
                // Set MTVAL (trap value)
                register_file.write_csr(CSR_MTVAL, interrupt.data);
                
                // Jump to interrupt handler
                u32 mtvec = register_file.get_csr(CSR_MTVEC);
                u32 handler_address = mtvec & ~0x3;  // Clear mode bits
                
                if ((mtvec & 0x1) == 1) {  // Vectored mode
                    handler_address += static_cast<u32>(interrupt.type) * 4;
                }
                
                register_file.set_pc(handler_address);
                
                // Update MSTATUS
                u32 mstatus = register_file.get_csr(CSR_MSTATUS);
                mstatus &= ~0x1888;  // Clear MIE, MPIE, MPP
                mstatus |= ((mstatus & 0x8) << 4);  // MPIE = MIE
                mstatus |= 0x1800;  // MPP = 11 (machine mode)
                register_file.write_csr(CSR_MSTATUS, mstatus);
                
                COMPONENT_LOG_DEBUG("Delivered interrupt {} to core {} (handler: 0x{:08X})",
                                   static_cast<int>(interrupt.type), static_cast<int>(core_id), handler_address);
            }
            
            // Clear processed interrupts
            interrupt_controller_->clear_processed_interrupts(core_id);
        }
    }
    
    return {};
}

}  // namespace m5tab5::emulator