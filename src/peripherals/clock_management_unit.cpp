#include "emulator/peripherals/clock_management_unit.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace m5tab5::emulator {

DECLARE_LOGGER("ClockManagementUnit");

ClockManagementUnit::ClockManagementUnit()
    : initialized_(false),
      current_power_profile_(PowerProfile::BALANCED),
      frequency_scaling_enabled_(true),
      low_power_mode_(false),
      interrupt_controller_(nullptr) {
    
    // Initialize default system configuration
    system_config_.cpu_clock_source = ClockSource::PLL_CPU;
    system_config_.cpu_frequency_mhz = 400;
    system_config_.lp_core_frequency_mhz = 40;
    system_config_.apb_frequency_mhz = 80;
    system_config_.ahb_frequency_mhz = 240;
    system_config_.power_profile = PowerProfile::BALANCED;
    
    // Initialize peripheral domains
    const std::array<std::string, NUM_PERIPHERAL_DOMAINS> domain_names = {
        "APB", "AHB", "UART", "SPI", "I2C", "GPIO", "PWM", "ADC", "DAC", "TIMER",
        "RTC", "WATCHDOG", "CRYPTO", "USB", "SDIO", "CAMERA", "DISPLAY", "AUDIO", "WIFI", "BLUETOOTH"
    };
    
    const std::array<u32, NUM_PERIPHERAL_DOMAINS> default_frequencies = {
        80000000,   // APB - 80MHz
        240000000,  // AHB - 240MHz  
        80000000,   // UART - 80MHz
        80000000,   // SPI - 80MHz
        80000000,   // I2C - 80MHz
        80000000,   // GPIO - 80MHz
        80000000,   // PWM - 80MHz
        20000000,   // ADC - 20MHz
        80000000,   // DAC - 80MHz
        80000000,   // TIMER - 80MHz
        32768,      // RTC - 32.768kHz
        80000000,   // WATCHDOG - 80MHz
        160000000,  // CRYPTO - 160MHz
        80000000,   // USB - 80MHz  
        80000000,   // SDIO - 80MHz
        120000000,  // CAMERA - 120MHz
        240000000,  // DISPLAY - 240MHz
        160000000,  // AUDIO - 160MHz
        160000000,  // WIFI - 160MHz
        80000000    // BLUETOOTH - 80MHz
    };
    
    for (size_t i = 0; i < NUM_PERIPHERAL_DOMAINS; ++i) {
        peripheral_domains_[i].name = domain_names[i];
        peripheral_domains_[i].frequency_hz = default_frequencies[i];
        peripheral_domains_[i].target_frequency_hz = default_frequencies[i];
        peripheral_domains_[i].source = ClockSource::PLL_CPU;
        peripheral_domains_[i].divider = 1;
        peripheral_domains_[i].enabled = true;
        peripheral_domains_[i].stable = true;
    }
    
    // Initialize CPU domains
    cpu_domains_[0].name = "HP_CORE_0";
    cpu_domains_[0].frequency_hz = 400000000;
    cpu_domains_[1].name = "HP_CORE_1"; 
    cpu_domains_[1].frequency_hz = 400000000;
    cpu_domains_[2].name = "LP_CORE";
    cpu_domains_[2].frequency_hz = 40000000;
    
    for (auto& domain : cpu_domains_) {
        domain.source = ClockSource::PLL_CPU;
        domain.enabled = true;
        domain.stable = true;
    }
    
    // Initialize PLL states
    pll_states_[ClockSource::PLL_CPU] = {};
    pll_states_[ClockSource::PLL_PERIPHERAL] = {};
    pll_states_[ClockSource::APLL] = {};
    
    auto& cpu_pll = pll_states_[ClockSource::PLL_CPU];
    cpu_pll.config.reference_source = ClockSource::XTAL_40MHZ;
    cpu_pll.config.reference_frequency_mhz = 40;
    cpu_pll.config.multiplier = 10;
    cpu_pll.config.output_frequency_mhz = 400;
    cpu_pll.enabled = true;
    cpu_pll.locked = true;
    cpu_pll.actual_frequency_hz = 400000000;
    
    // Initialize monitoring configuration
    monitoring_config_.frequency_monitoring_enabled = true;
    monitoring_config_.frequency_tolerance_percent = 1.0f;
    monitoring_config_.jitter_threshold_ps = 100;
    
    COMPONENT_LOG_DEBUG("Clock Management Unit created");
}

ClockManagementUnit::~ClockManagementUnit() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Clock Management Unit destroyed");
}

Result<void> ClockManagementUnit::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Clock Management Unit already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing Clock Management Unit");
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize registers with default values
    registers_ = {};
    registers_.system_control = 0x00000001; // Enable system
    registers_.pll_status = 0x00000007;     // All PLLs locked
    registers_.clock_gate = 0xFFFFFFFF;     // All clocks enabled
    registers_.power_control = 0x00000001;  // Normal power mode
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    last_calibration_ = last_update_;
    system_start_time_ = last_update_;
    
    // Initialize frequency monitors
    for (auto& [source, pll] : pll_states_) {
        frequency_monitors_[source] = {};
        frequency_monitors_[source].target_frequency_hz = pll.actual_frequency_hz;
        frequency_monitors_[source].enabled = monitoring_config_.frequency_monitoring_enabled;
    }
    
    // Clear statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Clock Management Unit initialized successfully");
    
    return {};
}

Result<void> ClockManagementUnit::shutdown() {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down Clock Management Unit");
    
    // Disable all PLLs
    for (auto& [source, pll] : pll_states_) {
        pll.enabled = false;
        pll.locked = false;
    }
    
    // Disable all peripheral clocks
    for (auto& domain : peripheral_domains_) {
        domain.enabled = false;
    }
    
    // Update registers
    registers_.system_control = 0x00000000;
    registers_.pll_status = 0x00000000;
    registers_.clock_gate = 0x00000000;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Clock Management Unit shutdown completed");
    return {};
}

Result<void> ClockManagementUnit::configure_system_clocks(const ClockConfiguration& config) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    system_config_ = config;
    current_power_profile_ = config.power_profile;
    frequency_scaling_enabled_ = config.enable_frequency_scaling;
    
    // Update CPU frequencies
    for (size_t i = 0; i < 2; ++i) { // HP cores
        cpu_domains_[i].target_frequency_hz = config.cpu_frequency_mhz * 1000000;
        cpu_domains_[i].frequency_hz = config.cpu_frequency_mhz * 1000000;
    }
    cpu_domains_[2].target_frequency_hz = config.lp_core_frequency_mhz * 1000000; // LP core
    cpu_domains_[2].frequency_hz = config.lp_core_frequency_mhz * 1000000;
    
    // Update peripheral frequencies
    peripheral_domains_[0].target_frequency_hz = config.apb_frequency_mhz * 1000000; // APB
    peripheral_domains_[0].frequency_hz = config.apb_frequency_mhz * 1000000;
    peripheral_domains_[1].target_frequency_hz = config.ahb_frequency_mhz * 1000000; // AHB
    peripheral_domains_[1].frequency_hz = config.ahb_frequency_mhz * 1000000;
    
    // Update PLL configuration
    auto& cpu_pll = pll_states_[ClockSource::PLL_CPU];
    cpu_pll.config.output_frequency_mhz = config.cpu_pll_frequency_mhz;
    cpu_pll.actual_frequency_hz = config.cpu_pll_frequency_mhz * 1000000;
    
    // Apply power profile settings
    apply_power_profile_settings();
    
    // Propagate changes to all clock domains
    propagate_clock_changes();
    
    COMPONENT_LOG_INFO("System clocks configured: CPU={}MHz, LP={}MHz, APB={}MHz, AHB={}MHz",
                      config.cpu_frequency_mhz, config.lp_core_frequency_mhz,
                      config.apb_frequency_mhz, config.ahb_frequency_mhz);
    
    return {};
}

Result<void> ClockManagementUnit::set_cpu_frequency(CPUCore core, u32 frequency_mhz) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    if (frequency_mhz < MIN_CPU_FREQUENCY / 1000000 || frequency_mhz > MAX_CPU_FREQUENCY / 1000000) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "CPU frequency out of range"));
    }
    
    size_t core_index = static_cast<size_t>(core);
    if (core_index >= cpu_domains_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid CPU core"));
    }
    
    cpu_domains_[core_index].target_frequency_hz = frequency_mhz * 1000000;
    cpu_domains_[core_index].frequency_hz = frequency_mhz * 1000000;
    
    statistics_.frequency_changes++;
    
    COMPONENT_LOG_DEBUG("CPU core {} frequency set to {} MHz", core_index, frequency_mhz);
    return {};
}

Result<u32> ClockManagementUnit::get_cpu_frequency(CPUCore core) const {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    size_t core_index = static_cast<size_t>(core);
    if (core_index >= cpu_domains_.size()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid CPU core"));
    }
    
    return cpu_domains_[core_index].frequency_hz / 1000000; // Convert to MHz
}

Result<void> ClockManagementUnit::set_peripheral_clock(PeripheralClockDomain domain, u32 frequency_mhz) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    size_t domain_index = static_cast<size_t>(domain);
    if (domain_index >= NUM_PERIPHERAL_DOMAINS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid peripheral domain"));
    }
    
    u32 target_frequency_hz = frequency_mhz * 1000000;
    
    // Special case for RTC which should be 32.768kHz
    if (domain == PeripheralClockDomain::RTC) {
        target_frequency_hz = XTAL_32KHZ_FREQUENCY;
    }
    
    peripheral_domains_[domain_index].target_frequency_hz = target_frequency_hz;
    peripheral_domains_[domain_index].frequency_hz = target_frequency_hz;
    
    statistics_.frequency_changes++;
    
    COMPONENT_LOG_DEBUG("Peripheral {} frequency set to {} MHz", 
                       peripheral_domains_[domain_index].name, frequency_mhz);
    
    return {};
}

Result<void> ClockManagementUnit::configure_cpu_pll(const PLLConfiguration& config) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    auto& cpu_pll = pll_states_[ClockSource::PLL_CPU];
    cpu_pll.config = config;
    cpu_pll.enabled = true;
    cpu_pll.locked = false; // Will need to re-lock
    cpu_pll.lock_time_remaining_us = config.lock_time_us;
    
    // Calculate actual output frequency
    calculate_pll_frequency(cpu_pll);
    
    COMPONENT_LOG_INFO("CPU PLL configured: ref={}MHz, mult={}, div={}, output={}MHz",
                      config.reference_frequency_mhz, config.multiplier, 
                      config.divider, cpu_pll.actual_frequency_hz / 1000000);
    
    return {};
}

Result<bool> ClockManagementUnit::is_pll_locked(ClockSource pll) const {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    auto it = pll_states_.find(pll);
    if (it == pll_states_.end()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid PLL source"));
    }
    
    return it->second.locked;
}

Result<void> ClockManagementUnit::enable_peripheral_clock(PeripheralClockDomain domain, bool enable) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    size_t domain_index = static_cast<size_t>(domain);
    if (domain_index >= NUM_PERIPHERAL_DOMAINS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid peripheral domain"));
    }
    
    peripheral_domains_[domain_index].enabled = enable;
    
    // Update clock gate register
    if (enable) {
        registers_.clock_gate |= (1UL << domain_index);
    } else {
        registers_.clock_gate &= ~(1UL << domain_index);
    }
    
    if (enable) {
        statistics_.clock_gating_events++;
    }
    
    COMPONENT_LOG_DEBUG("Peripheral {} clock {}", 
                       peripheral_domains_[domain_index].name,
                       enable ? "enabled" : "disabled");
    
    return {};
}

Result<void> ClockManagementUnit::set_power_profile(PowerProfile profile) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    current_power_profile_ = profile;
    apply_power_profile_settings();
    
    COMPONENT_LOG_INFO("Power profile changed to {}", static_cast<u8>(profile));
    return {};
}

Result<float> ClockManagementUnit::measure_frequency(ClockSource source) {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Clock Management Unit not initialized"));
    }
    
    auto it = pll_states_.find(source);
    if (it == pll_states_.end()) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid clock source"));
    }
    
    // Simulate measurement with small error
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> error_dist(0.0f, 0.001f); // ±0.1% error
    
    float measured_frequency = it->second.actual_frequency_hz * (1.0f + error_dist(gen));
    
    // Update frequency monitor
    auto& monitor = frequency_monitors_[source];
    monitor.measured_frequency_hz = static_cast<u32>(measured_frequency);
    monitor.error_percent = std::abs(measured_frequency - monitor.target_frequency_hz) / 
                           monitor.target_frequency_hz * 100.0f;
    monitor.in_spec = (monitor.error_percent <= monitor.tolerance_percent);
    monitor.last_measurement = std::chrono::steady_clock::now();
    
    return measured_frequency / 1000000.0f; // Return in MHz
}

void ClockManagementUnit::update() {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);
    last_update_ = now;
    
    // Update system clocks
    update_system_clocks();
    
    // Update PLL states
    update_pll_states();
    
    // Update frequency scaling
    if (frequency_scaling_enabled_) {
        update_frequency_scaling();
    }
    
    // Update frequency monitors (every 100ms)
    static auto last_monitor_update = now;
    auto monitor_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_monitor_update);
    if (monitor_elapsed.count() >= 100) {
        update_frequency_monitors();
        last_monitor_update = now;
    }
    
    // Apply clock gating
    apply_clock_gating();
    
    // Update statistics
    statistics_.cpu_cycles += elapsed.count() * (cpu_domains_[0].frequency_hz / 1000000); // Rough estimate
    
    // Auto-calibration (every 60 seconds)
    auto calibration_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_calibration_);
    if (calibration_elapsed.count() >= 60) {
        auto_calibrate_rc_oscillators();
        last_calibration_ = now;
    }
}

void ClockManagementUnit::update_system_clocks() {
    // Update all clock domains with their cycle counts
    auto now = std::chrono::steady_clock::now();
    
    for (auto& domain : cpu_domains_) {
        if (domain.enabled && domain.last_update != std::chrono::steady_clock::time_point{}) {
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - domain.last_update);
            u64 cycles = (elapsed.count() * domain.frequency_hz) / 1000000000ULL;
            domain.cycle_count += cycles;
        }
        domain.last_update = now;
    }
    
    for (auto& domain : peripheral_domains_) {
        if (domain.enabled && domain.last_update != std::chrono::steady_clock::time_point{}) {
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - domain.last_update);
            u64 cycles = (elapsed.count() * domain.frequency_hz) / 1000000000ULL;
            domain.cycle_count += cycles;
            statistics_.peripheral_accesses += cycles / 1000; // Rough estimate
        }
        domain.last_update = now;
    }
}

void ClockManagementUnit::update_pll_states() {
    for (auto& [source, pll] : pll_states_) {
        if (pll.enabled && !pll.locked) {
            // Simulate PLL locking process
            simulate_pll_locking(pll);
        }
        
        if (pll.enabled) {
            // Monitor PLL stability
            monitor_pll_stability(pll);
        }
    }
    
    // Update PLL status register
    registers_.pll_status = 0x00000000;
    if (pll_states_[ClockSource::PLL_CPU].locked) {
        registers_.pll_status |= 0x01;
    }
    if (pll_states_[ClockSource::PLL_PERIPHERAL].locked) {
        registers_.pll_status |= 0x02;
    }
    if (pll_states_[ClockSource::APLL].locked) {
        registers_.pll_status |= 0x04;
    }
}

void ClockManagementUnit::simulate_pll_locking(PLLState& pll) {
    if (pll.lock_time_remaining_us > 0.0f) {
        pll.lock_time_remaining_us -= 1.0f; // 1ms update interval approximation
        
        if (pll.lock_time_remaining_us <= 0.0f) {
            pll.locked = true;
            statistics_.pll_lock_events++;
            trigger_interrupt(CMUInterruptType::PLL_LOCK_ACQUIRED);
            
            COMPONENT_LOG_DEBUG("PLL locked successfully");
        }
    }
}

void ClockManagementUnit::calculate_pll_frequency(PLLState& pll) {
    // Calculate PLL output frequency: Fout = Fref * (N / M) / P
    u32 reference_hz = pll.config.reference_frequency_mhz * 1000000;
    pll.actual_frequency_hz = (reference_hz * pll.config.multiplier) / 
                             (pll.config.divider * pll.config.post_divider);
    
    // Calculate frequency error
    u32 target_hz = pll.config.output_frequency_mhz * 1000000;
    pll.frequency_error_ppm = calculate_frequency_error_ppm(pll.actual_frequency_hz, target_hz);
}

void ClockManagementUnit::monitor_pll_stability(PLLState& pll) {
    // Simulate PLL jitter and stability
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> jitter_dist(0.0f, 10.0f); // ±10ps typical jitter
    
    pll.jitter_ps = std::abs(jitter_dist(gen));
    
    // Check for lock loss (very rare)
    static std::uniform_int_distribution<int> lock_loss_dist(0, 1000000);
    if (lock_loss_dist(gen) == 0) { // 1 in 1M chance per update
        pll.locked = false;
        statistics_.pll_lock_events++;
        trigger_interrupt(CMUInterruptType::PLL_LOCK_LOST);
        handle_pll_lock_lost(ClockSource::PLL_CPU); // Assume CPU PLL for simplicity
    }
}

void ClockManagementUnit::handle_pll_lock_lost(ClockSource pll) {
    COMPONENT_LOG_ERROR("PLL lock lost, switching to backup clock source");
    
    // Switch affected domains to backup clock source
    for (auto& domain : peripheral_domains_) {
        if (domain.source == pll) {
            switch_to_backup_clock(static_cast<PeripheralClockDomain>(
                &domain - &peripheral_domains_[0]));
        }
    }
    
    statistics_.pll_lock_events++;
}

void ClockManagementUnit::switch_to_backup_clock(PeripheralClockDomain domain) {
    size_t domain_index = static_cast<size_t>(domain);
    if (domain_index < NUM_PERIPHERAL_DOMAINS) {
        // Switch to RC oscillator as backup
        peripheral_domains_[domain_index].source = ClockSource::RC_FAST;
        peripheral_domains_[domain_index].frequency_hz = RC_FAST_FREQUENCY;
        
        COMPONENT_LOG_WARN("Peripheral {} switched to backup clock source",
                          peripheral_domains_[domain_index].name);
    }
}

void ClockManagementUnit::update_frequency_scaling() {
    // Simple workload-based scaling simulation
    static auto last_scaling_update = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_scaling_update);
    
    if (elapsed.count() >= 5) { // Update every 5 seconds
        workload_based_scaling();
        last_scaling_update = now;
    }
}

void ClockManagementUnit::workload_based_scaling() {
    // Simulate CPU utilization
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> utilization_dist(0.1f, 0.9f);
    
    float cpu_utilization = utilization_dist(gen);
    statistics_.average_cpu_utilization = 
        (statistics_.average_cpu_utilization + cpu_utilization) / 2.0f;
    
    // Adjust frequency based on utilization and power profile
    u32 target_frequency_mhz = 0;
    
    switch (current_power_profile_) {
        case PowerProfile::MAXIMUM_PERFORMANCE:
            target_frequency_mhz = 400; // Always max frequency
            break;
        case PowerProfile::BALANCED:
            target_frequency_mhz = static_cast<u32>(200 + (cpu_utilization * 200)); // 200-400MHz
            break;
        case PowerProfile::POWER_EFFICIENT:
            target_frequency_mhz = static_cast<u32>(100 + (cpu_utilization * 200)); // 100-300MHz
            break;
        case PowerProfile::ULTRA_LOW_POWER:
            target_frequency_mhz = static_cast<u32>(50 + (cpu_utilization * 100)); // 50-150MHz
            break;
    }
    
    // Apply frequency change to CPU cores
    for (size_t i = 0; i < 2; ++i) { // HP cores only
        if (cpu_domains_[i].frequency_hz != target_frequency_mhz * 1000000) {
            cpu_domains_[i].frequency_hz = target_frequency_mhz * 1000000;
            cpu_domains_[i].target_frequency_hz = target_frequency_mhz * 1000000;
            statistics_.frequency_changes++;
        }
    }
}

void ClockManagementUnit::apply_power_profile_settings() {
    switch (current_power_profile_) {
        case PowerProfile::MAXIMUM_PERFORMANCE:
            // Disable clock gating for maximum performance
            for (auto& domain : peripheral_domains_) {
                domain.gating_mode = ClockGatingMode::ALWAYS_ON;
            }
            break;
            
        case PowerProfile::BALANCED:
            // Enable automatic clock gating
            for (auto& domain : peripheral_domains_) {
                domain.gating_mode = ClockGatingMode::AUTO_GATING;
            }
            break;
            
        case PowerProfile::POWER_EFFICIENT:
        case PowerProfile::ULTRA_LOW_POWER:
            // Aggressive clock gating
            for (auto& domain : peripheral_domains_) {
                if (domain.name != "RTC" && domain.name != "WATCHDOG") {
                    domain.gating_mode = ClockGatingMode::SOFTWARE_GATING;
                }
            }
            break;
    }
}

void ClockManagementUnit::apply_clock_gating() {
    for (auto& domain : peripheral_domains_) {
        bool should_gate = false;
        
        switch (domain.gating_mode) {
            case ClockGatingMode::ALWAYS_ON:
                should_gate = false;
                break;
            case ClockGatingMode::AUTO_GATING:
                // Simple idle detection simulation
                should_gate = (domain.cycle_count % 1000000 == 0); // Gate when idle
                break;
            case ClockGatingMode::SOFTWARE_GATING:
                should_gate = !domain.enabled;
                break;
            case ClockGatingMode::FORCE_OFF:
                should_gate = true;
                break;
        }
        
        if (should_gate && domain.enabled) {
            statistics_.clock_gating_events++;
        }
    }
}

void ClockManagementUnit::update_frequency_monitors() {
    for (auto& [source, monitor] : frequency_monitors_) {
        if (monitor.enabled) {
            // Measure frequency
            auto result = measure_frequency(source);
            if (result) {
                float measured_mhz = result.value();
                
                // Check if frequency is in specification
                if (!monitor.in_spec) {
                    trigger_interrupt(CMUInterruptType::FREQUENCY_ERROR);
                    statistics_.frequency_changes++; // Count as an error event
                }
            }
        }
    }
}

void ClockManagementUnit::auto_calibrate_rc_oscillators() {
    // Simulate RC oscillator calibration against crystal reference
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> calibration_error(0.0f, 0.002f); // ±0.2% error
    
    // Calibrate RC_FAST against XTAL_40MHZ
    float calibration_factor = 1.0f + calibration_error(gen);
    
    // Update frequency accuracy
    statistics_.frequency_accuracy_ppm = std::abs(calibration_factor - 1.0f) * 1000000.0f;
    
    trigger_interrupt(CMUInterruptType::CALIBRATION_COMPLETE);
    
    COMPONENT_LOG_DEBUG("RC oscillator calibration completed, accuracy: {:.1f} ppm", 
                       statistics_.frequency_accuracy_ppm);
}

void ClockManagementUnit::propagate_clock_changes() {
    // Recalculate all derived clocks
    for (auto& domain : peripheral_domains_) {
        // Calculate optimal divider for target frequency
        u32 source_frequency_hz = 0;
        
        switch (domain.source) {
            case ClockSource::PLL_CPU:
                source_frequency_hz = pll_states_[ClockSource::PLL_CPU].actual_frequency_hz;
                break;
            case ClockSource::XTAL_40MHZ:
                source_frequency_hz = XTAL_40MHZ_FREQUENCY;
                break;
            case ClockSource::XTAL_32KHZ:
                source_frequency_hz = XTAL_32KHZ_FREQUENCY;
                break;
            default:
                source_frequency_hz = RC_FAST_FREQUENCY;
                break;
        }
        
        domain.divider = calculate_optimal_divider(source_frequency_hz, domain.target_frequency_hz);
        domain.frequency_hz = calculate_divided_frequency(source_frequency_hz, domain.divider);
    }
}

u8 ClockManagementUnit::calculate_optimal_divider(u32 source_frequency_hz, u32 target_frequency_hz) {
    if (target_frequency_hz == 0) return 255; // Maximum divider
    
    u8 divider = static_cast<u8>(source_frequency_hz / target_frequency_hz);
    return std::max(static_cast<u8>(1), std::min(divider, static_cast<u8>(255)));
}

u32 ClockManagementUnit::calculate_divided_frequency(u32 source_frequency_hz, u8 divider) {
    return divider == 0 ? 0 : source_frequency_hz / divider;
}

float ClockManagementUnit::calculate_frequency_error_ppm(u32 actual_hz, u32 target_hz) {
    if (target_hz == 0) return 0.0f;
    return std::abs(static_cast<float>(actual_hz) - static_cast<float>(target_hz)) / 
           static_cast<float>(target_hz) * 1000000.0f;
}

void ClockManagementUnit::trigger_interrupt(CMUInterruptType interrupt_type) {
    if (interrupt_controller_) {
        // Map CMU interrupts to system interrupts
        interrupt_controller_->trigger_interrupt(InterruptType::CLOCK);
    }
    
    // Update interrupt status register
    registers_.interrupt_status |= static_cast<u8>(interrupt_type);
}

void ClockManagementUnit::update_cmu_registers() {
    // Update system control based on current state
    registers_.system_control = 0x00000001; // System enabled
    if (frequency_scaling_enabled_) {
        registers_.system_control |= 0x00000002;
    }
    if (low_power_mode_) {
        registers_.system_control |= 0x00000004;
    }
    
    // Update power control register
    registers_.power_control = static_cast<u32>(current_power_profile_);
    
    // Update calibration register
    registers_.calibration = static_cast<u32>(statistics_.frequency_accuracy_ppm);
}

void ClockManagementUnit::clear_statistics() {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    statistics_ = {};
}

void ClockManagementUnit::dump_status() const {
    std::lock_guard<std::mutex> lock(cmu_mutex_);
    
    COMPONENT_LOG_INFO("=== Clock Management Unit Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Power profile: {}", static_cast<u8>(current_power_profile_));
        COMPONENT_LOG_INFO("Frequency scaling enabled: {}", frequency_scaling_enabled_);
        COMPONENT_LOG_INFO("Low power mode: {}", low_power_mode_);
        
        COMPONENT_LOG_INFO("CPU Cores:");
        for (size_t i = 0; i < cpu_domains_.size(); ++i) {
            const auto& core = cpu_domains_[i];
            COMPONENT_LOG_INFO("  {}: {} MHz, enabled={}, cycles={}",
                              core.name, core.frequency_hz / 1000000, core.enabled, core.cycle_count);
        }
        
        COMPONENT_LOG_INFO("PLL Status:");
        for (const auto& [source, pll] : pll_states_) {
            COMPONENT_LOG_INFO("  Source {}: {} MHz, locked={}, enabled={}",
                              static_cast<u8>(source), pll.actual_frequency_hz / 1000000,
                              pll.locked, pll.enabled);
        }
        
        COMPONENT_LOG_INFO("Key Peripheral Clocks:");
        for (size_t i = 0; i < 10 && i < NUM_PERIPHERAL_DOMAINS; ++i) {
            const auto& domain = peripheral_domains_[i];
            COMPONENT_LOG_INFO("  {}: {} MHz, enabled={}, gating={}",
                              domain.name, domain.frequency_hz / 1000000, 
                              domain.enabled, static_cast<u8>(domain.gating_mode));
        }
        
        COMPONENT_LOG_INFO("Frequency Monitoring:");
        for (const auto& [source, monitor] : frequency_monitors_) {
            if (monitor.enabled) {
                COMPONENT_LOG_INFO("  Source {}: error={:.2f}%, in_spec={}",
                                  static_cast<u8>(source), monitor.error_percent, monitor.in_spec);
            }
        }
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  CPU cycles: {}", statistics_.cpu_cycles);
        COMPONENT_LOG_INFO("  Peripheral accesses: {}", statistics_.peripheral_accesses);
        COMPONENT_LOG_INFO("  PLL lock events: {}", statistics_.pll_lock_events);
        COMPONENT_LOG_INFO("  Clock gating events: {}", statistics_.clock_gating_events);
        COMPONENT_LOG_INFO("  Frequency changes: {}", statistics_.frequency_changes);
        COMPONENT_LOG_INFO("  Average CPU utilization: {:.1f}%", statistics_.average_cpu_utilization);
        COMPONENT_LOG_INFO("  Power consumption: {:.1f} mW", statistics_.power_consumption_mw);
        COMPONENT_LOG_INFO("  Frequency accuracy: {:.1f} ppm", statistics_.frequency_accuracy_ppm);
    }
}

}  // namespace m5tab5::emulator