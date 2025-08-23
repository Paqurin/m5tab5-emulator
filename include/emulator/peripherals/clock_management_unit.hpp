#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <atomic>
#include <array>
#include <unordered_map>

namespace m5tab5::emulator {

enum class ClockSource : u8 {
    XTAL_40MHZ = 0,              // External 40MHz crystal
    RC_FAST = 1,                 // Internal RC fast clock (~17.5MHz)
    RC_SLOW = 2,                 // Internal RC slow clock (~150kHz)
    XTAL_32KHZ = 3,              // External 32.768kHz RTC crystal
    PLL_CPU = 4,                 // CPU PLL output
    PLL_PERIPHERAL = 5,          // Peripheral PLL output
    APLL = 6,                    // Audio PLL output
    USB_PHY = 7                  // USB PHY reference clock
};

enum class CPUCore : u8 {
    HP_CORE_0 = 0,               // High Performance Core 0
    HP_CORE_1 = 1,               // High Performance Core 1  
    LP_CORE = 2                  // Low Power Core
};

enum class PeripheralClockDomain : u8 {
    APB = 0,                     // Advanced Peripheral Bus
    AHB = 1,                     // Advanced High-performance Bus
    UART = 2,                    // UART controllers
    SPI = 3,                     // SPI controllers
    I2C = 4,                     // I2C controllers
    GPIO = 5,                    // GPIO controller
    PWM = 6,                     // PWM controllers
    ADC = 7,                     // ADC controllers
    DAC = 8,                     // DAC controllers
    TIMER = 9,                   // Timer controllers
    RTC = 10,                    // Real-Time Clock
    WATCHDOG = 11,               // Watchdog timer
    CRYPTO = 12,                 // Cryptographic accelerator
    USB = 13,                    // USB controller
    SDIO = 14,                   // SDIO controller
    CAMERA = 15,                 // Camera interface
    DISPLAY = 16,                // Display controller
    AUDIO = 17,                  // Audio codec
    WIFI = 18,                   // WiFi subsystem
    BLUETOOTH = 19               // Bluetooth subsystem
};

enum class ClockGatingMode : u8 {
    ALWAYS_ON = 0,               // Clock always enabled
    AUTO_GATING = 1,             // Automatic clock gating when idle
    SOFTWARE_GATING = 2,         // Software controlled gating
    FORCE_OFF = 3                // Clock force disabled
};

enum class PowerProfile : u8 {
    MAXIMUM_PERFORMANCE = 0,     // Maximum performance, no power saving
    BALANCED = 1,                // Balanced performance and power
    POWER_EFFICIENT = 2,         // Optimize for power efficiency
    ULTRA_LOW_POWER = 3          // Ultra low power mode
};

enum class PLLOutput : u8 {
    CPU_PLL_400MHZ = 0,          // CPU PLL main output (400MHz)
    CPU_PLL_200MHZ = 1,          // CPU PLL divided by 2 (200MHz)
    CPU_PLL_100MHZ = 2,          // CPU PLL divided by 4 (100MHz)
    PERIPHERAL_PLL_240MHZ = 3,   // Peripheral PLL main output (240MHz)
    PERIPHERAL_PLL_120MHZ = 4,   // Peripheral PLL divided by 2 (120MHz)
    PERIPHERAL_PLL_60MHZ = 5,    // Peripheral PLL divided by 4 (60MHz)
    AUDIO_PLL_22MHZ = 6,         // Audio PLL for 22.05kHz family
    AUDIO_PLL_24MHZ = 7          // Audio PLL for 24kHz family
};

enum class CMUInterruptType : u8 {
    PLL_LOCK_LOST = 0x01,        // PLL lost lock
    PLL_LOCK_ACQUIRED = 0x02,    // PLL acquired lock
    CLOCK_FAILURE = 0x04,        // Clock source failure
    FREQUENCY_ERROR = 0x08,      // Frequency monitoring error
    THERMAL_THROTTLE = 0x10,     // Thermal throttling event
    POWER_MODE_CHANGE = 0x20,    // Power mode transition
    CALIBRATION_COMPLETE = 0x40, // Clock calibration complete
    JITTER_ALARM = 0x80          // Clock jitter alarm
};

struct ClockConfiguration {
    ClockSource cpu_clock_source = ClockSource::PLL_CPU;
    u32 cpu_frequency_mhz = 400;         // CPU frequency (MHz)
    u32 lp_core_frequency_mhz = 40;      // Low-power core frequency (MHz)
    u32 apb_frequency_mhz = 80;          // APB bus frequency (MHz)  
    u32 ahb_frequency_mhz = 240;         // AHB bus frequency (MHz)
    
    // PLL configuration
    u32 cpu_pll_frequency_mhz = 400;     // CPU PLL output frequency
    u32 peripheral_pll_frequency_mhz = 240; // Peripheral PLL frequency
    u32 audio_pll_frequency_mhz = 22;    // Audio PLL frequency
    
    // Clock dividers
    u8 cpu_divider = 1;                  // CPU clock divider
    u8 apb_divider = 5;                  // APB clock divider (from CPU PLL)
    u8 ahb_divider = 2;                  // AHB clock divider (from PLL)
    
    PowerProfile power_profile = PowerProfile::BALANCED;
    bool enable_clock_gating = true;     // Enable automatic clock gating
    bool enable_frequency_scaling = true; // Enable dynamic frequency scaling
};

struct PLLConfiguration {
    ClockSource reference_source = ClockSource::XTAL_40MHZ;
    u32 reference_frequency_mhz = 40;    // Reference clock frequency
    u16 multiplier = 10;                 // PLL multiplier (N)
    u8 divider = 1;                      // PLL divider (M)
    u8 post_divider = 1;                 // Post divider (P)
    u32 output_frequency_mhz = 400;      // Calculated output frequency
    bool lock_detector_enabled = true;   // Enable PLL lock detection
    u8 lock_time_us = 100;               // PLL lock time (microseconds)
};

struct ClockMonitoring {
    bool frequency_monitoring_enabled = true;
    float frequency_tolerance_percent = 1.0f; // ±1% tolerance
    u32 jitter_threshold_ps = 100;       // Jitter threshold (picoseconds)
    bool jitter_monitoring_enabled = true;
    u32 monitoring_period_ms = 1000;     // Monitoring update period
};

struct ClockStatistics {
    u64 cpu_cycles = 0;                  // Total CPU cycles executed
    u64 peripheral_accesses = 0;         // Peripheral clock cycles
    u64 pll_lock_events = 0;             // PLL lock/unlock events
    u64 clock_gating_events = 0;         // Clock gating on/off events
    u64 frequency_changes = 0;           // Frequency change events
    u64 thermal_throttle_events = 0;     // Thermal throttling events
    float average_cpu_utilization = 0.0f; // Average CPU utilization (%)
    float power_consumption_mw = 0.0f;   // Clock tree power consumption
    double jitter_measurement_ps = 0.0;  // Current jitter measurement
    float frequency_accuracy_ppm = 0.0f; // Frequency accuracy (ppm)
};

class ClockManagementUnit {
public:
    static constexpr u32 XTAL_40MHZ_FREQUENCY = 40000000;    // 40MHz external crystal
    static constexpr u32 XTAL_32KHZ_FREQUENCY = 32768;      // 32.768kHz RTC crystal
    static constexpr u32 RC_FAST_FREQUENCY = 17500000;      // ~17.5MHz RC oscillator
    static constexpr u32 RC_SLOW_FREQUENCY = 150000;        // ~150kHz RC oscillator
    static constexpr u32 MAX_CPU_FREQUENCY = 400000000;     // Maximum CPU frequency (400MHz)
    static constexpr u32 MIN_CPU_FREQUENCY = 10000000;      // Minimum CPU frequency (10MHz)
    static constexpr size_t NUM_PERIPHERAL_DOMAINS = 20;
    
    ClockManagementUnit();
    ~ClockManagementUnit();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    // Clock configuration
    Result<void> configure_system_clocks(const ClockConfiguration& config);
    Result<void> set_cpu_frequency(CPUCore core, u32 frequency_mhz);
    Result<u32> get_cpu_frequency(CPUCore core) const;
    Result<void> set_peripheral_clock(PeripheralClockDomain domain, u32 frequency_mhz);
    Result<u32> get_peripheral_clock(PeripheralClockDomain domain) const;
    
    // PLL management
    Result<void> configure_cpu_pll(const PLLConfiguration& config);
    Result<void> configure_peripheral_pll(const PLLConfiguration& config);
    Result<void> configure_audio_pll(const PLLConfiguration& config);
    Result<bool> is_pll_locked(ClockSource pll) const;
    Result<void> reset_pll(ClockSource pll);
    Result<u32> get_pll_frequency(ClockSource pll) const;
    
    // Clock gating control
    Result<void> set_clock_gating(PeripheralClockDomain domain, ClockGatingMode mode);
    Result<ClockGatingMode> get_clock_gating(PeripheralClockDomain domain) const;
    Result<void> enable_peripheral_clock(PeripheralClockDomain domain, bool enable);
    Result<bool> is_peripheral_clock_enabled(PeripheralClockDomain domain) const;
    
    // Dynamic frequency scaling
    Result<void> enable_frequency_scaling(bool enable);
    Result<void> set_power_profile(PowerProfile profile);
    Result<PowerProfile> get_power_profile() const;
    Result<void> throttle_cpu_frequency(u8 throttle_percent);
    Result<void> boost_cpu_frequency(u8 boost_percent);
    
    // Clock source management
    Result<void> set_clock_source(PeripheralClockDomain domain, ClockSource source);
    Result<ClockSource> get_clock_source(PeripheralClockDomain domain) const;
    Result<void> calibrate_rc_oscillator();
    Result<bool> is_clock_source_stable(ClockSource source) const;
    
    // Clock monitoring and measurement
    Result<void> configure_monitoring(const ClockMonitoring& config);
    Result<float> measure_frequency(ClockSource source);
    Result<float> measure_jitter(ClockSource source);
    Result<void> enable_frequency_monitor(ClockSource source, bool enable);
    Result<bool> detect_clock_failure(ClockSource source);
    
    // Advanced features
    Result<void> enable_spread_spectrum(ClockSource pll, bool enable, float modulation_percent = 0.5f);
    Result<void> configure_clock_output(u8 output_pin, ClockSource source, u8 divider);
    Result<void> synchronize_clocks(); // Synchronize all clock domains
    Result<void> emergency_clock_recovery(); // Recovery from clock failure
    
    // Power management integration
    Result<void> prepare_for_sleep(); // Prepare clocks for sleep mode
    Result<void> restore_from_sleep(); // Restore clocks after wake
    Result<void> set_low_power_mode(bool enable);
    Result<float> get_clock_power_consumption() const;
    
    // Interrupt handling
    Result<void> enable_interrupt(CMUInterruptType interrupt_type);
    Result<void> disable_interrupt(CMUInterruptType interrupt_type);
    Result<u8> get_interrupt_status();
    Result<void> clear_interrupt(CMUInterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    const ClockStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct CMURegisters {
        u32 system_control = 0x00000001;     // 0x00 - System clock control
        u32 cpu_control = 0x00000000;        // 0x04 - CPU clock control
        u32 peripheral_control = 0x00000000; // 0x08 - Peripheral clock control
        u32 pll_control = 0x00000000;        // 0x0C - PLL control register
        u32 pll_status = 0x00000000;         // 0x10 - PLL status (read-only)
        u32 clock_gate = 0xFFFFFFFF;         // 0x14 - Clock gating control
        u32 frequency_monitor = 0x00000000;  // 0x18 - Frequency monitoring
        u32 jitter_monitor = 0x00000000;     // 0x1C - Jitter monitoring
        u32 interrupt_enable = 0x00000000;   // 0x20 - Interrupt enable
        u32 interrupt_status = 0x00000000;   // 0x24 - Interrupt status
        u32 calibration = 0x00000000;        // 0x28 - Oscillator calibration
        u32 power_control = 0x00000000;      // 0x2C - Power control
    };

    struct ClockDomain {
        std::string name;
        ClockSource source = ClockSource::XTAL_40MHZ;
        u32 frequency_hz = 40000000;
        u32 target_frequency_hz = 40000000;
        u8 divider = 1;
        ClockGatingMode gating_mode = ClockGatingMode::ALWAYS_ON;
        bool enabled = true;
        bool stable = true;
        u64 cycle_count = 0;
        std::chrono::steady_clock::time_point last_update;
    };

    struct PLLState {
        PLLConfiguration config;
        bool locked = false;
        bool enabled = false;
        float lock_time_remaining_us = 0.0f;
        u32 actual_frequency_hz = 0;
        float frequency_error_ppm = 0.0f;
        float jitter_ps = 0.0f;
        std::chrono::steady_clock::time_point last_lock_check;
    };

    struct FrequencyMonitor {
        bool enabled = false;
        u32 target_frequency_hz = 0;
        u32 measured_frequency_hz = 0;
        float error_percent = 0.0f;
        float tolerance_percent = 1.0f;
        bool in_spec = true;
        std::chrono::steady_clock::time_point last_measurement;
    };

    // Core clock management
    void update_system_clocks();
    void update_pll_states();
    void update_frequency_scaling();
    void apply_clock_gating();
    
    // PLL control algorithms
    void simulate_pll_locking(PLLState& pll);
    void calculate_pll_frequency(PLLState& pll);
    void monitor_pll_stability(PLLState& pll);
    void handle_pll_lock_lost(ClockSource pll);
    
    // Frequency scaling algorithms
    void apply_power_profile_settings();
    void dynamic_frequency_adjustment();
    void thermal_frequency_throttling();
    void workload_based_scaling();
    
    // Clock monitoring implementation
    void update_frequency_monitors();
    void update_jitter_monitors();
    void detect_clock_anomalies();
    void log_clock_event(const std::string& event);
    
    // Clock distribution simulation
    void propagate_clock_changes();
    void calculate_clock_skew();
    void simulate_clock_jitter();
    void update_power_consumption();
    
    // Calibration algorithms
    void auto_calibrate_rc_oscillators();
    void trim_oscillator_frequency(ClockSource source, float target_frequency_mhz);
    void temperature_compensate_clocks();
    
    // Error handling and recovery
    void handle_clock_failure(ClockSource source);
    void switch_to_backup_clock(PeripheralClockDomain domain);
    void emergency_frequency_reduction();
    void reset_clock_system();
    
    // Utility functions
    u32 calculate_divided_frequency(u32 source_frequency_hz, u8 divider);
    u8 calculate_optimal_divider(u32 source_frequency_hz, u32 target_frequency_hz);
    float calculate_frequency_error_ppm(u32 actual_hz, u32 target_hz);
    bool is_frequency_in_tolerance(u32 actual_hz, u32 target_hz, float tolerance_percent);
    
    void trigger_interrupt(CMUInterruptType interrupt_type);
    void update_cmu_registers();
    
    bool initialized_;
    ClockConfiguration system_config_;
    PowerProfile current_power_profile_;
    bool frequency_scaling_enabled_;
    bool low_power_mode_;
    
    CMURegisters registers_;
    InterruptController* interrupt_controller_;
    
    // Clock domain management
    std::array<ClockDomain, NUM_PERIPHERAL_DOMAINS> peripheral_domains_;
    std::array<ClockDomain, 3> cpu_domains_; // HP_CORE_0, HP_CORE_1, LP_CORE
    
    // PLL management
    std::unordered_map<ClockSource, PLLState> pll_states_;
    std::unordered_map<ClockSource, FrequencyMonitor> frequency_monitors_;
    
    ClockMonitoring monitoring_config_;
    
    // Timing and synchronization
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_calibration_;
    std::chrono::steady_clock::time_point system_start_time_;
    
    ClockStatistics statistics_;
    
    mutable std::mutex cmu_mutex_;
};

}  // namespace m5tab5::emulator