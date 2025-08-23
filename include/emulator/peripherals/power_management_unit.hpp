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

namespace m5tab5::emulator {

enum class PowerState : u8 {
    ACTIVE = 0,                  // Full power, all systems active
    IDLE = 1,                    // CPU idle, peripherals active
    LIGHT_SLEEP = 2,             // CPU stopped, RAM retained, RTC active
    DEEP_SLEEP = 3,              // CPU stopped, most peripherals off
    HIBERNATION = 4,             // Minimal power, only wake sources active
    SHUTDOWN = 5                 // System powered off
};

enum class WakeupSource : u16 {
    NONE = 0x0000,
    TOUCH_PANEL = 0x0001,        // GT911 touch controller
    GPIO_PIN = 0x0002,           // GPIO wake pin
    RTC_ALARM = 0x0004,          // RTC alarm
    TIMER_WAKE = 0x0008,         // Wake timer
    USB_CONNECT = 0x0010,        // USB cable connection
    POWER_BUTTON = 0x0020,       // Power button press
    ACCELEROMETER = 0x0040,      // Motion detection (BMI270)
    UART_RX = 0x0080,            // UART receive data
    WIFI_BEACON = 0x0100,        // WiFi beacon frame
    BLUETOOTH_ADV = 0x0200,      // Bluetooth advertisement
    LOW_BATTERY = 0x0400,        // Low battery condition
    CHARGER_CONNECT = 0x0800,    // Charger connection
    THERMAL_ALERT = 0x1000,      // Thermal management alert
    SYSTEM_FAULT = 0x2000,       // System fault condition
    EXTERNAL_INTERRUPT = 0x4000, // External interrupt source
    SOFTWARE_WAKE = 0x8000       // Software-triggered wake
};

// Bitwise operators for WakeupSource
inline WakeupSource operator&(WakeupSource a, WakeupSource b) {
    return static_cast<WakeupSource>(static_cast<u16>(a) & static_cast<u16>(b));
}

inline WakeupSource operator|(WakeupSource a, WakeupSource b) {
    return static_cast<WakeupSource>(static_cast<u16>(a) | static_cast<u16>(b));
}

inline WakeupSource operator^(WakeupSource a, WakeupSource b) {
    return static_cast<WakeupSource>(static_cast<u16>(a) ^ static_cast<u16>(b));
}

inline WakeupSource operator~(WakeupSource a) {
    return static_cast<WakeupSource>(~static_cast<u16>(a));
}

inline WakeupSource& operator&=(WakeupSource& a, WakeupSource b) {
    return a = a & b;
}

inline WakeupSource& operator|=(WakeupSource& a, WakeupSource b) {
    return a = a | b;
}

inline WakeupSource& operator^=(WakeupSource& a, WakeupSource b) {
    return a = a ^ b;
}

enum class BatteryType : u8 {
    NP_F550 = 0,                 // M5Stack Tab5 default battery (2900mAh)
    NP_F570 = 1,                 // Extended battery (3880mAh)
    NP_F970 = 2,                 // High capacity battery (6600mAh)
    EXTERNAL_POWER = 3,          // External power supply
    USB_POWER = 4                // USB-C power delivery
};

enum class ChargingState : u8 {
    NOT_CHARGING = 0,
    TRICKLE_CHARGE = 1,          // Pre-charge phase
    CONSTANT_CURRENT = 2,        // CC charging phase
    CONSTANT_VOLTAGE = 3,        // CV charging phase
    CHARGE_COMPLETE = 4,         // Battery fully charged
    CHARGE_FAULT = 5,            // Charging fault detected
    THERMAL_REGULATION = 6,      // Temperature-limited charging
    INPUT_CURRENT_LIMIT = 7      // Input current limited
};

enum class PowerRail : u8 {
    VDD_CORE = 0,                // Core CPU voltage (1.1V)
    VDD_IO = 1,                  // I/O voltage (3.3V)
    VDD_MEM = 2,                 // Memory voltage (1.2V)  
    VDD_RF = 3,                  // RF section voltage (3.3V)
    VDD_USB = 4,                 // USB PHY voltage (3.3V)
    VDD_DISPLAY = 5,             // Display voltage (3.3V)
    VDD_BACKLIGHT = 6,           // Backlight LED voltage (variable)
    VDD_AUDIO = 7,               // Audio codec voltage (3.3V)
    VDD_SENSORS = 8,             // Sensor voltage (3.3V)
    VDD_CAMERA = 9,              // Camera module voltage (2.8V)
    VDD_EXTERNAL_5V = 10,        // External 5V rail
    VDD_EXTERNAL_3V3 = 11        // External 3.3V rail
};

enum class PMUInterruptType : u16 {
    BATTERY_LOW = 0x0001,        // Battery voltage below threshold
    BATTERY_CRITICAL = 0x0002,   // Battery critically low
    CHARGER_CONNECTED = 0x0004,  // Charger cable connected
    CHARGER_DISCONNECTED = 0x0008, // Charger cable disconnected
    CHARGING_COMPLETE = 0x0010,  // Battery charging complete
    CHARGING_FAULT = 0x0020,     // Charging fault detected
    OVERCURRENT = 0x0040,        // Overcurrent protection
    OVERVOLTAGE = 0x0080,        // Overvoltage protection
    THERMAL_WARNING = 0x0100,    // Temperature warning
    THERMAL_SHUTDOWN = 0x0200,   // Thermal shutdown
    POWER_BUTTON = 0x0400,       // Power button pressed
    USB_PLUGGED = 0x0800,        // USB cable plugged in
    USB_UNPLUGGED = 0x1000,      // USB cable unplugged
    VOLTAGE_REGULATOR_FAULT = 0x2000, // Voltage regulator fault
    POWER_GOOD = 0x4000,         // All power rails stable
    WAKE_EVENT = 0x8000          // System wake event
};

struct BatteryInfo {
    BatteryType type = BatteryType::NP_F550;
    float voltage_v = 7.4f;              // Current battery voltage (V)
    float current_ma = 0.0f;             // Current battery current (mA, negative = discharging)
    float capacity_mah = 2900.0f;        // Battery capacity (mAh)
    float remaining_mah = 2900.0f;       // Remaining capacity (mAh)
    float charge_percentage = 100.0f;    // Battery charge level (%)
    float temperature_c = 25.0f;         // Battery temperature (°C)
    u32 cycle_count = 0;                 // Battery charge cycle count
    float health_percentage = 100.0f;    // Battery health (%)
    ChargingState charging_state = ChargingState::NOT_CHARGING;
    bool is_present = true;              // Battery physically present
    std::chrono::steady_clock::time_point last_update;
};

struct PowerConsumption {
    float cpu_core_mw = 200.0f;          // CPU core power (mW)
    float memory_mw = 50.0f;             // Memory power (mW)
    float display_mw = 300.0f;           // Display power (mW)
    float backlight_mw = 500.0f;         // Backlight power (mW)
    float wifi_mw = 100.0f;              // WiFi module power (mW)
    float bluetooth_mw = 20.0f;          // Bluetooth power (mW)
    float audio_mw = 30.0f;              // Audio codec power (mW)
    float sensors_mw = 10.0f;            // Sensors power (mW)
    float peripherals_mw = 50.0f;        // Other peripherals (mW)
    float total_mw = 1260.0f;            // Total system power (mW)
    float efficiency_percentage = 85.0f;  // Power conversion efficiency (%)
};

struct ThermalInfo {
    float cpu_temperature_c = 40.0f;     // CPU temperature (°C)
    float battery_temperature_c = 25.0f; // Battery temperature (°C)
    float ambient_temperature_c = 22.0f; // Ambient temperature (°C)  
    float pmu_temperature_c = 35.0f;     // PMU IC temperature (°C)
    float thermal_throttle_threshold_c = 85.0f; // Thermal throttling threshold
    float thermal_shutdown_threshold_c = 95.0f; // Thermal shutdown threshold
    bool thermal_throttling_active = false;
    bool thermal_warning_active = false;
};

struct PowerStatistics {
    u64 total_runtime_seconds = 0;       // Total device runtime
    u64 battery_runtime_seconds = 0;     // Runtime on battery power  
    u64 charging_cycles = 0;             // Total charging cycles
    u64 deep_sleep_entries = 0;          // Number of deep sleep entries
    u64 thermal_events = 0;              // Thermal management events
    u64 power_faults = 0;                // Power fault events
    float average_power_consumption_mw = 1260.0f; // Average power consumption
    float peak_power_consumption_mw = 1260.0f;    // Peak power consumption
    double energy_consumed_wh = 0.0;     // Total energy consumed (Wh)
    double energy_charged_wh = 0.0;      // Total energy charged (Wh)
};

class PowerManagementUnit {
public:
    static constexpr float MIN_BATTERY_VOLTAGE = 6.0f;      // Minimum battery voltage (V)
    static constexpr float MAX_BATTERY_VOLTAGE = 8.4f;      // Maximum battery voltage (V)
    static constexpr float LOW_BATTERY_THRESHOLD = 6.5f;    // Low battery warning (V)
    static constexpr float CRITICAL_BATTERY_THRESHOLD = 6.2f; // Critical battery level (V)
    static constexpr float MAX_CHARGING_CURRENT = 2000.0f;  // Maximum charging current (mA)
    static constexpr float MAX_SYSTEM_CURRENT = 3000.0f;    // Maximum system current (mA)
    static constexpr size_t NUM_POWER_RAILS = 12;
    
    PowerManagementUnit();
    ~PowerManagementUnit();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    // Power state management
    Result<void> set_power_state(PowerState state);
    Result<PowerState> get_power_state() const;
    Result<void> request_system_shutdown();
    Result<void> request_system_reboot();
    Result<void> configure_wakeup_sources(WakeupSource sources);
    Result<WakeupSource> get_wakeup_source() const;
    
    // Battery management
    Result<BatteryInfo> get_battery_info() const;
    Result<void> set_battery_type(BatteryType type);
    Result<float> get_battery_percentage() const;
    Result<float> get_battery_voltage() const;
    Result<float> get_battery_current() const;
    Result<u32> get_estimated_runtime_minutes() const;
    
    // Charging control
    Result<void> start_charging();
    Result<void> stop_charging(); 
    Result<void> set_charging_current(float current_ma);
    Result<ChargingState> get_charging_state() const;
    Result<bool> is_charger_connected() const;
    Result<bool> is_usb_connected() const;
    
    // Power rail management
    Result<void> enable_power_rail(PowerRail rail, bool enable);
    Result<bool> is_power_rail_enabled(PowerRail rail) const;
    Result<void> set_rail_voltage(PowerRail rail, float voltage_v);
    Result<float> get_rail_voltage(PowerRail rail) const;
    Result<float> get_rail_current(PowerRail rail) const;
    
    // Power consumption monitoring
    Result<PowerConsumption> get_power_consumption() const;
    Result<void> set_cpu_frequency_mhz(u32 frequency_mhz);
    Result<void> enable_power_saving_mode(bool enable);
    Result<void> configure_backlight_power(u8 brightness_level);
    
    // Thermal management
    Result<ThermalInfo> get_thermal_info() const;
    Result<void> set_thermal_policy(bool aggressive_throttling);
    Result<void> trigger_thermal_shutdown();
    Result<bool> is_thermal_throttling_active() const;
    
    // Sleep and wake management
    Result<void> enter_light_sleep();
    Result<void> enter_deep_sleep();
    Result<void> enter_hibernation();
    Result<void> wake_from_sleep(WakeupSource source);
    Result<void> set_sleep_timer(u32 seconds);
    
    // USB power delivery (USB-C)
    Result<void> configure_usb_pd(bool enable, u8 voltage_profile = 0); // 5V, 9V, 12V, 15V, 20V
    Result<float> get_usb_input_voltage() const;
    Result<float> get_usb_input_current() const;
    Result<u8> get_usb_pd_profile() const;
    
    // Power button and GPIO wake
    Result<void> simulate_power_button_press(u32 duration_ms);
    Result<void> configure_gpio_wakeup(u8 gpio_pin, bool rising_edge);
    Result<bool> is_power_button_pressed() const;
    
    // Advanced power features
    Result<void> enable_dynamic_voltage_scaling(bool enable);
    Result<void> enable_clock_gating(bool enable);
    Result<void> set_power_profile(const std::string& profile_name); // "performance", "balanced", "power_save"
    Result<void> calibrate_battery_gauge();
    
    // Interrupt handling
    Result<void> enable_interrupt(PMUInterruptType interrupt_type);
    Result<void> disable_interrupt(PMUInterruptType interrupt_type);
    Result<u16> get_interrupt_status();
    Result<void> clear_interrupt(PMUInterruptType interrupt_type);
    
    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_system_powered() const { return current_power_state_ != PowerState::SHUTDOWN; }
    const PowerStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct PMURegisters {
        u8 power_control = 0x01;          // 0x00 - Main power control
        u8 power_status = 0x00;           // 0x01 - Power status (read-only)
        u8 battery_status = 0x00;         // 0x02 - Battery status
        u8 charging_control = 0x00;       // 0x03 - Charging control
        u8 charging_status = 0x00;        // 0x04 - Charging status  
        u8 wakeup_enable = 0x00;          // 0x05 - Wakeup source enable
        u8 wakeup_status = 0x00;          // 0x06 - Wakeup source status
        u8 interrupt_enable = 0x00;       // 0x07 - Interrupt enable mask
        u8 interrupt_status = 0x00;       // 0x08 - Interrupt status
        u8 thermal_control = 0x00;        // 0x09 - Thermal management
        u8 rail_enable = 0xFF;            // 0x0A - Power rail enable mask
        u8 rail_status = 0xFF;            // 0x0B - Power rail status
        u16 battery_voltage = 0x1D4C;     // 0x0C-0x0D - Battery voltage (mV)
        u16 battery_current = 0x0000;     // 0x0E-0x0F - Battery current (mA)  
        u8 battery_percentage = 100;      // 0x10 - Battery charge percentage
        u8 usb_status = 0x00;             // 0x11 - USB power status
        u8 thermal_status = 0x00;         // 0x12 - Thermal status
        u8 power_profile = 0x01;          // 0x13 - Power profile selection
    };

    struct PowerRailInfo {
        bool enabled = true;
        float target_voltage_v = 3.3f;
        float actual_voltage_v = 3.3f;
        float current_ma = 0.0f;
        float power_mw = 0.0f;
        bool fault_detected = false;
        std::string name;
    };

    struct ChargingController {
        bool charging_enabled = false;
        bool charger_connected = false;
        float charging_current_ma = 0.0f;
        float target_voltage_v = 8.4f;
        ChargingState state = ChargingState::NOT_CHARGING;
        std::chrono::steady_clock::time_point charging_start_time;
        float total_charge_mah = 0.0f;
    };

    struct ThermalController {
        bool monitoring_enabled = true;
        bool throttling_enabled = true;
        float throttle_temperature_c = 85.0f;
        float shutdown_temperature_c = 95.0f;
        u32 cpu_frequency_original_mhz = 400;
        u32 cpu_frequency_throttled_mhz = 200;
        std::chrono::steady_clock::time_point last_thermal_check;
    };

    // Core power management
    void update_power_state_machine();
    void process_power_transitions();
    void handle_wakeup_events();
    void manage_power_rails();
    
    // Battery simulation and management
    void simulate_battery_discharge();
    void simulate_battery_charging();
    void update_battery_statistics();
    void calculate_remaining_runtime();
    void perform_battery_health_check();
    
    // Thermal management
    void update_thermal_monitoring();
    void apply_thermal_throttling();
    void simulate_temperature_sensors();
    void handle_thermal_events();
    
    // Power consumption calculation
    void calculate_power_consumption();
    void update_power_efficiency();
    void apply_power_saving_measures();
    void optimize_power_rails();
    
    // Sleep mode implementation
    void prepare_for_sleep(PowerState target_state);
    void restore_from_sleep();
    void configure_wakeup_hardware();
    void simulate_sleep_power_consumption();
    
    // USB power delivery
    void negotiate_usb_pd_contract();
    void monitor_usb_power_input();
    void handle_usb_connection_events();
    
    // Safety and protection
    void monitor_overcurrent_protection();
    void monitor_overvoltage_protection();
    void handle_thermal_protection();
    void perform_safety_shutdown();
    
    // Simulation helpers
    void simulate_charger_connection(bool connected);
    void simulate_usb_connection(bool connected);
    void simulate_battery_removal(bool removed);
    void add_power_consumption_noise();
    
    void trigger_interrupt(PMUInterruptType interrupt_type);
    void update_pmu_registers();
    void log_power_event(const std::string& event);
    
    bool initialized_;
    PowerState current_power_state_;
    PowerState previous_power_state_;
    WakeupSource enabled_wakeup_sources_;
    WakeupSource last_wakeup_source_;
    
    PMURegisters registers_;
    InterruptController* interrupt_controller_;
    
    // Power management components
    BatteryInfo battery_info_;
    PowerConsumption power_consumption_;
    ThermalInfo thermal_info_;
    ChargingController charging_controller_;
    ThermalController thermal_controller_;
    std::array<PowerRailInfo, NUM_POWER_RAILS> power_rails_;
    
    // State tracking
    bool power_button_pressed_;
    bool usb_connected_;
    bool charger_connected_;
    bool in_thermal_throttling_;
    std::string active_power_profile_;
    
    // Timing control
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point sleep_timer_start_;
    std::chrono::steady_clock::time_point battery_update_time_;
    u32 sleep_timer_duration_seconds_;
    
    PowerStatistics statistics_;
    
    mutable std::mutex pmu_mutex_;
};

}  // namespace m5tab5::emulator