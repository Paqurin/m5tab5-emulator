#include "emulator/peripherals/power_management_unit.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace m5tab5::emulator {

DECLARE_LOGGER("PowerManagementUnit");

PowerManagementUnit::PowerManagementUnit()
    : initialized_(false),
      current_power_state_(PowerState::ACTIVE),
      previous_power_state_(PowerState::SHUTDOWN),
      enabled_wakeup_sources_(WakeupSource::NONE),
      last_wakeup_source_(WakeupSource::NONE),
      interrupt_controller_(nullptr),
      power_button_pressed_(false),
      usb_connected_(false),
      charger_connected_(false),
      in_thermal_throttling_(false),
      active_power_profile_("balanced"),
      sleep_timer_duration_seconds_(0) {
    
    // Initialize battery info with default NP-F550 battery
    battery_info_.type = BatteryType::NP_F550;
    battery_info_.voltage_v = 7.4f;
    battery_info_.capacity_mah = 2900.0f;
    battery_info_.remaining_mah = 2900.0f;
    battery_info_.charge_percentage = 100.0f;
    battery_info_.temperature_c = 25.0f;
    battery_info_.health_percentage = 100.0f;
    battery_info_.is_present = true;
    
    // Initialize power consumption
    power_consumption_.cpu_core_mw = 200.0f;
    power_consumption_.memory_mw = 50.0f;
    power_consumption_.display_mw = 300.0f;
    power_consumption_.backlight_mw = 500.0f;
    power_consumption_.total_mw = 1260.0f;
    
    // Initialize thermal info
    thermal_info_.cpu_temperature_c = 40.0f;
    thermal_info_.battery_temperature_c = 25.0f;
    thermal_info_.ambient_temperature_c = 22.0f;
    thermal_info_.pmu_temperature_c = 35.0f;
    
    // Initialize power rails
    const std::array<std::string, NUM_POWER_RAILS> rail_names = {
        "VDD_CORE", "VDD_IO", "VDD_MEM", "VDD_RF", "VDD_USB", "VDD_DISPLAY",
        "VDD_BACKLIGHT", "VDD_AUDIO", "VDD_SENSORS", "VDD_CAMERA", "VDD_EXTERNAL_5V", "VDD_EXTERNAL_3V3"
    };
    
    const std::array<float, NUM_POWER_RAILS> rail_voltages = {
        1.1f, 3.3f, 1.2f, 3.3f, 3.3f, 3.3f, 12.0f, 3.3f, 3.3f, 2.8f, 5.0f, 3.3f
    };
    
    for (size_t i = 0; i < NUM_POWER_RAILS; ++i) {
        power_rails_[i].name = rail_names[i];
        power_rails_[i].target_voltage_v = rail_voltages[i];
        power_rails_[i].actual_voltage_v = rail_voltages[i];
        power_rails_[i].enabled = true;
    }
    
    // Initialize charging controller
    charging_controller_.charging_enabled = false;
    charging_controller_.charger_connected = false;
    charging_controller_.state = ChargingState::NOT_CHARGING;
    
    // Initialize thermal controller
    thermal_controller_.monitoring_enabled = true;
    thermal_controller_.throttling_enabled = true;
    thermal_controller_.throttle_temperature_c = 85.0f;
    thermal_controller_.shutdown_temperature_c = 95.0f;
    thermal_controller_.cpu_frequency_original_mhz = 400;
    thermal_controller_.cpu_frequency_throttled_mhz = 200;
    
    COMPONENT_LOG_DEBUG("Power Management Unit created");
}

PowerManagementUnit::~PowerManagementUnit() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Power Management Unit destroyed");
}

Result<void> PowerManagementUnit::initialize(const Configuration& config, InterruptController* interrupt_controller) {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Power Management Unit already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing Power Management Unit");
    
    interrupt_controller_ = interrupt_controller;
    
    // Initialize registers with default values
    registers_ = {};
    registers_.power_control = 0x01; // Power on
    registers_.battery_voltage = static_cast<u16>(battery_info_.voltage_v * 1000); // mV
    registers_.battery_percentage = static_cast<u8>(battery_info_.charge_percentage);
    registers_.rail_enable = 0xFF; // All rails enabled
    registers_.rail_status = 0xFF; // All rails OK
    
    // Initialize timing
    last_update_ = std::chrono::steady_clock::now();
    battery_update_time_ = last_update_;
    
    // Clear statistics
    statistics_ = {};
    
    // Set initial power state
    current_power_state_ = PowerState::ACTIVE;
    previous_power_state_ = PowerState::SHUTDOWN;
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Power Management Unit initialized successfully");
    
    return {};
}

Result<void> PowerManagementUnit::shutdown() {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down Power Management Unit");
    
    // Force power off
    current_power_state_ = PowerState::SHUTDOWN;
    registers_.power_control = 0x00;
    
    // Disable all power rails
    for (auto& rail : power_rails_) {
        rail.enabled = false;
    }
    registers_.rail_enable = 0x00;
    
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Power Management Unit shutdown completed");
    return {};
}

Result<void> PowerManagementUnit::set_power_state(PowerState state) {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    if (state == current_power_state_) {
        return {}; // Already in requested state
    }
    
    previous_power_state_ = current_power_state_;
    current_power_state_ = state;
    
    // Update power control register
    switch (state) {
        case PowerState::ACTIVE:
            registers_.power_control = 0x01;
            break;
        case PowerState::IDLE:
            registers_.power_control = 0x02;
            break;
        case PowerState::LIGHT_SLEEP:
            registers_.power_control = 0x04;
            prepare_for_sleep(state);
            break;
        case PowerState::DEEP_SLEEP:
            registers_.power_control = 0x08;
            prepare_for_sleep(state);
            break;
        case PowerState::HIBERNATION:
            registers_.power_control = 0x10;
            prepare_for_sleep(state);
            break;
        case PowerState::SHUTDOWN:
            registers_.power_control = 0x00;
            break;
    }
    
    // Update power consumption based on state
    calculate_power_consumption();
    
    // Trigger power mode change interrupt
    trigger_interrupt(PMUInterruptType::POWER_GOOD);
    
    COMPONENT_LOG_INFO("Power state changed from {} to {}", 
                      static_cast<u8>(previous_power_state_), static_cast<u8>(state));
    
    return {};
}

Result<PowerState> PowerManagementUnit::get_power_state() const {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    return current_power_state_;
}

Result<BatteryInfo> PowerManagementUnit::get_battery_info() const {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    return battery_info_;
}

Result<void> PowerManagementUnit::start_charging() {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    if (!charger_connected_) {
        return unexpected(MAKE_ERROR(INVALID_OPERATION,
            "Charger not connected"));
    }
    
    if (battery_info_.charge_percentage >= 100.0f) {
        charging_controller_.state = ChargingState::CHARGE_COMPLETE;
        return {}; // Battery already full
    }
    
    charging_controller_.charging_enabled = true;
    charging_controller_.state = ChargingState::CONSTANT_CURRENT;
    charging_controller_.charging_current_ma = 1500.0f; // 1.5A charging current
    charging_controller_.charging_start_time = std::chrono::steady_clock::now();
    
    registers_.charging_control = 0x01; // Enable charging
    registers_.charging_status = static_cast<u8>(charging_controller_.state);
    
    trigger_interrupt(PMUInterruptType::CHARGING_COMPLETE);
    
    COMPONENT_LOG_INFO("Battery charging started");
    return {};
}

Result<void> PowerManagementUnit::stop_charging() {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    charging_controller_.charging_enabled = false;
    charging_controller_.state = ChargingState::NOT_CHARGING;
    charging_controller_.charging_current_ma = 0.0f;
    
    registers_.charging_control = 0x00; // Disable charging
    registers_.charging_status = static_cast<u8>(charging_controller_.state);
    
    COMPONENT_LOG_INFO("Battery charging stopped");
    return {};
}

Result<void> PowerManagementUnit::enable_power_rail(PowerRail rail, bool enable) {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    size_t rail_index = static_cast<size_t>(rail);
    if (rail_index >= NUM_POWER_RAILS) {
        return unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid power rail"));
    }
    
    power_rails_[rail_index].enabled = enable;
    
    // Update register bits
    if (enable) {
        registers_.rail_enable |= (1 << rail_index);
        registers_.rail_status |= (1 << rail_index);
    } else {
        registers_.rail_enable &= ~(1 << rail_index);
        registers_.rail_status &= ~(1 << rail_index);
    }
    
    COMPONENT_LOG_DEBUG("Power rail {} {}: {}", 
                       power_rails_[rail_index].name, 
                       enable ? "enabled" : "disabled",
                       power_rails_[rail_index].target_voltage_v);
    
    return {};
}

Result<ThermalInfo> PowerManagementUnit::get_thermal_info() const {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    return thermal_info_;
}

Result<void> PowerManagementUnit::simulate_power_button_press(u32 duration_ms) {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Power Management Unit not initialized"));
    }
    
    power_button_pressed_ = true;
    
    // Trigger power button interrupt
    trigger_interrupt(PMUInterruptType::POWER_BUTTON);
    
    // Handle power button logic based on current state and duration
    if (duration_ms > 3000) { // Long press (3+ seconds)
        if (current_power_state_ == PowerState::SHUTDOWN) {
            // Power on from shutdown
            set_power_state(PowerState::ACTIVE);
        } else {
            // Force shutdown
            set_power_state(PowerState::SHUTDOWN);
        }
    } else if (duration_ms > 100) { // Short press
        if (current_power_state_ == PowerState::ACTIVE) {
            // Enter sleep mode
            set_power_state(PowerState::LIGHT_SLEEP);
        } else if (current_power_state_ == PowerState::LIGHT_SLEEP) {
            // Wake up
            wake_from_sleep(WakeupSource::POWER_BUTTON);
        }
    }
    
    COMPONENT_LOG_INFO("Power button pressed for {} ms", duration_ms);
    
    // Simulate button release after duration
    // In real implementation, this would be handled by a timer
    power_button_pressed_ = false;
    
    return {};
}

void PowerManagementUnit::update() {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    if (!initialized_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);
    last_update_ = now;
    
    // Update power state machine
    update_power_state_machine();
    
    // Update battery simulation (every 5 seconds)
    auto battery_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - battery_update_time_);
    if (battery_elapsed.count() >= 5) {
        simulate_battery_discharge();
        simulate_battery_charging();
        update_battery_statistics();
        battery_update_time_ = now;
    }
    
    // Update thermal monitoring (every 1 second)
    static auto last_thermal_update = now;
    auto thermal_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_thermal_update);
    if (thermal_elapsed.count() >= 1) {
        update_thermal_monitoring();
        last_thermal_update = now;
    }
    
    // Update power consumption calculation
    calculate_power_consumption();
    
    // Update statistics
    statistics_.total_runtime_seconds += elapsed.count() / 1000.0;
    if (!charging_controller_.charging_enabled) {
        statistics_.battery_runtime_seconds += elapsed.count() / 1000.0;
    }
}

void PowerManagementUnit::update_power_state_machine() {
    // Handle automatic power state transitions
    switch (current_power_state_) {
        case PowerState::ACTIVE:
            // No automatic transitions from active
            break;
            
        case PowerState::IDLE:
            // Could transition to light sleep after timeout
            break;
            
        case PowerState::LIGHT_SLEEP:
        case PowerState::DEEP_SLEEP:
        case PowerState::HIBERNATION:
            // Check for wakeup conditions
            handle_wakeup_events();
            break;
            
        case PowerState::SHUTDOWN:
            // Only wakeup sources can exit shutdown
            handle_wakeup_events();
            break;
    }
}

void PowerManagementUnit::handle_wakeup_events() {
    // Check enabled wakeup sources
    if ((enabled_wakeup_sources_ & WakeupSource::POWER_BUTTON) != WakeupSource::NONE && power_button_pressed_) {
        wake_from_sleep(WakeupSource::POWER_BUTTON);
    }
    
    if ((enabled_wakeup_sources_ & WakeupSource::USB_CONNECT) != WakeupSource::NONE && usb_connected_) {
        wake_from_sleep(WakeupSource::USB_CONNECT);
    }
    
    if ((enabled_wakeup_sources_ & WakeupSource::CHARGER_CONNECT) != WakeupSource::NONE && charger_connected_) {
        wake_from_sleep(WakeupSource::CHARGER_CONNECT);
    }
    
    // Sleep timer check
    if (sleep_timer_duration_seconds_ > 0) {
        auto now = std::chrono::steady_clock::now();
        auto timer_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - sleep_timer_start_);
        if (timer_elapsed.count() >= sleep_timer_duration_seconds_) {
            wake_from_sleep(WakeupSource::TIMER_WAKE);
            sleep_timer_duration_seconds_ = 0; // Clear timer
        }
    }
}

void PowerManagementUnit::simulate_battery_discharge() {
    if (charging_controller_.charging_enabled || battery_info_.charge_percentage <= 0.0f) {
        return; // Don't discharge while charging or if already empty
    }
    
    // Calculate discharge rate based on power consumption
    float discharge_rate_mah_per_hour = power_consumption_.total_mw / battery_info_.voltage_v; // Rough approximation
    float discharge_per_update = discharge_rate_mah_per_hour / (60.0f * 12.0f); // 5-second updates
    
    // Apply power state modifier
    switch (current_power_state_) {
        case PowerState::ACTIVE:
            // Full discharge rate
            break;
        case PowerState::IDLE:
            discharge_per_update *= 0.7f;
            break;
        case PowerState::LIGHT_SLEEP:
            discharge_per_update *= 0.1f;
            break;
        case PowerState::DEEP_SLEEP:
            discharge_per_update *= 0.05f;
            break;
        case PowerState::HIBERNATION:
            discharge_per_update *= 0.01f;
            break;
        case PowerState::SHUTDOWN:
            discharge_per_update *= 0.001f;
            break;
    }
    
    // Update battery capacity
    battery_info_.remaining_mah = std::max(0.0f, battery_info_.remaining_mah - discharge_per_update);
    battery_info_.charge_percentage = (battery_info_.remaining_mah / battery_info_.capacity_mah) * 100.0f;
    
    // Update voltage (simple linear model)
    battery_info_.voltage_v = MIN_BATTERY_VOLTAGE + 
        (battery_info_.charge_percentage / 100.0f) * (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
    
    battery_info_.current_ma = -discharge_per_update * 12.0f; // Negative for discharge
    
    // Check for low battery conditions
    if (battery_info_.voltage_v <= LOW_BATTERY_THRESHOLD && battery_info_.voltage_v > CRITICAL_BATTERY_THRESHOLD) {
        trigger_interrupt(PMUInterruptType::BATTERY_LOW);
    } else if (battery_info_.voltage_v <= CRITICAL_BATTERY_THRESHOLD) {
        trigger_interrupt(PMUInterruptType::BATTERY_CRITICAL);
        // Auto-shutdown on critical battery
        if (current_power_state_ != PowerState::SHUTDOWN) {
            set_power_state(PowerState::SHUTDOWN);
        }
    }
    
    // Update registers
    registers_.battery_voltage = static_cast<u16>(battery_info_.voltage_v * 1000);
    registers_.battery_current = static_cast<u16>(std::abs(battery_info_.current_ma));
    registers_.battery_percentage = static_cast<u8>(battery_info_.charge_percentage);
}

void PowerManagementUnit::simulate_battery_charging() {
    if (!charging_controller_.charging_enabled || !charger_connected_) {
        return;
    }
    
    if (battery_info_.charge_percentage >= 100.0f) {
        charging_controller_.state = ChargingState::CHARGE_COMPLETE;
        registers_.charging_status = static_cast<u8>(charging_controller_.state);
        trigger_interrupt(PMUInterruptType::CHARGING_COMPLETE);
        return;
    }
    
    // Calculate charge rate
    float charge_rate_mah = charging_controller_.charging_current_ma / 12.0f; // 5-second updates
    
    // Apply charging efficiency
    charge_rate_mah *= 0.85f; // 85% charging efficiency
    
    // Update battery capacity
    battery_info_.remaining_mah = std::min(battery_info_.capacity_mah, 
                                          battery_info_.remaining_mah + charge_rate_mah);
    battery_info_.charge_percentage = (battery_info_.remaining_mah / battery_info_.capacity_mah) * 100.0f;
    
    // Update voltage
    battery_info_.voltage_v = MIN_BATTERY_VOLTAGE + 
        (battery_info_.charge_percentage / 100.0f) * (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
    
    battery_info_.current_ma = charging_controller_.charging_current_ma; // Positive for charge
    
    // Update charging state based on voltage
    if (battery_info_.voltage_v >= 8.2f) {
        charging_controller_.state = ChargingState::CONSTANT_VOLTAGE;
    } else {
        charging_controller_.state = ChargingState::CONSTANT_CURRENT;
    }
    
    registers_.charging_status = static_cast<u8>(charging_controller_.state);
    
    // Update total charge accumulated
    charging_controller_.total_charge_mah += charge_rate_mah;
    statistics_.energy_charged_wh += (charge_rate_mah * battery_info_.voltage_v) / 1000.0;
}

void PowerManagementUnit::update_battery_statistics() {
    battery_info_.last_update = std::chrono::steady_clock::now();
    
    // Update energy consumed
    statistics_.energy_consumed_wh += (power_consumption_.total_mw * 5.0) / (1000.0 * 3600.0); // 5 second interval
    
    // Update average power consumption
    statistics_.average_power_consumption_mw = 
        (statistics_.average_power_consumption_mw + power_consumption_.total_mw) / 2.0f;
    
    // Update peak power consumption
    statistics_.peak_power_consumption_mw = 
        std::max(statistics_.peak_power_consumption_mw, power_consumption_.total_mw);
}

void PowerManagementUnit::update_thermal_monitoring() {
    // Simulate temperature changes based on power consumption and environment
    float heat_generation = power_consumption_.total_mw / 1000.0f; // Watts to thermal load
    
    // Update CPU temperature
    thermal_info_.cpu_temperature_c = thermal_info_.ambient_temperature_c + heat_generation * 15.0f;
    
    // Update PMU temperature  
    thermal_info_.pmu_temperature_c = thermal_info_.ambient_temperature_c + heat_generation * 8.0f;
    
    // Add some random variation
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> temp_noise(0.0f, 1.0f);
    
    thermal_info_.cpu_temperature_c += temp_noise(gen);
    thermal_info_.pmu_temperature_c += temp_noise(gen);
    
    // Check for thermal events
    bool should_throttle = thermal_info_.cpu_temperature_c >= thermal_controller_.throttle_temperature_c;
    bool should_shutdown = thermal_info_.cpu_temperature_c >= thermal_controller_.shutdown_temperature_c;
    
    if (should_shutdown) {
        thermal_info_.thermal_warning_active = true;
        trigger_interrupt(PMUInterruptType::THERMAL_SHUTDOWN);
        set_power_state(PowerState::SHUTDOWN);
        statistics_.thermal_events++;
    } else if (should_throttle && !in_thermal_throttling_) {
        thermal_info_.thermal_throttling_active = true;
        in_thermal_throttling_ = true;
        trigger_interrupt(PMUInterruptType::THERMAL_WARNING);
        statistics_.thermal_events++;
    } else if (!should_throttle && in_thermal_throttling_) {
        thermal_info_.thermal_throttling_active = false;
        in_thermal_throttling_ = false;
    }
    
    // Update thermal status register
    registers_.thermal_status = 0x00;
    if (thermal_info_.thermal_warning_active) {
        registers_.thermal_status |= 0x01;
    }
    if (thermal_info_.thermal_throttling_active) {
        registers_.thermal_status |= 0x02;
    }
}

void PowerManagementUnit::calculate_power_consumption() {
    power_consumption_.total_mw = 0.0f;
    
    // Base power consumption based on power state
    switch (current_power_state_) {
        case PowerState::ACTIVE:
            power_consumption_.cpu_core_mw = 200.0f;
            power_consumption_.memory_mw = 50.0f;
            break;
        case PowerState::IDLE:
            power_consumption_.cpu_core_mw = 50.0f;
            power_consumption_.memory_mw = 30.0f;
            break;
        case PowerState::LIGHT_SLEEP:
            power_consumption_.cpu_core_mw = 5.0f;
            power_consumption_.memory_mw = 10.0f;
            power_consumption_.display_mw = 0.0f;
            power_consumption_.backlight_mw = 0.0f;
            break;
        case PowerState::DEEP_SLEEP:
            power_consumption_.cpu_core_mw = 1.0f;
            power_consumption_.memory_mw = 2.0f;
            power_consumption_.display_mw = 0.0f;
            power_consumption_.backlight_mw = 0.0f;
            power_consumption_.peripherals_mw = 1.0f;
            break;
        case PowerState::HIBERNATION:
            power_consumption_.cpu_core_mw = 0.1f;
            power_consumption_.memory_mw = 0.5f;
            power_consumption_.display_mw = 0.0f;
            power_consumption_.backlight_mw = 0.0f;
            power_consumption_.peripherals_mw = 0.1f;
            break;
        case PowerState::SHUTDOWN:
            // Only minimal leakage current
            power_consumption_.cpu_core_mw = 0.01f;
            power_consumption_.memory_mw = 0.01f;
            power_consumption_.display_mw = 0.0f;
            power_consumption_.backlight_mw = 0.0f;
            power_consumption_.peripherals_mw = 0.01f;
            break;
    }
    
    // Sum all components
    power_consumption_.total_mw = 
        power_consumption_.cpu_core_mw +
        power_consumption_.memory_mw +
        power_consumption_.display_mw +
        power_consumption_.backlight_mw +
        power_consumption_.wifi_mw +
        power_consumption_.bluetooth_mw +
        power_consumption_.audio_mw +
        power_consumption_.sensors_mw +
        power_consumption_.peripherals_mw;
    
    // Apply efficiency loss
    power_consumption_.total_mw /= (power_consumption_.efficiency_percentage / 100.0f);
}

void PowerManagementUnit::prepare_for_sleep(PowerState target_state) {
    // Disable non-essential power rails
    switch (target_state) {
        case PowerState::LIGHT_SLEEP:
            // Keep most rails on
            break;
        case PowerState::DEEP_SLEEP:
            // Turn off display and some peripherals
            enable_power_rail(PowerRail::VDD_DISPLAY, false);
            enable_power_rail(PowerRail::VDD_BACKLIGHT, false);
            enable_power_rail(PowerRail::VDD_AUDIO, false);
            break;
        case PowerState::HIBERNATION:
            // Turn off most rails except core
            for (size_t i = 2; i < NUM_POWER_RAILS; ++i) { // Keep VDD_CORE and VDD_IO
                enable_power_rail(static_cast<PowerRail>(i), false);
            }
            break;
        default:
            break;
    }
    
    statistics_.deep_sleep_entries++;
}

Result<void> PowerManagementUnit::wake_from_sleep(WakeupSource source) {
    if (current_power_state_ == PowerState::ACTIVE) {
        return {}; // Already awake
    }
    
    last_wakeup_source_ = source;
    
    // Restore power rails
    for (size_t i = 0; i < NUM_POWER_RAILS; ++i) {
        enable_power_rail(static_cast<PowerRail>(i), true);
    }
    
    // Transition to active state
    set_power_state(PowerState::ACTIVE);
    
    // Trigger wakeup interrupt
    trigger_interrupt(PMUInterruptType::WAKE_EVENT);
    
    COMPONENT_LOG_INFO("System woke up from {} due to {}", 
                      static_cast<u8>(previous_power_state_), static_cast<u16>(source));
    
    return {};
}

void PowerManagementUnit::trigger_interrupt(PMUInterruptType interrupt_type) {
    if (interrupt_controller_) {
        // Map PMU interrupts to system interrupts
        interrupt_controller_->trigger_interrupt(InterruptType::PMU);
    }
    
    // Update interrupt status register
    registers_.interrupt_status |= static_cast<u8>(interrupt_type);
}

void PowerManagementUnit::clear_statistics() {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    statistics_ = {};
}

void PowerManagementUnit::dump_status() const {
    std::lock_guard<std::mutex> lock(pmu_mutex_);
    
    COMPONENT_LOG_INFO("=== Power Management Unit Status ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (initialized_) {
        COMPONENT_LOG_INFO("Power state: {}", static_cast<u8>(current_power_state_));
        COMPONENT_LOG_INFO("Wakeup sources enabled: 0x{:04X}", static_cast<u16>(enabled_wakeup_sources_));
        COMPONENT_LOG_INFO("Last wakeup source: 0x{:04X}", static_cast<u16>(last_wakeup_source_));
        
        COMPONENT_LOG_INFO("Battery:");
        COMPONENT_LOG_INFO("  Type: {}", static_cast<u8>(battery_info_.type));
        COMPONENT_LOG_INFO("  Voltage: {:.2f} V", battery_info_.voltage_v);
        COMPONENT_LOG_INFO("  Current: {:.1f} mA", battery_info_.current_ma);
        COMPONENT_LOG_INFO("  Charge: {:.1f}%", battery_info_.charge_percentage);
        COMPONENT_LOG_INFO("  Capacity: {:.0f}/{:.0f} mAh", 
                          battery_info_.remaining_mah, battery_info_.capacity_mah);
        COMPONENT_LOG_INFO("  Temperature: {:.1f}°C", battery_info_.temperature_c);
        COMPONENT_LOG_INFO("  Health: {:.1f}%", battery_info_.health_percentage);
        
        COMPONENT_LOG_INFO("Charging:");
        COMPONENT_LOG_INFO("  Enabled: {}", charging_controller_.charging_enabled);
        COMPONENT_LOG_INFO("  State: {}", static_cast<u8>(charging_controller_.state));
        COMPONENT_LOG_INFO("  Current: {:.1f} mA", charging_controller_.charging_current_ma);
        COMPONENT_LOG_INFO("  Charger connected: {}", charger_connected_);
        COMPONENT_LOG_INFO("  USB connected: {}", usb_connected_);
        
        COMPONENT_LOG_INFO("Power consumption:");
        COMPONENT_LOG_INFO("  CPU: {:.1f} mW", power_consumption_.cpu_core_mw);
        COMPONENT_LOG_INFO("  Memory: {:.1f} mW", power_consumption_.memory_mw);
        COMPONENT_LOG_INFO("  Display: {:.1f} mW", power_consumption_.display_mw);
        COMPONENT_LOG_INFO("  Backlight: {:.1f} mW", power_consumption_.backlight_mw);
        COMPONENT_LOG_INFO("  Total: {:.1f} mW", power_consumption_.total_mw);
        COMPONENT_LOG_INFO("  Efficiency: {:.1f}%", power_consumption_.efficiency_percentage);
        
        COMPONENT_LOG_INFO("Thermal:");
        COMPONENT_LOG_INFO("  CPU: {:.1f}°C", thermal_info_.cpu_temperature_c);
        COMPONENT_LOG_INFO("  Battery: {:.1f}°C", thermal_info_.battery_temperature_c);
        COMPONENT_LOG_INFO("  PMU: {:.1f}°C", thermal_info_.pmu_temperature_c);
        COMPONENT_LOG_INFO("  Throttling: {}", thermal_info_.thermal_throttling_active);
        COMPONENT_LOG_INFO("  Warning: {}", thermal_info_.thermal_warning_active);
        
        COMPONENT_LOG_INFO("Statistics:");
        COMPONENT_LOG_INFO("  Total runtime: {:.0f} s", statistics_.total_runtime_seconds);
        COMPONENT_LOG_INFO("  Battery runtime: {:.0f} s", statistics_.battery_runtime_seconds);
        COMPONENT_LOG_INFO("  Charging cycles: {}", statistics_.charging_cycles);
        COMPONENT_LOG_INFO("  Energy consumed: {:.3f} Wh", statistics_.energy_consumed_wh);
        COMPONENT_LOG_INFO("  Energy charged: {:.3f} Wh", statistics_.energy_charged_wh);
        COMPONENT_LOG_INFO("  Average power: {:.1f} mW", statistics_.average_power_consumption_mw);
        COMPONENT_LOG_INFO("  Peak power: {:.1f} mW", statistics_.peak_power_consumption_mw);
        COMPONENT_LOG_INFO("  Thermal events: {}", statistics_.thermal_events);
    }
}

}  // namespace m5tab5::emulator