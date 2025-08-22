#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <chrono>

namespace m5tab5::emulator {

enum class PWMMode : u8 {
    EDGE_ALIGNED = 0,
    CENTER_ALIGNED = 1,
    ASYMMETRIC = 2
};

enum class PWMPolarity : u8 {
    ACTIVE_HIGH = 0,
    ACTIVE_LOW = 1
};

enum class PWMInterruptType : u8 {
    PERIOD_COMPLETE = 0x01,
    COMPARE_MATCH = 0x02,
    UPDATE_EVENT = 0x04,
    TRIGGER_EVENT = 0x08,
    BREAK_EVENT = 0x10,
    FAULT_EVENT = 0x20
};

struct PWMChannel {
    u8 channel_id;
    bool enabled = false;
    u32 duty_cycle = 0;        // 0-10000 (0.01% resolution)
    PWMPolarity polarity = PWMPolarity::ACTIVE_HIGH;
    bool complementary_enabled = false;
    u32 dead_time_ns = 0;
    u32 compare_value = 0;
    bool interrupt_enabled = false;
    u64 pulse_count = 0;
};

struct PWMStatistics {
    u64 periods_generated = 0;
    u64 interrupts_triggered = 0;
    u64 duty_cycle_changes = 0;
    u64 frequency_changes = 0;
    double average_frequency_hz = 0.0;
    double total_runtime_ms = 0.0;
};

class PWMController {
public:
    static constexpr size_t MAX_CHANNELS = 8;
    static constexpr u32 MAX_FREQUENCY = 40000000;  // 40 MHz
    static constexpr u32 MIN_FREQUENCY = 1;         // 1 Hz
    static constexpr u32 DEFAULT_FREQUENCY = 1000;  // 1 kHz
    static constexpr u32 DUTY_CYCLE_MAX = 10000;    // 100.00%
    
    PWMController(u8 controller_id);
    ~PWMController();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    Result<void> configure(u32 frequency_hz, PWMMode mode);
    Result<void> set_period(u32 period_us);
    Result<u32> get_period() const;
    
    Result<void> enable_channel(u8 channel, bool enable);
    Result<void> set_duty_cycle(u8 channel, u32 duty_cycle_percent_x100);
    Result<void> set_compare_value(u8 channel, u32 compare_value);
    Result<void> set_polarity(u8 channel, PWMPolarity polarity);
    Result<void> set_dead_time(u8 channel, u32 dead_time_ns);
    Result<void> enable_complementary(u8 channel, bool enable);
    
    Result<void> start();
    Result<void> stop();
    Result<void> reset();
    
    Result<void> enable_interrupt(u8 channel, PWMInterruptType interrupt_type);
    Result<void> disable_interrupt(u8 channel, PWMInterruptType interrupt_type);
    
    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);

    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_running() const { return running_; }
    u32 get_frequency() const { return frequency_hz_; }
    const PWMChannel& get_channel(u8 channel) const;
    const PWMStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct PWMRegisters {
        u32 control = 0;          // Control register
        u32 status = 0;           // Status register
        u32 interrupt_enable = 0; // Interrupt enable register
        u32 interrupt_status = 0; // Interrupt status register
        u32 prescaler = 0;        // Clock prescaler register
        u32 period = 0;           // Period register
        u32 counter = 0;          // Counter register
        u32 channels[MAX_CHANNELS] = {0}; // Channel control registers
        u32 compare[MAX_CHANNELS] = {0};  // Compare registers
        u32 dead_time = 0;        // Dead time register
        u32 break_control = 0;    // Break and dead-time register
    };

    void update_counter();
    void process_channels();
    void trigger_interrupt(PWMInterruptType interrupt_type, u8 channel = 0xFF);
    void update_status_register();
    u32 calculate_compare_value(u8 channel) const;
    bool is_channel_active(u8 channel) const;

    u8 controller_id_;
    bool initialized_;
    bool running_;
    u32 frequency_hz_;
    PWMMode mode_;
    u32 counter_period_;
    
    PWMRegisters registers_;
    InterruptController* interrupt_controller_;
    
    std::vector<PWMChannel> channels_;
    
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point start_time_;
    u32 current_counter_;
    bool counter_direction_up_;
    
    PWMStatistics statistics_;
    
    mutable std::mutex controller_mutex_;
};

}  // namespace m5tab5::emulator