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
#include <random>

namespace m5tab5::emulator {

enum class ADCResolution : u8 {
    BITS_8 = 8,
    BITS_10 = 10,
    BITS_12 = 12,
    BITS_14 = 14,
    BITS_16 = 16
};

enum class ADCMode : u8 {
    SINGLE_CONVERSION = 0,
    CONTINUOUS = 1,
    SCAN = 2,
    DISCONTINUOUS = 3
};

enum class ADCTrigger : u8 {
    SOFTWARE = 0,
    TIMER = 1,
    EXTERNAL = 2,
    PWM = 3
};

enum class ADCAlignment : u8 {
    RIGHT = 0,
    LEFT = 1
};

enum class ADCInterruptType : u8 {
    CONVERSION_COMPLETE = 0x01,
    END_OF_SEQUENCE = 0x02,
    ANALOG_WATCHDOG = 0x04,
    OVERRUN = 0x08,
    INJECTED_COMPLETE = 0x10,
    FIFO_THRESHOLD = 0x20,
    CALIBRATION_COMPLETE = 0x40,
    ERROR = 0x80
};

struct ADCChannel {
    u8 channel_id;
    bool enabled = false;
    u32 sampling_time = 1;     // Sampling time in ADC clock cycles
    u32 offset = 0;            // Offset value
    float gain = 1.0f;         // Gain multiplier
    u32 threshold_low = 0;     // Analog watchdog low threshold
    u32 threshold_high = 4095; // Analog watchdog high threshold
    bool watchdog_enabled = false;
    u64 conversions_completed = 0;
    u32 last_value = 0;
};

struct ADCConversion {
    u8 channel;
    u32 value;
    std::chrono::steady_clock::time_point timestamp;
    bool is_injected = false;
};

struct ADCStatistics {
    u64 total_conversions = 0;
    u64 regular_conversions = 0;
    u64 injected_conversions = 0;
    u64 overrun_errors = 0;
    u64 watchdog_events = 0;
    u64 calibrations_performed = 0;
    double average_conversion_time_us = 0.0;
    double conversion_rate_sps = 0.0; // Samples per second
};

class ADCController {
public:
    static constexpr size_t MAX_CHANNELS = 16;
    static constexpr size_t MAX_SEQUENCE_LENGTH = 16;
    static constexpr size_t FIFO_SIZE = 64;
    static constexpr u32 MAX_CLOCK_RATE = 40000000; // 40 MHz
    static constexpr u32 MIN_CLOCK_RATE = 1000000;  // 1 MHz
    static constexpr u32 DEFAULT_CLOCK_RATE = 10000000; // 10 MHz
    
    ADCController(u8 controller_id);
    ~ADCController();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    Result<void> configure(ADCResolution resolution, ADCMode mode, ADCAlignment alignment);
    Result<void> set_clock_rate(u32 clock_rate_hz);
    Result<void> set_trigger(ADCTrigger trigger);
    
    Result<void> enable_channel(u8 channel, bool enable);
    Result<void> set_channel_sampling_time(u8 channel, u32 sampling_cycles);
    Result<void> set_channel_offset(u8 channel, u32 offset);
    Result<void> set_channel_gain(u8 channel, float gain);
    
    Result<void> configure_sequence(const std::vector<u8>& channels);
    Result<void> configure_injected_sequence(const std::vector<u8>& channels);
    
    Result<void> enable_analog_watchdog(u8 channel, u32 low_threshold, u32 high_threshold);
    Result<void> disable_analog_watchdog(u8 channel);
    
    Result<void> start_conversion();
    Result<void> stop_conversion();
    Result<u32> read_conversion_result(u8 channel = 0xFF);
    Result<std::vector<ADCConversion>> read_fifo_data(size_t max_count = 0);
    
    Result<void> calibrate();
    Result<bool> is_calibration_complete() const;
    
    Result<void> enable_interrupt(ADCInterruptType interrupt_type);
    Result<void> disable_interrupt(ADCInterruptType interrupt_type);
    
    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);

    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_converting() const { return converting_; }
    ADCResolution get_resolution() const { return resolution_; }
    u32 get_max_value() const;
    const ADCStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct ADCRegisters {
        u32 control1 = 0;         // Control register 1
        u32 control2 = 0;         // Control register 2
        u32 status = 0;           // Status register
        u32 interrupt_enable = 0; // Interrupt enable register
        u32 interrupt_status = 0; // Interrupt status register
        u32 data = 0;             // Data register
        u32 sequence1 = 0;        // Regular sequence register 1
        u32 sequence2 = 0;        // Regular sequence register 2
        u32 sequence3 = 0;        // Regular sequence register 3
        u32 injected_sequence = 0; // Injected sequence register
        u32 offset[4] = {0};      // Offset registers
        u32 watchdog_high = 0;    // Analog watchdog high threshold
        u32 watchdog_low = 0;     // Analog watchdog low threshold
        u32 sampling_time1 = 0;   // Sampling time register 1
        u32 sampling_time2 = 0;   // Sampling time register 2
    };

    void update_conversion();
    void process_regular_sequence();
    void process_injected_sequence();
    void complete_conversion(u8 channel, u32 value, bool is_injected = false);
    void trigger_interrupt(ADCInterruptType interrupt_type);
    void update_status_register();
    u32 simulate_channel_value(u8 channel);
    u32 apply_channel_processing(u8 channel, u32 raw_value);
    u32 calculate_conversion_time_us(u8 channel) const;
    void check_analog_watchdog(u8 channel, u32 value);

    u8 controller_id_;
    bool initialized_;
    bool converting_;
    bool calibrating_;
    ADCResolution resolution_;
    ADCMode mode_;
    ADCAlignment alignment_;
    ADCTrigger trigger_;
    u32 clock_rate_hz_;
    
    ADCRegisters registers_;
    InterruptController* interrupt_controller_;
    
    std::vector<ADCChannel> channels_;
    std::vector<u8> regular_sequence_;
    std::vector<u8> injected_sequence_;
    
    std::queue<ADCConversion> conversion_fifo_;
    
    // Conversion state
    size_t current_sequence_index_;
    size_t current_injected_index_;
    std::chrono::steady_clock::time_point conversion_start_;
    std::chrono::steady_clock::time_point last_conversion_;
    
    // Calibration state
    std::chrono::steady_clock::time_point calibration_start_;
    u32 calibration_duration_us_;
    
    // Simulation state
    std::mt19937 random_generator_;
    std::uniform_real_distribution<float> noise_distribution_;
    
    ADCStatistics statistics_;
    
    mutable std::mutex controller_mutex_;
};

}  // namespace m5tab5::emulator