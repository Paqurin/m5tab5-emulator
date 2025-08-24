#pragma once

#include "emulator/gui/emulator_gui.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <vector>
#include <map>
#include <functional>

namespace m5tab5::emulator::gui {

class MainWindow;

/**
 * @brief Base class for dockable GUI panels
 */
class DockWidget {
public:
    explicit DockWidget(MainWindow& parent, const std::string& title);
    virtual ~DockWidget() = default;

    virtual void render() = 0;
    virtual void update() {}

    const std::string& get_title() const { return title_; }
    void set_title(const std::string& title) { title_ = title; }
    
    bool is_visible() const { return visible_; }
    void set_visible(bool visible) { visible_ = visible; }

protected:
    MainWindow& parent_;
    std::string title_;
    bool visible_ = true;
};

/**
 * @brief Control panel for emulator power state and basic operations
 */
class ControlPanel : public DockWidget {
public:
    explicit ControlPanel(MainWindow& parent);
    ~ControlPanel() override = default;

    void render() override;
    void update() override;

private:
    // State tracking
    bool emulator_running_ = false;
    bool firmware_loaded_ = false;
    float execution_speed_ = 1.0f;
    u64 cycles_executed_ = 0;
    
    // UI elements
    void render_power_controls();
    void render_execution_controls();
    void render_statistics();
    
    // Actions
    void handle_start_stop();
    void handle_pause_resume();
    void handle_reset();
    void handle_speed_change(float new_speed);
};

/**
 * @brief Firmware management with ELF loading and metadata display
 */
class FirmwareManager : public DockWidget {
public:
    struct FirmwareInfo {
        std::string path;
        std::string filename;
        u64 file_size = 0;
        std::string build_date;
        std::string version;
        u32 entry_point = 0;
        std::vector<std::string> sections;
        bool loaded = false;
    };

    explicit FirmwareManager(MainWindow& parent);
    ~FirmwareManager() override = default;

    void render() override;
    void update() override;

    Result<void> load_firmware(const std::string& path);
    void clear_firmware();

private:
    FirmwareInfo current_firmware_;
    std::vector<std::string> recent_files_;
    bool show_file_dialog_ = false;
    
    void render_load_controls();
    void render_firmware_info();
    void render_recent_files();
    void render_file_dialog();
    
    void add_to_recent(const std::string& path);
    void load_recent_files();
    void save_recent_files();
    
    Result<FirmwareInfo> parse_elf_file(const std::string& path);
};

/**
 * @brief GPIO pin visualization and control interface
 */
class GPIOViewer : public DockWidget {
public:
    enum class PinMode {
        Input, Output, InputPullUp, InputPullDown, 
        OpenDrain, AnalogInput, PWM
    };

    struct PinState {
        PinMode mode = PinMode::Input;
        bool level = false;
        u16 pwm_duty = 0;     // 0-1023
        u16 analog_value = 0;  // 0-4095
        bool interrupt_enabled = false;
        bool has_interrupt = false;
    };

    explicit GPIOViewer(MainWindow& parent);
    ~GPIOViewer() override = default;

    void render() override;
    void update() override;

private:
    static constexpr u32 GPIO_PIN_COUNT = 55; // ESP32-P4
    std::array<PinState, GPIO_PIN_COUNT> pin_states_;
    
    // UI state
    u32 selected_pin_ = 0;
    bool show_analog_pins_only_ = false;
    bool show_pwm_pins_only_ = false;
    std::string pin_filter_;
    
    void render_pin_grid();
    void render_pin_details();
    void render_pin_controls();
    void render_filters();
    
    void render_pin_button(u32 pin);
    void handle_pin_click(u32 pin);
    void handle_pin_mode_change(u32 pin, PinMode mode);
    void handle_pin_value_change(u32 pin, bool value);
    
    u32 get_pin_color(const PinState& state) const;
    const char* get_pin_mode_name(PinMode mode) const;
};

/**
 * @brief CPU monitoring with core utilization and performance metrics
 */
class CPUMonitor : public DockWidget {
public:
    struct CoreStats {
        float utilization = 0.0f;        // 0-100%
        u32 frequency = 400000000;       // Hz
        u64 instructions_executed = 0;
        u64 cache_hits = 0;
        u64 cache_misses = 0;
        u32 pipeline_stalls = 0;
    };

    explicit CPUMonitor(MainWindow& parent);
    ~CPUMonitor() override = default;

    void render() override;
    void update() override;

private:
    std::array<CoreStats, 2> core_stats_; // Dual-core ESP32-P4
    std::vector<float> utilization_history_[2]; // For graphs
    static constexpr u32 HISTORY_SIZE = 100;
    
    void render_core_overview();
    void render_core_details(u32 core_id);
    void render_utilization_graph(u32 core_id);
    void render_cache_stats();
    void render_performance_metrics();
    
    void update_statistics();
    float calculate_cache_hit_rate(u32 core_id) const;
    u32 calculate_ipc(u32 core_id) const; // Instructions per cycle
};

/**
 * @brief Memory usage visualization and inspector
 */
class MemoryViewer : public DockWidget {
public:
    enum class MemoryRegion {
        Flash,   // 16MB Flash
        PSRAM,   // 32MB PSRAM  
        SRAM,    // 768KB SRAM
        Cache    // L1/L2 Cache
    };

    struct MemoryStats {
        u64 total_size = 0;
        u64 used_size = 0;
        u64 free_size = 0;
        u32 allocations = 0;
        u32 deallocations = 0;
        float fragmentation = 0.0f;
    };

    explicit MemoryViewer(MainWindow& parent);
    ~MemoryViewer() override = default;

    void render() override;
    void update() override;

private:
    std::map<MemoryRegion, MemoryStats> memory_stats_;
    MemoryRegion selected_region_ = MemoryRegion::SRAM;
    
    // Memory map visualization
    u32 memory_map_address_ = 0x4FF00000; // SRAM start
    u32 bytes_per_row_ = 16;
    u32 visible_rows_ = 16;
    
    void render_memory_overview();
    void render_memory_bars();
    void render_region_selector();
    void render_memory_map();
    void render_hex_view();
    
    void update_memory_statistics();
    u32 get_region_color(MemoryRegion region, float usage) const;
    std::string format_memory_size(u64 size) const;
    void handle_address_change(u32 new_address);
};

/**
 * @brief Peripheral status monitoring (I2C, SPI, UART, etc.)
 */
class PeripheralStatus : public DockWidget {
public:
    enum class PeripheralType {
        I2C, SPI, UART, PWM, ADC, DAC, Timer, DMA
    };

    struct PeripheralInfo {
        PeripheralType type;
        std::string name;
        bool enabled = false;
        bool active = false;
        u32 transactions = 0;
        std::string status_text;
        u32 status_color = 0x757575; // Gray
    };

    explicit PeripheralStatus(MainWindow& parent);
    ~PeripheralStatus() override = default;

    void render() override;
    void update() override;

private:
    std::vector<PeripheralInfo> peripherals_;
    PeripheralType selected_type_ = PeripheralType::I2C;
    
    void render_peripheral_list();
    void render_peripheral_details();
    void render_activity_indicators();
    
    void update_peripheral_status();
    void setup_peripheral_list();
    u32 get_status_color(const PeripheralInfo& info) const;
    const char* get_peripheral_type_name(PeripheralType type) const;
};

/**
 * @brief Integrated log viewer with filtering and search
 */
class LogViewer : public DockWidget {
public:
    enum class LogLevel {
        Trace, Debug, Info, Warn, Error
    };

    struct LogEntry {
        u64 timestamp;
        LogLevel level;
        std::string component;
        std::string message;
        u32 thread_id = 0;
    };

    explicit LogViewer(MainWindow& parent);
    ~LogViewer() override = default;

    void render() override;
    void update() override;

    void add_log_entry(const LogEntry& entry);
    void clear_logs();

private:
    std::vector<LogEntry> log_entries_;
    static constexpr u32 MAX_LOG_ENTRIES = 10000;
    
    // Filtering
    LogLevel min_level_ = LogLevel::Info;
    std::string component_filter_;
    std::string search_text_;
    bool auto_scroll_ = true;
    bool word_wrap_ = false;
    
    void render_controls();
    void render_log_list();
    void render_entry(const LogEntry& entry);
    
    bool should_show_entry(const LogEntry& entry) const;
    u32 get_level_color(LogLevel level) const;
    const char* get_level_name(LogLevel level) const;
    std::string format_timestamp(u64 timestamp) const;
};

} // namespace m5tab5::emulator::gui