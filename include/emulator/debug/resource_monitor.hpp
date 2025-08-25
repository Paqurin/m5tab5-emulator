#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <unordered_map>

namespace emulator::debug {

class ResourceMonitor {
public:
    struct SystemMetrics {
        double cpu_usage_percent;
        size_t memory_usage_bytes;
        size_t memory_available_bytes;
        size_t virtual_memory_usage_bytes;
        size_t resident_memory_usage_bytes;
        double memory_usage_percent;
        
        uint64_t context_switches;
        uint64_t page_faults;
        uint64_t voluntary_context_switches;
        uint64_t involuntary_context_switches;
        
        size_t open_file_descriptors;
        size_t max_file_descriptors;
        
        uint32_t thread_count;
        double load_average_1min;
        double load_average_5min;
        double load_average_15min;
        
        std::chrono::high_resolution_clock::time_point timestamp;
    };
    
    struct ProcessMetrics {
        uint32_t pid;
        std::string name;
        double cpu_usage_percent;
        size_t memory_usage_bytes;
        size_t virtual_memory_bytes;
        size_t resident_memory_bytes;
        uint32_t thread_count;
        uint64_t context_switches;
        std::chrono::milliseconds cpu_time;
        std::chrono::high_resolution_clock::time_point timestamp;
    };
    
    struct ThreadMetrics {
        uint32_t tid;
        std::string name;
        double cpu_usage_percent;
        std::chrono::milliseconds cpu_time;
        std::chrono::milliseconds user_time;
        std::chrono::milliseconds system_time;
        uint64_t context_switches;
        int priority;
        std::string state;
        std::chrono::high_resolution_clock::time_point timestamp;
    };
    
    struct NetworkMetrics {
        uint64_t bytes_sent;
        uint64_t bytes_received;
        uint64_t packets_sent;
        uint64_t packets_received;
        uint64_t errors_sent;
        uint64_t errors_received;
        uint64_t drops_sent;
        uint64_t drops_received;
        std::chrono::high_resolution_clock::time_point timestamp;
    };
    
    struct DiskMetrics {
        uint64_t read_bytes;
        uint64_t write_bytes;
        uint64_t read_operations;
        uint64_t write_operations;
        std::chrono::milliseconds read_time;
        std::chrono::milliseconds write_time;
        double disk_usage_percent;
        size_t available_space_bytes;
        std::chrono::high_resolution_clock::time_point timestamp;
    };
    
    struct PerformanceAlert {
        enum class Type {
            CPU_HIGH,
            MEMORY_HIGH,
            DISK_FULL,
            THREAD_LIMIT,
            CONTEXT_SWITCH_HIGH,
            CUSTOM
        };
        
        Type type;
        std::string message;
        double threshold;
        double current_value;
        std::chrono::high_resolution_clock::time_point timestamp;
        bool resolved;
    };
    
    using MetricsCallback = std::function<void(const SystemMetrics&)>;
    using AlertCallback = std::function<void(const PerformanceAlert&)>;
    
    static expected<std::unique_ptr<ResourceMonitor>, utils::ErrorCode> create();
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    expected<void, utils::ErrorCode> start_monitoring();
    void stop_monitoring();
    
    void set_sampling_interval(std::chrono::milliseconds interval);
    void set_metrics_callback(MetricsCallback callback);
    void set_alert_callback(AlertCallback callback);
    
    expected<SystemMetrics, utils::ErrorCode> get_current_system_metrics() const;
    expected<ProcessMetrics, utils::ErrorCode> get_current_process_metrics() const;
    expected<std::vector<ThreadMetrics>, utils::ErrorCode> get_current_thread_metrics() const;
    expected<NetworkMetrics, utils::ErrorCode> get_current_network_metrics() const;
    expected<DiskMetrics, utils::ErrorCode> get_current_disk_metrics() const;
    
    expected<std::vector<SystemMetrics>, utils::ErrorCode> get_system_history(
        std::chrono::milliseconds duration) const;
    expected<std::vector<ProcessMetrics>, utils::ErrorCode> get_process_history(
        std::chrono::milliseconds duration) const;
    
    void set_cpu_alert_threshold(double percent);
    void set_memory_alert_threshold(double percent);
    void set_disk_alert_threshold(double percent);
    void set_thread_count_alert_threshold(uint32_t count);
    void set_context_switch_alert_threshold(uint64_t switches_per_second);
    
    void add_custom_alert(const std::string& name, double threshold, 
                         std::function<double()> value_getter);
    void remove_custom_alert(const std::string& name);
    
    expected<std::vector<PerformanceAlert>, utils::ErrorCode> get_active_alerts() const;
    expected<std::vector<PerformanceAlert>, utils::ErrorCode> get_alert_history(
        std::chrono::milliseconds duration) const;
    
    void acknowledge_alert(const PerformanceAlert& alert);
    void clear_alert_history();
    
    expected<void, utils::ErrorCode> export_metrics(const std::string& filename,
        std::chrono::milliseconds duration) const;
    expected<std::string, utils::ErrorCode> generate_resource_report(
        std::chrono::milliseconds duration) const;
    
    void reset_counters();
    bool is_monitoring() const { return is_monitoring_; }
    
private:
    ResourceMonitor() = default;
    
    void monitoring_loop();
    void check_alerts(const SystemMetrics& metrics);
    void trigger_alert(PerformanceAlert::Type type, const std::string& message,
                      double threshold, double current_value);
    
    SystemMetrics collect_system_metrics() const;
    ProcessMetrics collect_process_metrics() const;
    std::vector<ThreadMetrics> collect_thread_metrics() const;
    NetworkMetrics collect_network_metrics() const;
    DiskMetrics collect_disk_metrics() const;
    
    double calculate_cpu_usage() const;
    size_t get_memory_usage() const;
    size_t get_available_memory() const;
    uint32_t get_thread_count() const;
    std::vector<double> get_load_averages() const;
    
    std::string format_metrics(const SystemMetrics& metrics) const;
    std::string format_alert(const PerformanceAlert& alert) const;
    
    mutable std::mutex metrics_mutex_;
    std::vector<SystemMetrics> system_metrics_history_;
    std::vector<ProcessMetrics> process_metrics_history_;
    
    mutable std::mutex alerts_mutex_;
    std::vector<PerformanceAlert> active_alerts_;
    std::vector<PerformanceAlert> alert_history_;
    
    std::atomic<bool> is_monitoring_{false};
    std::thread monitoring_thread_;
    std::chrono::milliseconds sampling_interval_{std::chrono::milliseconds(1000)};
    
    MetricsCallback metrics_callback_;
    AlertCallback alert_callback_;
    
    // Alert thresholds
    std::atomic<double> cpu_alert_threshold_{80.0};
    std::atomic<double> memory_alert_threshold_{90.0};
    std::atomic<double> disk_alert_threshold_{95.0};
    std::atomic<uint32_t> thread_count_alert_threshold_{1000};
    std::atomic<uint64_t> context_switch_alert_threshold_{10000};
    
    // Custom alerts
    struct CustomAlert {
        std::string name;
        double threshold;
        std::function<double()> value_getter;
    };
    std::vector<CustomAlert> custom_alerts_;
    
    // Baseline metrics for delta calculations
    mutable SystemMetrics last_system_metrics_;
    mutable std::chrono::high_resolution_clock::time_point last_sample_time_;
    
    // Platform-specific data
    uint32_t process_id_;
    size_t page_size_;
    long clock_ticks_per_second_;
};

} // namespace emulator::debug