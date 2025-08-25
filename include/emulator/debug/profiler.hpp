#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <chrono>
#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <functional>
#include <thread>

namespace m5tab5::emulator {

class Profiler {
public:
    struct ProfileEntry {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        std::chrono::high_resolution_clock::time_point end_time;
        std::chrono::nanoseconds duration;
        uint32_t thread_id;
        size_t call_count;
        size_t memory_allocated;
        size_t memory_freed;
        std::string component;
        std::string function;
    };
    
    struct PerformanceMetrics {
        double cpu_usage_percent;
        size_t memory_usage_bytes;
        size_t peak_memory_bytes;
        double operations_per_second;
        double average_latency_ns;
        double min_latency_ns;
        double max_latency_ns;
        size_t total_operations;
        std::chrono::nanoseconds total_execution_time;
        std::unordered_map<std::string, double> component_breakdown;
    };
    
    struct ComponentStats {
        std::string name;
        size_t call_count;
        std::chrono::nanoseconds total_time;
        std::chrono::nanoseconds min_time;
        std::chrono::nanoseconds max_time;
        std::chrono::nanoseconds average_time;
        size_t memory_allocated;
        size_t memory_freed;
        double cpu_percentage;
    };
    
    class ProfileScope {
    public:
        ProfileScope(Profiler& profiler, const std::string& name, 
                    const std::string& component = "", const std::string& function = "");
        ~ProfileScope();
        
        void add_memory_allocation(size_t bytes);
        void add_memory_deallocation(size_t bytes);
        void add_custom_metric(const std::string& key, double value);
        
    private:
        Profiler& profiler_;
        std::string name_;
        std::string component_;
        std::string function_;
        std::chrono::high_resolution_clock::time_point start_time_;
        size_t memory_allocated_{0};
        size_t memory_freed_{0};
        bool finished_{false};
    };
    
    static Result<std::unique_ptr<Profiler>> create();
    
    Result<void> initialize();
    void shutdown();
    
    void start_profiling(const std::string& session_name);
    void stop_profiling();
    void pause_profiling();
    void resume_profiling();
    
    void begin_sample(const std::string& name, const std::string& component = "", 
                     const std::string& function = "");
    void end_sample(const std::string& name);
    
    ProfileScope create_scope(const std::string& name, const std::string& component = "", 
                             const std::string& function = "");
    
    void record_memory_allocation(size_t bytes, const std::string& component = "");
    void record_memory_deallocation(size_t bytes, const std::string& component = "");
    void record_custom_metric(const std::string& name, double value);
    
    Result<PerformanceMetrics> get_performance_metrics() const;
    Result<std::vector<ComponentStats>> get_component_stats() const;
    Result<std::vector<ProfileEntry>> get_profile_entries() const;
    
    Result<void> export_profile_data(const std::string& filename) const;
    Result<void> import_profile_data(const std::string& filename);
    
    void set_sampling_rate(double samples_per_second);
    void set_memory_tracking_enabled(bool enabled);
    void set_component_filter(const std::vector<std::string>& components);
    
    Result<void> start_continuous_monitoring();
    void stop_continuous_monitoring();
    
    using PerformanceCallback = std::function<void(const PerformanceMetrics&)>;
    void set_performance_callback(PerformanceCallback callback);
    
    Result<std::string> generate_performance_report() const;
    Result<void> save_performance_report(const std::string& filename) const;
    
    void reset_statistics();
    bool is_profiling() const { return is_profiling_; }
    
private:
    Profiler() = default;
    
    void update_metrics();
    void process_completed_entries();
    std::string format_duration(std::chrono::nanoseconds duration) const;
    std::string format_memory(size_t bytes) const;
    
    mutable std::mutex entries_mutex_;
    std::vector<ProfileEntry> profile_entries_;
    std::unordered_map<std::string, ProfileEntry> active_samples_;
    
    std::atomic<bool> is_profiling_{false};
    std::atomic<bool> is_paused_{false};
    std::atomic<bool> memory_tracking_enabled_{true};
    std::atomic<double> sampling_rate_{1000.0}; // samples per second
    
    std::string current_session_name_;
    std::chrono::high_resolution_clock::time_point session_start_time_;
    
    std::vector<std::string> component_filter_;
    PerformanceCallback performance_callback_;
    
    mutable std::mutex metrics_mutex_;
    PerformanceMetrics cached_metrics_;
    std::chrono::high_resolution_clock::time_point last_metrics_update_;
    
    std::atomic<size_t> total_memory_allocated_{0};
    std::atomic<size_t> total_memory_freed_{0};
    std::atomic<size_t> peak_memory_usage_{0};
    
    std::thread monitoring_thread_;
    std::atomic<bool> continuous_monitoring_{false};
    
    static thread_local uint32_t thread_id_counter_;
    static uint32_t get_thread_id();
};

// Convenience macros for profiling
#define PROFILE_SCOPE(profiler, name) \
    auto _prof_scope = profiler.create_scope(name, __FILE__, __FUNCTION__)

#define PROFILE_FUNCTION(profiler) \
    auto _prof_scope = profiler.create_scope(__FUNCTION__, __FILE__, __FUNCTION__)

#define PROFILE_COMPONENT(profiler, component, name) \
    auto _prof_scope = profiler.create_scope(name, component, __FUNCTION__)

} // namespace m5tab5::emulator