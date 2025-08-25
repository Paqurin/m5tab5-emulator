#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/debug/profiler.hpp"
#include "emulator/debug/resource_monitor.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <functional>

namespace emulator::debug {

class OptimizationAnalyzer {
public:
    enum class OptimizationType {
        CPU_OPTIMIZATION,
        MEMORY_OPTIMIZATION,
        IO_OPTIMIZATION,
        CACHE_OPTIMIZATION,
        THREAD_OPTIMIZATION,
        ALGORITHM_OPTIMIZATION
    };
    
    struct PerformanceBottleneck {
        std::string component;
        std::string function;
        OptimizationType type;
        double severity_score;  // 0-100, higher is worse
        std::string description;
        std::chrono::nanoseconds total_time;
        size_t call_count;
        double cpu_percentage;
        size_t memory_usage;
        std::vector<std::string> recommendations;
        std::chrono::high_resolution_clock::time_point detected_at;
    };
    
    struct OptimizationOpportunity {
        std::string component;
        OptimizationType type;
        double potential_improvement_percent;
        std::string description;
        std::vector<std::string> action_items;
        enum class Priority { LOW, MEDIUM, HIGH, CRITICAL } priority;
        size_t estimated_effort_hours;
        double confidence_score;  // 0-1, confidence in the recommendation
    };
    
    struct PerformanceComparison {
        std::string baseline_session;
        std::string current_session;
        std::chrono::high_resolution_clock::time_point comparison_time;
        
        struct ComponentComparison {
            std::string component;
            double baseline_cpu_percent;
            double current_cpu_percent;
            double cpu_change_percent;
            std::chrono::nanoseconds baseline_avg_time;
            std::chrono::nanoseconds current_avg_time;
            double time_change_percent;
            size_t baseline_memory;
            size_t current_memory;
            double memory_change_percent;
            enum class Trend { IMPROVED, DEGRADED, STABLE } trend;
        };
        
        std::vector<ComponentComparison> component_comparisons;
        double overall_performance_change_percent;
        std::vector<std::string> significant_changes;
    };
    
    struct CacheAnalysis {
        std::string component;
        double hit_rate;
        double miss_rate;
        size_t total_accesses;
        size_t hits;
        size_t misses;
        std::chrono::nanoseconds average_hit_time;
        std::chrono::nanoseconds average_miss_time;
        size_t cache_size_bytes;
        double efficiency_score;  // 0-100
        std::vector<std::string> recommendations;
    };
    
    struct AlgorithmicComplexity {
        std::string function;
        std::string component;
        
        enum class ComplexityClass {
            CONSTANT,      // O(1)
            LOGARITHMIC,   // O(log n)
            LINEAR,        // O(n)
            LINEARITHMIC,  // O(n log n)
            QUADRATIC,     // O(n²)
            CUBIC,         // O(n³)
            EXPONENTIAL,   // O(2^n)
            UNKNOWN
        } estimated_complexity;
        
        double confidence;
        size_t sample_size;
        std::vector<std::pair<size_t, std::chrono::nanoseconds>> data_points;  // (input_size, execution_time)
        std::string analysis_description;
        std::vector<std::string> optimization_suggestions;
    };
    
    static expected<std::unique_ptr<OptimizationAnalyzer>, utils::ErrorCode> create(
        std::shared_ptr<Profiler> profiler, std::shared_ptr<ResourceMonitor> resource_monitor);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Bottleneck detection
    expected<std::vector<PerformanceBottleneck>, utils::ErrorCode> detect_bottlenecks(
        double severity_threshold = 10.0) const;
    expected<std::vector<OptimizationOpportunity>, utils::ErrorCode> identify_opportunities() const;
    
    // Performance comparison
    expected<void, utils::ErrorCode> create_performance_baseline(const std::string& session_name);
    expected<PerformanceComparison, utils::ErrorCode> compare_with_baseline(
        const std::string& current_session) const;
    expected<std::vector<PerformanceComparison>, utils::ErrorCode> get_performance_trends(
        std::chrono::milliseconds duration) const;
    
    // Cache analysis
    expected<std::vector<CacheAnalysis>, utils::ErrorCode> analyze_cache_performance() const;
    expected<void, utils::ErrorCode> simulate_cache_configurations(
        const std::vector<size_t>& cache_sizes) const;
    
    // Algorithmic complexity analysis
    expected<std::vector<AlgorithmicComplexity>, utils::ErrorCode> analyze_algorithmic_complexity() const;
    expected<void, utils::ErrorCode> profile_function_complexity(
        const std::string& function_name, const std::string& component,
        std::function<void(size_t)> test_function, const std::vector<size_t>& input_sizes);
    
    // Thread and concurrency analysis
    expected<void, utils::ErrorCode> analyze_thread_contention() const;
    expected<void, utils::ErrorCode> analyze_lock_contention() const;
    expected<std::vector<std::string>, utils::ErrorCode> suggest_parallelization_opportunities() const;
    
    // Memory analysis
    expected<void, utils::ErrorCode> analyze_memory_patterns() const;
    expected<std::vector<std::string>, utils::ErrorCode> detect_memory_leaks() const;
    expected<void, utils::ErrorCode> analyze_allocation_patterns() const;
    
    // I/O analysis
    expected<void, utils::ErrorCode> analyze_io_patterns() const;
    expected<std::vector<std::string>, utils::ErrorCode> suggest_io_optimizations() const;
    
    // Report generation
    expected<std::string, utils::ErrorCode> generate_optimization_report() const;
    expected<std::string, utils::ErrorCode> generate_bottleneck_report() const;
    expected<std::string, utils::ErrorCode> generate_comparison_report(
        const PerformanceComparison& comparison) const;
    
    expected<void, utils::ErrorCode> export_analysis_data(const std::string& filename) const;
    expected<void, utils::ErrorCode> import_analysis_data(const std::string& filename);
    
    // Configuration
    void set_bottleneck_thresholds(double cpu_threshold, double memory_threshold,
                                  std::chrono::nanoseconds time_threshold);
    void set_optimization_priorities(const std::vector<OptimizationType>& priorities);
    void enable_continuous_analysis(bool enabled);
    void set_analysis_interval(std::chrono::milliseconds interval);
    
    // Callbacks
    using BottleneckCallback = std::function<void(const PerformanceBottleneck&)>;
    using OpportunityCallback = std::function<void(const OptimizationOpportunity&)>;
    
    void set_bottleneck_callback(BottleneckCallback callback);
    void set_opportunity_callback(OpportunityCallback callback);
    
private:
    OptimizationAnalyzer(std::shared_ptr<Profiler> profiler, 
                        std::shared_ptr<ResourceMonitor> resource_monitor);
    
    void continuous_analysis_loop();
    
    // Analysis helpers
    double calculate_severity_score(const Profiler::ComponentStats& stats,
                                   const ResourceMonitor::SystemMetrics& metrics) const;
    OptimizationType classify_bottleneck(const Profiler::ComponentStats& stats) const;
    std::vector<std::string> generate_recommendations(const PerformanceBottleneck& bottleneck) const;
    
    AlgorithmicComplexity::ComplexityClass estimate_complexity(
        const std::vector<std::pair<size_t, std::chrono::nanoseconds>>& data_points) const;
    double calculate_goodness_of_fit(const std::vector<std::pair<size_t, std::chrono::nanoseconds>>& data,
                                   AlgorithmicComplexity::ComplexityClass complexity) const;
    
    std::string format_bottleneck(const PerformanceBottleneck& bottleneck) const;
    std::string format_opportunity(const OptimizationOpportunity& opportunity) const;
    std::string format_comparison(const PerformanceComparison::ComponentComparison& comp) const;
    
    std::shared_ptr<Profiler> profiler_;
    std::shared_ptr<ResourceMonitor> resource_monitor_;
    
    // Configuration
    double cpu_threshold_{20.0};        // CPU usage threshold for bottleneck detection
    double memory_threshold_{100*1024*1024}; // 100MB memory threshold
    std::chrono::nanoseconds time_threshold_{std::chrono::milliseconds(10)}; // 10ms time threshold
    
    std::vector<OptimizationType> optimization_priorities_{
        OptimizationType::CPU_OPTIMIZATION,
        OptimizationType::MEMORY_OPTIMIZATION,
        OptimizationType::CACHE_OPTIMIZATION,
        OptimizationType::ALGORITHM_OPTIMIZATION,
        OptimizationType::THREAD_OPTIMIZATION,
        OptimizationType::IO_OPTIMIZATION
    };
    
    std::atomic<bool> continuous_analysis_enabled_{false};
    std::chrono::milliseconds analysis_interval_{std::chrono::seconds(30)};
    std::thread analysis_thread_;
    
    // Callbacks
    BottleneckCallback bottleneck_callback_;
    OpportunityCallback opportunity_callback_;
    
    // Baseline storage
    mutable std::mutex baseline_mutex_;
    std::unordered_map<std::string, std::pair<Profiler::PerformanceMetrics, 
                                            std::vector<Profiler::ComponentStats>>> baselines_;
    
    // Analysis history
    mutable std::mutex history_mutex_;
    std::vector<PerformanceComparison> comparison_history_;
    std::vector<PerformanceBottleneck> detected_bottlenecks_;
    std::vector<OptimizationOpportunity> identified_opportunities_;
};

} // namespace emulator::debug