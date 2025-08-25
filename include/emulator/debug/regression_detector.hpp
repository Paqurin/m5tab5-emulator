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
#include <queue>

namespace emulator::debug {

class RegressionDetector {
public:
    enum class RegressionType {
        PERFORMANCE_DEGRADATION,
        MEMORY_LEAK,
        CPU_SPIKE,
        THROUGHPUT_DROP,
        LATENCY_INCREASE,
        RESOURCE_EXHAUSTION,
        STABILITY_ISSUE
    };
    
    enum class Severity {
        INFO,     // <5% change
        WARNING,  // 5-15% change
        CRITICAL, // 15-30% change
        BLOCKING  // >30% change
    };
    
    struct RegressionAlert {
        RegressionType type;
        Severity severity;
        std::string component;
        std::string metric_name;
        double baseline_value;
        double current_value;
        double change_percent;
        std::string description;
        std::chrono::high_resolution_clock::time_point detected_at;
        std::chrono::high_resolution_clock::time_point first_occurrence;
        size_t occurrence_count;
        bool confirmed;
        bool false_positive;
        std::vector<std::string> related_changes;
        std::string commit_hash;  // If available
        std::string build_version; // If available
    };
    
    struct PerformanceTrend {
        std::string metric_name;
        std::string component;
        std::vector<std::pair<std::chrono::high_resolution_clock::time_point, double>> data_points;
        
        enum class TrendDirection { 
            IMPROVING, 
            STABLE, 
            DEGRADING, 
            VOLATILE 
        } direction;
        
        double trend_slope;        // Rate of change per day
        double volatility;         // Standard deviation
        double confidence;         // 0-1, confidence in trend direction
        std::chrono::milliseconds trend_duration;
        std::string analysis;
    };
    
    struct StatisticalThresholds {
        double mean;
        double standard_deviation;
        double median;
        double percentile_95;
        double percentile_99;
        double min_value;
        double max_value;
        size_t sample_count;
        std::chrono::milliseconds measurement_period;
    };
    
    struct RegressionTest {
        std::string name;
        std::string description;
        std::function<bool()> test_function;
        std::chrono::milliseconds timeout;
        int max_retries;
        bool enabled;
        std::chrono::milliseconds run_interval;
        std::chrono::high_resolution_clock::time_point last_run;
        std::vector<bool> recent_results;  // History of recent test results
    };
    
    static expected<std::unique_ptr<RegressionDetector>, utils::ErrorCode> create(
        std::shared_ptr<Profiler> profiler, std::shared_ptr<ResourceMonitor> resource_monitor);
    
    utils::expected<void, utils::ErrorCode> initialize();
    void shutdown();
    
    // Baseline management
    expected<void, utils::ErrorCode> capture_baseline(const std::string& baseline_name);
    expected<void, utils::ErrorCode> load_baseline(const std::string& baseline_name);
    expected<std::vector<std::string>, utils::ErrorCode> list_baselines() const;
    expected<void, utils::ErrorCode> delete_baseline(const std::string& baseline_name);
    
    // Regression detection
    expected<void, utils::ErrorCode> start_continuous_monitoring();
    void stop_continuous_monitoring();
    
    expected<std::vector<RegressionAlert>, utils::ErrorCode> detect_regressions() const;
    expected<std::vector<RegressionAlert>, utils::ErrorCode> get_active_alerts() const;
    expected<std::vector<RegressionAlert>, utils::ErrorCode> get_alert_history(
        std::chrono::milliseconds duration) const;
    
    // Trend analysis
    expected<std::vector<PerformanceTrend>, utils::ErrorCode> analyze_performance_trends(
        std::chrono::milliseconds duration) const;
    expected<PerformanceTrend, utils::ErrorCode> analyze_metric_trend(
        const std::string& metric_name, const std::string& component,
        std::chrono::milliseconds duration) const;
    
    // Statistical analysis
    expected<StatisticalThresholds, utils::ErrorCode> calculate_statistical_thresholds(
        const std::string& metric_name, const std::string& component,
        std::chrono::milliseconds period) const;
    expected<bool, utils::ErrorCode> is_value_anomalous(
        const std::string& metric_name, const std::string& component, double value) const;
    
    // Custom regression tests
    expected<void, utils::ErrorCode> add_regression_test(const RegressionTest& test);
    expected<void, utils::ErrorCode> remove_regression_test(const std::string& test_name);
    expected<void, utils::ErrorCode> run_regression_tests();
    expected<void, utils::ErrorCode> enable_test(const std::string& test_name, bool enabled);
    
    // Alert management
    expected<void, utils::ErrorCode> acknowledge_alert(const RegressionAlert& alert);
    expected<void, utils::ErrorCode> mark_false_positive(const RegressionAlert& alert);
    expected<void, utils::ErrorCode> suppress_alerts_for_component(
        const std::string& component, std::chrono::milliseconds duration);
    
    // Configuration
    void set_regression_thresholds(double warning_percent, double critical_percent, double blocking_percent);
    void set_detection_sensitivity(double sensitivity);  // 0-1, higher = more sensitive
    void set_minimum_sample_size(size_t min_samples);
    void set_statistical_confidence(double confidence);  // 0-1, typically 0.95
    void enable_auto_baseline_update(bool enabled);
    
    // Reporting
    expected<std::string, utils::ErrorCode> generate_regression_report() const;
    expected<std::string, utils::ErrorCode> generate_trend_report(
        std::chrono::milliseconds duration) const;
    expected<void, utils::ErrorCode> export_regression_data(const std::string& filename) const;
    
    // Callbacks
    using RegressionCallback = std::function<void(const RegressionAlert&)>;
    using TrendCallback = std::function<void(const PerformanceTrend&)>;
    
    void set_regression_callback(RegressionCallback callback);
    void set_trend_callback(TrendCallback callback);
    
    // Integration with CI/CD
    expected<int, utils::ErrorCode> get_regression_exit_code() const;  // 0 = no issues, >0 = issues
    expected<void, utils::ErrorCode> set_build_metadata(const std::string& commit_hash, 
                                                       const std::string& build_version);
    
    bool is_monitoring() const { return is_monitoring_; }
    
private:
    RegressionDetector(std::shared_ptr<Profiler> profiler, 
                      std::shared_ptr<ResourceMonitor> resource_monitor);
    
    void monitoring_loop();
    void run_regression_tests_loop();
    
    // Detection algorithms
    bool detect_performance_regression(const std::string& metric_name, double current_value,
                                     const StatisticalThresholds& thresholds) const;
    bool detect_memory_leak(const std::vector<std::pair<std::chrono::high_resolution_clock::time_point, size_t>>& memory_data) const;
    bool detect_throughput_drop(const std::vector<double>& throughput_data) const;
    
    Severity calculate_severity(double change_percent) const;
    RegressionType classify_regression(const std::string& metric_name, double change_percent) const;
    
    // Trend analysis helpers
    double calculate_trend_slope(const std::vector<std::pair<std::chrono::high_resolution_clock::time_point, double>>& data) const;
    double calculate_volatility(const std::vector<double>& values) const;
    PerformanceTrend::TrendDirection classify_trend(double slope, double volatility) const;
    
    // Statistical helpers
    StatisticalThresholds calculate_thresholds(const std::vector<double>& values) const;
    double calculate_z_score(double value, double mean, double std_dev) const;
    bool is_outlier(double value, const StatisticalThresholds& thresholds, double confidence) const;
    
    // Data management
    void update_metric_history(const std::string& metric_name, const std::string& component, double value);
    void cleanup_old_data();
    
    std::string format_alert(const RegressionAlert& alert) const;
    std::string format_trend(const PerformanceTrend& trend) const;
    
    std::shared_ptr<Profiler> profiler_;
    std::shared_ptr<ResourceMonitor> resource_monitor_;
    
    // Monitoring state
    std::atomic<bool> is_monitoring_{false};
    std::thread monitoring_thread_;
    std::thread test_runner_thread_;
    std::chrono::milliseconds monitoring_interval_{std::chrono::seconds(60)};
    
    // Thresholds and configuration
    double warning_threshold_percent_{5.0};
    double critical_threshold_percent_{15.0};
    double blocking_threshold_percent_{30.0};
    double detection_sensitivity_{0.8};
    size_t minimum_sample_size_{10};
    double statistical_confidence_{0.95};
    bool auto_baseline_update_enabled_{false};
    
    // Data storage
    mutable std::mutex data_mutex_;
    std::unordered_map<std::string, StatisticalThresholds> baselines_;
    std::unordered_map<std::string, std::vector<std::pair<std::chrono::high_resolution_clock::time_point, double>>> metric_history_;
    
    // Alerts and trends
    mutable std::mutex alerts_mutex_;
    std::vector<RegressionAlert> active_alerts_;
    std::vector<RegressionAlert> alert_history_;
    std::vector<PerformanceTrend> detected_trends_;
    
    // Regression tests
    mutable std::mutex tests_mutex_;
    std::unordered_map<std::string, RegressionTest> regression_tests_;
    
    // Suppression management
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> suppressed_components_;
    
    // Callbacks
    RegressionCallback regression_callback_;
    TrendCallback trend_callback_;
    
    // Build metadata
    std::string current_commit_hash_;
    std::string current_build_version_;
    
    // Constants
    static constexpr std::chrono::hours DATA_RETENTION_PERIOD{24 * 7}; // 1 week
    static constexpr size_t MAX_HISTORY_SIZE{10000};
};

} // namespace emulator::debug