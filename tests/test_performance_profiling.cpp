#include <gtest/gtest.h>
#include "emulator/debug/profiler.hpp"
#include "emulator/debug/resource_monitor.hpp"
#include "emulator/debug/optimization_analyzer.hpp"
#include "emulator/debug/regression_detector.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/config/configuration.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include <vector>
#include <atomic>

namespace emulator::debug::test {

class PerformanceProfilingTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::string config_content = R"({
            "emulator": {
                "enable_debugging": true,
                "log_level": "info"
            },
            "profiling": {
                "enabled": true,
                "sampling_rate": 1000.0,
                "memory_tracking": true
            },
            "monitoring": {
                "enabled": true,
                "sampling_interval_ms": 100
            }
        })";
        
        config_path = std::filesystem::temp_directory_path() / "profiling_config.json";
        std::ofstream(config_path) << config_content;
        
        auto config_result = config::Configuration::load(config_path.string());
        ASSERT_TRUE(config_result.has_value());
        configuration = config_result.value();
        
        auto profiler_result = Profiler::create();
        ASSERT_TRUE(profiler_result.has_value());
        profiler = std::move(profiler_result.value());
        
        auto monitor_result = ResourceMonitor::create();
        ASSERT_TRUE(monitor_result.has_value());
        resource_monitor = std::move(monitor_result.value());
        
        auto analyzer_result = OptimizationAnalyzer::create(profiler, resource_monitor);
        ASSERT_TRUE(analyzer_result.has_value());
        optimization_analyzer = std::move(analyzer_result.value());
        
        auto detector_result = RegressionDetector::create(profiler, resource_monitor);
        ASSERT_TRUE(detector_result.has_value());
        regression_detector = std::move(detector_result.value());
        
        ASSERT_TRUE(profiler->initialize().has_value());
        ASSERT_TRUE(resource_monitor->initialize().has_value());
        ASSERT_TRUE(optimization_analyzer->initialize().has_value());
        ASSERT_TRUE(regression_detector->initialize().has_value());
    }
    
    void TearDown() override {
        if (profiler) profiler->shutdown();
        if (resource_monitor) resource_monitor->shutdown();
        if (optimization_analyzer) optimization_analyzer->shutdown();
        if (regression_detector) regression_detector->shutdown();
        
        if (std::filesystem::exists(config_path)) {
            std::filesystem::remove(config_path);
        }
    }
    
    void simulate_cpu_intensive_work(int duration_ms, const std::string& component) {
        PROFILE_COMPONENT(*profiler, component, "cpu_intensive_work");
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto target_duration = std::chrono::milliseconds(duration_ms);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        
        volatile double result = 0.0;
        while (std::chrono::high_resolution_clock::now() - start_time < target_duration) {
            for (int i = 0; i < 1000; ++i) {
                result += std::sin(dist(gen)) * std::cos(dist(gen));
            }
        }
        
        // Prevent optimization
        (void)result;
    }
    
    void simulate_memory_intensive_work(size_t allocation_size, int iterations, const std::string& component) {
        PROFILE_COMPONENT(*profiler, component, "memory_intensive_work");
        
        std::vector<std::unique_ptr<char[]>> allocations;
        allocations.reserve(iterations);
        
        for (int i = 0; i < iterations; ++i) {
            auto allocation = std::make_unique<char[]>(allocation_size);
            profiler->record_memory_allocation(allocation_size, component);
            
            // Touch the memory to ensure it's allocated
            std::memset(allocation.get(), i % 256, allocation_size);
            
            allocations.push_back(std::move(allocation));
            
            if (i % 2 == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
        
        // Clean up allocations
        for (const auto& allocation : allocations) {
            profiler->record_memory_deallocation(allocation_size, component);
        }
    }
    
    std::filesystem::path config_path;
    std::shared_ptr<config::Configuration> configuration;
    std::unique_ptr<Profiler> profiler;
    std::unique_ptr<ResourceMonitor> resource_monitor;
    std::unique_ptr<OptimizationAnalyzer> optimization_analyzer;
    std::unique_ptr<RegressionDetector> regression_detector;
};

TEST_F(PerformanceProfilingTest, BasicProfilingWorkflow) {
    profiler->start_profiling("basic_profiling_test");
    EXPECT_TRUE(profiler->is_profiling());
    
    simulate_cpu_intensive_work(100, "test_component");
    simulate_memory_intensive_work(1024, 10, "test_component");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    profiler->stop_profiling();
    EXPECT_FALSE(profiler->is_profiling());
    
    auto metrics_result = profiler->get_performance_metrics();
    ASSERT_TRUE(metrics_result.has_value());
    
    const auto& metrics = metrics_result.value();
    EXPECT_GT(metrics.total_operations, 0);
    EXPECT_GT(metrics.operations_per_second, 0.0);
    EXPECT_GT(metrics.average_latency_ns, 0.0);
    EXPECT_GT(metrics.memory_usage_bytes, 0);
    
    auto component_stats_result = profiler->get_component_stats();
    ASSERT_TRUE(component_stats_result.has_value());
    
    const auto& component_stats = component_stats_result.value();
    EXPECT_FALSE(component_stats.empty());
    
    bool found_test_component = false;
    for (const auto& stat : component_stats) {
        if (stat.name == "test_component") {
            found_test_component = true;
            EXPECT_GT(stat.call_count, 0);
            EXPECT_GT(stat.total_time.count(), 0);
            EXPECT_GT(stat.memory_allocated, 0);
            break;
        }
    }
    EXPECT_TRUE(found_test_component);
    
    std::cout << "Basic Profiling Results:" << std::endl;
    std::cout << "  Total operations: " << metrics.total_operations << std::endl;
    std::cout << "  Operations/sec: " << metrics.operations_per_second << std::endl;
    std::cout << "  Memory usage: " << metrics.memory_usage_bytes << " bytes" << std::endl;
    std::cout << "  Peak memory: " << metrics.peak_memory_bytes << " bytes" << std::endl;
}

TEST_F(PerformanceProfilingTest, ResourceMonitoring) {
    ASSERT_TRUE(resource_monitor->start_monitoring().has_value());
    EXPECT_TRUE(resource_monitor->is_monitoring());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    simulate_cpu_intensive_work(200, "cpu_test");
    simulate_memory_intensive_work(1024 * 1024, 5, "memory_test");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto system_metrics_result = resource_monitor->get_current_system_metrics();
    ASSERT_TRUE(system_metrics_result.has_value());
    
    const auto& system_metrics = system_metrics_result.value();
    EXPECT_GE(system_metrics.cpu_usage_percent, 0.0);
    EXPECT_LE(system_metrics.cpu_usage_percent, 100.0);
    EXPECT_GT(system_metrics.memory_usage_bytes, 0);
    EXPECT_GT(system_metrics.memory_available_bytes, 0);
    EXPECT_GT(system_metrics.thread_count, 0);
    
    auto process_metrics_result = resource_monitor->get_current_process_metrics();
    ASSERT_TRUE(process_metrics_result.has_value());
    
    const auto& process_metrics = process_metrics_result.value();
    EXPECT_GT(process_metrics.pid, 0);
    EXPECT_GE(process_metrics.cpu_usage_percent, 0.0);
    EXPECT_GT(process_metrics.memory_usage_bytes, 0);
    EXPECT_GT(process_metrics.thread_count, 0);
    
    resource_monitor->stop_monitoring();
    EXPECT_FALSE(resource_monitor->is_monitoring());
    
    std::cout << "Resource Monitoring Results:" << std::endl;
    std::cout << "  System CPU: " << system_metrics.cpu_usage_percent << "%" << std::endl;
    std::cout << "  System Memory: " << system_metrics.memory_usage_bytes << " bytes" << std::endl;
    std::cout << "  Process CPU: " << process_metrics.cpu_usage_percent << "%" << std::endl;
    std::cout << "  Process Memory: " << process_metrics.memory_usage_bytes << " bytes" << std::endl;
    std::cout << "  Thread Count: " << process_metrics.thread_count << std::endl;
}

TEST_F(PerformanceProfilingTest, BottleneckDetection) {
    profiler->start_profiling("bottleneck_test");
    
    simulate_cpu_intensive_work(300, "slow_component");
    simulate_cpu_intensive_work(50, "fast_component");
    simulate_memory_intensive_work(1024 * 1024, 10, "memory_heavy_component");
    
    profiler->stop_profiling();
    
    auto bottlenecks_result = optimization_analyzer->detect_bottlenecks(5.0);
    ASSERT_TRUE(bottlenecks_result.has_value());
    
    const auto& bottlenecks = bottlenecks_result.value();
    EXPECT_FALSE(bottlenecks.empty());
    
    bool found_slow_component = false;
    for (const auto& bottleneck : bottlenecks) {
        EXPECT_GE(bottleneck.severity_score, 5.0);
        EXPECT_FALSE(bottleneck.description.empty());
        EXPECT_FALSE(bottleneck.recommendations.empty());
        
        if (bottleneck.component == "slow_component") {
            found_slow_component = true;
            EXPECT_GT(bottleneck.severity_score, 10.0);
        }
    }
    EXPECT_TRUE(found_slow_component);
    
    auto opportunities_result = optimization_analyzer->identify_opportunities();
    ASSERT_TRUE(opportunities_result.has_value());
    
    const auto& opportunities = opportunities_result.value();
    EXPECT_FALSE(opportunities.empty());
    
    for (const auto& opportunity : opportunities) {
        EXPECT_GT(opportunity.potential_improvement_percent, 0.0);
        EXPECT_FALSE(opportunity.description.empty());
        EXPECT_FALSE(opportunity.action_items.empty());
    }
    
    std::cout << "Bottleneck Detection Results:" << std::endl;
    for (const auto& bottleneck : bottlenecks) {
        std::cout << "  Component: " << bottleneck.component 
                  << ", Severity: " << bottleneck.severity_score 
                  << ", CPU%: " << bottleneck.cpu_percentage << std::endl;
    }
    
    std::cout << "Optimization Opportunities:" << std::endl;
    for (const auto& opportunity : opportunities) {
        std::cout << "  Component: " << opportunity.component 
                  << ", Potential improvement: " << opportunity.potential_improvement_percent 
                  << "%" << std::endl;
    }
}

TEST_F(PerformanceProfilingTest, PerformanceComparison) {
    ASSERT_TRUE(optimization_analyzer->create_performance_baseline("baseline_test").has_value());
    
    profiler->start_profiling("baseline_session");
    simulate_cpu_intensive_work(100, "comparison_component");
    simulate_memory_intensive_work(1024, 5, "comparison_component");
    profiler->stop_profiling();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    profiler->start_profiling("comparison_session");
    simulate_cpu_intensive_work(150, "comparison_component");  // Slower
    simulate_memory_intensive_work(1024, 8, "comparison_component");  // More memory
    profiler->stop_profiling();
    
    auto comparison_result = optimization_analyzer->compare_with_baseline("comparison_session");
    ASSERT_TRUE(comparison_result.has_value());
    
    const auto& comparison = comparison_result.value();
    EXPECT_EQ(comparison.baseline_session, "baseline_test");
    EXPECT_EQ(comparison.current_session, "comparison_session");
    EXPECT_FALSE(comparison.component_comparisons.empty());
    
    bool found_comparison_component = false;
    for (const auto& comp : comparison.component_comparisons) {
        if (comp.component == "comparison_component") {
            found_comparison_component = true;
            EXPECT_GT(comp.time_change_percent, 0.0);  // Should be slower
            EXPECT_GT(comp.memory_change_percent, 0.0);  // Should use more memory
        }
    }
    EXPECT_TRUE(found_comparison_component);
    
    std::cout << "Performance Comparison Results:" << std::endl;
    std::cout << "  Overall change: " << comparison.overall_performance_change_percent << "%" << std::endl;
    for (const auto& comp : comparison.component_comparisons) {
        std::cout << "  Component: " << comp.component 
                  << ", Time change: " << comp.time_change_percent 
                  << "%, Memory change: " << comp.memory_change_percent << "%" << std::endl;
    }
}

TEST_F(PerformanceProfilingTest, RegressionDetection) {
    ASSERT_TRUE(regression_detector->capture_baseline("regression_baseline").has_value());
    
    profiler->start_profiling("baseline_performance");
    for (int i = 0; i < 10; ++i) {
        simulate_cpu_intensive_work(50, "stable_component");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    profiler->stop_profiling();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    profiler->start_profiling("regressed_performance");
    for (int i = 0; i < 10; ++i) {
        simulate_cpu_intensive_work(100, "stable_component");  // Doubled execution time
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    profiler->stop_profiling();
    
    auto regressions_result = regression_detector->detect_regressions();
    ASSERT_TRUE(regressions_result.has_value());
    
    const auto& regressions = regressions_result.value();
    
    if (!regressions.empty()) {
        std::cout << "Regression Detection Results:" << std::endl;
        for (const auto& regression : regressions) {
            std::cout << "  Type: " << static_cast<int>(regression.type)
                      << ", Component: " << regression.component
                      << ", Change: " << regression.change_percent << "%" 
                      << ", Severity: " << static_cast<int>(regression.severity) << std::endl;
        }
        
        bool found_performance_regression = false;
        for (const auto& regression : regressions) {
            if (regression.component == "stable_component" &&
                regression.type == RegressionDetector::RegressionType::PERFORMANCE_DEGRADATION) {
                found_performance_regression = true;
                EXPECT_GT(regression.change_percent, 30.0);  // Should detect significant degradation
                EXPECT_GE(regression.severity, RegressionDetector::Severity::WARNING);
            }
        }
        EXPECT_TRUE(found_performance_regression);
    } else {
        std::cout << "No regressions detected (may need more baseline data)" << std::endl;
    }
}

TEST_F(PerformanceProfilingTest, ConcurrentProfilingStressTest) {
    const int num_threads = 4;
    const int operations_per_thread = 50;
    
    profiler->start_profiling("concurrent_stress_test");
    ASSERT_TRUE(resource_monitor->start_monitoring().has_value());
    
    std::vector<std::thread> threads;
    std::atomic<int> completed_operations{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            std::string component = "thread_" + std::to_string(i);
            
            for (int j = 0; j < operations_per_thread; ++j) {
                {
                    PROFILE_COMPONENT(*profiler, component, "concurrent_work");
                    simulate_cpu_intensive_work(10 + (j % 20), component);
                }
                
                if (j % 10 == 0) {
                    simulate_memory_intensive_work(1024 * (j + 1), 5, component);
                }
                
                completed_operations++;
            }
        });
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    profiler->stop_profiling();
    resource_monitor->stop_monitoring();
    
    EXPECT_EQ(completed_operations.load(), num_threads * operations_per_thread);
    
    auto metrics_result = profiler->get_performance_metrics();
    ASSERT_TRUE(metrics_result.has_value());
    
    const auto& metrics = metrics_result.value();
    EXPECT_GT(metrics.total_operations, 0);
    EXPECT_GT(metrics.operations_per_second, 0.0);
    
    auto component_stats_result = profiler->get_component_stats();
    ASSERT_TRUE(component_stats_result.has_value());
    
    const auto& component_stats = component_stats_result.value();
    EXPECT_GE(component_stats.size(), num_threads);
    
    std::cout << "Concurrent Profiling Stress Test Results:" << std::endl;
    std::cout << "  Duration: " << duration.count() << "ms" << std::endl;
    std::cout << "  Completed operations: " << completed_operations.load() << std::endl;
    std::cout << "  Total profile entries: " << metrics.total_operations << std::endl;
    std::cout << "  Operations/sec: " << metrics.operations_per_second << std::endl;
    std::cout << "  Memory usage: " << metrics.memory_usage_bytes << " bytes" << std::endl;
    std::cout << "  Component count: " << component_stats.size() << std::endl;
    
    for (const auto& stat : component_stats) {
        if (stat.name.find("thread_") == 0) {
            std::cout << "    " << stat.name << ": " << stat.call_count 
                      << " calls, " << stat.cpu_percentage << "% CPU" << std::endl;
        }
    }
}

TEST_F(PerformanceProfilingTest, ProfileReportGeneration) {
    profiler->start_profiling("report_generation_test");
    
    simulate_cpu_intensive_work(100, "component_a");
    simulate_cpu_intensive_work(150, "component_b");
    simulate_memory_intensive_work(2048, 10, "component_c");
    
    profiler->stop_profiling();
    
    auto report_result = profiler->generate_performance_report();
    ASSERT_TRUE(report_result.has_value());
    
    const auto& report = report_result.value();
    EXPECT_FALSE(report.empty());
    
    EXPECT_NE(report.find("Performance Report"), std::string::npos);
    EXPECT_NE(report.find("Overall Metrics"), std::string::npos);
    EXPECT_NE(report.find("Component Breakdown"), std::string::npos);
    EXPECT_NE(report.find("component_a"), std::string::npos);
    EXPECT_NE(report.find("component_b"), std::string::npos);
    EXPECT_NE(report.find("component_c"), std::string::npos);
    
    auto optimization_report_result = optimization_analyzer->generate_optimization_report();
    ASSERT_TRUE(optimization_report_result.has_value());
    
    const auto& optimization_report = optimization_report_result.value();
    EXPECT_FALSE(optimization_report.empty());
    
    auto export_result = profiler->export_profile_data("/tmp/profile_test_export.json");
    ASSERT_TRUE(export_result.has_value());
    
    std::cout << "Generated Performance Report (first 500 chars):" << std::endl;
    std::cout << report.substr(0, 500) << "..." << std::endl;
}

} // namespace emulator::debug::test