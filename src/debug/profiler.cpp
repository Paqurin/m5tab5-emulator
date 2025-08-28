#include "emulator/debug/profiler.hpp"
#include "emulator/utils/logging.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <cmath>

namespace m5tab5::emulator {

DECLARE_LOGGER("Profiler");

thread_local uint32_t Profiler::thread_id_counter_ = 0;

uint32_t Profiler::get_thread_id() {
    static std::atomic<uint32_t> global_counter{0};
    if (thread_id_counter_ == 0) {
        thread_id_counter_ = ++global_counter;
    }
    return thread_id_counter_;
}

Profiler::ProfileScope::ProfileScope(Profiler& profiler, const std::string& name, 
                                   const std::string& component, const std::string& function)
    : profiler_(profiler), name_(name), component_(component), function_(function),
      start_time_(std::chrono::high_resolution_clock::now()) {
    
    if (profiler_.is_profiling_ && !profiler_.is_paused_) {
        profiler_.begin_sample(name_, component_, function_);
    }
}

Profiler::ProfileScope::~ProfileScope() {
    if (!finished_ && profiler_.is_profiling_ && !profiler_.is_paused_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
        
        std::lock_guard<std::mutex> lock(profiler_.entries_mutex_);
        
        ProfileEntry entry;
        entry.name = name_;
        entry.component = component_;
        entry.function = function_;
        entry.start_time = start_time_;
        entry.end_time = end_time;
        entry.duration = duration;
        entry.thread_id = get_thread_id();
        entry.call_count = 1;
        entry.memory_allocated = memory_allocated_;
        entry.memory_freed = memory_freed_;
        
        profiler_.profile_entries_.push_back(entry);
        finished_ = true;
        
        // Update total memory tracking
        profiler_.total_memory_allocated_ += memory_allocated_;
        profiler_.total_memory_freed_ += memory_freed_;
        
        size_t current_usage = profiler_.total_memory_allocated_ - profiler_.total_memory_freed_;
        size_t peak = profiler_.peak_memory_usage_.load();
        while (current_usage > peak && 
               !profiler_.peak_memory_usage_.compare_exchange_weak(peak, current_usage)) {
            // CAS loop
        }
    }
}

void Profiler::ProfileScope::add_memory_allocation(size_t bytes) {
    memory_allocated_ += bytes;
}

void Profiler::ProfileScope::add_memory_deallocation(size_t bytes) {
    memory_freed_ += bytes;
}

void Profiler::ProfileScope::add_custom_metric(const std::string& key, double value) {
    // Custom metrics can be logged here
    COMPONENT_LOG_DEBUG("Custom metric {}: {}", key, value);
}

Result<std::unique_ptr<Profiler>> Profiler::create() {
    auto profiler = std::unique_ptr<Profiler>(new Profiler());
    
    if (auto result = profiler->initialize(); !result.has_value()) {
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to create profiler"));
    }
    
    return profiler;
}

Result<void> Profiler::initialize() {
    COMPONENT_LOG_INFO("Initializing profiler system");
    
    profile_entries_.reserve(10000);  // Pre-allocate for performance
    
    last_metrics_update_ = std::chrono::high_resolution_clock::now();
    
    return {};
}

void Profiler::shutdown() {
    COMPONENT_LOG_INFO("Shutting down profiler system");
    
    stop_profiling();
    stop_continuous_monitoring();
}

void Profiler::start_profiling(const std::string& session_name) {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    current_session_name_ = session_name;
    session_start_time_ = std::chrono::high_resolution_clock::now();
    is_profiling_ = true;
    is_paused_ = false;
    
    profile_entries_.clear();
    active_samples_.clear();
    
    COMPONENT_LOG_INFO("Started profiling session: {}", session_name);
}

void Profiler::stop_profiling() {
    is_profiling_ = false;
    is_paused_ = false;
    
    process_completed_entries();
    
    COMPONENT_LOG_INFO("Stopped profiling session: {}", current_session_name_);
}

void Profiler::pause_profiling() {
    is_paused_ = true;
    COMPONENT_LOG_INFO("Paused profiling session: {}", current_session_name_);
}

void Profiler::resume_profiling() {
    is_paused_ = false;
    COMPONENT_LOG_INFO("Resumed profiling session: {}", current_session_name_);
}

void Profiler::begin_sample(const std::string& name, const std::string& component, 
                           const std::string& function) {
    if (!is_profiling_ || is_paused_) return;
    
    // Check component filter
    if (!component_filter_.empty() && component != "" &&
        std::find(component_filter_.begin(), component_filter_.end(), component) == component_filter_.end()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    ProfileEntry entry;
    entry.name = name;
    entry.component = component;
    entry.function = function;
    entry.start_time = std::chrono::high_resolution_clock::now();
    entry.thread_id = get_thread_id();
    entry.call_count = 1;
    entry.memory_allocated = 0;
    entry.memory_freed = 0;
    
    active_samples_[name] = entry;
}

void Profiler::end_sample(const std::string& name) {
    if (!is_profiling_ || is_paused_) return;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    auto it = active_samples_.find(name);
    if (it != active_samples_.end()) {
        ProfileEntry& entry = it->second;
        entry.end_time = end_time;
        entry.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - entry.start_time);
        
        profile_entries_.push_back(entry);
        active_samples_.erase(it);
    }
}

Profiler::ProfileScope Profiler::create_scope(const std::string& name, const std::string& component, 
                                             const std::string& function) {
    return ProfileScope(*this, name, component, function);
}

void Profiler::record_memory_allocation(size_t bytes, const std::string& component) {
    if (!memory_tracking_enabled_) return;
    
    total_memory_allocated_ += bytes;
    
    // Update peak usage
    size_t current_usage = total_memory_allocated_ - total_memory_freed_;
    size_t peak = peak_memory_usage_.load();
    while (current_usage > peak && 
           !peak_memory_usage_.compare_exchange_weak(peak, current_usage)) {
        // CAS loop
    }
}

void Profiler::record_memory_deallocation(size_t bytes, const std::string& component) {
    if (!memory_tracking_enabled_) return;
    
    total_memory_freed_ += bytes;
}

void Profiler::record_custom_metric(const std::string& name, double value) {
    COMPONENT_LOG_DEBUG("Custom metric {}: {}", name, value);
}

Result<Profiler::PerformanceMetrics> Profiler::get_performance_metrics() const {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    
    PerformanceMetrics metrics;
    
    if (profile_entries_.empty()) {
        metrics.cpu_usage_percent = 0.0;
        metrics.memory_usage_bytes = total_memory_allocated_ - total_memory_freed_;
        metrics.peak_memory_bytes = peak_memory_usage_;
        metrics.operations_per_second = 0.0;
        metrics.average_latency_ns = 0.0;
        metrics.min_latency_ns = 0.0;
        metrics.max_latency_ns = 0.0;
        metrics.total_operations = 0;
        metrics.total_execution_time = std::chrono::nanoseconds{0};
        return metrics;
    }
    
    auto total_time = std::chrono::nanoseconds{0};
    auto min_time = std::chrono::nanoseconds::max();
    auto max_time = std::chrono::nanoseconds::min();
    
    std::unordered_map<std::string, std::chrono::nanoseconds> component_times;
    
    for (const auto& entry : profile_entries_) {
        total_time += entry.duration;
        min_time = std::min(min_time, entry.duration);
        max_time = std::max(max_time, entry.duration);
        
        if (!entry.component.empty()) {
            component_times[entry.component] += entry.duration;
        }
    }
    
    auto session_duration = std::chrono::high_resolution_clock::now() - session_start_time_;
    auto session_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(session_duration);
    
    metrics.cpu_usage_percent = (double(total_time.count()) / double(session_duration_ns.count())) * 100.0;
    metrics.memory_usage_bytes = total_memory_allocated_ - total_memory_freed_;
    metrics.peak_memory_bytes = peak_memory_usage_;
    metrics.operations_per_second = double(profile_entries_.size()) / (double(session_duration_ns.count()) / 1e9);
    metrics.average_latency_ns = double(total_time.count()) / profile_entries_.size();
    metrics.min_latency_ns = double(min_time.count());
    metrics.max_latency_ns = double(max_time.count());
    metrics.total_operations = profile_entries_.size();
    metrics.total_execution_time = total_time;
    
    // Calculate component breakdown
    for (const auto& [component, time] : component_times) {
        double percentage = (double(time.count()) / double(total_time.count())) * 100.0;
        metrics.component_breakdown[component] = percentage;
    }
    
    return metrics;
}

Result<std::vector<Profiler::ComponentStats>> Profiler::get_component_stats() const {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    std::unordered_map<std::string, std::vector<ProfileEntry>> component_entries;
    
    for (const auto& entry : profile_entries_) {
        if (!entry.component.empty()) {
            component_entries[entry.component].push_back(entry);
        }
    }
    
    std::vector<ComponentStats> stats;
    stats.reserve(component_entries.size());
    
    auto total_time = std::chrono::nanoseconds{0};
    for (const auto& entry : profile_entries_) {
        total_time += entry.duration;
    }
    
    for (const auto& [component_name, entries] : component_entries) {
        ComponentStats stat;
        stat.name = component_name;
        stat.call_count = entries.size();
        
        auto component_total = std::chrono::nanoseconds{0};
        auto min_time = std::chrono::nanoseconds::max();
        auto max_time = std::chrono::nanoseconds::min();
        size_t total_allocated = 0;
        size_t total_freed = 0;
        
        for (const auto& entry : entries) {
            component_total += entry.duration;
            min_time = std::min(min_time, entry.duration);
            max_time = std::max(max_time, entry.duration);
            total_allocated += entry.memory_allocated;
            total_freed += entry.memory_freed;
        }
        
        stat.total_time = component_total;
        stat.min_time = min_time;
        stat.max_time = max_time;
        stat.average_time = component_total / entries.size();
        stat.memory_allocated = total_allocated;
        stat.memory_freed = total_freed;
        stat.cpu_percentage = total_time.count() > 0 ? 
            (double(component_total.count()) / double(total_time.count())) * 100.0 : 0.0;
        
        stats.push_back(stat);
    }
    
    // Sort by total time descending
    std::sort(stats.begin(), stats.end(), 
        [](const ComponentStats& a, const ComponentStats& b) {
            return a.total_time > b.total_time;
        });
    
    return stats;
}

Result<std::vector<Profiler::ProfileEntry>> Profiler::get_profile_entries() const {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    return profile_entries_;
}

Result<void> Profiler::export_profile_data(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        return unexpected(MAKE_ERROR(FILE_ERROR, "Failed to export profile data"));
    }
    
    // Export as JSON
    file << "{\n";
    file << "  \"session\": \"" << current_session_name_ << "\",\n";
    file << "  \"start_time\": " << std::chrono::duration_cast<std::chrono::nanoseconds>(
        session_start_time_.time_since_epoch()).count() << ",\n";
    file << "  \"entries\": [\n";
    
    for (size_t i = 0; i < profile_entries_.size(); ++i) {
        const auto& entry = profile_entries_[i];
        file << "    {\n";
        file << "      \"name\": \"" << entry.name << "\",\n";
        file << "      \"component\": \"" << entry.component << "\",\n";
        file << "      \"function\": \"" << entry.function << "\",\n";
        file << "      \"duration_ns\": " << entry.duration.count() << ",\n";
        file << "      \"thread_id\": " << entry.thread_id << ",\n";
        file << "      \"memory_allocated\": " << entry.memory_allocated << ",\n";
        file << "      \"memory_freed\": " << entry.memory_freed << "\n";
        file << "    }";
        if (i < profile_entries_.size() - 1) file << ",";
        file << "\n";
    }
    
    file << "  ]\n";
    file << "}\n";
    
    return {};
}

Result<std::string> Profiler::generate_performance_report() const {
    auto metrics_result = get_performance_metrics();
    if (!metrics_result.has_value()) {
        return unexpected(metrics_result.error());
    }
    
    auto component_stats_result = get_component_stats();
    if (!component_stats_result.has_value()) {
        return unexpected(component_stats_result.error());
    }
    
    const auto& metrics = metrics_result.value();
    const auto& component_stats = component_stats_result.value();
    
    std::ostringstream report;
    
    report << "=== M5Stack Tab5 Emulator Performance Report ===\n\n";
    report << "Session: " << current_session_name_ << "\n";
    report << "Duration: " << format_duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now() - session_start_time_)) << "\n\n";
    
    report << "=== Overall Metrics ===\n";
    report << "CPU Usage: " << std::fixed << std::setprecision(2) << metrics.cpu_usage_percent << "%\n";
    report << "Memory Usage: " << format_memory(metrics.memory_usage_bytes) << "\n";
    report << "Peak Memory: " << format_memory(metrics.peak_memory_bytes) << "\n";
    report << "Operations/sec: " << std::fixed << std::setprecision(2) << metrics.operations_per_second << "\n";
    report << "Average Latency: " << format_duration(std::chrono::nanoseconds(static_cast<int64_t>(metrics.average_latency_ns))) << "\n";
    report << "Min Latency: " << format_duration(std::chrono::nanoseconds(static_cast<int64_t>(metrics.min_latency_ns))) << "\n";
    report << "Max Latency: " << format_duration(std::chrono::nanoseconds(static_cast<int64_t>(metrics.max_latency_ns))) << "\n";
    report << "Total Operations: " << metrics.total_operations << "\n\n";
    
    report << "=== Component Breakdown ===\n";
    for (const auto& stat : component_stats) {
        report << stat.name << ":\n";
        report << "  Calls: " << stat.call_count << "\n";
        report << "  Total Time: " << format_duration(stat.total_time) << "\n";
        report << "  Avg Time: " << format_duration(stat.average_time) << "\n";
        report << "  Min Time: " << format_duration(stat.min_time) << "\n";
        report << "  Max Time: " << format_duration(stat.max_time) << "\n";
        report << "  CPU %: " << std::fixed << std::setprecision(2) << stat.cpu_percentage << "%\n";
        report << "  Memory Allocated: " << format_memory(stat.memory_allocated) << "\n";
        report << "  Memory Freed: " << format_memory(stat.memory_freed) << "\n\n";
    }
    
    return report.str();
}

std::string Profiler::format_duration(std::chrono::nanoseconds duration) const {
    auto ns = duration.count();
    
    if (ns < 1000) {
        return std::to_string(ns) + "ns";
    } else if (ns < 1000000) {
        return std::to_string(ns / 1000) + "." + std::to_string((ns % 1000) / 100) + "μs";
    } else if (ns < 1000000000) {
        return std::to_string(ns / 1000000) + "." + std::to_string((ns % 1000000) / 100000) + "ms";
    } else {
        return std::to_string(ns / 1000000000) + "." + std::to_string((ns % 1000000000) / 100000000) + "s";
    }
}

std::string Profiler::format_memory(size_t bytes) const {
    if (bytes < 1024) {
        return std::to_string(bytes) + "B";
    } else if (bytes < 1024 * 1024) {
        return std::to_string(bytes / 1024) + "." + std::to_string((bytes % 1024) / 102) + "KB";
    } else if (bytes < 1024 * 1024 * 1024) {
        return std::to_string(bytes / (1024 * 1024)) + "." + 
               std::to_string((bytes % (1024 * 1024)) / (102 * 1024)) + "MB";
    } else {
        return std::to_string(bytes / (1024 * 1024 * 1024)) + "." + 
               std::to_string((bytes % (1024 * 1024 * 1024)) / (102 * 1024 * 1024)) + "GB";
    }
}

void Profiler::set_sampling_rate(double samples_per_second) {
    sampling_rate_ = samples_per_second;
}

void Profiler::set_memory_tracking_enabled(bool enabled) {
    memory_tracking_enabled_ = enabled;
}

void Profiler::set_component_filter(const std::vector<std::string>& components) {
    component_filter_ = components;
}

void Profiler::reset_statistics() {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    profile_entries_.clear();
    active_samples_.clear();
    total_memory_allocated_ = 0;
    total_memory_freed_ = 0;
    peak_memory_usage_ = 0;
}

void Profiler::process_completed_entries() {
    // Process any remaining active samples
    std::lock_guard<std::mutex> lock(entries_mutex_);
    
    auto now = std::chrono::high_resolution_clock::now();
    for (auto& [name, entry] : active_samples_) {
        entry.end_time = now;
        entry.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - entry.start_time);
        profile_entries_.push_back(entry);
    }
    active_samples_.clear();
}

Result<void> Profiler::start_continuous_monitoring() {
    if (continuous_monitoring_.load()) {
        COMPONENT_LOG_WARN("Continuous monitoring is already running");
        return {};
    }
    
    continuous_monitoring_ = true;
    
    try {
        monitoring_thread_ = std::thread([this]() {
            COMPONENT_LOG_INFO("Started continuous monitoring thread");
            
            const auto update_interval = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / sampling_rate_));
            
            while (continuous_monitoring_.load()) {
                try {
                    update_metrics();
                    
                    // Call performance callback if set
                    if (performance_callback_) {
                        auto metrics_result = get_performance_metrics();
                        if (metrics_result.has_value()) {
                            performance_callback_(metrics_result.value());
                        }
                    }
                    
                    std::this_thread::sleep_for(update_interval);
                } catch (const std::exception& e) {
                    COMPONENT_LOG_ERROR("Error in continuous monitoring: {}", e.what());
                }
            }
            
            COMPONENT_LOG_INFO("Stopped continuous monitoring thread");
        });
    } catch (const std::exception& e) {
        continuous_monitoring_ = false;
        return unexpected(MAKE_ERROR(OPERATION_FAILED, "Failed to start monitoring thread: " + std::string(e.what())));
    }
    
    COMPONENT_LOG_INFO("Started continuous monitoring");
    return {};
}

void Profiler::stop_continuous_monitoring() {
    if (!continuous_monitoring_.load()) {
        return;
    }
    
    continuous_monitoring_ = false;
    
    if (monitoring_thread_.joinable()) {
        monitoring_thread_.join();
    }
    
    COMPONENT_LOG_INFO("Stopped continuous monitoring");
}

void Profiler::set_performance_callback(PerformanceCallback callback) {
    performance_callback_ = std::move(callback);
    COMPONENT_LOG_INFO("Performance callback set");
}

Result<void> Profiler::save_performance_report(const std::string& filename) const {
    auto report_result = generate_performance_report();
    if (!report_result.has_value()) {
        return unexpected(report_result.error());
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        return unexpected(MAKE_ERROR(FILE_ERROR, "Failed to save performance report"));
    }
    
    file << report_result.value();
    
    COMPONENT_LOG_INFO("Performance report saved to: {}", filename);
    return {};
}

Result<void> Profiler::import_profile_data(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return unexpected(MAKE_ERROR(FILE_ERROR, "Failed to import profile data"));
    }
    
    // This is a simplified JSON parser for the specific format we export
    // A full JSON parser would be better for production use
    std::string line;
    std::vector<ProfileEntry> imported_entries;
    
    bool in_entries = false;
    bool in_entry = false;
    ProfileEntry current_entry;
    
    while (std::getline(file, line)) {
        // Remove whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.find("\"entries\": [") != std::string::npos) {
            in_entries = true;
            continue;
        }
        
        if (in_entries && line == "{") {
            in_entry = true;
            current_entry = ProfileEntry{};
            continue;
        }
        
        if (in_entry && (line == "}" || line == "},")) {
            imported_entries.push_back(current_entry);
            in_entry = false;
            continue;
        }
        
        if (in_entry) {
            // Parse key-value pairs
            auto colon_pos = line.find(":");
            if (colon_pos != std::string::npos) {
                std::string key = line.substr(0, colon_pos);
                std::string value = line.substr(colon_pos + 1);
                
                // Remove quotes and whitespace
                key.erase(std::remove(key.begin(), key.end(), '"'), key.end());
                key.erase(std::remove(key.begin(), key.end(), ' '), key.end());
                value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
                value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
                if (value.back() == ',') value.pop_back();
                
                if (key == "name") {
                    current_entry.name = value;
                } else if (key == "component") {
                    current_entry.component = value;
                } else if (key == "function") {
                    current_entry.function = value;
                } else if (key == "duration_ns") {
                    current_entry.duration = std::chrono::nanoseconds(std::stoll(value));
                } else if (key == "thread_id") {
                    current_entry.thread_id = std::stoul(value);
                } else if (key == "memory_allocated") {
                    current_entry.memory_allocated = std::stoull(value);
                } else if (key == "memory_freed") {
                    current_entry.memory_freed = std::stoull(value);
                }
            }
        }
        
        if (line == "]") {
            break;
        }
    }
    
    // Merge imported entries with current ones
    std::lock_guard<std::mutex> lock(entries_mutex_);
    profile_entries_.insert(profile_entries_.end(), imported_entries.begin(), imported_entries.end());
    
    COMPONENT_LOG_INFO("Imported {} profile entries from: {}", imported_entries.size(), filename);
    return {};
}

void Profiler::update_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    auto now = std::chrono::high_resolution_clock::now();
    
    // Only update if enough time has passed
    auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_metrics_update_);
    if (time_since_update < std::chrono::milliseconds(100)) {  // Update at most every 100ms
        return;
    }
    
    // Update cached metrics
    auto metrics_result = get_performance_metrics();
    if (metrics_result.has_value()) {
        cached_metrics_ = metrics_result.value();
        last_metrics_update_ = now;
    }
}

} // namespace m5tab5::emulator