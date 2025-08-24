/**
 * @file esp_timer_api.cpp
 * @brief ESP-IDF timer API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible high-resolution timer functions
 * that provide microsecond-precision timing and periodic callbacks.
 */

#include "emulator/esp_idf/esp_timer.h"
#include "emulator/utils/logging.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <memory>
#include <map>

namespace {
    using namespace m5tab5::emulator;
    
    /**
     * @brief Timer state enumeration
     */
    enum class TimerState {
        STOPPED,
        RUNNING_ONCE,
        RUNNING_PERIODIC
    };
    
    /**
     * @brief Internal timer structure
     */
    struct TimerInfo {
        esp_timer_cb_t callback;
        void* arg;
        std::string name;
        bool skip_unhandled_events;
        TimerState state;
        uint64_t period_us;
        uint64_t next_alarm_us;
        uint32_t callback_count;
        
        TimerInfo(const esp_timer_create_args_t* args)
            : callback(args->callback)
            , arg(args->arg)
            , name(args->name ? args->name : "unnamed")
            , skip_unhandled_events(args->skip_unhandled_events)
            , state(TimerState::STOPPED)
            , period_us(0)
            , next_alarm_us(0)
            , callback_count(0)
        {}
    };
    
    // Global timer system state
    static std::atomic<bool> timer_system_initialized{false};
    static std::vector<std::shared_ptr<TimerInfo>> timers;
    static std::mutex timers_mutex;
    static std::thread timer_thread;
    static std::atomic<bool> timer_thread_running{false};
    static std::condition_variable timer_cv;
    
    // System start time for absolute time calculations
    static auto system_start_time = std::chrono::high_resolution_clock::now();
    
    /**
     * @brief Get current time in microseconds since system start
     */
    uint64_t get_current_time_us() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - system_start_time);
        return static_cast<uint64_t>(duration.count());
    }
    
    /**
     * @brief Timer thread main loop
     */
    void timer_thread_loop() {
        LOG_DEBUG("esp_timer: timer thread started");
        
        while (timer_thread_running.load()) {
            uint64_t current_time = get_current_time_us();
            uint64_t next_alarm = UINT64_MAX;
            std::vector<std::shared_ptr<TimerInfo>> ready_timers;
            
            // Check for expired timers
            {
                std::lock_guard<std::mutex> lock(timers_mutex);
                
                for (auto& timer : timers) {
                    if (timer->state == TimerState::STOPPED) {
                        continue;
                    }
                    
                    if (timer->next_alarm_us <= current_time) {
                        // Timer has expired
                        ready_timers.push_back(timer);
                        
                        if (timer->state == TimerState::RUNNING_PERIODIC) {
                            // Schedule next alarm for periodic timer
                            timer->next_alarm_us = current_time + timer->period_us;
                        } else {
                            // One-shot timer - mark as stopped
                            timer->state = TimerState::STOPPED;
                        }
                    }
                    
                    // Track next alarm time
                    if (timer->state != TimerState::STOPPED && timer->next_alarm_us < next_alarm) {
                        next_alarm = timer->next_alarm_us;
                    }
                }
            }
            
            // Execute timer callbacks (outside of mutex to avoid deadlock)
            for (auto& timer : ready_timers) {
                if (timer->callback) {
                    try {
                        timer->callback(timer->arg);
                        timer->callback_count++;
                        LOG_DEBUG("esp_timer: executed callback for timer '{}'", timer->name);
                    } catch (...) {
                        LOG_ERROR("esp_timer: callback threw exception for timer '{}'", timer->name);
                    }
                }
            }
            
            // Sleep until next alarm or timeout
            if (next_alarm != UINT64_MAX && next_alarm > current_time) {
                uint64_t sleep_us = next_alarm - current_time;
                if (sleep_us > 1000000) { // Cap sleep time to 1 second
                    sleep_us = 1000000;
                }
                
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            } else {
                // No active timers, sleep for a short time
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        LOG_DEBUG("esp_timer: timer thread stopped");
    }
    
    /**
     * @brief Find timer by handle
     */
    std::shared_ptr<TimerInfo> find_timer(esp_timer_handle_t handle) {
        auto timer_info = static_cast<TimerInfo*>(handle);
        
        for (auto& timer : timers) {
            if (timer.get() == timer_info) {
                return timer;
            }
        }
        
        return nullptr;
    }
}

// ============================================================================
// ESP-IDF Timer API Implementation
// ============================================================================

extern "C" {

esp_err_t esp_timer_init(void) {
    if (timer_system_initialized.load()) {
        LOG_WARN("esp_timer_init: timer system already initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_timer_init: initializing high-resolution timer system");
    
    // Record system start time
    system_start_time = std::chrono::high_resolution_clock::now();
    
    // Clear any existing timers
    {
        std::lock_guard<std::mutex> lock(timers_mutex);
        timers.clear();
    }
    
    // Start timer thread
    timer_thread_running.store(true);
    timer_thread = std::thread(timer_thread_loop);
    
    timer_system_initialized.store(true);
    
    LOG_INFO("esp_timer_init: timer system initialization completed");
    return ESP_OK;
}

esp_err_t esp_timer_deinit(void) {
    if (!timer_system_initialized.load()) {
        LOG_WARN("esp_timer_deinit: timer system not initialized");
        return ESP_OK;
    }
    
    LOG_INFO("esp_timer_deinit: shutting down timer system");
    
    // Stop timer thread
    timer_thread_running.store(false);
    timer_cv.notify_all();
    
    if (timer_thread.joinable()) {
        timer_thread.join();
    }
    
    // Clear all timers
    {
        std::lock_guard<std::mutex> lock(timers_mutex);
        timers.clear();
    }
    
    timer_system_initialized.store(false);
    
    LOG_INFO("esp_timer_deinit: timer system shutdown completed");
    return ESP_OK;
}

esp_err_t esp_timer_create(const esp_timer_create_args_t* create_args, esp_timer_handle_t* out_handle) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_create: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!create_args || !create_args->callback || !out_handle) {
        LOG_ERROR("esp_timer_create: invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUG("esp_timer_create: creating timer '{}'", create_args->name ? create_args->name : "unnamed");
    
    auto timer = std::make_shared<TimerInfo>(create_args);
    
    {
        std::lock_guard<std::mutex> lock(timers_mutex);
        timers.push_back(timer);
    }
    
    *out_handle = static_cast<esp_timer_handle_t>(timer.get());
    
    LOG_DEBUG("esp_timer_create: timer '{}' created successfully", timer->name);
    return ESP_OK;
}

esp_err_t esp_timer_delete(esp_timer_handle_t timer) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_delete: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timer) {
        LOG_ERROR("esp_timer_delete: null timer handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    auto it = std::find_if(timers.begin(), timers.end(),
                          [timer](const std::shared_ptr<TimerInfo>& t) {
                              return t.get() == static_cast<TimerInfo*>(timer);
                          });
    
    if (it == timers.end()) {
        LOG_ERROR("esp_timer_delete: timer not found");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::string timer_name = (*it)->name;
    (*it)->state = TimerState::STOPPED;
    timers.erase(it);
    
    LOG_DEBUG("esp_timer_delete: timer '{}' deleted successfully", timer_name);
    return ESP_OK;
}

esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_start_once: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timer) {
        LOG_ERROR("esp_timer_start_once: null timer handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    auto timer_info = find_timer(timer);
    if (!timer_info) {
        LOG_ERROR("esp_timer_start_once: timer not found");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (timer_info->state != TimerState::STOPPED) {
        LOG_ERROR("esp_timer_start_once: timer already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint64_t current_time = get_current_time_us();
    timer_info->state = TimerState::RUNNING_ONCE;
    timer_info->period_us = timeout_us;
    timer_info->next_alarm_us = current_time + timeout_us;
    
    LOG_DEBUG("esp_timer_start_once: started one-shot timer '{}' for {} us", 
              timer_info->name, timeout_us);
    
    timer_cv.notify_one();
    return ESP_OK;
}

esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_start_periodic: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timer) {
        LOG_ERROR("esp_timer_start_periodic: null timer handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (period_us == 0) {
        LOG_ERROR("esp_timer_start_periodic: period cannot be zero");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    auto timer_info = find_timer(timer);
    if (!timer_info) {
        LOG_ERROR("esp_timer_start_periodic: timer not found");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (timer_info->state != TimerState::STOPPED) {
        LOG_ERROR("esp_timer_start_periodic: timer already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint64_t current_time = get_current_time_us();
    timer_info->state = TimerState::RUNNING_PERIODIC;
    timer_info->period_us = period_us;
    timer_info->next_alarm_us = current_time + period_us;
    
    LOG_DEBUG("esp_timer_start_periodic: started periodic timer '{}' with period {} us", 
              timer_info->name, period_us);
    
    timer_cv.notify_one();
    return ESP_OK;
}

esp_err_t esp_timer_stop(esp_timer_handle_t timer) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_stop: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timer) {
        LOG_ERROR("esp_timer_stop: null timer handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    auto timer_info = find_timer(timer);
    if (!timer_info) {
        LOG_ERROR("esp_timer_stop: timer not found");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (timer_info->state == TimerState::STOPPED) {
        LOG_WARN("esp_timer_stop: timer '{}' already stopped", timer_info->name);
        return ESP_OK;
    }
    
    timer_info->state = TimerState::STOPPED;
    
    LOG_DEBUG("esp_timer_stop: stopped timer '{}'", timer_info->name);
    return ESP_OK;
}

esp_err_t esp_timer_restart(esp_timer_handle_t timer, uint64_t timeout_us) {
    // Stop the timer first (ignore errors)
    esp_timer_stop(timer);
    
    // Start it again
    return esp_timer_start_once(timer, timeout_us);
}

// ============================================================================
// Time Query Functions Implementation
// ============================================================================

uint64_t esp_timer_get_time(void) {
    return get_current_time_us();
}

uint64_t esp_timer_get_next_alarm(void) {
    if (!timer_system_initialized.load()) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    uint64_t next_alarm = UINT64_MAX;
    for (const auto& timer : timers) {
        if (timer->state != TimerState::STOPPED && timer->next_alarm_us < next_alarm) {
            next_alarm = timer->next_alarm_us;
        }
    }
    
    return (next_alarm == UINT64_MAX) ? 0 : next_alarm;
}

bool esp_timer_is_active(esp_timer_handle_t timer) {
    if (!timer_system_initialized.load() || !timer) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    auto timer_info = find_timer(timer);
    if (!timer_info) {
        return false;
    }
    
    return timer_info->state != TimerState::STOPPED;
}

// ============================================================================
// Timer Information Functions Implementation
// ============================================================================

esp_err_t esp_timer_dump(esp_timer_info_t* timer_info, size_t size) {
    if (!timer_system_initialized.load()) {
        LOG_ERROR("esp_timer_dump: timer system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timer_info || size == 0) {
        LOG_ERROR("esp_timer_dump: invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    size_t count = std::min(size, timers.size());
    
    for (size_t i = 0; i < count; i++) {
        const auto& timer = timers[i];
        timer_info[i].name = timer->name.c_str();
        timer_info[i].period = (timer->state == TimerState::RUNNING_PERIODIC) ? timer->period_us : 0;
        timer_info[i].next_alarm = (timer->state != TimerState::STOPPED) ? timer->next_alarm_us : 0;
        timer_info[i].is_active = (timer->state != TimerState::STOPPED);
        timer_info[i].callback_count = timer->callback_count;
    }
    
    return ESP_OK;
}

void esp_timer_dump_all(void) {
    if (!timer_system_initialized.load()) {
        LOG_INFO("esp_timer_dump_all: timer system not initialized");
        return;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex);
    
    LOG_INFO("esp_timer_dump_all: {} timers registered", timers.size());
    
    for (const auto& timer : timers) {
        const char* state_str = "STOPPED";
        if (timer->state == TimerState::RUNNING_ONCE) {
            state_str = "ONE_SHOT";
        } else if (timer->state == TimerState::RUNNING_PERIODIC) {
            state_str = "PERIODIC";
        }
        
        LOG_INFO("  Timer '{}': state={}, period={} us, next_alarm={} us, callbacks={}",
                 timer->name, state_str, timer->period_us, timer->next_alarm_us, timer->callback_count);
    }
}

// ============================================================================
// Delay Functions Implementation
// ============================================================================

void esp_timer_delay_us(uint32_t us) {
    if (us == 0) {
        return;
    }
    
    // Use high-resolution sleep for microsecond delays
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

uint32_t esp_timer_get_resolution(void) {
    // Return 1 microsecond resolution
    return 1;
}

} // extern "C"