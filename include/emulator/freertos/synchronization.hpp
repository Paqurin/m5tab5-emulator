#pragma once

#include "emulator/freertos/task.hpp"
#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

namespace m5tab5::emulator::freertos {

// Forward declarations
class TaskScheduler;

// FreeRTOS synchronization handle types
using SemaphoreHandle_t = void*;
using QueueHandle_t = void*;
using MutexHandle_t = void*;
using EventGroupHandle_t = void*;
using EventBits_t = uint32_t;

/**
 * @brief FreeRTOS Semaphore implementation
 * 
 * Supports both binary and counting semaphores with proper blocking behavior.
 */
class Semaphore {
public:
    enum class SemaphoreType {
        BINARY,
        COUNTING,
        MUTEX,
        RECURSIVE_MUTEX
    };

    explicit Semaphore(SemaphoreType type, UBaseType_t max_count = 1, UBaseType_t initial_count = 0);
    ~Semaphore();

    // Core semaphore operations
    BaseType_t take(TickType_t ticks_to_wait);
    BaseType_t take_from_isr(BaseType_t* higher_priority_task_woken);
    BaseType_t give();
    BaseType_t give_from_isr(BaseType_t* higher_priority_task_woken);
    
    // State queries
    UBaseType_t get_count() const { return count_; }
    UBaseType_t get_max_count() const { return max_count_; }
    UBaseType_t get_waiting_tasks() const;
    
    // Mutex-specific operations (for recursive mutexes)
    BaseType_t give_recursive();
    BaseType_t take_recursive(TickType_t ticks_to_wait);
    TaskHandle_t get_mutex_holder() const { return mutex_holder_; }
    UBaseType_t get_recursive_count() const { return recursive_count_; }

private:
    SemaphoreType type_;
    std::atomic<UBaseType_t> count_;
    UBaseType_t max_count_;
    
    // Waiting tasks queue (FIFO order)
    mutable std::mutex waiting_mutex_;
    std::queue<TaskHandle_t> waiting_tasks_;
    std::condition_variable semaphore_cv_;
    
    // Mutex-specific data
    std::atomic<TaskHandle_t> mutex_holder_{nullptr};
    std::atomic<UBaseType_t> recursive_count_{0};
    
    // Internal helpers
    void wake_waiting_task();
    bool is_mutex_type() const;
};

/**
 * @brief FreeRTOS Queue implementation
 * 
 * Thread-safe message queue with blocking send/receive operations.
 */
class Queue {
public:
    explicit Queue(UBaseType_t queue_length, UBaseType_t item_size);
    ~Queue();

    // Queue operations
    BaseType_t send(const void* item, TickType_t ticks_to_wait);
    BaseType_t send_from_isr(const void* item, BaseType_t* higher_priority_task_woken);
    BaseType_t send_to_back(const void* item, TickType_t ticks_to_wait);
    BaseType_t send_to_front(const void* item, TickType_t ticks_to_wait);
    
    BaseType_t receive(void* buffer, TickType_t ticks_to_wait);
    BaseType_t receive_from_isr(void* buffer, BaseType_t* higher_priority_task_woken);
    BaseType_t peek(void* buffer, TickType_t ticks_to_wait);
    BaseType_t peek_from_isr(void* buffer);
    
    // Queue state
    UBaseType_t get_messages_waiting() const;
    UBaseType_t get_spaces_available() const;
    UBaseType_t get_length() const { return queue_length_; }
    UBaseType_t get_item_size() const { return item_size_; }
    
    // Queue management
    BaseType_t reset();
    BaseType_t is_full() const;
    BaseType_t is_empty() const;

private:
    UBaseType_t queue_length_;
    UBaseType_t item_size_;
    
    // Queue storage
    std::unique_ptr<uint8_t[]> queue_storage_;
    std::atomic<UBaseType_t> head_{0};
    std::atomic<UBaseType_t> tail_{0};
    std::atomic<UBaseType_t> items_in_queue_{0};
    
    // Synchronization
    mutable std::mutex queue_mutex_;
    std::condition_variable send_cv_;
    std::condition_variable receive_cv_;
    
    // Waiting tasks
    std::queue<TaskHandle_t> send_waiting_tasks_;
    std::queue<TaskHandle_t> receive_waiting_tasks_;
    
    // Internal operations
    void copy_item_to_queue(const void* item, UBaseType_t position);
    void copy_item_from_queue(void* buffer, UBaseType_t position);
    UBaseType_t get_next_position(UBaseType_t current) const;
    UBaseType_t get_prev_position(UBaseType_t current) const;
};

/**
 * @brief FreeRTOS Event Groups implementation
 * 
 * Bit-based synchronization mechanism for coordinating multiple tasks.
 */
class EventGroup {
public:
    using EventBits_t = uint32_t;
    
    explicit EventGroup();
    ~EventGroup();

    // Event group operations
    EventBits_t wait_bits(
        const EventBits_t bits_to_wait_for,
        const BaseType_t clear_on_exit,
        const BaseType_t wait_for_all_bits,
        TickType_t ticks_to_wait
    );
    
    EventBits_t set_bits(const EventBits_t bits_to_set);
    EventBits_t set_bits_from_isr(const EventBits_t bits_to_set, BaseType_t* higher_priority_task_woken);
    EventBits_t clear_bits(const EventBits_t bits_to_clear);
    EventBits_t clear_bits_from_isr(const EventBits_t bits_to_clear);
    EventBits_t get_bits() const { return event_bits_; }
    EventBits_t get_bits_from_isr() const { return event_bits_; }
    
    // Synchronization
    EventBits_t sync(
        const EventBits_t bits_to_set,
        const EventBits_t bits_to_wait_for,
        TickType_t ticks_to_wait
    );

private:
    std::atomic<EventBits_t> event_bits_{0};
    
    // Waiting tasks with their wait conditions
    struct WaitingTask {
        TaskHandle_t task;
        EventBits_t bits_to_wait_for;
        bool clear_on_exit;
        bool wait_for_all_bits;
        TickType_t wake_time;
    };
    
    mutable std::mutex waiting_mutex_;
    std::vector<WaitingTask> waiting_tasks_;
    std::condition_variable event_cv_;
    
    // Internal helpers
    void check_waiting_tasks();
    bool are_bits_satisfied(const WaitingTask& waiting_task) const;
};

/**
 * @brief FreeRTOS synchronization API compatibility layer
 * 
 * Provides the standard FreeRTOS synchronization APIs that ESP-IDF expects.
 */
class SynchronizationAPI {
public:
    // Semaphore APIs
    static SemaphoreHandle_t xSemaphoreCreateBinary();
    static SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t max_count, UBaseType_t initial_count);
    static SemaphoreHandle_t xSemaphoreCreateMutex();
    static SemaphoreHandle_t xSemaphoreCreateRecursiveMutex();
    static void vSemaphoreDelete(SemaphoreHandle_t semaphore);
    
    static BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks_to_wait);
    static BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t semaphore, BaseType_t* higher_priority_task_woken);
    static BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore);
    static BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t semaphore, BaseType_t* higher_priority_task_woken);
    
    static BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t semaphore, TickType_t ticks_to_wait);
    static BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t semaphore);
    static UBaseType_t uxSemaphoreGetCount(SemaphoreHandle_t semaphore);
    
    // Queue APIs
    static QueueHandle_t xQueueCreate(UBaseType_t queue_length, UBaseType_t item_size);
    static void vQueueDelete(QueueHandle_t queue);
    
    static BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t ticks_to_wait);
    static BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void* item, BaseType_t* higher_priority_task_woken);
    static BaseType_t xQueueSendToBack(QueueHandle_t queue, const void* item, TickType_t ticks_to_wait);
    static BaseType_t xQueueSendToFront(QueueHandle_t queue, const void* item, TickType_t ticks_to_wait);
    
    static BaseType_t xQueueReceive(QueueHandle_t queue, void* buffer, TickType_t ticks_to_wait);
    static BaseType_t xQueueReceiveFromISR(QueueHandle_t queue, void* buffer, BaseType_t* higher_priority_task_woken);
    static BaseType_t xQueuePeek(QueueHandle_t queue, void* buffer, TickType_t ticks_to_wait);
    static BaseType_t xQueuePeekFromISR(QueueHandle_t queue, void* buffer);
    
    static UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue);
    static UBaseType_t uxQueueSpacesAvailable(QueueHandle_t queue);
    static BaseType_t xQueueReset(QueueHandle_t queue);
    
    // Event Group APIs
    static EventGroupHandle_t xEventGroupCreate();
    static void vEventGroupDelete(EventGroupHandle_t event_group);
    
    static EventBits_t xEventGroupWaitBits(
        EventGroupHandle_t event_group,
        const EventBits_t bits_to_wait_for,
        const BaseType_t clear_on_exit,
        const BaseType_t wait_for_all_bits,
        TickType_t ticks_to_wait
    );
    
    static EventBits_t xEventGroupSetBits(EventGroupHandle_t event_group, const EventBits_t bits_to_set);
    static EventBits_t xEventGroupSetBitsFromISR(EventGroupHandle_t event_group, const EventBits_t bits_to_set, BaseType_t* higher_priority_task_woken);
    static EventBits_t xEventGroupClearBits(EventGroupHandle_t event_group, const EventBits_t bits_to_clear);
    static EventBits_t xEventGroupClearBitsFromISR(EventGroupHandle_t event_group, const EventBits_t bits_to_clear);
    static EventBits_t xEventGroupGetBits(EventGroupHandle_t event_group);
    static EventBits_t xEventGroupGetBitsFromISR(EventGroupHandle_t event_group);
    
    static EventBits_t xEventGroupSync(
        EventGroupHandle_t event_group,
        const EventBits_t bits_to_set,
        const EventBits_t bits_to_wait_for,
        TickType_t ticks_to_wait
    );

private:
    // Handle conversion helpers
    static Semaphore* handle_to_semaphore(SemaphoreHandle_t handle);
    static Queue* handle_to_queue(QueueHandle_t handle);
    static EventGroup* handle_to_event_group(EventGroupHandle_t handle);
    
    static SemaphoreHandle_t semaphore_to_handle(Semaphore* semaphore);
    static QueueHandle_t queue_to_handle(Queue* queue);
    static EventGroupHandle_t event_group_to_handle(EventGroup* event_group);
};

} // namespace m5tab5::emulator::freertos