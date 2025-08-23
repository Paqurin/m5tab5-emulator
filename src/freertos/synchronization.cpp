#include "emulator/freertos/freertos_kernel.hpp"
#include "emulator/freertos/synchronization.hpp"
#include "emulator/freertos/scheduler.hpp"
#include "emulator/utils/logging.hpp"
#include <chrono>
#include <cstring>

namespace m5tab5::emulator::freertos {

// Semaphore Implementation
Semaphore::Semaphore(SemaphoreType type, UBaseType_t max_count, UBaseType_t initial_count)
    : type_(type), count_(initial_count), max_count_(max_count), mutex_holder_(nullptr), recursive_count_(0) {
    
    LOG_DEBUG("Created semaphore: type={}, max_count={}, initial_count={}", 
              static_cast<int>(type), max_count, initial_count);
}

Semaphore::~Semaphore() {
    // Wake any waiting tasks
    semaphore_cv_.notify_all();
    LOG_DEBUG("Destroyed semaphore");
}

BaseType_t Semaphore::take(TickType_t ticks_to_wait) {
    std::unique_lock<std::mutex> lock(waiting_mutex_);
    
    // For mutex types, check if already held by current task
    if (is_mutex_type()) {
        TaskHandle_t current_task = TaskAPI::xTaskGetCurrentTaskHandle();
        
        if (type_ == SemaphoreType::RECURSIVE_MUTEX && mutex_holder_ == current_task) {
            recursive_count_++;
            LOG_DEBUG("Recursive mutex taken (count: {})", recursive_count_.load());
            return pdTRUE;
        }
        
        if (mutex_holder_ != nullptr && mutex_holder_ != current_task) {
            // Mutex is held by another task
            if (ticks_to_wait == 0) {
                return pdFALSE;
            }
            
            // Wait for mutex to be released
            TaskHandle_t waiting_task = current_task;
            waiting_tasks_.push(waiting_task);
            
            if (ticks_to_wait == portMAX_DELAY) {
                semaphore_cv_.wait(lock, [this] { return count_ > 0; });
            } else {
                auto timeout = std::chrono::milliseconds(ticks_to_wait);
                if (!semaphore_cv_.wait_for(lock, timeout, [this] { return count_ > 0; })) {
                    // Remove from waiting queue if timeout
                    std::queue<TaskHandle_t> temp_queue;
                    while (!waiting_tasks_.empty()) {
                        if (waiting_tasks_.front() != waiting_task) {
                            temp_queue.push(waiting_tasks_.front());
                        }
                        waiting_tasks_.pop();
                    }
                    waiting_tasks_ = temp_queue;
                    return pdFALSE;
                }
            }
        }
        
        // Take the mutex
        if (count_ > 0) {
            count_--;
            mutex_holder_ = current_task;
            recursive_count_ = 1;
            LOG_DEBUG("Mutex taken by task");
            return pdTRUE;
        }
    } else {
        // Binary or counting semaphore
        if (count_ > 0) {
            count_--;
            LOG_DEBUG("Semaphore taken (remaining: {})", count_.load());
            return pdTRUE;
        }
        
        if (ticks_to_wait == 0) {
            return pdFALSE;
        }
        
        // Wait for semaphore
        TaskHandle_t current_task = TaskAPI::xTaskGetCurrentTaskHandle();
        waiting_tasks_.push(current_task);
        
        if (ticks_to_wait == portMAX_DELAY) {
            semaphore_cv_.wait(lock, [this] { return count_ > 0; });
        } else {
            auto timeout = std::chrono::milliseconds(ticks_to_wait);
            if (!semaphore_cv_.wait_for(lock, timeout, [this] { return count_ > 0; })) {
                // Remove from waiting queue if timeout
                std::queue<TaskHandle_t> temp_queue;
                while (!waiting_tasks_.empty()) {
                    if (waiting_tasks_.front() != current_task) {
                        temp_queue.push(waiting_tasks_.front());
                    }
                    waiting_tasks_.pop();
                }
                waiting_tasks_ = temp_queue;
                return pdFALSE;
            }
        }
        
        if (count_ > 0) {
            count_--;
            LOG_DEBUG("Semaphore taken after wait (remaining: {})", count_.load());
            return pdTRUE;
        }
    }
    
    return pdFALSE;
}

BaseType_t Semaphore::give() {
    std::lock_guard<std::mutex> lock(waiting_mutex_);
    
    if (is_mutex_type()) {
        TaskHandle_t current_task = TaskAPI::xTaskGetCurrentTaskHandle();
        
        // Check if current task holds the mutex
        if (mutex_holder_ != current_task) {
            LOG_ERROR("Attempt to give mutex not owned by current task");
            return pdFALSE;
        }
        
        if (type_ == SemaphoreType::RECURSIVE_MUTEX) {
            recursive_count_--;
            if (recursive_count_ > 0) {
                LOG_DEBUG("Recursive mutex partially released (count: {})", recursive_count_.load());
                return pdTRUE;
            }
        }
        
        // Release the mutex
        mutex_holder_ = nullptr;
        recursive_count_ = 0;
        count_++;
        
        LOG_DEBUG("Mutex released");
    } else {
        // Binary or counting semaphore
        if (count_ < max_count_) {
            count_++;
            LOG_DEBUG("Semaphore given (count: {})", count_.load());
        } else {
            LOG_WARN("Semaphore already at maximum count");
            return pdFALSE;
        }
    }
    
    // Wake waiting task
    wake_waiting_task();
    return pdTRUE;
}

BaseType_t Semaphore::take_from_isr(BaseType_t* higher_priority_task_woken) {
    // Simplified ISR version - no blocking
    if (count_ > 0) {
        count_--;
        if (higher_priority_task_woken) {
            *higher_priority_task_woken = pdFALSE; // Simplified
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t Semaphore::give_from_isr(BaseType_t* higher_priority_task_woken) {
    if (is_mutex_type()) {
        LOG_ERROR("Cannot give mutex from ISR");
        return pdFALSE;
    }
    
    if (count_ < max_count_) {
        count_++;
        wake_waiting_task();
        
        if (higher_priority_task_woken) {
            *higher_priority_task_woken = pdTRUE; // Assume higher priority task was woken
        }
        return pdTRUE;
    }
    return pdFALSE;
}

void Semaphore::wake_waiting_task() {
    if (!waiting_tasks_.empty()) {
        waiting_tasks_.pop(); // Remove one waiting task
        semaphore_cv_.notify_one();
    }
}

bool Semaphore::is_mutex_type() const {
    return type_ == SemaphoreType::MUTEX || type_ == SemaphoreType::RECURSIVE_MUTEX;
}

UBaseType_t Semaphore::get_waiting_tasks() const {
    std::lock_guard<std::mutex> lock(waiting_mutex_);
    return waiting_tasks_.size();
}

// Queue Implementation
Queue::Queue(UBaseType_t queue_length, UBaseType_t item_size)
    : queue_length_(queue_length), item_size_(item_size), head_(0), tail_(0), items_in_queue_(0) {
    
    // Allocate storage for queue items
    queue_storage_ = std::make_unique<uint8_t[]>(queue_length * item_size);
    std::memset(queue_storage_.get(), 0, queue_length * item_size);
    
    LOG_DEBUG("Created queue: length={}, item_size={}", queue_length, item_size);
}

Queue::~Queue() {
    // Wake all waiting tasks
    send_cv_.notify_all();
    receive_cv_.notify_all();
    LOG_DEBUG("Destroyed queue");
}

BaseType_t Queue::send(const void* item, TickType_t ticks_to_wait) {
    if (!item) {
        return pdFAIL;
    }
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Check if queue is full
    if (items_in_queue_ >= queue_length_) {
        if (ticks_to_wait == 0) {
            return pdFAIL;
        }
        
        // Wait for space
        TaskHandle_t current_task = TaskAPI::xTaskGetCurrentTaskHandle();
        send_waiting_tasks_.push(current_task);
        
        if (ticks_to_wait == portMAX_DELAY) {
            send_cv_.wait(lock, [this] { return items_in_queue_ < queue_length_; });
        } else {
            auto timeout = std::chrono::milliseconds(ticks_to_wait);
            if (!send_cv_.wait_for(lock, timeout, [this] { return items_in_queue_ < queue_length_; })) {
                // Remove from waiting queue
                std::queue<TaskHandle_t> temp_queue;
                while (!send_waiting_tasks_.empty()) {
                    if (send_waiting_tasks_.front() != current_task) {
                        temp_queue.push(send_waiting_tasks_.front());
                    }
                    send_waiting_tasks_.pop();
                }
                send_waiting_tasks_ = temp_queue;
                return pdFAIL;
            }
        }
    }
    
    // Add item to queue
    if (items_in_queue_ < queue_length_) {
        copy_item_to_queue(item, tail_);
        tail_ = get_next_position(tail_);
        items_in_queue_++;
        
        LOG_DEBUG("Item added to queue (items: {})", items_in_queue_.load());
        
        // Wake waiting receiver
        if (!receive_waiting_tasks_.empty()) {
            receive_waiting_tasks_.pop();
            receive_cv_.notify_one();
        }
        
        return pdTRUE;
    }
    
    return pdFAIL;
}

BaseType_t Queue::receive(void* buffer, TickType_t ticks_to_wait) {
    if (!buffer) {
        return pdFAIL;
    }
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Check if queue is empty
    if (items_in_queue_ == 0) {
        if (ticks_to_wait == 0) {
            return pdFAIL;
        }
        
        // Wait for item
        TaskHandle_t current_task = TaskAPI::xTaskGetCurrentTaskHandle();
        receive_waiting_tasks_.push(current_task);
        
        if (ticks_to_wait == portMAX_DELAY) {
            receive_cv_.wait(lock, [this] { return items_in_queue_ > 0; });
        } else {
            auto timeout = std::chrono::milliseconds(ticks_to_wait);
            if (!receive_cv_.wait_for(lock, timeout, [this] { return items_in_queue_ > 0; })) {
                // Remove from waiting queue
                std::queue<TaskHandle_t> temp_queue;
                while (!receive_waiting_tasks_.empty()) {
                    if (receive_waiting_tasks_.front() != current_task) {
                        temp_queue.push(receive_waiting_tasks_.front());
                    }
                    receive_waiting_tasks_.pop();
                }
                receive_waiting_tasks_ = temp_queue;
                return pdFAIL;
            }
        }
    }
    
    // Remove item from queue
    if (items_in_queue_ > 0) {
        copy_item_from_queue(buffer, head_);
        head_ = get_next_position(head_);
        items_in_queue_--;
        
        LOG_DEBUG("Item received from queue (items: {})", items_in_queue_.load());
        
        // Wake waiting sender
        if (!send_waiting_tasks_.empty()) {
            send_waiting_tasks_.pop();
            send_cv_.notify_one();
        }
        
        return pdTRUE;
    }
    
    return pdFAIL;
}

BaseType_t Queue::send_to_back(const void* item, TickType_t ticks_to_wait) {
    // Same as regular send
    return send(item, ticks_to_wait);
}

BaseType_t Queue::send_to_front(const void* item, TickType_t ticks_to_wait) {
    if (!item) {
        return pdFAIL;
    }
    
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // Check if queue is full
    if (items_in_queue_ >= queue_length_) {
        if (ticks_to_wait == 0) {
            return pdFAIL;
        }
        
        // Wait for space (simplified - same logic as send)
        if (ticks_to_wait != portMAX_DELAY) {
            return pdFAIL; // Simplified timeout handling
        }
    }
    
    // Add item to front of queue
    if (items_in_queue_ < queue_length_) {
        head_ = get_prev_position(head_);
        copy_item_to_queue(item, head_);
        items_in_queue_++;
        
        LOG_DEBUG("Item added to front of queue (items: {})", items_in_queue_.load());
        
        // Wake waiting receiver
        receive_cv_.notify_one();
        return pdTRUE;
    }
    
    return pdFAIL;
}

void Queue::copy_item_to_queue(const void* item, UBaseType_t position) {
    uint8_t* dest = queue_storage_.get() + (position * item_size_);
    std::memcpy(dest, item, item_size_);
}

void Queue::copy_item_from_queue(void* buffer, UBaseType_t position) {
    const uint8_t* src = queue_storage_.get() + (position * item_size_);
    std::memcpy(buffer, src, item_size_);
}

UBaseType_t Queue::get_next_position(UBaseType_t current) const {
    return (current + 1) % queue_length_;
}

UBaseType_t Queue::get_prev_position(UBaseType_t current) const {
    return (current == 0) ? queue_length_ - 1 : current - 1;
}

// SynchronizationAPI Implementation
SemaphoreHandle_t SynchronizationAPI::xSemaphoreCreateBinary() {
    auto semaphore = new Semaphore(Semaphore::SemaphoreType::BINARY, 1, 0);
    LOG_DEBUG("Created binary semaphore");
    return semaphore_to_handle(semaphore);
}

SemaphoreHandle_t SynchronizationAPI::xSemaphoreCreateCounting(UBaseType_t max_count, UBaseType_t initial_count) {
    auto semaphore = new Semaphore(Semaphore::SemaphoreType::COUNTING, max_count, initial_count);
    LOG_DEBUG("Created counting semaphore (max: {}, initial: {})", max_count, initial_count);
    return semaphore_to_handle(semaphore);
}

SemaphoreHandle_t SynchronizationAPI::xSemaphoreCreateMutex() {
    auto semaphore = new Semaphore(Semaphore::SemaphoreType::MUTEX, 1, 1);
    LOG_DEBUG("Created mutex");
    return semaphore_to_handle(semaphore);
}

QueueHandle_t SynchronizationAPI::xQueueCreate(UBaseType_t queue_length, UBaseType_t item_size) {
    auto queue = new Queue(queue_length, item_size);
    LOG_DEBUG("Created queue (length: {}, item_size: {})", queue_length, item_size);
    return queue_to_handle(queue);
}

BaseType_t SynchronizationAPI::xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks_to_wait) {
    Semaphore* sem = handle_to_semaphore(semaphore);
    if (sem) {
        return sem->take(ticks_to_wait);
    }
    return pdFAIL;
}

BaseType_t SynchronizationAPI::xSemaphoreGive(SemaphoreHandle_t semaphore) {
    Semaphore* sem = handle_to_semaphore(semaphore);
    if (sem) {
        return sem->give();
    }
    return pdFAIL;
}

BaseType_t SynchronizationAPI::xQueueSend(QueueHandle_t queue, const void* item, TickType_t ticks_to_wait) {
    Queue* q = handle_to_queue(queue);
    if (q) {
        return q->send(item, ticks_to_wait);
    }
    return pdFAIL;
}

BaseType_t SynchronizationAPI::xQueueReceive(QueueHandle_t queue, void* buffer, TickType_t ticks_to_wait) {
    Queue* q = handle_to_queue(queue);
    if (q) {
        return q->receive(buffer, ticks_to_wait);
    }
    return pdFAIL;
}

// Handle conversion helpers
Semaphore* SynchronizationAPI::handle_to_semaphore(SemaphoreHandle_t handle) {
    return static_cast<Semaphore*>(handle);
}

Queue* SynchronizationAPI::handle_to_queue(QueueHandle_t handle) {
    return static_cast<Queue*>(handle);
}

SemaphoreHandle_t SynchronizationAPI::semaphore_to_handle(Semaphore* semaphore) {
    return static_cast<SemaphoreHandle_t>(semaphore);
}

QueueHandle_t SynchronizationAPI::queue_to_handle(Queue* queue) {
    return static_cast<QueueHandle_t>(queue);
}

} // namespace m5tab5::emulator::freertos