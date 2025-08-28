/**
 * @file esp_heap_caps_api.cpp
 * @brief ESP-IDF Heap Memory Management API implementation for M5Stack Tab5 Emulator
 * 
 * This file implements ESP-IDF compatible heap management functions that integrate
 * with the emulator's memory controller, providing advanced memory allocation with
 * capability-based selection and comprehensive heap statistics.
 */

#include "emulator/esp_idf/esp_heap_caps.h"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/utils/logging.hpp"
#include <mutex>
#include <unordered_map>
#include <atomic>
#include <cstring>
#include <algorithm>
#include <chrono>

namespace {
    using namespace m5tab5::emulator;
    
    // Heap statistics tracking
    struct HeapStats {
        std::atomic<size_t> total_allocated{0};
        std::atomic<size_t> total_free_bytes{32 * 1024 * 1024}; // 32MB total simulated heap
        std::atomic<size_t> minimum_free_bytes{32 * 1024 * 1024};
        std::atomic<size_t> allocation_count{0};
        std::atomic<size_t> free_count{0};
        std::atomic<size_t> largest_free_block{32 * 1024 * 1024};
    };
    
    // Per-capability heap tracking
    static std::unordered_map<uint32_t, HeapStats> heap_stats_by_caps;
    static std::mutex heap_stats_mutex;
    
    // Allocation tracking for debugging
    struct AllocationInfo {
        size_t size;
        uint32_t caps;
        std::chrono::steady_clock::time_point timestamp;
    };
    static std::unordered_map<void*, AllocationInfo> allocation_tracker;
    static std::mutex allocation_tracker_mutex;
    
    /**
     * @brief Get memory controller instance
     */
    MemoryController* get_memory_controller() {
        static MemoryController* controller = nullptr;
        if (!controller) {
            // Try to get from emulator core if available
            // For now, use standard malloc as fallback
        }
        return controller;
    }
    
    /**
     * @brief Update heap statistics after allocation
     */
    void update_stats_after_alloc(uint32_t caps, size_t size) {
        std::lock_guard<std::mutex> lock(heap_stats_mutex);
        auto& stats = heap_stats_by_caps[caps];
        
        stats.total_allocated += size;
        stats.total_free_bytes -= size;
        stats.allocation_count++;
        
        // Update minimum free bytes
        size_t current_free = stats.total_free_bytes.load();
        size_t min_free = stats.minimum_free_bytes.load();
        while (current_free < min_free && 
               !stats.minimum_free_bytes.compare_exchange_weak(min_free, current_free)) {
            // Retry if another thread updated min_free
        }
        
        // Update largest free block (simplified estimation)
        stats.largest_free_block = std::min(stats.largest_free_block.load(), current_free);
        
        LOG_DEBUG("Heap alloc: {} bytes, caps=0x{:x}, total_allocated={}, free_bytes={}", 
                  size, caps, stats.total_allocated.load(), stats.total_free_bytes.load());
    }
    
    /**
     * @brief Update heap statistics after free
     */
    void update_stats_after_free(uint32_t caps, size_t size) {
        std::lock_guard<std::mutex> lock(heap_stats_mutex);
        auto& stats = heap_stats_by_caps[caps];
        
        stats.total_allocated -= size;
        stats.total_free_bytes += size;
        stats.free_count++;
        
        // Update largest free block (simplified estimation)
        stats.largest_free_block = std::max(stats.largest_free_block.load(), stats.total_free_bytes.load());
        
        LOG_DEBUG("Heap free: {} bytes, caps=0x{:x}, total_allocated={}, free_bytes={}", 
                  size, caps, stats.total_allocated.load(), stats.total_free_bytes.load());
    }
    
    /**
     * @brief Check if capabilities are supported
     */
    bool are_caps_supported(uint32_t caps) {
        // For emulation, we support all standard capabilities
        const uint32_t supported_caps = 
            MALLOC_CAP_EXEC | MALLOC_CAP_32BIT | MALLOC_CAP_8BIT |
            MALLOC_CAP_DMA | MALLOC_CAP_PID2 | MALLOC_CAP_PID3 |
            MALLOC_CAP_PID4 | MALLOC_CAP_PID5 | MALLOC_CAP_PID6 |
            MALLOC_CAP_PID7 | MALLOC_CAP_SPIRAM | MALLOC_CAP_INTERNAL |
            MALLOC_CAP_DEFAULT | MALLOC_CAP_IRAM_8BIT | MALLOC_CAP_RETENTION;
            
        return (caps & ~supported_caps) == 0;
    }
}

extern "C" {

// ============================================================================
// Core Memory Allocation Functions
// ============================================================================

void* heap_caps_malloc(size_t size, uint32_t caps) {
    if (size == 0) {
        LOG_DEBUG("heap_caps_malloc: zero size requested");
        return nullptr;
    }
    
    if (!are_caps_supported(caps)) {
        LOG_ERROR("heap_caps_malloc: unsupported capabilities 0x{:x}", caps);
        return nullptr;
    }
    
    LOG_DEBUG("heap_caps_malloc: {} bytes with caps 0x{:x}", size, caps);
    
    // For emulation, use standard malloc with capability simulation
    void* ptr = malloc(size);
    if (ptr) {
        // Track allocation
        {
            std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
            allocation_tracker[ptr] = {size, caps, std::chrono::steady_clock::now()};
        }
        
        update_stats_after_alloc(caps, size);
        
        // For DMA capability, ensure alignment
        if (caps & MALLOC_CAP_DMA) {
            // DMA memory should be properly aligned
            if (reinterpret_cast<uintptr_t>(ptr) % 4 != 0) {
                LOG_WARN("heap_caps_malloc: DMA memory not 4-byte aligned");
            }
        }
    } else {
        LOG_ERROR("heap_caps_malloc: failed to allocate {} bytes with caps 0x{:x}", size, caps);
    }
    
    return ptr;
}

void* heap_caps_calloc(size_t n, size_t size, uint32_t caps) {
    if (n == 0 || size == 0) {
        LOG_DEBUG("heap_caps_calloc: zero count or size requested");
        return nullptr;
    }
    
    // Check for overflow
    if (n > SIZE_MAX / size) {
        LOG_ERROR("heap_caps_calloc: size overflow");
        return nullptr;
    }
    
    size_t total_size = n * size;
    void* ptr = heap_caps_malloc(total_size, caps);
    if (ptr) {
        memset(ptr, 0, total_size);
    }
    
    return ptr;
}

void* heap_caps_realloc(void* ptr, size_t size, uint32_t caps) {
    if (ptr == nullptr) {
        return heap_caps_malloc(size, caps);
    }
    
    if (size == 0) {
        heap_caps_free(ptr);
        return nullptr;
    }
    
    // Get original allocation info
    AllocationInfo orig_info = {0};
    {
        std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
        auto it = allocation_tracker.find(ptr);
        if (it != allocation_tracker.end()) {
            orig_info = it->second;
        }
    }
    
    LOG_DEBUG("heap_caps_realloc: {} bytes to {} bytes with caps 0x{:x}", 
              orig_info.size, size, caps);
    
    void* new_ptr = realloc(ptr, size);
    if (new_ptr) {
        // Update tracking
        {
            std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
            if (new_ptr != ptr) {
                allocation_tracker.erase(ptr);
            }
            allocation_tracker[new_ptr] = {size, caps, std::chrono::steady_clock::now()};
        }
        
        // Update statistics
        if (orig_info.size > 0) {
            update_stats_after_free(orig_info.caps, orig_info.size);
        }
        update_stats_after_alloc(caps, size);
    } else {
        LOG_ERROR("heap_caps_realloc: failed to reallocate {} bytes", size);
    }
    
    return new_ptr;
}

void heap_caps_free(void* ptr) {
    if (ptr == nullptr) {
        LOG_DEBUG("heap_caps_free: null pointer");
        return;
    }
    
    // Get allocation info for statistics
    AllocationInfo info = {0};
    {
        std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
        auto it = allocation_tracker.find(ptr);
        if (it != allocation_tracker.end()) {
            info = it->second;
            allocation_tracker.erase(it);
        }
    }
    
    if (info.size > 0) {
        update_stats_after_free(info.caps, info.size);
        LOG_DEBUG("heap_caps_free: {} bytes with caps 0x{:x}", info.size, info.caps);
    } else {
        LOG_WARN("heap_caps_free: freeing untracked pointer {}", ptr);
    }
    
    free(ptr);
}

// ============================================================================
// Heap Information and Statistics Functions
// ============================================================================

size_t heap_caps_get_free_size(uint32_t caps) {
    std::lock_guard<std::mutex> lock(heap_stats_mutex);
    auto it = heap_stats_by_caps.find(caps);
    if (it != heap_stats_by_caps.end()) {
        size_t free_size = it->second.total_free_bytes.load();
        LOG_DEBUG("heap_caps_get_free_size: {} bytes for caps 0x{:x}", free_size, caps);
        return free_size;
    }
    
    // Default free size for new capability sets
    const size_t default_free = 16 * 1024 * 1024; // 16MB default
    LOG_DEBUG("heap_caps_get_free_size: {} bytes (default) for caps 0x{:x}", default_free, caps);
    return default_free;
}

size_t heap_caps_get_minimum_free_size(uint32_t caps) {
    std::lock_guard<std::mutex> lock(heap_stats_mutex);
    auto it = heap_stats_by_caps.find(caps);
    if (it != heap_stats_by_caps.end()) {
        size_t min_free = it->second.minimum_free_bytes.load();
        LOG_DEBUG("heap_caps_get_minimum_free_size: {} bytes for caps 0x{:x}", min_free, caps);
        return min_free;
    }
    
    const size_t default_min_free = 16 * 1024 * 1024; // 16MB default
    LOG_DEBUG("heap_caps_get_minimum_free_size: {} bytes (default) for caps 0x{:x}", default_min_free, caps);
    return default_min_free;
}

size_t heap_caps_get_largest_free_block(uint32_t caps) {
    std::lock_guard<std::mutex> lock(heap_stats_mutex);
    auto it = heap_stats_by_caps.find(caps);
    if (it != heap_stats_by_caps.end()) {
        size_t largest = it->second.largest_free_block.load();
        LOG_DEBUG("heap_caps_get_largest_free_block: {} bytes for caps 0x{:x}", largest, caps);
        return largest;
    }
    
    const size_t default_largest = 16 * 1024 * 1024; // 16MB default
    LOG_DEBUG("heap_caps_get_largest_free_block: {} bytes (default) for caps 0x{:x}", default_largest, caps);
    return default_largest;
}

void heap_caps_get_info(multi_heap_info_t* info, uint32_t caps) {
    if (!info) {
        LOG_ERROR("heap_caps_get_info: null info pointer");
        return;
    }
    
    std::lock_guard<std::mutex> lock(heap_stats_mutex);
    auto it = heap_stats_by_caps.find(caps);
    
    if (it != heap_stats_by_caps.end()) {
        const auto& stats = it->second;
        info->total_free_bytes = stats.total_free_bytes.load();
        info->total_allocated_bytes = stats.total_allocated.load();
        info->largest_free_block = stats.largest_free_block.load();
        info->minimum_free_bytes = stats.minimum_free_bytes.load();
        info->allocated_blocks = stats.allocation_count.load() - stats.free_count.load();
        info->free_blocks = 1000; // Simulated block count
        info->total_blocks = info->allocated_blocks + info->free_blocks;
    } else {
        // Default values for untracked capability sets
        info->total_free_bytes = 16 * 1024 * 1024;
        info->total_allocated_bytes = 0;
        info->largest_free_block = 16 * 1024 * 1024;
        info->minimum_free_bytes = 16 * 1024 * 1024;
        info->allocated_blocks = 0;
        info->free_blocks = 1000;
        info->total_blocks = 1000;
    }
    
    LOG_DEBUG("heap_caps_get_info: caps=0x{:x}, free={}, allocated={}, largest_free={}", 
              caps, info->total_free_bytes, info->total_allocated_bytes, info->largest_free_block);
}

void heap_caps_print_heap_info(uint32_t caps) {
    multi_heap_info_t info;
    heap_caps_get_info(&info, caps);
    
    LOG_INFO("Heap Info for capabilities 0x{:08x}:", caps);
    LOG_INFO("  Total free bytes:     {}", info.total_free_bytes);
    LOG_INFO("  Total allocated bytes: {}", info.total_allocated_bytes);
    LOG_INFO("  Largest free block:   {}", info.largest_free_block);
    LOG_INFO("  Minimum free bytes:   {}", info.minimum_free_bytes);
    LOG_INFO("  Allocated blocks:     {}", info.allocated_blocks);
    LOG_INFO("  Free blocks:          {}", info.free_blocks);
    LOG_INFO("  Total blocks:         {}", info.total_blocks);
}

bool heap_caps_check_integrity_all(bool print_errors) {
    LOG_DEBUG("heap_caps_check_integrity_all: checking all heaps (print_errors={})", print_errors);
    
    // For emulation, assume integrity is always OK
    // In real implementation, this would walk heap structures
    
    if (print_errors) {
        LOG_INFO("Heap integrity check: All heaps OK");
    }
    
    return true;
}

bool heap_caps_check_integrity(uint32_t caps, bool print_errors) {
    LOG_DEBUG("heap_caps_check_integrity: checking caps 0x{:x} (print_errors={})", caps, print_errors);
    
    if (!are_caps_supported(caps)) {
        if (print_errors) {
            LOG_ERROR("Heap integrity check: Unsupported capabilities 0x{:x}", caps);
        }
        return false;
    }
    
    if (print_errors) {
        LOG_INFO("Heap integrity check for caps 0x{:x}: OK", caps);
    }
    
    return true;
}

// ============================================================================
// Memory Alignment Functions
// ============================================================================

void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps) {
    if (size == 0) {
        LOG_DEBUG("heap_caps_aligned_alloc: zero size requested");
        return nullptr;
    }
    
    if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
        LOG_ERROR("heap_caps_aligned_alloc: invalid alignment {}", alignment);
        return nullptr;
    }
    
    if (!are_caps_supported(caps)) {
        LOG_ERROR("heap_caps_aligned_alloc: unsupported capabilities 0x{:x}", caps);
        return nullptr;
    }
    
    LOG_DEBUG("heap_caps_aligned_alloc: {} bytes aligned to {} with caps 0x{:x}", 
              size, alignment, caps);
    
#if defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 16
    void* ptr = aligned_alloc(alignment, size);
#elif defined(_WIN32)
    void* ptr = _aligned_malloc(size, alignment);
#else
    void* ptr;
    int ret = posix_memalign(&ptr, alignment, size);
    if (ret != 0) {
        ptr = nullptr;
    }
#endif
    
    if (ptr) {
        // Track allocation
        {
            std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
            allocation_tracker[ptr] = {size, caps, std::chrono::steady_clock::now()};
        }
        
        update_stats_after_alloc(caps, size);
        
        LOG_DEBUG("heap_caps_aligned_alloc: allocated {} bytes at {} (alignment {})", 
                  size, ptr, alignment);
    } else {
        LOG_ERROR("heap_caps_aligned_alloc: failed to allocate {} bytes aligned to {}", size, alignment);
    }
    
    return ptr;
}

void heap_caps_aligned_free(void* ptr) {
    if (ptr == nullptr) {
        LOG_DEBUG("heap_caps_aligned_free: null pointer");
        return;
    }
    
    // Get allocation info for statistics
    AllocationInfo info = {0};
    {
        std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
        auto it = allocation_tracker.find(ptr);
        if (it != allocation_tracker.end()) {
            info = it->second;
            allocation_tracker.erase(it);
        }
    }
    
    if (info.size > 0) {
        update_stats_after_free(info.caps, info.size);
        LOG_DEBUG("heap_caps_aligned_free: {} bytes with caps 0x{:x}", info.size, info.caps);
    } else {
        LOG_WARN("heap_caps_aligned_free: freeing untracked aligned pointer {}", ptr);
    }
    
#if defined(_WIN32)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

// ============================================================================
// Convenience Functions
// ============================================================================

void* heap_caps_malloc_default(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}

void* heap_caps_realloc_default(void* ptr, size_t size) {
    return heap_caps_realloc(ptr, size, MALLOC_CAP_DEFAULT);
}

void* heap_caps_calloc_default(size_t n, size_t size) {
    return heap_caps_calloc(n, size, MALLOC_CAP_DEFAULT);
}

// ============================================================================
// Debug and Analysis Functions
// ============================================================================

size_t heap_caps_get_allocated_size(void* ptr) {
    if (ptr == nullptr) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
    auto it = allocation_tracker.find(ptr);
    if (it != allocation_tracker.end()) {
        size_t size = it->second.size;
        LOG_DEBUG("heap_caps_get_allocated_size: {} bytes for pointer {}", size, ptr);
        return size;
    }
    
    LOG_WARN("heap_caps_get_allocated_size: untracked pointer {}", ptr);
    return 0;
}

esp_err_t heap_caps_dump(uint32_t caps) {
    LOG_INFO("=== Heap Dump for capabilities 0x{:08x} ===", caps);
    
    multi_heap_info_t info;
    heap_caps_get_info(&info, caps);
    
    LOG_INFO("Total free bytes:      {}", info.total_free_bytes);
    LOG_INFO("Total allocated bytes: {}", info.total_allocated_bytes);
    LOG_INFO("Largest free block:    {}", info.largest_free_block);
    LOG_INFO("Minimum free bytes:    {}", info.minimum_free_bytes);
    LOG_INFO("Allocated blocks:      {}", info.allocated_blocks);
    LOG_INFO("Free blocks:           {}", info.free_blocks);
    LOG_INFO("Total blocks:          {}", info.total_blocks);
    
    // Dump allocation tracker info for debugging
    std::lock_guard<std::mutex> lock(allocation_tracker_mutex);
    LOG_INFO("Currently tracked allocations: {}", allocation_tracker.size());
    
    size_t total_tracked = 0;
    for (const auto& pair : allocation_tracker) {
        if ((pair.second.caps & caps) == caps) {
            total_tracked += pair.second.size;
        }
    }
    LOG_INFO("Total tracked allocation size for caps: {}", total_tracked);
    
    LOG_INFO("=== End Heap Dump ===");
    return ESP_OK;
}

esp_err_t heap_caps_dump_all() {
    LOG_INFO("=== Complete Heap Dump ===");
    
    std::lock_guard<std::mutex> lock(heap_stats_mutex);
    for (const auto& pair : heap_stats_by_caps) {
        heap_caps_dump(pair.first);
    }
    
    // Also dump default capabilities
    heap_caps_dump(MALLOC_CAP_DEFAULT);
    
    LOG_INFO("=== End Complete Heap Dump ===");
    return ESP_OK;
}

} // extern "C"