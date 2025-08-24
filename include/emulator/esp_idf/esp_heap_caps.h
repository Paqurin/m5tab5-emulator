/**
 * @file esp_heap_caps.h
 * @brief ESP-IDF heap capabilities API for M5Stack Tab5 Emulator
 * 
 * This header provides ESP-IDF compatible memory allocation functions with
 * capability-based memory management for different memory regions.
 */

#pragma once

#include "esp_system.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Memory capabilities flags
 */
#define MALLOC_CAP_EXEC             (1<<0)  ///< Memory must be able to run executable code
#define MALLOC_CAP_32BIT            (1<<1)  ///< Memory must allow for aligned 32-bit data accesses
#define MALLOC_CAP_8BIT             (1<<2)  ///< Memory must allow for 8/16/...-bit data accesses
#define MALLOC_CAP_DMA              (1<<3)  ///< Memory must be able to accessed by DMA
#define MALLOC_CAP_PID2             (1<<4)  ///< Memory must be mapped to PID2 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID3             (1<<5)  ///< Memory must be mapped to PID3 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID4             (1<<6)  ///< Memory must be mapped to PID4 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID5             (1<<7)  ///< Memory must be mapped to PID5 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID6             (1<<8)  ///< Memory must be mapped to PID6 memory space (PIDs are not currently used)
#define MALLOC_CAP_PID7             (1<<9)  ///< Memory must be mapped to PID7 memory space (PIDs are not currently used)
#define MALLOC_CAP_SPIRAM           (1<<10) ///< Memory must be in SPI RAM
#define MALLOC_CAP_INTERNAL         (1<<11) ///< Memory must be internal; specifically it should not disappear when flash/spiram cache is switched off
#define MALLOC_CAP_DEFAULT          (1<<12) ///< Memory can be returned in a non-capability-specific memory allocation (malloc(), calloc(), etc.) call
#define MALLOC_CAP_IRAM_8BIT        (1<<13) ///< Memory must be in IRAM and allow unaligned access
#define MALLOC_CAP_RETENTION        (1<<14) ///< Memory must be able to accessed by retention DMA
#define MALLOC_CAP_RTCRAM           (1<<15) ///< Memory must be in RTC fast memory

/**
 * @brief Convenience macros for common capability combinations
 */
#define MALLOC_CAP_IRAM             (MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL)
#define MALLOC_CAP_DRAM             (MALLOC_CAP_8BIT | MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL)

/**
 * @brief Multi-heap information structure
 */
typedef struct {
    uint32_t total_free_bytes;      ///< Total free bytes in all regions
    uint32_t total_allocated_bytes; ///< Total allocated bytes in all regions
    uint32_t largest_free_block;    ///< Size of largest free block
    uint32_t minimum_free_bytes;    ///< Minimum free bytes ever available
    uint32_t allocated_blocks;      ///< Number of allocated blocks
    uint32_t free_blocks;           ///< Number of free blocks
    uint32_t total_blocks;          ///< Total number of blocks
} multi_heap_info_t;

/**
 * @brief Heap region information structure
 */
typedef struct {
    void* start_addr;               ///< Start address of the region
    size_t total_size;             ///< Total size of the region
    uint32_t caps;                 ///< Capabilities flags for this region
    const char* type_desc;         ///< Type description string
} heap_caps_region_info_t;

// ============================================================================
// Memory Allocation Functions
// ============================================================================

/**
 * @brief Allocate memory with capabilities
 * 
 * @param size Size to allocate
 * @param caps Capabilities flags
 * @return Pointer to allocated memory, or NULL on failure
 */
void* heap_caps_malloc(size_t size, uint32_t caps);

/**
 * @brief Allocate aligned memory with capabilities
 * 
 * @param alignment Required alignment (must be power of 2)
 * @param size Size to allocate
 * @param caps Capabilities flags
 * @return Pointer to allocated memory, or NULL on failure
 */
void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps);

/**
 * @brief Allocate zero-initialized memory with capabilities
 * 
 * @param n Number of elements
 * @param size Size of each element
 * @param caps Capabilities flags
 * @return Pointer to allocated memory, or NULL on failure
 */
void* heap_caps_calloc(size_t n, size_t size, uint32_t caps);

/**
 * @brief Reallocate memory with capabilities
 * 
 * @param ptr Existing memory pointer (can be NULL)
 * @param size New size
 * @param caps Capabilities flags
 * @return Pointer to reallocated memory, or NULL on failure
 */
void* heap_caps_realloc(void* ptr, size_t size, uint32_t caps);

/**
 * @brief Free memory allocated with heap_caps functions
 * 
 * @param ptr Pointer to memory to free
 */
void heap_caps_free(void* ptr);

// ============================================================================
// Memory Information Functions
// ============================================================================

/**
 * @brief Get free heap size for specific capabilities
 * 
 * @param caps Capabilities flags
 * @return Free heap size in bytes
 */
size_t heap_caps_get_free_size(uint32_t caps);

/**
 * @brief Get minimum ever free heap size for specific capabilities
 * 
 * @param caps Capabilities flags
 * @return Minimum free heap size in bytes
 */
size_t heap_caps_get_minimum_free_size(uint32_t caps);

/**
 * @brief Get largest free block size for specific capabilities
 * 
 * @param caps Capabilities flags
 * @return Largest free block size in bytes
 */
size_t heap_caps_get_largest_free_block(uint32_t caps);

/**
 * @brief Get total heap size for specific capabilities
 * 
 * @param caps Capabilities flags
 * @return Total heap size in bytes
 */
size_t heap_caps_get_total_size(uint32_t caps);

/**
 * @brief Get size of allocated memory block
 * 
 * @param ptr Pointer to allocated memory
 * @return Size of the block, or 0 if invalid
 */
size_t heap_caps_get_allocated_size(const void* ptr);

// ============================================================================
// Multi-heap Information Functions
// ============================================================================

/**
 * @brief Get multi-heap information for specific capabilities
 * 
 * @param info Structure to store information
 * @param caps Capabilities flags
 */
void heap_caps_get_info(multi_heap_info_t* info, uint32_t caps);

/**
 * @brief Print heap information to console
 * 
 * @param caps Capabilities flags (0 for all heaps)
 */
void heap_caps_print_heap_info(uint32_t caps);

/**
 * @brief Check heap integrity for specific capabilities
 * 
 * @param caps Capabilities flags (0 for all heaps)
 * @param print_errors Whether to print errors to console
 * @return true if heap is valid
 */
bool heap_caps_check_integrity(uint32_t caps, bool print_errors);

/**
 * @brief Check if heap corruption has been detected
 * 
 * @param caps Capabilities flags
 * @return true if corruption detected
 */
bool heap_caps_check_integrity_all(bool print_errors);

/**
 * @brief Dump heap information in detail
 * 
 * @param caps Capabilities flags
 */
void heap_caps_dump(uint32_t caps);

/**
 * @brief Dump all heap regions information
 * 
 * @param caps Capabilities flags
 */
void heap_caps_dump_all(void);

// ============================================================================
// Region Management Functions
// ============================================================================

/**
 * @brief Get number of heap regions
 * 
 * @return Number of heap regions
 */
size_t heap_caps_get_regions(heap_caps_region_info_t* regions, size_t max_regions);

/**
 * @brief Add memory region to heap
 * 
 * @param start Start address of region
 * @param size Size of region
 * @param caps Capabilities flags for this region
 * @return ESP_OK on success
 */
esp_err_t heap_caps_add_region(intptr_t start, size_t size, uint32_t caps);

// ============================================================================
// Debug and Testing Functions
// ============================================================================

/**
 * @brief Set heap allocation failure simulation
 * 
 * For testing purposes only. Makes allocation fail after N successful allocations.
 * 
 * @param fail_after Number of successful allocations before failing (0 to disable)
 */
void heap_caps_set_allocation_failure_after(size_t fail_after);

/**
 * @brief Check if pointer was allocated with heap_caps functions
 * 
 * @param ptr Pointer to check
 * @return true if allocated with heap_caps functions
 */
bool heap_caps_check_allocated(const void* ptr);

/**
 * @brief Walk through all allocated blocks
 * 
 * @param visitor Function to call for each block
 * @param user_data User data passed to visitor function
 */
typedef void (*heap_caps_walker_cb_t)(void* ptr, size_t size, void* user_data);
void heap_caps_walk_all(heap_caps_walker_cb_t visitor, void* user_data);

/**
 * @brief Get heap statistics
 * 
 * @param caps Capabilities flags
 * @param heap_stats Structure to store statistics
 * @return ESP_OK on success
 */
typedef struct {
    size_t total_free;      ///< Total free memory
    size_t total_allocated; ///< Total allocated memory
    size_t largest_free;    ///< Largest free block
    size_t minimum_free;    ///< Minimum ever free
    size_t allocation_count; ///< Number of allocations
    size_t free_count;      ///< Number of frees
    size_t failed_allocs;   ///< Number of failed allocations
} heap_caps_stats_t;

esp_err_t heap_caps_get_stats(heap_caps_stats_t* heap_stats, uint32_t caps);

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Check if capabilities are available
 * 
 * @param caps Capabilities flags to check
 * @return true if capabilities are available
 */
bool heap_caps_match(uint32_t caps);

/**
 * @brief Get description string for capabilities
 * 
 * @param caps Capabilities flags
 * @return Description string
 */
const char* heap_caps_get_description(uint32_t caps);

// ============================================================================
// Standard C Library Compatibility
// ============================================================================

// Note: Standard C library compatibility is disabled to avoid conflicts
// with existing memory allocation. Use heap_caps_* functions directly.
// 
// #ifndef CONFIG_HEAP_DISABLE_IRAM
// #define malloc(size) heap_caps_malloc(size, MALLOC_CAP_DEFAULT)
// #define calloc(n, size) heap_caps_calloc(n, size, MALLOC_CAP_DEFAULT)
// #define realloc(ptr, size) heap_caps_realloc(ptr, size, MALLOC_CAP_DEFAULT)
// #define free(ptr) heap_caps_free(ptr)
// #endif
//
// // Aligned allocation functions
// #define aligned_alloc(alignment, size) heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_DEFAULT)
// #define memalign(alignment, size) heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_DEFAULT)

#ifdef __cplusplus
}
#endif