#pragma once

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"

#include <memory>
#include <vector>
#include <array>
#include <queue>
#include <mutex>
#include <atomic>
#include <functional>
#include <string>

namespace m5tab5::emulator {

// Forward declarations
class MemoryController;
class InterruptController;

/**
 * @brief DMA (Direct Memory Access) controller for ESP32-P4 emulation
 * 
 * Implements high-speed data transfer between memory and peripherals
 * without CPU intervention, supporting both 2D and linked-list DMA modes.
 */

// DMA transfer modes
enum class DmaTransferMode {
    NORMAL,         // Simple memory-to-memory or peripheral transfers
    CIRCULAR,       // Continuous circular buffer transfers
    LINKED_LIST,    // Chained transfers using linked descriptor lists
    TWO_DIMENSIONAL // 2D block transfers with stride support
};

// DMA transfer directions
enum class DmaDirection {
    MEMORY_TO_MEMORY,
    MEMORY_TO_PERIPHERAL,
    PERIPHERAL_TO_MEMORY,
    PERIPHERAL_TO_PERIPHERAL
};

// DMA channel priorities
enum class DmaPriority {
    LOW = 0,
    MEDIUM = 1,
    HIGH = 2,
    VERY_HIGH = 3
};

// DMA address modes
enum class DmaAddressMode {
    INCREMENT,
    DECREMENT,
    FIXED
};

// DMA transfer width
enum class DmaTransferWidth {
    BYTE = 1,
    HALFWORD = 2,
    WORD = 4
};

// DMA channel state
enum class DmaState {
    IDLE,
    CONFIGURED,
    ACTIVE,
    COMPLETED,
    ABORTED,
    ERROR
};

// DMA transfer configuration
struct DmaTransferConfig {
    Address source_address;
    Address destination_address;
    u32 transfer_size;
    DmaTransferWidth transfer_width;
    DmaAddressMode source_address_mode;
    DmaAddressMode destination_address_mode;
    bool interrupt_on_complete;
    
    DmaTransferConfig()
        : source_address(0), destination_address(0), transfer_size(0),
          transfer_width(DmaTransferWidth::WORD),
          source_address_mode(DmaAddressMode::INCREMENT),
          destination_address_mode(DmaAddressMode::INCREMENT),
          interrupt_on_complete(true) {}
};

// DMA channel status
struct DmaChannelStatus {
    u8 channel_id;
    DmaState state;
    u32 bytes_transferred;
    u32 remaining_bytes;
    Address current_source_address;
    Address current_destination_address;
    std::string error_message;
    
    DmaChannelStatus()
        : channel_id(0), state(DmaState::IDLE), bytes_transferred(0),
          remaining_bytes(0), current_source_address(0),
          current_destination_address(0) {}
};

// DMA controller statistics
struct DmaStatistics {
    u64 transfers_started;
    u64 transfers_completed;
    u64 transfers_aborted;
    u64 transfer_errors;
    u64 bytes_transferred;
    
    DmaStatistics()
        : transfers_started(0), transfers_completed(0), transfers_aborted(0),
          transfer_errors(0), bytes_transferred(0) {}
};

// DMA transfer descriptor for linked-list mode
struct DmaDescriptor {
    Address next_descriptor;    // Address of next descriptor (0 = end)
    Address source_address;     // Source memory address
    Address dest_address;       // Destination address
    u32 transfer_size;         // Number of bytes to transfer
    u32 control_flags;         // Transfer control flags
    
    // 2D transfer parameters (when applicable)
    u32 source_stride;         // Source row stride in bytes
    u32 dest_stride;           // Destination row stride in bytes
    u32 block_width;           // Block width in bytes
    u32 block_height;          // Number of rows
    
    DmaDescriptor() 
        : next_descriptor(0), source_address(0), dest_address(0), 
          transfer_size(0), control_flags(0), source_stride(0),
          dest_stride(0), block_width(0), block_height(0) {}
};

// DMA channel configuration
struct DmaChannelConfig {
    DmaTransferMode mode;
    DmaDirection direction;
    DmaPriority priority;
    u32 peripheral_id;         // For peripheral transfers
    bool interrupt_enable;
    bool error_interrupt_enable;
    
    DmaChannelConfig()
        : mode(DmaTransferMode::NORMAL), direction(DmaDirection::MEMORY_TO_MEMORY),
          priority(DmaPriority::MEDIUM), peripheral_id(0),
          interrupt_enable(true), error_interrupt_enable(true) {}
};

// DMA channel state
enum class DmaChannelState {
    IDLE,
    CONFIGURED,
    RUNNING,
    PAUSED,
    COMPLETED,
    ERROR
};

// DMA channel statistics
struct DmaChannelStats {
    u64 transfers_completed;
    u64 bytes_transferred;
    u64 errors_count;
    u64 avg_transfer_rate;     // Bytes per second
    
    DmaChannelStats() : transfers_completed(0), bytes_transferred(0),
                       errors_count(0), avg_transfer_rate(0) {}
};

// DMA transfer completion callback
using DmaCompletionCallback = std::function<void(u32 channel_id, bool success, u32 bytes_transferred)>;

/**
 * @brief Individual DMA channel
 */
class DmaChannel {
public:
    static constexpr u32 INVALID_CHANNEL = 0xFFFFFFFF;
    
    DmaChannel(u8 channel_id);
    ~DmaChannel() = default;
    
    // Lifecycle
    void reset();
    void allocate(DmaPriority priority);
    void free();
    
    // Configuration
    Result<void> configure(const DmaTransferConfig& config);
    Result<void> set_addresses(Address source, Address dest, u32 size);
    Result<void> set_descriptor_list(Address first_descriptor);
    Result<void> set_2d_parameters(u32 src_stride, u32 dst_stride, 
                                  u32 block_width, u32 block_height);
    
    // Control
    Result<void> start();
    void abort();
    void complete();
    void set_error(const std::string& error);
    
    // Status
    bool is_allocated() const;
    bool is_configured() const;
    bool is_active() const;
    DmaPriority get_priority() const;
    const DmaTransferConfig& get_config() const;
    const DmaChannelStatus& get_status() const;
    DmaChannelStatus& get_mutable_status();
    
    // Legacy compatibility
    DmaChannelState get_state() const { return static_cast<DmaChannelState>(status_.state); }
    bool is_busy() const { return is_active(); }
    u32 get_remaining_bytes() const { return status_.remaining_bytes; }
    float get_progress_percent() const {
        if (config_.transfer_size == 0) return 0.0f;
        return (static_cast<float>(status_.bytes_transferred) / config_.transfer_size) * 100.0f;
    }
    
    // Callbacks
    void set_completion_callback(DmaCompletionCallback callback) { 
        completion_callback_ = std::move(callback); 
    }
    
    // Statistics
    const DmaChannelStats& get_statistics() const { return stats_; }
    void reset_statistics();
    
    // Internal execution (called by DMA controller)
    Result<u32> execute_transfer(MemoryController& memory, u32 max_bytes);

private:
    u8 channel_id_;
    bool allocated_;
    bool configured_;
    bool active_;
    DmaPriority priority_;
    DmaTransferConfig config_;
    DmaChannelStatus status_;
    std::string error_message_;
    
    // Legacy members
    std::atomic<DmaChannelState> state_;
    Address current_source_;
    Address current_dest_;
    u32 remaining_bytes_;
    u32 total_transfer_size_;
    
    // Linked-list mode
    Address current_descriptor_;
    DmaDescriptor current_desc_;
    
    // Callbacks and statistics
    DmaCompletionCallback completion_callback_;
    DmaChannelStats stats_;
    
    // Internal helpers
    Result<void> load_next_descriptor(MemoryController& memory);
    Result<u32> execute_normal_transfer(MemoryController& memory, u32 max_bytes);
    Result<u32> execute_2d_transfer(MemoryController& memory, u32 max_bytes);
    void update_statistics(u32 bytes_transferred);
    void trigger_completion(bool success);
};

/**
 * @brief Main DMA controller
 */
class DmaController {
public:
    static constexpr u32 MAX_DMA_CHANNELS = 8;
    static constexpr u32 DMA_QUANTUM_BYTES = 1024; // Bytes per execution quantum
    static constexpr u32 MAX_TRANSFER_SIZE = 0x100000; // 1MB max transfer
    static constexpr u32 MAX_BYTES_PER_CYCLE = 1024; // Max bytes per processing cycle

    DmaController();
    ~DmaController();
    
    // Lifecycle
    Result<void> initialize(const Configuration& config,
                           MemoryController& memory_controller,
                           InterruptController* interrupt_controller = nullptr);
    Result<void> shutdown();
    Result<void> reset();
    
    // Channel management
    Result<u8> allocate_channel(DmaPriority priority = DmaPriority::MEDIUM);
    Result<void> free_channel(u8 channel_id);
    Result<DmaChannel*> get_channel(u32 channel_id);
    
    // Transfer operations
    Result<void> configure_transfer(u8 channel_id, const DmaTransferConfig& config);
    Result<void> start_transfer(u8 channel_id);
    Result<void> abort_transfer(u8 channel_id);
    Result<DmaChannelStatus> get_channel_status(u8 channel_id) const;
    
    // Update and processing
    Result<void> update();
    
    // Global control
    void set_enabled(bool enabled) { enabled_ = enabled; }
    bool is_enabled() const { return enabled_; }
    
    // Execution (called by system tick)
    Result<void> tick(ClockCycle cycle);
    
    // Statistics and monitoring
    const DmaStatistics& get_statistics() const;
    void clear_statistics();
    u32 get_active_channels() const;
    u64 get_total_bytes_transferred() const;
    void dump_channel_states() const;
    
    // Configuration
    void set_quantum_size(u32 bytes) { quantum_bytes_ = bytes; }
    u32 get_quantum_size() const { return quantum_bytes_; }

private:
    bool initialized_;
    bool enabled_;
    MemoryController* memory_controller_;
    InterruptController* interrupt_controller_;
    
    // Channel management
    std::array<std::unique_ptr<DmaChannel>, MAX_DMA_CHANNELS> channels_;
    std::queue<u32> free_channels_;
    mutable std::mutex channels_mutex_;
    
    // Execution scheduling
    u32 quantum_bytes_;
    u32 current_channel_index_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    DmaStatistics statistics_;
    u64 total_bytes_transferred_;
    
    // Internal helpers
    Result<void> setup_channels();
    Result<void> validate_transfer_config(const DmaTransferConfig& config) const;
    Result<void> process_channel_transfer(u8 channel_id);
    void schedule_next_channel();
    void handle_channel_completion(u32 channel_id, bool success);
    bool is_valid_channel_id(u32 channel_id) const;
};

} // namespace m5tab5::emulator