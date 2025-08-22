#include "emulator/memory/dma_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("DmaController");

DmaController::DmaController() 
    : initialized_(false),
      memory_controller_(nullptr),
      interrupt_controller_(nullptr) {
    COMPONENT_LOG_DEBUG("DmaController created");
}

DmaController::~DmaController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("DmaController destroyed");
}

Result<void> DmaController::initialize(const Configuration& config, 
                                       MemoryController& memory_controller,
                                       InterruptController* interrupt_controller) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "DMA controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing DMA controller");
    
    memory_controller_ = &memory_controller;
    interrupt_controller_ = interrupt_controller;
    
    // Initialize DMA channels
    for (int i = 0; i < MAX_DMA_CHANNELS; ++i) {
        channels_[i] = std::make_unique<DmaChannel>(i);
        channels_[i]->reset();
    }
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("DMA controller initialized with {} channels", MAX_DMA_CHANNELS);
    
    return {};
}

Result<void> DmaController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down DMA controller");
    
    // Stop all active transfers
    for (auto& channel : channels_) {
        if (channel && channel->is_active()) {
            channel->abort();
        }
        channel.reset();
    }
    
    memory_controller_ = nullptr;
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("DMA controller shutdown completed");
    return {};
}

Result<void> DmaController::reset() {
    COMPONENT_LOG_INFO("Resetting DMA controller");
    
    // Reset all channels
    for (auto& channel : channels_) {
        if (channel) {
            channel->reset();
        }
    }
    
    // Reset statistics
    statistics_ = {};
    
    return {};
}

Result<u8> DmaController::allocate_channel(DmaPriority priority) {
    // Find first available channel
    for (int i = 0; i < MAX_DMA_CHANNELS; ++i) {
        if (channels_[i] && !channels_[i]->is_allocated()) {
            channels_[i]->allocate(priority);
            COMPONENT_LOG_DEBUG("Allocated DMA channel {} with priority {}", 
                               i, static_cast<int>(priority));
            return static_cast<u8>(i);
        }
    }
    
    return std::unexpected(MAKE_ERROR(SYSTEM_RESOURCE_EXHAUSTED,
        "No DMA channels available"));
}

Result<void> DmaController::free_channel(u8 channel_id) {
    if (channel_id >= MAX_DMA_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel ID: " + std::to_string(channel_id)));
    }
    
    auto& channel = channels_[channel_id];
    if (!channel || !channel->is_allocated()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "DMA channel not allocated: " + std::to_string(channel_id)));
    }
    
    // Abort any active transfer
    if (channel->is_active()) {
        channel->abort();
    }
    
    channel->free();
    COMPONENT_LOG_DEBUG("Freed DMA channel {}", channel_id);
    
    return {};
}

Result<void> DmaController::configure_transfer(u8 channel_id, const DmaTransferConfig& config) {
    if (channel_id >= MAX_DMA_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel ID: " + std::to_string(channel_id)));
    }
    
    auto& channel = channels_[channel_id];
    if (!channel || !channel->is_allocated()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "DMA channel not allocated: " + std::to_string(channel_id)));
    }
    
    if (channel->is_active()) {
        return std::unexpected(MAKE_ERROR(PERIPHERAL_BUSY,
            "DMA channel is active: " + std::to_string(channel_id)));
    }
    
    // Validate transfer configuration
    RETURN_IF_ERROR(validate_transfer_config(config));
    
    // Configure the channel
    RETURN_IF_ERROR(channel->configure(config));
    
    COMPONENT_LOG_DEBUG("Configured DMA channel {} for transfer: src=0x{:08X} dst=0x{:08X} size={}",
                       channel_id, config.source_address, config.destination_address, config.transfer_size);
    
    return {};
}

Result<void> DmaController::start_transfer(u8 channel_id) {
    if (channel_id >= MAX_DMA_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel ID: " + std::to_string(channel_id)));
    }
    
    auto& channel = channels_[channel_id];
    if (!channel || !channel->is_allocated()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "DMA channel not allocated: " + std::to_string(channel_id)));
    }
    
    if (!channel->is_configured()) {
        return std::unexpected(MAKE_ERROR(PERIPHERAL_INVALID_CONFIG,
            "DMA channel not configured: " + std::to_string(channel_id)));
    }
    
    RETURN_IF_ERROR(channel->start());
    
    statistics_.transfers_started++;
    COMPONENT_LOG_DEBUG("Started DMA transfer on channel {}", channel_id);
    
    return {};
}

Result<void> DmaController::abort_transfer(u8 channel_id) {
    if (channel_id >= MAX_DMA_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel ID: " + std::to_string(channel_id)));
    }
    
    auto& channel = channels_[channel_id];
    if (!channel || !channel->is_allocated()) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "DMA channel not allocated: " + std::to_string(channel_id)));
    }
    
    if (!channel->is_active()) {
        return {}; // Already stopped
    }
    
    channel->abort();
    statistics_.transfers_aborted++;
    COMPONENT_LOG_DEBUG("Aborted DMA transfer on channel {}", channel_id);
    
    return {};
}

Result<DmaChannelStatus> DmaController::get_channel_status(u8 channel_id) const {
    if (channel_id >= MAX_DMA_CHANNELS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel ID: " + std::to_string(channel_id)));
    }
    
    const auto& channel = channels_[channel_id];
    if (!channel) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid DMA channel: " + std::to_string(channel_id)));
    }
    
    return channel->get_status();
}

Result<void> DmaController::update() {
    if (!initialized_) {
        return {};
    }
    
    // Process active DMA transfers
    for (int i = 0; i < MAX_DMA_CHANNELS; ++i) {
        auto& channel = channels_[i];
        if (channel && channel->is_active()) {
            RETURN_IF_ERROR(process_channel_transfer(i));
        }
    }
    
    return {};
}

const DmaStatistics& DmaController::get_statistics() const {
    return statistics_;
}

void DmaController::clear_statistics() {
    statistics_ = {};
}

Result<void> DmaController::validate_transfer_config(const DmaTransferConfig& config) const {
    // Validate addresses
    if (!memory_controller_->is_valid_address(config.source_address).value_or(false)) {
        return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Invalid source address: 0x" + std::to_string(config.source_address)));
    }
    
    if (!memory_controller_->is_valid_address(config.destination_address).value_or(false)) {
        return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Invalid destination address: 0x" + std::to_string(config.destination_address)));
    }
    
    // Validate transfer size
    if (config.transfer_size == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Transfer size cannot be zero"));
    }
    
    if (config.transfer_size > MAX_TRANSFER_SIZE) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Transfer size too large: " + std::to_string(config.transfer_size)));
    }
    
    // Validate alignment based on transfer width
    u32 alignment = static_cast<u32>(config.transfer_width);
    if (config.source_address % alignment != 0) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Source address not aligned for transfer width"));
    }
    
    if (config.destination_address % alignment != 0) {
        return std::unexpected(MAKE_ERROR(MEMORY_ALIGNMENT_ERROR,
            "Destination address not aligned for transfer width"));
    }
    
    return {};
}

Result<void> DmaController::process_channel_transfer(u8 channel_id) {
    auto& channel = channels_[channel_id];
    const auto& config = channel->get_config();
    auto& status = channel->get_mutable_status();
    
    // Calculate how much to transfer this cycle
    size_t chunk_size = std::min(static_cast<size_t>(config.transfer_width), 
                                status.remaining_bytes);
    chunk_size = std::min(chunk_size, MAX_BYTES_PER_CYCLE);
    
    // Perform the transfer
    std::vector<u8> buffer(chunk_size);
    
    // Read from source
    auto read_result = memory_controller_->read_bytes(status.current_source_address, chunk_size);
    if (!read_result) {
        channel->set_error("Memory read failed");
        statistics_.transfer_errors++;
        return std::unexpected(read_result.error());
    }
    
    buffer = read_result.value();
    
    // Write to destination
    auto write_result = memory_controller_->write_bytes(status.current_destination_address, 
                                                       buffer.data(), chunk_size);
    if (!write_result) {
        channel->set_error("Memory write failed");
        statistics_.transfer_errors++;
        return std::unexpected(write_result.error());
    }
    
    // Update transfer state
    status.bytes_transferred += chunk_size;
    status.remaining_bytes -= chunk_size;
    
    // Update addresses based on address mode
    switch (config.source_address_mode) {
        case DmaAddressMode::INCREMENT:
            status.current_source_address += chunk_size;
            break;
        case DmaAddressMode::DECREMENT:
            status.current_source_address -= chunk_size;
            break;
        case DmaAddressMode::FIXED:
            // Address stays the same
            break;
    }
    
    switch (config.destination_address_mode) {
        case DmaAddressMode::INCREMENT:
            status.current_destination_address += chunk_size;
            break;
        case DmaAddressMode::DECREMENT:
            status.current_destination_address -= chunk_size;
            break;
        case DmaAddressMode::FIXED:
            // Address stays the same
            break;
    }
    
    statistics_.bytes_transferred += chunk_size;
    
    // Check if transfer is complete
    if (status.remaining_bytes == 0) {
        channel->complete();
        statistics_.transfers_completed++;
        
        COMPONENT_LOG_DEBUG("DMA transfer completed on channel {}: {} bytes transferred",
                           channel_id, status.bytes_transferred);
        
        // Trigger interrupt if enabled
        if (config.interrupt_on_complete && interrupt_controller_) {
            auto interrupt_result = interrupt_controller_->trigger_interrupt(
                CoreId::CORE_0, InterruptType::DMA, channel_id);
            if (!interrupt_result) {
                COMPONENT_LOG_WARN("Failed to trigger DMA completion interrupt: {}",
                                  interrupt_result.error().to_string());
            }
        }
    }
    
    return {};
}

// DmaChannel implementation

DmaChannel::DmaChannel(u8 channel_id) 
    : channel_id_(channel_id),
      allocated_(false),
      configured_(false),
      active_(false),
      priority_(DmaPriority::LOW) {
    reset();
}

void DmaChannel::reset() {
    allocated_ = false;
    configured_ = false;
    active_ = false;
    priority_ = DmaPriority::LOW;
    config_ = {};
    status_ = {};
    error_message_.clear();
}

void DmaChannel::allocate(DmaPriority priority) {
    allocated_ = true;
    priority_ = priority;
    configured_ = false;
    active_ = false;
    error_message_.clear();
}

void DmaChannel::free() {
    if (active_) {
        abort();
    }
    reset();
}

Result<void> DmaChannel::configure(const DmaTransferConfig& config) {
    if (!allocated_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Channel not allocated"));
    }
    
    if (active_) {
        return std::unexpected(MAKE_ERROR(PERIPHERAL_BUSY,
            "Channel is active"));
    }
    
    config_ = config;
    configured_ = true;
    
    // Initialize status
    status_.channel_id = channel_id_;
    status_.state = DmaState::CONFIGURED;
    status_.bytes_transferred = 0;
    status_.remaining_bytes = config.transfer_size;
    status_.current_source_address = config.source_address;
    status_.current_destination_address = config.destination_address;
    status_.error_message.clear();
    
    return {};
}

Result<void> DmaChannel::start() {
    if (!allocated_ || !configured_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Channel not properly configured"));
    }
    
    if (active_) {
        return std::unexpected(MAKE_ERROR(PERIPHERAL_BUSY,
            "Channel already active"));
    }
    
    active_ = true;
    status_.state = DmaState::ACTIVE;
    error_message_.clear();
    
    return {};
}

void DmaChannel::abort() {
    active_ = false;
    status_.state = DmaState::ABORTED;
}

void DmaChannel::complete() {
    active_ = false;
    status_.state = DmaState::COMPLETED;
}

void DmaChannel::set_error(const std::string& error) {
    active_ = false;
    status_.state = DmaState::ERROR;
    error_message_ = error;
    status_.error_message = error;
}

bool DmaChannel::is_allocated() const {
    return allocated_;
}

bool DmaChannel::is_configured() const {
    return configured_;
}

bool DmaChannel::is_active() const {
    return active_;
}

DmaPriority DmaChannel::get_priority() const {
    return priority_;
}

const DmaTransferConfig& DmaChannel::get_config() const {
    return config_;
}

const DmaChannelStatus& DmaChannel::get_status() const {
    return status_;
}

DmaChannelStatus& DmaChannel::get_mutable_status() {
    return status_;
}

}  // namespace m5tab5::emulator