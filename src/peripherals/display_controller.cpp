#include "emulator/peripherals/display_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("DisplayController");

DisplayController::DisplayController()
    : initialized_(false),
      enabled_(false),
      memory_controller_(nullptr),
      interrupt_controller_(nullptr),
      framebuffer_(nullptr) {
    COMPONENT_LOG_DEBUG("DisplayController created");
}

DisplayController::~DisplayController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("DisplayController destroyed");
}

Result<void> DisplayController::initialize(const Configuration& config,
                                           MemoryController& memory_controller,
                                           InterruptController* interrupt_controller) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Display controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing display controller");
    
    memory_controller_ = &memory_controller;
    interrupt_controller_ = interrupt_controller;
    
    // Initialize display configuration from config
    display_config_.width = config.get_display_width();
    display_config_.height = config.get_display_height();
    display_config_.refresh_rate = config.get_display_refresh_rate();
    display_config_.pixel_format = PixelFormat::RGB888; // Default format
    display_config_.vsync_enabled = config.is_vsync_enabled();
    
    // Initialize MIPI-DSI configuration
    mipi_config_.lanes = 4; // 4-lane MIPI-DSI
    mipi_config_.pixel_clock = display_config_.width * display_config_.height * 
                               display_config_.refresh_rate * 3; // RGB888
    mipi_config_.escape_clock = 20000000; // 20MHz escape clock
    mipi_config_.command_mode = false; // Video mode
    
    // Initialize framebuffer
    framebuffer_ = std::make_unique<Framebuffer>();
    RETURN_IF_ERROR(framebuffer_->initialize(
        display_config_.width,
        display_config_.height,
        display_config_.pixel_format,
        true // Double buffered
    ));
    
    // Initialize registers
    RETURN_IF_ERROR(setup_registers());
    
    // Initialize timing parameters
    RETURN_IF_ERROR(calculate_timing_parameters());
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    enabled_ = false;
    
    COMPONENT_LOG_INFO("Display controller initialized: {}x{} @{}Hz",
                      display_config_.width, display_config_.height, display_config_.refresh_rate);
    
    return {};
}

Result<void> DisplayController::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down display controller");
    
    // Disable display
    enabled_ = false;
    
    // Shutdown framebuffer
    if (framebuffer_) {
        framebuffer_->shutdown();
        framebuffer_.reset();
    }
    
    // Clear registers
    registers_.clear();
    
    memory_controller_ = nullptr;
    interrupt_controller_ = nullptr;
    initialized_ = false;
    
    COMPONENT_LOG_INFO("Display controller shutdown completed");
    return {};
}

Result<void> DisplayController::enable() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    if (enabled_) {
        return {}; // Already enabled
    }
    
    COMPONENT_LOG_INFO("Enabling display controller");
    
    // Initialize MIPI-DSI link
    RETURN_IF_ERROR(initialize_mipi_dsi());
    
    // Configure display timing
    RETURN_IF_ERROR(configure_display_timing());
    
    // Start display refresh
    enabled_ = true;
    last_frame_time_ = std::chrono::steady_clock::now();
    
    COMPONENT_LOG_INFO("Display controller enabled");
    return {};
}

Result<void> DisplayController::disable() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    if (!enabled_) {
        return {}; // Already disabled
    }
    
    COMPONENT_LOG_INFO("Disabling display controller");
    
    enabled_ = false;
    
    COMPONENT_LOG_INFO("Display controller disabled");
    return {};
}

Result<void> DisplayController::update() {
    if (!initialized_ || !enabled_) {
        return {};
    }
    
    auto current_time = std::chrono::steady_clock::now();
    auto frame_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_frame_time_);
    
    // Check if it's time for a new frame
    auto target_frame_time = std::chrono::microseconds(1000000 / display_config_.refresh_rate);
    
    if (frame_duration >= target_frame_time) {
        RETURN_IF_ERROR(process_frame());
        last_frame_time_ = current_time;
        statistics_.frames_processed++;
    }
    
    // Handle pending display commands
    RETURN_IF_ERROR(process_pending_commands());
    
    return {};
}

Result<void> DisplayController::set_framebuffer_address(Address address) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    // Validate address
    if (!memory_controller_->is_valid_address(address).value_or(false)) {
        return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Invalid framebuffer address: 0x" + std::to_string(address)));
    }
    
    framebuffer_address_ = address;
    
    // Write to register
    write_register(DISPLAY_REG_FRAMEBUFFER_ADDR, address);
    
    COMPONENT_LOG_DEBUG("Framebuffer address set to 0x{:08X}", address);
    return {};
}

Result<void> DisplayController::set_display_mode(u32 width, u32 height, u32 refresh_rate) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    // Validate parameters
    if (width == 0 || height == 0 || refresh_rate == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid display mode parameters"));
    }
    
    if (width > MAX_DISPLAY_WIDTH || height > MAX_DISPLAY_HEIGHT) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Display resolution exceeds maximum supported"));
    }
    
    COMPONENT_LOG_INFO("Setting display mode: {}x{} @{}Hz", width, height, refresh_rate);
    
    bool was_enabled = enabled_;
    if (was_enabled) {
        RETURN_IF_ERROR(disable());
    }
    
    // Update configuration
    display_config_.width = width;
    display_config_.height = height;
    display_config_.refresh_rate = refresh_rate;
    
    // Recalculate timing parameters
    RETURN_IF_ERROR(calculate_timing_parameters());
    
    // Resize framebuffer
    RETURN_IF_ERROR(framebuffer_->shutdown());
    RETURN_IF_ERROR(framebuffer_->initialize(width, height, display_config_.pixel_format, true));
    
    // Update registers
    write_register(DISPLAY_REG_WIDTH, width);
    write_register(DISPLAY_REG_HEIGHT, height);
    write_register(DISPLAY_REG_REFRESH_RATE, refresh_rate);
    
    if (was_enabled) {
        RETURN_IF_ERROR(enable());
    }
    
    COMPONENT_LOG_INFO("Display mode updated successfully");
    return {};
}

Result<void> DisplayController::send_command(u8 command, const std::vector<u8>& parameters) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    DisplayCommand cmd{};
    cmd.command = command;
    cmd.parameters = parameters;
    cmd.timestamp = std::chrono::steady_clock::now();
    
    command_queue_.push_back(cmd);
    
    COMPONENT_LOG_TRACE("Display command queued: 0x{:02X} with {} parameters",
                       command, parameters.size());
    
    return {};
}

Result<std::vector<u8>> DisplayController::read_status() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    std::vector<u8> status;
    
    // Status byte 0: General status
    u8 general_status = 0;
    if (enabled_) general_status |= 0x01;
    if (framebuffer_address_ != 0) general_status |= 0x02;
    if (!command_queue_.empty()) general_status |= 0x04;
    status.push_back(general_status);
    
    // Status byte 1: Error flags
    u8 error_flags = 0;
    // Add error flags as needed
    status.push_back(error_flags);
    
    // Status bytes 2-3: Current line (for debugging)
    u16 current_line = statistics_.frames_processed % display_config_.height;
    status.push_back(static_cast<u8>(current_line & 0xFF));
    status.push_back(static_cast<u8>((current_line >> 8) & 0xFF));
    
    return status;
}

const DisplayStatistics& DisplayController::get_statistics() const {
    return statistics_;
}

void DisplayController::clear_statistics() {
    statistics_ = {};
}

Framebuffer* DisplayController::get_framebuffer() const {
    return framebuffer_.get();
}

const DisplayConfiguration& DisplayController::get_display_config() const {
    return display_config_;
}

Result<void> DisplayController::handle_mmio_write(Address address, u32 value) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    // Handle register writes
    u32 reg_offset = address - DISPLAY_CONTROLLER_BASE_ADDR;
    
    switch (reg_offset) {
        case DISPLAY_REG_CONTROL:
            return handle_control_register_write(value);
            
        case DISPLAY_REG_FRAMEBUFFER_ADDR:
            return set_framebuffer_address(value);
            
        case DISPLAY_REG_WIDTH:
            if (value != display_config_.width) {
                return set_display_mode(value, display_config_.height, display_config_.refresh_rate);
            }
            break;
            
        case DISPLAY_REG_HEIGHT:
            if (value != display_config_.height) {
                return set_display_mode(display_config_.width, value, display_config_.refresh_rate);
            }
            break;
            
        case DISPLAY_REG_REFRESH_RATE:
            if (value != display_config_.refresh_rate) {
                return set_display_mode(display_config_.width, display_config_.height, value);
            }
            break;
            
        case DISPLAY_REG_COMMAND:
            return handle_command_register_write(value);
            
        default:
            COMPONENT_LOG_WARN("Write to unknown display register: 0x{:08X} = 0x{:08X}",
                              address, value);
            break;
    }
    
    write_register(reg_offset, value);
    return {};
}

Result<u32> DisplayController::handle_mmio_read(Address address) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Display controller not initialized"));
    }
    
    u32 reg_offset = address - DISPLAY_CONTROLLER_BASE_ADDR;
    
    switch (reg_offset) {
        case DISPLAY_REG_STATUS: {
            auto status_result = read_status();
            if (!status_result) {
                return std::unexpected(status_result.error());
            }
            
            auto status = status_result.value();
            u32 status_word = 0;
            for (size_t i = 0; i < std::min(status.size(), size_t(4)); ++i) {
                status_word |= (static_cast<u32>(status[i]) << (i * 8));
            }
            return status_word;
        }
        
        default:
            return read_register(reg_offset);
    }
}

Result<void> DisplayController::setup_registers() {
    // Initialize register map
    registers_[DISPLAY_REG_CONTROL] = 0;
    registers_[DISPLAY_REG_STATUS] = 0;
    registers_[DISPLAY_REG_FRAMEBUFFER_ADDR] = 0;
    registers_[DISPLAY_REG_WIDTH] = display_config_.width;
    registers_[DISPLAY_REG_HEIGHT] = display_config_.height;
    registers_[DISPLAY_REG_REFRESH_RATE] = display_config_.refresh_rate;
    registers_[DISPLAY_REG_PIXEL_FORMAT] = static_cast<u32>(display_config_.pixel_format);
    registers_[DISPLAY_REG_COMMAND] = 0;
    registers_[DISPLAY_REG_COMMAND_DATA] = 0;
    
    return {};
}

Result<void> DisplayController::calculate_timing_parameters() {
    // Calculate MIPI-DSI timing parameters
    timing_.pixel_clock = display_config_.width * display_config_.height * 
                         display_config_.refresh_rate * 3; // RGB888
    
    // Standard MIPI-DSI timing (simplified)
    timing_.h_sync_width = 10;
    timing_.h_back_porch = 20;
    timing_.h_front_porch = 20;
    timing_.v_sync_width = 3;
    timing_.v_back_porch = 7;
    timing_.v_front_porch = 10;
    
    timing_.h_total = display_config_.width + timing_.h_sync_width + 
                     timing_.h_back_porch + timing_.h_front_porch;
    timing_.v_total = display_config_.height + timing_.v_sync_width + 
                     timing_.v_back_porch + timing_.v_front_porch;
    
    COMPONENT_LOG_DEBUG("Display timing: {}x{} total, pixel_clock={}",
                       timing_.h_total, timing_.v_total, timing_.pixel_clock);
    
    return {};
}

Result<void> DisplayController::initialize_mipi_dsi() {
    COMPONENT_LOG_DEBUG("Initializing MIPI-DSI interface");
    
    // Initialize MIPI-DSI PHY (simulated)
    mipi_state_.phy_enabled = true;
    mipi_state_.lanes_active = mipi_config_.lanes;
    mipi_state_.clock_active = true;
    
    // Send initialization sequence to display panel (simulated)
    std::vector<u8> init_commands = {
        0x11, // Sleep out
        0x29, // Display on
        0x3A, 0x77, // Pixel format: RGB888
    };
    
    for (size_t i = 0; i < init_commands.size(); ++i) {
        std::vector<u8> params;
        if (i + 1 < init_commands.size() && init_commands[i] == 0x3A) {
            params.push_back(init_commands[++i]);
        }
        RETURN_IF_ERROR(send_command(init_commands[i], params));
    }
    
    COMPONENT_LOG_DEBUG("MIPI-DSI initialization completed");
    return {};
}

Result<void> DisplayController::configure_display_timing() {
    COMPONENT_LOG_DEBUG("Configuring display timing");
    
    // Configure timing registers (simulated)
    write_register(DISPLAY_REG_H_SYNC_WIDTH, timing_.h_sync_width);
    write_register(DISPLAY_REG_H_BACK_PORCH, timing_.h_back_porch);
    write_register(DISPLAY_REG_H_FRONT_PORCH, timing_.h_front_porch);
    write_register(DISPLAY_REG_V_SYNC_WIDTH, timing_.v_sync_width);
    write_register(DISPLAY_REG_V_BACK_PORCH, timing_.v_back_porch);
    write_register(DISPLAY_REG_V_FRONT_PORCH, timing_.v_front_porch);
    write_register(DISPLAY_REG_PIXEL_CLOCK, timing_.pixel_clock);
    
    return {};
}

Result<void> DisplayController::process_frame() {
    if (framebuffer_address_ == 0) {
        return {}; // No framebuffer configured
    }
    
    // Read framebuffer data from memory
    size_t framebuffer_size = display_config_.width * display_config_.height * 3; // RGB888
    
    auto framebuffer_data = memory_controller_->read_bytes(framebuffer_address_, framebuffer_size);
    if (!framebuffer_data) {
        statistics_.frame_errors++;
        return std::unexpected(framebuffer_data.error());
    }
    
    // Update internal framebuffer
    RETURN_IF_ERROR(framebuffer_->blit(
        framebuffer_data.value().data(),
        display_config_.width,
        display_config_.height,
        0, 0,
        PixelFormat::RGB888
    ));
    
    // Swap buffers
    RETURN_IF_ERROR(framebuffer_->swap_buffers());
    
    // Generate VSync interrupt if enabled
    if (interrupt_controller_) {
        auto interrupt_result = interrupt_controller_->trigger_interrupt(
            CoreId::CORE_0, InterruptType::EXTERNAL, DISPLAY_VSYNC_INTERRUPT_ID);
        if (!interrupt_result) {
            COMPONENT_LOG_WARN("Failed to trigger VSync interrupt: {}",
                              interrupt_result.error().to_string());
        }
    }
    
    statistics_.vsync_interrupts++;
    return {};
}

Result<void> DisplayController::process_pending_commands() {
    while (!command_queue_.empty()) {
        auto& cmd = command_queue_.front();
        
        // Process command (simplified)
        switch (cmd.command) {
            case 0x11: // Sleep out
                COMPONENT_LOG_DEBUG("Display: Sleep out");
                break;
                
            case 0x29: // Display on
                COMPONENT_LOG_DEBUG("Display: Display on");
                break;
                
            case 0x3A: // Set pixel format
                if (!cmd.parameters.empty()) {
                    COMPONENT_LOG_DEBUG("Display: Set pixel format 0x{:02X}", cmd.parameters[0]);
                }
                break;
                
            default:
                COMPONENT_LOG_DEBUG("Display: Unknown command 0x{:02X}", cmd.command);
                break;
        }
        
        command_queue_.pop_front();
        statistics_.commands_processed++;
    }
    
    return {};
}

Result<void> DisplayController::handle_control_register_write(u32 value) {
    bool new_enabled = (value & 0x01) != 0;
    
    if (new_enabled != enabled_) {
        if (new_enabled) {
            return enable();
        } else {
            return disable();
        }
    }
    
    return {};
}

Result<void> DisplayController::handle_command_register_write(u32 value) {
    u8 command = static_cast<u8>(value & 0xFF);
    
    // For now, just queue the command with no parameters
    // In a full implementation, parameters would be written to a separate register
    return send_command(command, {});
}

void DisplayController::write_register(u32 offset, u32 value) {
    registers_[offset] = value;
}

u32 DisplayController::read_register(u32 offset) const {
    auto it = registers_.find(offset);
    return (it != registers_.end()) ? it->second : 0;
}

}  // namespace m5tab5::emulator