#include "emulator/debug/gdb_server.hpp"
#include "emulator/core/emulator_core.hpp"
#include "emulator/cpu/dual_core_manager.hpp"
#include "emulator/debug/debugger.hpp"
#include "emulator/memory/memory_controller.hpp"
#include "emulator/utils/logging.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace m5tab5::emulator {

DECLARE_LOGGER("GDBServer");

GDBServer::GDBServer(EmulatorCore& core)
    : config_(), core_(core), running_(false) {
    COMPONENT_LOG_DEBUG("GDBServer created with default config");
    
    // Initialize command handlers
    command_handlers_["qSupported"] = [this](const std::vector<std::string>& args) {
        return cmd_query_supported(args);
    };
    command_handlers_["?"] = [this](const std::vector<std::string>& args) {
        return cmd_halt_reason(args);
    };
    command_handlers_["c"] = [this](const std::vector<std::string>& args) {
        return cmd_continue(args);
    };
    command_handlers_["s"] = [this](const std::vector<std::string>& args) {
        return cmd_step(args);
    };
    command_handlers_["g"] = [this](const std::vector<std::string>& args) {
        return cmd_read_registers(args);
    };
    command_handlers_["m"] = [this](const std::vector<std::string>& args) {
        return cmd_read_memory(args);
    };
    command_handlers_["M"] = [this](const std::vector<std::string>& args) {
        return cmd_write_memory(args);
    };
    command_handlers_["Z"] = [this](const std::vector<std::string>& args) {
        return cmd_set_breakpoint(args);
    };
    command_handlers_["z"] = [this](const std::vector<std::string>& args) {
        return cmd_remove_breakpoint(args);
    };
    command_handlers_["H"] = [this](const std::vector<std::string>& args) {
        return cmd_set_thread(args);
    };
    command_handlers_["qfThreadInfo"] = [this](const std::vector<std::string>& args) {
        return cmd_thread_info(args);
    };
    command_handlers_["qsThreadInfo"] = [this](const std::vector<std::string>& args) {
        return "l"; // No more threads
    };
    command_handlers_["D"] = [this](const std::vector<std::string>& args) {
        return cmd_detach(args);
    };
    command_handlers_["k"] = [this](const std::vector<std::string>& args) {
        return cmd_kill(args);
    };
}

GDBServer::GDBServer(EmulatorCore& core, const Config& config)
    : GDBServer(core) {
    config_ = config;
    COMPONENT_LOG_DEBUG("GDBServer created with config: port={}, enabled={}", 
                       config_.port, config_.enabled);
}

GDBServer::~GDBServer() {
    if (running_) {
        stop();
    }
    COMPONENT_LOG_DEBUG("GDBServer destroyed");
}

bool GDBServer::start() {
    if (running_) {
        COMPONENT_LOG_WARN("GDBServer already running");
        return false;
    }
    
    if (!config_.enabled) {
        COMPONENT_LOG_DEBUG("GDBServer disabled in configuration");
        return false;
    }
    
    if (!create_server_socket()) {
        COMPONENT_LOG_ERROR("Failed to create server socket");
        return false;
    }
    
    running_ = true;
    server_thread_ = std::thread(&GDBServer::server_thread, this);
    
    COMPONENT_LOG_INFO("🚀 GDB Server started on {}:{}", config_.bind_address, config_.port);
    COMPONENT_LOG_INFO("Connect GDB with: (gdb) target remote {}:{}", config_.bind_address, config_.port);
    
    return true;
}

void GDBServer::stop() {
    if (!running_) {
        return;
    }
    
    COMPONENT_LOG_INFO("Stopping GDBServer");
    running_ = false;
    
    close_server_socket();
    
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    
    // Wait for all client threads to finish
    std::lock_guard<std::mutex> lock(client_threads_mutex_);
    for (auto& thread : client_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    client_threads_.clear();
    
    COMPONENT_LOG_INFO("GDBServer stopped");
}

bool GDBServer::is_running() const {
    return running_;
}

void GDBServer::set_port(uint16_t port) {
    if (running_) {
        COMPONENT_LOG_WARN("Cannot change port while GDBServer is running");
        return;
    }
    config_.port = port;
}

uint16_t GDBServer::get_port() const {
    return config_.port;
}

void GDBServer::set_config(const Config& config) {
    if (running_) {
        COMPONENT_LOG_WARN("Cannot change config while GDBServer is running");
        return;
    }
    config_ = config;
}

//
// TCP Server Management
//

bool GDBServer::create_server_socket() {
    server_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_fd_ < 0) {
        COMPONENT_LOG_ERROR("Failed to create socket: {}", strerror(errno));
        return false;
    }
    
    // Set socket options for reuse
    int opt = 1;
    if (setsockopt(server_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        COMPONENT_LOG_WARN("Failed to set SO_REUSEADDR: {}", strerror(errno));
    }
    
    // Bind to address and port
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(config_.port);
    
    if (config_.bind_address == "localhost" || config_.bind_address == "127.0.0.1") {
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    } else {
        server_addr.sin_addr.s_addr = INADDR_ANY;
    }
    
    if (bind(server_socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        COMPONENT_LOG_ERROR("Failed to bind to {}:{}: {}", 
                           config_.bind_address, config_.port, strerror(errno));
        close(server_socket_fd_);
        server_socket_fd_ = -1;
        return false;
    }
    
    // Start listening
    if (listen(server_socket_fd_, config_.max_connections) < 0) {
        COMPONENT_LOG_ERROR("Failed to listen on socket: {}", strerror(errno));
        close(server_socket_fd_);
        server_socket_fd_ = -1;
        return false;
    }
    
    return true;
}

void GDBServer::close_server_socket() {
    if (server_socket_fd_ >= 0) {
        close(server_socket_fd_);
        server_socket_fd_ = -1;
    }
}

void GDBServer::server_thread() {
    COMPONENT_LOG_DEBUG("GDB server thread started");
    
    while (running_) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        // Use poll to check for incoming connections with timeout
        struct pollfd pfd;
        pfd.fd = server_socket_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        
        int poll_result = poll(&pfd, 1, 1000); // 1 second timeout
        
        if (poll_result < 0) {
            if (errno != EINTR) {
                COMPONENT_LOG_ERROR("Poll error: {}", strerror(errno));
            }
            continue;
        }
        
        if (poll_result == 0) {
            continue; // Timeout, check running_ again
        }
        
        int client_socket = accept(server_socket_fd_, (struct sockaddr*)&client_addr, &client_len);
        if (client_socket < 0) {
            if (errno != EINTR) {
                COMPONENT_LOG_ERROR("Failed to accept connection: {}", strerror(errno));
            }
            continue;
        }
        
        std::string client_ip = inet_ntoa(client_addr.sin_addr);
        COMPONENT_LOG_INFO("New GDB connection from {}:{}", client_ip, ntohs(client_addr.sin_port));
        
        // Create client thread
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        client_threads_.emplace_back(&GDBServer::client_handler, this, client_socket);
    }
    
    COMPONENT_LOG_DEBUG("GDB server thread finished");
}

void GDBServer::client_handler(int client_socket) {
    ClientConnection client;
    client.socket_fd = client_socket;
    client.current_thread_id = 0; // Default to core 0
    
    char buffer[4096];
    std::string packet_buffer;
    
    COMPONENT_LOG_DEBUG("GDB client handler started for socket {}", client_socket);
    
    try {
        while (running_) {
            // Use poll for non-blocking read with timeout
            struct pollfd pfd;
            pfd.fd = client_socket;
            pfd.events = POLLIN;
            pfd.revents = 0;
            
            int poll_result = poll(&pfd, 1, 1000); // 1 second timeout
            
            if (poll_result < 0) {
                if (errno != EINTR) {
                    COMPONENT_LOG_ERROR("Client poll error: {}", strerror(errno));
                    break;
                }
                continue;
            }
            
            if (poll_result == 0) {
                continue; // Timeout, check running_ again
            }
            
            ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received <= 0) {
                if (bytes_received == 0) {
                    COMPONENT_LOG_INFO("GDB client disconnected");
                } else {
                    COMPONENT_LOG_ERROR("Client recv error: {}", strerror(errno));
                }
                break;
            }
            
            buffer[bytes_received] = '\0';
            packet_buffer += buffer;
            
            // Process complete packets
            size_t packet_start = 0;
            while (true) {
                size_t dollar_pos = packet_buffer.find('$', packet_start);
                if (dollar_pos == std::string::npos) {
                    break;
                }
                
                size_t hash_pos = packet_buffer.find('#', dollar_pos);
                if (hash_pos == std::string::npos || hash_pos + 2 >= packet_buffer.size()) {
                    break; // Incomplete packet
                }
                
                // Extract complete packet
                std::string packet_data = packet_buffer.substr(dollar_pos, hash_pos - dollar_pos + 3);
                RSPPacket packet = parse_packet(packet_data);
                
                if (packet.valid) {
                    // Send acknowledgment
                    send(client_socket, "+", 1, 0);
                    
                    // Handle command
                    std::string response = handle_command(packet, client);
                    
                    if (!response.empty()) {
                        std::string formatted_response = format_response(response);
                        send(client_socket, formatted_response.c_str(), formatted_response.size(), 0);
                        
                        if (config_.verbose_logging) {
                            COMPONENT_LOG_DEBUG("GDB -> {}", formatted_response);
                        }
                    }
                } else {
                    // Send negative acknowledgment
                    send(client_socket, "-", 1, 0);
                }
                
                packet_start = hash_pos + 3;
            }
            
            // Remove processed packets from buffer
            if (packet_start > 0) {
                packet_buffer.erase(0, packet_start);
            }
        }
    } catch (const std::exception& e) {
        COMPONENT_LOG_ERROR("Exception in GDB client handler: {}", e.what());
    }
    
    close(client_socket);
    COMPONENT_LOG_DEBUG("GDB client handler finished for socket {}", client_socket);
}

//
// RSP Packet Handling
//

GDBServer::RSPPacket GDBServer::parse_packet(const std::string& data) {
    RSPPacket packet;
    
    if (data.size() < 4 || data[0] != '$' || data[data.size()-3] != '#') {
        return packet; // Invalid packet format
    }
    
    // Extract checksum
    std::string checksum_str = data.substr(data.size() - 2);
    packet.checksum = static_cast<uint8_t>(std::stoul(checksum_str, nullptr, 16));
    
    // Extract and unescape payload
    std::string payload = data.substr(1, data.size() - 4);
    std::string unescaped_payload;
    
    for (size_t i = 0; i < payload.size(); ++i) {
        if (payload[i] == '}' && i + 1 < payload.size()) {
            unescaped_payload += static_cast<char>(payload[i + 1] ^ 0x20);
            ++i; // Skip the next character
        } else {
            unescaped_payload += payload[i];
        }
    }
    
    // Verify checksum
    uint8_t calculated_checksum = 0;
    for (char c : payload) {
        calculated_checksum += static_cast<uint8_t>(c);
    }
    
    if (calculated_checksum != packet.checksum) {
        COMPONENT_LOG_WARN("GDB packet checksum mismatch: calculated=0x{:02x}, received=0x{:02x}",
                          calculated_checksum, packet.checksum);
        return packet;
    }
    
    // Parse command and arguments
    packet.raw_data = unescaped_payload;
    if (!unescaped_payload.empty()) {
        // Extract command (first character or first word before ':')
        size_t colon_pos = unescaped_payload.find(':');
        if (colon_pos != std::string::npos) {
            packet.command = unescaped_payload.substr(0, colon_pos);
            
            // Split arguments by commas
            std::string args_str = unescaped_payload.substr(colon_pos + 1);
            std::stringstream ss(args_str);
            std::string arg;
            while (std::getline(ss, arg, ',')) {
                packet.arguments.push_back(arg);
            }
        } else {
            packet.command = unescaped_payload;
        }
    }
    
    packet.valid = true;
    
    if (config_.verbose_logging) {
        COMPONENT_LOG_DEBUG("GDB <- ${}", unescaped_payload);
    }
    
    return packet;
}

std::string GDBServer::format_response(const std::string& response) {
    std::string formatted = "$";
    
    // Escape special characters
    for (char c : response) {
        if (c == '$' || c == '#' || c == '}') {
            formatted += '}';
            formatted += static_cast<char>(c ^ 0x20);
        } else {
            formatted += c;
        }
    }
    
    // Calculate and append checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < formatted.size(); ++i) {
        checksum += static_cast<uint8_t>(formatted[i]);
    }
    
    formatted += "#";
    formatted += format_hex32(checksum).substr(6); // Get last 2 hex digits
    
    return formatted;
}

//
// GDB Command Handlers
//

std::string GDBServer::handle_command(const RSPPacket& packet, ClientConnection& client) {
    auto it = command_handlers_.find(packet.command);
    if (it != command_handlers_.end()) {
        return it->second(packet.arguments);
    }
    
    // Handle single character commands
    if (packet.command.size() == 1) {
        auto single_it = command_handlers_.find(packet.command);
        if (single_it != command_handlers_.end()) {
            // Parse full packet for single character commands
            std::vector<std::string> args;
            if (packet.raw_data.size() > 1) {
                args.push_back(packet.raw_data.substr(1));
            }
            return single_it->second(args);
        }
    }
    
    COMPONENT_LOG_WARN("Unhandled GDB command: {}", packet.command);
    return ""; // Empty response for unhandled commands
}

std::string GDBServer::cmd_query_supported(const std::vector<std::string>& args) {
    // Advertise our capabilities
    return "PacketSize=4000;qXfer:features:read+;multiprocess+";
}

std::string GDBServer::cmd_halt_reason(const std::vector<std::string>& args) {
    // Return halt reason (SIGTRAP for breakpoint)
    if (execution_stopped_) {
        return "S05"; // SIGTRAP
    }
    return "S00"; // No signal
}

std::string GDBServer::cmd_continue(const std::vector<std::string>& args) {
    auto debugger = get_debugger();
    if (!debugger) {
        return "E01";
    }
    
    auto result = debugger->continueExecution();
    if (result != EmulatorError::Success) {
        return "E02";
    }
    
    execution_stopped_ = false;
    return ""; // No immediate response - will send stop reply when halted
}

std::string GDBServer::cmd_step(const std::vector<std::string>& args) {
    auto debugger = get_debugger();
    if (!debugger) {
        return "E01";
    }
    
    auto result = debugger->stepInstruction();
    if (result != EmulatorError::Success) {
        return "E02";
    }
    
    // Single step should immediately halt
    return "S05"; // SIGTRAP
}

std::string GDBServer::cmd_read_registers(const std::vector<std::string>& args) {
    return format_risc_v_registers(active_core_);
}

std::string GDBServer::cmd_read_memory(const std::vector<std::string>& args) {
    if (args.size() < 1) {
        return "E01";
    }
    
    std::string addr_len = args[0];
    size_t comma_pos = addr_len.find(',');
    if (comma_pos == std::string::npos) {
        return "E02";
    }
    
    Address address = parse_address(addr_len.substr(0, comma_pos));
    size_t length = std::stoull(addr_len.substr(comma_pos + 1), nullptr, 16);
    
    auto debugger = get_debugger();
    if (!debugger) {
        return "E03";
    }
    
    std::vector<uint8_t> data;
    auto result = debugger->readMemory(address, length, data);
    if (result != EmulatorError::Success) {
        return "E04";
    }
    
    return hex_encode(data);
}

std::string GDBServer::cmd_write_memory(const std::vector<std::string>& args) {
    if (args.size() < 1) {
        return "E01";
    }
    
    std::string full_arg = args[0];
    size_t colon_pos = full_arg.find(':');
    if (colon_pos == std::string::npos) {
        return "E02";
    }
    
    std::string addr_len = full_arg.substr(0, colon_pos);
    std::string hex_data = full_arg.substr(colon_pos + 1);
    
    size_t comma_pos = addr_len.find(',');
    if (comma_pos == std::string::npos) {
        return "E03";
    }
    
    Address address = parse_address(addr_len.substr(0, comma_pos));
    size_t length = std::stoull(addr_len.substr(comma_pos + 1), nullptr, 16);
    
    std::vector<uint8_t> data = hex_decode(hex_data);
    if (data.size() != length) {
        return "E04";
    }
    
    auto debugger = get_debugger();
    if (!debugger) {
        return "E05";
    }
    
    auto result = debugger->writeMemory(address, data);
    if (result != EmulatorError::Success) {
        return "E06";
    }
    
    return "OK";
}

std::string GDBServer::cmd_set_breakpoint(const std::vector<std::string>& args) {
    if (args.size() < 1) {
        return "E01";
    }
    
    std::string full_arg = args[0];
    std::vector<std::string> bp_args;
    std::stringstream ss(full_arg);
    std::string token;
    while (std::getline(ss, token, ',')) {
        bp_args.push_back(token);
    }
    
    if (bp_args.size() < 3) {
        return "E02";
    }
    
    // Parse breakpoint type, address, length
    int type = std::stoi(bp_args[0]);
    Address address = parse_address(bp_args[1]);
    size_t length = std::stoull(bp_args[2], nullptr, 16);
    
    auto debugger = get_debugger();
    if (!debugger) {
        return "E03";
    }
    
    // Map GDB breakpoint types to our types
    BreakpointType bp_type = BreakpointType::EXECUTION;
    switch (type) {
        case 0: bp_type = BreakpointType::EXECUTION; break;
        case 2: bp_type = BreakpointType::WRITE; break;
        case 3: bp_type = BreakpointType::READ; break;
        case 4: bp_type = BreakpointType::ACCESS; break;
        default: return "E04";
    }
    
    debugger->setBreakpoint(bp_type, address, length);
    return "OK";
}

std::string GDBServer::cmd_remove_breakpoint(const std::vector<std::string>& args) {
    // Similar to set_breakpoint but remove instead
    // For now, return OK (would need breakpoint ID tracking)
    return "OK";
}

std::string GDBServer::cmd_write_register(const std::vector<std::string>& args) {
    // TODO: Implement register write
    return "OK";
}

std::string GDBServer::cmd_set_thread(const std::vector<std::string>& args) {
    if (args.size() < 1) {
        return "E01";
    }
    
    std::string full_arg = args[0];
    if (full_arg.size() < 2) {
        return "E02";
    }
    
    char operation = full_arg[0];
    std::string thread_id_str = full_arg.substr(1);
    
    if (thread_id_str == "-1") {
        // All threads - use current active core
        return "OK";
    }
    
    uint32_t thread_id = std::stoul(thread_id_str, nullptr, 16);
    if (thread_id >= 2) { // ESP32-P4 has 2 cores
        return "E03";
    }
    
    if (operation == 'g' || operation == 'c') {
        set_active_core(thread_id);
        return "OK";
    }
    
    return "E04";
}

std::string GDBServer::cmd_thread_info(const std::vector<std::string>& args) {
    return format_thread_list();
}

std::string GDBServer::cmd_detach(const std::vector<std::string>& args) {
    return "OK";
}

std::string GDBServer::cmd_kill(const std::vector<std::string>& args) {
    // Stop emulation
    auto debugger = get_debugger();
    if (debugger) {
        debugger->pauseExecution();
    }
    return "OK";
}

//
// Utility Methods  
//

std::string GDBServer::format_risc_v_registers(uint32_t core_id) {
    auto debugger = get_debugger();
    if (!debugger) {
        return "E01";
    }
    
    std::string result;
    
    // Read all 32 RISC-V general purpose registers + PC
    for (uint32_t reg = 0; reg < RISCV_REG_COUNT; ++reg) {
        uint32_t value = 0;
        auto read_result = debugger->readRegister(reg, value);
        if (read_result != EmulatorError::Success) {
            // Return zeros for unreadable registers
            value = 0;
        }
        
        // Convert to little-endian hex string (8 hex digits)
        result += format_hex32(value);
    }
    
    return result;
}

std::string GDBServer::format_thread_list() {
    // Return both cores as threads
    return "m0,1";
}

uint32_t GDBServer::get_active_core() const {
    return active_core_;
}

void GDBServer::set_active_core(uint32_t core_id) {
    if (core_id < 2) { // ESP32-P4 has 2 cores
        active_core_ = core_id;
        COMPONENT_LOG_DEBUG("GDB active core set to {}", core_id);
    }
}

std::string GDBServer::hex_encode(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (uint8_t byte : data) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
    }
    return oss.str();
}

std::vector<uint8_t> GDBServer::hex_decode(const std::string& hex) {
    std::vector<uint8_t> data;
    for (size_t i = 0; i < hex.size(); i += 2) {
        if (i + 1 < hex.size()) {
            uint8_t byte = static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16));
            data.push_back(byte);
        }
    }
    return data;
}

Address GDBServer::parse_address(const std::string& hex) {
    return static_cast<Address>(std::stoull(hex, nullptr, 16));
}

std::string GDBServer::format_address(Address addr) {
    std::ostringstream oss;
    oss << std::hex << std::setw(8) << std::setfill('0') << addr;
    return oss.str();
}

uint32_t GDBServer::parse_hex32(const std::string& hex) {
    return static_cast<uint32_t>(std::stoul(hex, nullptr, 16));
}

std::string GDBServer::format_hex32(uint32_t value) {
    std::ostringstream oss;
    oss << std::hex << std::setw(8) << std::setfill('0') << value;
    return oss.str();
}

//
// Component Access Helpers
//

std::shared_ptr<DualCoreManager> GDBServer::get_cpu_manager() {
    return core_.getComponent<DualCoreManager>();
}

std::shared_ptr<Debugger> GDBServer::get_debugger() {
    return core_.getComponent<Debugger>();
}

//
// Debug State Callbacks
//

void GDBServer::on_breakpoint_hit(uint32_t core_id, Address pc) {
    execution_stopped_ = true;
    last_stop_pc_ = pc;
    last_stop_reason_ = "breakpoint";
    
    // Notify GDB clients
    std::string stop_reply = "S05"; // SIGTRAP
    // Would send this to connected clients
    
    COMPONENT_LOG_INFO("🛑 Breakpoint hit on core {} at PC=0x{:08x}", core_id, pc);
}

void GDBServer::on_execution_stopped(uint32_t core_id, Address pc, const std::string& reason) {
    execution_stopped_ = true;
    last_stop_pc_ = pc;
    last_stop_reason_ = reason;
    
    COMPONENT_LOG_INFO("⏸️  Execution stopped on core {} at PC=0x{:08x}: {}", core_id, pc, reason);
}

void GDBServer::on_execution_started(uint32_t core_id) {
    execution_stopped_ = false;
    
    COMPONENT_LOG_DEBUG("▶️  Execution started on core {}", core_id);
}

} // namespace m5tab5::emulator