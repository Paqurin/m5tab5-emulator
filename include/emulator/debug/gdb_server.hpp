#pragma once

#include "emulator/core/types.hpp"
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <unordered_map>
#include <functional>

namespace m5tab5::emulator {

// Forward declarations
class EmulatorCore;
class Debugger;
class DualCoreManager;

/**
 * @brief GDB Remote Serial Protocol (RSP) server implementation
 * 
 * Implements the GDB remote debugging protocol for ESP32-P4 RISC-V emulator.
 * Provides full debugging capabilities including breakpoints, memory inspection,
 * register access, and execution control.
 * 
 * Features:
 * - TCP server for GDB connections (default port 3333)
 * - RSP packet parsing and response generation
 * - Multi-core debugging support (ESP32-P4 dual cores)
 * - RISC-V register mapping and instruction stepping
 * - Breakpoint management and execution control
 * - Memory and register inspection
 * - Symbol table integration
 */
class GDBServer {
public:
    struct Config {
        uint16_t port = 3333;
        bool enabled = false;
        std::string bind_address = "localhost";
        bool multi_core_support = true;
        size_t max_connections = 1;
        bool verbose_logging = false;
    };

    // GDB RSP packet structure
    struct RSPPacket {
        std::string command;
        std::vector<std::string> arguments;
        std::string raw_data;
        uint8_t checksum;
        bool valid = false;
        
        RSPPacket() : checksum(0) {}
    };

    // Client connection state
    struct ClientConnection {
        int socket_fd = -1;
        bool authenticated = false;
        uint32_t current_thread_id = 0;  // 0 = core 0, 1 = core 1
        bool extended_mode = false;
        std::string client_address;
        
        ClientConnection() = default;
    };

    explicit GDBServer(EmulatorCore& core);
    explicit GDBServer(EmulatorCore& core, const Config& config);
    ~GDBServer();

    // Basic lifecycle
    bool start();
    void stop();
    bool is_running() const;

    // Configuration
    void set_port(uint16_t port);
    uint16_t get_port() const;
    void set_config(const Config& config);

    // Debug state callbacks (called by debugger)
    void on_breakpoint_hit(uint32_t core_id, Address pc);
    void on_execution_stopped(uint32_t core_id, Address pc, const std::string& reason);
    void on_execution_started(uint32_t core_id);

private:
    // TCP server management
    void server_thread();
    void client_handler(int client_socket);
    bool create_server_socket();
    void close_server_socket();

    // RSP packet handling
    RSPPacket parse_packet(const std::string& data);
    std::string format_response(const std::string& response);
    std::string calculate_checksum(const std::string& data);
    bool verify_checksum(const std::string& packet, uint8_t expected_checksum);
    std::string escape_binary_data(const std::vector<uint8_t>& data);
    std::vector<uint8_t> unescape_binary_data(const std::string& data);

    // GDB command handlers
    std::string handle_command(const RSPPacket& packet, ClientConnection& client);
    
    // Essential GDB commands
    std::string cmd_query_supported(const std::vector<std::string>& args);
    std::string cmd_halt_reason(const std::vector<std::string>& args);
    std::string cmd_continue(const std::vector<std::string>& args);
    std::string cmd_step(const std::vector<std::string>& args);
    std::string cmd_set_breakpoint(const std::vector<std::string>& args);
    std::string cmd_remove_breakpoint(const std::vector<std::string>& args);
    std::string cmd_read_registers(const std::vector<std::string>& args);
    std::string cmd_write_register(const std::vector<std::string>& args);
    std::string cmd_read_memory(const std::vector<std::string>& args);
    std::string cmd_write_memory(const std::vector<std::string>& args);
    std::string cmd_thread_info(const std::vector<std::string>& args);
    std::string cmd_set_thread(const std::vector<std::string>& args);
    std::string cmd_detach(const std::vector<std::string>& args);
    std::string cmd_kill(const std::vector<std::string>& args);

    // RISC-V specific handlers
    std::string format_risc_v_registers(uint32_t core_id);
    bool parse_risc_v_registers(const std::string& data, uint32_t core_id);
    std::string get_thread_extra_info(uint32_t thread_id);
    
    // Multi-core support
    uint32_t get_active_core() const;
    void set_active_core(uint32_t core_id);
    std::string format_thread_list();

    // Utility methods
    std::string hex_encode(const std::vector<uint8_t>& data);
    std::vector<uint8_t> hex_decode(const std::string& hex);
    Address parse_address(const std::string& hex);
    std::string format_address(Address addr);
    uint32_t parse_hex32(const std::string& hex);
    std::string format_hex32(uint32_t value);

    // Component access helpers
    std::shared_ptr<DualCoreManager> get_cpu_manager();
    std::shared_ptr<Debugger> get_debugger();

    // Configuration and state
    Config config_;
    EmulatorCore& core_;
    std::atomic<bool> running_{false};
    
    // TCP server state
    int server_socket_fd_ = -1;
    std::thread server_thread_;
    std::vector<std::thread> client_threads_;
    std::mutex client_threads_mutex_;
    
    // Client connections
    std::vector<ClientConnection> client_connections_;
    std::mutex connections_mutex_;
    
    // Debug state
    std::atomic<bool> execution_stopped_{false};
    std::atomic<uint32_t> active_core_{0};
    Address last_stop_pc_ = 0;
    std::string last_stop_reason_;
    
    // GDB command table
    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string>&)>> command_handlers_;
    
    // RISC-V register mapping (32 general purpose + PC + CSRs)
    static constexpr uint32_t RISCV_REG_COUNT = 33;
    static constexpr uint32_t RISCV_REG_PC = 32;
    
    // Synchronization with emulator
    std::condition_variable stop_condition_;
    std::mutex stop_mutex_;
};

} // namespace m5tab5::emulator