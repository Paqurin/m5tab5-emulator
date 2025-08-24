#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/peripherals/interrupt_controller.hpp"
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace m5tab5::emulator {

enum class I2CMode : u8 {
    MASTER = 0,
    SLAVE = 1
};

enum class I2CSpeed : u32 {
    STANDARD = 100000,    // 100 kHz
    FAST = 400000,        // 400 kHz
    FAST_PLUS = 1000000,  // 1 MHz
    HIGH_SPEED = 3400000  // 3.4 MHz
};

enum class I2CState : u8 {
    IDLE = 0,
    START_SENT,
    ADDRESS_SENT,
    DATA_SENDING,
    DATA_RECEIVING,
    STOP_SENT,
    ERROR
};

enum class I2CInterruptType : u8 {
    TRANSACTION_COMPLETE = 0x01,
    ARBITRATION_LOST = 0x02,
    ADDRESS_NACK = 0x04,
    DATA_NACK = 0x08,
    TIMEOUT = 0x10,
    FIFO_OVERFLOW = 0x20,
    FIFO_UNDERFLOW = 0x40,
    SLAVE_ADDRESSED = 0x80
};

struct I2CTransaction {
    u8 slave_address;
    bool is_read;
    std::vector<u8> data;
    bool is_complete;
    std::chrono::steady_clock::time_point start_time;
    Result<void> result;
};

struct I2CStatistics {
    u64 transactions_completed = 0;
    u64 bytes_transmitted = 0;
    u64 bytes_received = 0;
    u64 arbitration_losses = 0;
    u64 timeout_errors = 0;
    u64 nack_errors = 0;
    u64 fifo_overflows = 0;
    u64 fifo_underflows = 0;
    double average_transaction_time_us = 0.0;
};

class I2CController {
public:
    static constexpr size_t TX_FIFO_SIZE = 32;
    static constexpr size_t RX_FIFO_SIZE = 32;
    static constexpr u32 DEFAULT_TIMEOUT_MS = 1000;
    
    I2CController(u8 controller_id);
    ~I2CController();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    Result<void> configure(I2CMode mode, I2CSpeed speed);
    Result<void> configure(u32 frequency, I2CMode mode);  // ESP-IDF compatible overload
    Result<void> set_timeout(u32 timeout_ms);
    
    Result<void> start_transaction(u8 slave_address, bool is_read, const std::vector<u8>& data = {});
    Result<std::vector<u8>> read_data(size_t num_bytes);
    Result<void> write_data(const std::vector<u8>& data);
    Result<void> stop_transaction();
    
    // ESP-IDF compatible convenience methods
    Result<std::vector<u8>> read(u8 device_addr, u8 reg_addr, size_t len);
    Result<void> write(u8 device_addr, const std::vector<u8>& data);

    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);

    void update();
    
    bool is_initialized() const { return initialized_; }
    I2CState get_state() const { return state_; }
    const I2CStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct I2CRegisters {
        u32 control = 0;          // Control register
        u32 status = 0;           // Status register
        u32 interrupt_enable = 0; // Interrupt enable register
        u32 interrupt_status = 0; // Interrupt status register
        u32 clock_divider = 0;    // Clock divider register
        u32 slave_address = 0;    // Slave address register
        u32 data = 0;             // Data register
        u32 timeout = 0;          // Timeout register
        u32 fifo_control = 0;     // FIFO control register
        u32 fifo_status = 0;      // FIFO status register
    };

    void update_transaction();
    void complete_transaction(Result<void> result);
    void trigger_interrupt(I2CInterruptType interrupt_type);
    void update_status_register();
    void process_tx_fifo();
    void process_rx_fifo();
    bool simulate_device_response(u8 slave_address, bool is_read, const std::vector<u8>& data, std::vector<u8>& response);

    u8 controller_id_;
    bool initialized_;
    I2CMode mode_;
    I2CSpeed speed_;
    I2CState state_;
    u32 timeout_ms_;
    
    I2CRegisters registers_;
    InterruptController* interrupt_controller_;
    
    std::queue<u8> tx_fifo_;
    std::queue<u8> rx_fifo_;
    
    std::unique_ptr<I2CTransaction> current_transaction_;
    std::chrono::steady_clock::time_point transaction_start_;
    
    I2CStatistics statistics_;
    
    mutable std::mutex controller_mutex_;
    std::condition_variable transaction_cv_;
};

}  // namespace m5tab5::emulator