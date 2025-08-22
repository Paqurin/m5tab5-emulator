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

enum class SPIMode : u8 {
    MASTER = 0,
    SLAVE = 1
};

enum class SPIClockPolarity : u8 {
    IDLE_LOW = 0,   // CPOL = 0
    IDLE_HIGH = 1   // CPOL = 1
};

enum class SPIClockPhase : u8 {
    FIRST_EDGE = 0,  // CPHA = 0
    SECOND_EDGE = 1  // CPHA = 1
};

enum class SPIBitOrder : u8 {
    MSB_FIRST = 0,
    LSB_FIRST = 1
};

enum class SPIDataSize : u8 {
    BITS_8 = 8,
    BITS_16 = 16,
    BITS_32 = 32
};

enum class SPIInterruptType : u8 {
    TRANSFER_COMPLETE = 0x01,
    RX_FIFO_NOT_EMPTY = 0x02,
    TX_FIFO_EMPTY = 0x04,
    RX_FIFO_OVERFLOW = 0x08,
    TX_FIFO_UNDERFLOW = 0x10,
    MODE_FAULT = 0x20,
    SLAVE_SELECT = 0x40,
    ERROR = 0x80
};

struct SPITransfer {
    std::vector<u8> tx_data;
    std::vector<u8> rx_data;
    size_t bytes_transferred;
    bool is_complete;
    std::chrono::steady_clock::time_point start_time;
    Result<void> result;
};

struct SPIStatistics {
    u64 transfers_completed = 0;
    u64 bytes_transmitted = 0;
    u64 bytes_received = 0;
    u64 mode_faults = 0;
    u64 fifo_overflows = 0;
    u64 fifo_underflows = 0;
    u64 cs_toggles = 0;
    double average_transfer_time_us = 0.0;
};

class SPIController {
public:
    static constexpr size_t TX_FIFO_SIZE = 64;
    static constexpr size_t RX_FIFO_SIZE = 64;
    static constexpr u32 MAX_CLOCK_RATE = 40000000; // 40 MHz
    static constexpr u32 MIN_CLOCK_RATE = 1000;     // 1 kHz
    
    SPIController(u8 controller_id);
    ~SPIController();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    Result<void> configure(SPIMode mode, u32 clock_rate, SPIClockPolarity cpol, 
                          SPIClockPhase cpha, SPIBitOrder bit_order, SPIDataSize data_size);
    
    Result<void> set_chip_select(u8 cs_pin, bool active);
    Result<void> start_transfer(const std::vector<u8>& tx_data);
    Result<std::vector<u8>> get_received_data();
    Result<void> abort_transfer();

    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);

    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_busy() const { return current_transfer_ != nullptr; }
    const SPIStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct SPIRegisters {
        u32 control1 = 0;         // Control register 1
        u32 control2 = 0;         // Control register 2
        u32 status = 0;           // Status register
        u32 interrupt_enable = 0; // Interrupt enable register
        u32 interrupt_status = 0; // Interrupt status register
        u32 clock_control = 0;    // Clock control register
        u32 chip_select = 0;      // Chip select register
        u32 data = 0;             // Data register
        u32 fifo_control = 0;     // FIFO control register
        u32 fifo_status = 0;      // FIFO status register
        u32 dma_control = 0;      // DMA control register
        u32 timing_control = 0;   // Timing control register
    };

    void update_transfer();
    void complete_transfer(Result<void> result);
    void trigger_interrupt(SPIInterruptType interrupt_type);
    void update_status_register();
    void process_tx_fifo();
    void process_rx_fifo();
    void simulate_device_response(const std::vector<u8>& tx_data, std::vector<u8>& rx_data);
    u32 calculate_bit_time_ns() const;

    u8 controller_id_;
    bool initialized_;
    SPIMode mode_;
    u32 clock_rate_;
    SPIClockPolarity clock_polarity_;
    SPIClockPhase clock_phase_;
    SPIBitOrder bit_order_;
    SPIDataSize data_size_;
    
    SPIRegisters registers_;
    InterruptController* interrupt_controller_;
    
    std::queue<u8> tx_fifo_;
    std::queue<u8> rx_fifo_;
    
    std::unique_ptr<SPITransfer> current_transfer_;
    std::chrono::steady_clock::time_point transfer_start_;
    
    u8 active_cs_pin_;
    bool cs_active_;
    
    SPIStatistics statistics_;
    
    mutable std::mutex controller_mutex_;
    std::condition_variable transfer_cv_;
};

}  // namespace m5tab5::emulator