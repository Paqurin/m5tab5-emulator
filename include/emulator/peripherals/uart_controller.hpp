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
#include <string>

namespace m5tab5::emulator {

enum class UARTDataBits : u8 {
    BITS_5 = 5,
    BITS_6 = 6,
    BITS_7 = 7,
    BITS_8 = 8,
    BITS_9 = 9
};

enum class UARTStopBits : u8 {
    STOP_1 = 1,
    STOP_1_5 = 2,
    STOP_2 = 3
};

enum class UARTParity : u8 {
    NONE = 0,
    ODD = 1,
    EVEN = 2,
    MARK = 3,
    SPACE = 4
};

enum class UARTFlowControl : u8 {
    NONE = 0,
    RTS_CTS = 1,
    XON_XOFF = 2
};

enum class UARTInterruptType : u8 {
    RX_DATA_AVAILABLE = 0x01,
    TX_HOLDING_EMPTY = 0x02,
    LINE_STATUS = 0x04,
    MODEM_STATUS = 0x08,
    RX_TIMEOUT = 0x10,
    BREAK_DETECTED = 0x20,
    PARITY_ERROR = 0x40,
    FRAMING_ERROR = 0x80
};

enum class UARTLineStatus : u8 {
    DATA_READY = 0x01,
    OVERRUN_ERROR = 0x02,
    PARITY_ERROR = 0x04,
    FRAMING_ERROR = 0x08,
    BREAK_INTERRUPT = 0x10,
    TX_HOLDING_EMPTY = 0x20,
    TX_EMPTY = 0x40,
    RX_FIFO_ERROR = 0x80
};

struct UARTStatistics {
    u64 bytes_transmitted = 0;
    u64 bytes_received = 0;
    u64 frames_sent = 0;
    u64 frames_received = 0;
    u64 parity_errors = 0;
    u64 framing_errors = 0;
    u64 overrun_errors = 0;
    u64 break_conditions = 0;
    u64 fifo_overflows = 0;
    u64 fifo_underflows = 0;
    double average_throughput_bps = 0.0;
};

class UARTController {
public:
    static constexpr size_t TX_FIFO_SIZE = 128;
    static constexpr size_t RX_FIFO_SIZE = 128;
    static constexpr u32 MAX_BAUD_RATE = 5000000; // 5 Mbps
    static constexpr u32 MIN_BAUD_RATE = 110;     // 110 bps
    static constexpr u32 DEFAULT_TIMEOUT_MS = 100;
    
    UARTController(u8 controller_id);
    ~UARTController();

    Result<void> initialize(const Configuration& config, InterruptController* interrupt_controller);
    Result<void> shutdown();

    Result<void> configure(u32 baud_rate, UARTDataBits data_bits, UARTStopBits stop_bits,
                          UARTParity parity, UARTFlowControl flow_control);
    Result<void> set_timeout(u32 timeout_ms);
    
    Result<void> send_data(const std::vector<u8>& data);
    Result<void> send_string(const std::string& str);
    Result<std::vector<u8>> receive_data(size_t max_bytes = 0);
    Result<std::string> receive_string();
    Result<void> send_break(u32 duration_ms);

    Result<void> set_rts(bool state);
    Result<void> set_dtr(bool state);
    bool get_cts() const;
    bool get_dsr() const;
    bool get_cd() const;
    bool get_ri() const;

    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);

    void update();
    
    bool is_initialized() const { return initialized_; }
    bool is_tx_busy() const;
    bool is_rx_data_available() const;
    size_t get_rx_data_count() const;
    const UARTStatistics& get_statistics() const { return statistics_; }
    void clear_statistics();

    void dump_status() const;

private:
    struct UARTRegisters {
        u32 data = 0;             // Data register (RX/TX)
        u32 interrupt_enable = 0; // Interrupt enable register
        u32 interrupt_id = 0;     // Interrupt identification register
        u32 fifo_control = 0;     // FIFO control register
        u32 line_control = 0;     // Line control register
        u32 modem_control = 0;    // Modem control register
        u32 line_status = 0;      // Line status register
        u32 modem_status = 0;     // Modem status register
        u32 scratch = 0;          // Scratch register
        u32 divisor_latch_low = 0;  // Divisor latch low byte
        u32 divisor_latch_high = 0; // Divisor latch high byte
        u32 enhanced_features = 0;  // Enhanced features register
    };

    struct UARTFrame {
        u8 data;
        std::chrono::steady_clock::time_point timestamp;
        bool has_parity_error = false;
        bool has_framing_error = false;
        bool is_break = false;
    };

    void update_transmission();
    void update_reception();
    void process_tx_fifo();
    void process_rx_fifo();
    void trigger_interrupt(UARTInterruptType interrupt_type);
    void update_line_status();
    void update_modem_status();
    void simulate_external_device();
    u32 calculate_frame_time_us() const;
    bool validate_frame(const UARTFrame& frame) const;
    void handle_break_condition();

    u8 controller_id_;
    bool initialized_;
    u32 baud_rate_;
    UARTDataBits data_bits_;
    UARTStopBits stop_bits_;
    UARTParity parity_;
    UARTFlowControl flow_control_;
    u32 timeout_ms_;
    
    UARTRegisters registers_;
    InterruptController* interrupt_controller_;
    
    std::queue<UARTFrame> tx_fifo_;
    std::queue<UARTFrame> rx_fifo_;
    
    // Modem control signals
    bool rts_state_;
    bool dtr_state_;
    bool cts_state_;
    bool dsr_state_;
    bool cd_state_;
    bool ri_state_;
    
    // Transmission state
    std::unique_ptr<UARTFrame> current_tx_frame_;
    std::unique_ptr<UARTFrame> current_rx_frame_;
    std::chrono::steady_clock::time_point tx_start_time_;
    std::chrono::steady_clock::time_point rx_start_time_;
    std::chrono::steady_clock::time_point last_rx_time_;
    
    UARTStatistics statistics_;
    
    mutable std::mutex controller_mutex_;
    std::condition_variable data_cv_;
};

}  // namespace m5tab5::emulator