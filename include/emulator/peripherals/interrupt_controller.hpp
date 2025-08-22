#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include <vector>
#include <functional>
#include <mutex>

namespace m5tab5::emulator {

enum class InterruptType : u8 {
    // System interrupts
    RESET = 0,
    NMI = 1,
    HARD_FAULT = 2,
    
    // Timer interrupts
    TIMER0 = 16,
    TIMER1 = 17,
    TIMER2 = 18,
    TIMER3 = 19,
    
    // Communication interrupts
    I2C0 = 32,
    I2C1 = 33,
    SPI0 = 48,
    SPI1 = 49,
    SPI2 = 50,
    UART0 = 64,
    UART1 = 65,
    UART2 = 66,
    UART3 = 67,
    UART4 = 68,
    
    // GPIO interrupts
    GPIO_BANK0 = 96,
    GPIO_BANK1 = 97,
    GPIO_BANK2 = 98,
    
    // Display interrupts
    DISPLAY_VSYNC = 112,
    DISPLAY_ERROR = 113,
    
    // Touch interrupts
    TOUCH_IRQ = 128,
    
    // DMA interrupts
    DMA_CH0 = 144,
    DMA_CH1 = 145,
    DMA_CH2 = 146,
    DMA_CH3 = 147,
    DMA_CH4 = 148,
    DMA_CH5 = 149,
    DMA_CH6 = 150,
    DMA_CH7 = 151,
    
    // Memory interrupts
    MEMORY_ERROR = 160,
    CACHE_ERROR = 161,
    
    MAX_INTERRUPT = 255
};

enum class InterruptPriority : u8 {
    HIGHEST = 0,
    HIGH = 1,
    MEDIUM = 2,
    LOW = 3,
    LOWEST = 4
};

using InterruptHandler = std::function<void(InterruptType)>;

class InterruptController {
public:
    static constexpr size_t MAX_INTERRUPTS = static_cast<size_t>(InterruptType::MAX_INTERRUPT) + 1;
    
    InterruptController();
    ~InterruptController();

    Result<void> initialize();
    Result<void> shutdown();

    Result<void> register_handler(InterruptType type, InterruptHandler handler);
    Result<void> unregister_handler(InterruptType type);
    
    Result<void> trigger_interrupt(InterruptType type);
    Result<void> clear_interrupt(InterruptType type);
    
    Result<void> enable_interrupt(InterruptType type);
    Result<void> disable_interrupt(InterruptType type);
    
    Result<void> set_priority(InterruptType type, InterruptPriority priority);
    Result<InterruptPriority> get_priority(InterruptType type) const;
    
    bool is_interrupt_enabled(InterruptType type) const;
    bool is_interrupt_pending(InterruptType type) const;
    
    void process_interrupts();
    
    void dump_status() const;

private:
    struct InterruptEntry {
        InterruptHandler handler;
        InterruptPriority priority = InterruptPriority::MEDIUM;
        bool enabled = false;
        bool pending = false;
    };

    bool initialized_;
    std::vector<InterruptEntry> interrupts_;
    mutable std::mutex controller_mutex_;
};

}  // namespace m5tab5::emulator