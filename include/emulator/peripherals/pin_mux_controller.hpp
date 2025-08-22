#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/config/configuration.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

namespace m5tab5::emulator {

enum class PinFunction : u8 {
    GPIO = 0,
    UART_TX = 1,
    UART_RX = 2,
    UART_RTS = 3,
    UART_CTS = 4,
    SPI_CLK = 5,
    SPI_MOSI = 6,
    SPI_MISO = 7,
    SPI_CS = 8,
    I2C_SDA = 9,
    I2C_SCL = 10,
    PWM_OUT = 11,
    PWM_COMP = 12,
    ADC_IN = 13,
    TOUCH_INT = 14,
    TOUCH_RST = 15,
    USB_DP = 16,
    USB_DM = 17,
    MIPI_CLK = 18,
    MIPI_DATA = 19,
    JTAG_TCK = 20,
    JTAG_TMS = 21,
    JTAG_TDI = 22,
    JTAG_TDO = 23,
    RESERVED = 24,
    INVALID = 255
};

enum class PinDriveStrength : u8 {
    WEAK = 0,    // 2mA
    MEDIUM = 1,  // 4mA
    STRONG = 2,  // 8mA
    MAXIMUM = 3  // 12mA
};

enum class PinSlewRate : u8 {
    SLOW = 0,
    FAST = 1
};

struct PinConfiguration {
    u8 pin_number;
    PinFunction function = PinFunction::GPIO;
    PinDriveStrength drive_strength = PinDriveStrength::MEDIUM;
    PinSlewRate slew_rate = PinSlewRate::SLOW;
    bool input_enable = true;
    bool output_enable = false;
    bool pull_up_enable = false;
    bool pull_down_enable = false;
    bool schmitt_trigger = false;
    bool open_drain = false;
    u8 peripheral_instance = 0; // Which instance of the peripheral (e.g., UART0, UART1)
    u8 peripheral_channel = 0;  // Which channel within the peripheral
};

struct PinMapping {
    u8 pin_number;
    PinFunction function;
    u8 peripheral_instance;
    u8 peripheral_channel;
    bool is_configurable;
    std::vector<PinFunction> available_functions;
};

class PinMuxController {
public:
    static constexpr size_t MAX_PINS = 55;
    static constexpr size_t MAX_PERIPHERAL_INSTANCES = 8;
    
    PinMuxController();
    ~PinMuxController();

    Result<void> initialize(const Configuration& config);
    Result<void> shutdown();

    Result<void> configure_pin(u8 pin_number, PinFunction function, 
                              u8 peripheral_instance = 0, u8 peripheral_channel = 0);
    Result<void> set_pin_properties(u8 pin_number, PinDriveStrength drive_strength,
                                   PinSlewRate slew_rate, bool schmitt_trigger, bool open_drain);
    Result<void> set_pin_pull(u8 pin_number, bool pull_up, bool pull_down);
    Result<void> enable_pin_input(u8 pin_number, bool enable);
    Result<void> enable_pin_output(u8 pin_number, bool enable);
    
    Result<PinConfiguration> get_pin_configuration(u8 pin_number) const;
    Result<std::vector<u8>> get_pins_for_function(PinFunction function, 
                                                  u8 peripheral_instance = 0) const;
    Result<std::vector<PinFunction>> get_available_functions(u8 pin_number) const;
    
    Result<bool> is_function_conflict(u8 pin_number, PinFunction function,
                                     u8 peripheral_instance = 0) const;
    Result<void> resolve_conflicts(u8 pin_number, PinFunction new_function);
    
    Result<void> load_default_configuration();
    Result<void> save_configuration_to_file(const std::string& filename) const;
    Result<void> load_configuration_from_file(const std::string& filename);
    
    Result<void> handle_mmio_write(Address address, u32 value);
    Result<u32> handle_mmio_read(Address address);
    
    bool is_initialized() const { return initialized_; }
    void dump_configuration() const;
    void dump_pin_status(u8 pin_number) const;

private:
    void initialize_pin_mappings();
    void initialize_default_configuration();
    bool validate_pin_function(u8 pin_number, PinFunction function) const;
    void update_peripheral_routing(u8 pin_number, PinFunction old_function, PinFunction new_function);
    
    bool initialized_;
    std::vector<PinConfiguration> pin_configurations_;
    std::vector<PinMapping> pin_mappings_;
    
    // Peripheral routing tables
    std::unordered_map<std::string, std::vector<u8>> peripheral_pin_assignments_;
    
    mutable std::mutex controller_mutex_;
};

}  // namespace m5tab5::emulator