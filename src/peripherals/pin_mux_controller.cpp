#include "emulator/peripherals/pin_mux_controller.hpp"
#include "emulator/utils/logging.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>

namespace m5tab5::emulator {

DECLARE_LOGGER("PinMuxController");

PinMuxController::PinMuxController()
    : initialized_(false) {
    pin_configurations_.resize(MAX_PINS);
    pin_mappings_.resize(MAX_PINS);
    
    COMPONENT_LOG_DEBUG("Pin mux controller created");
}

PinMuxController::~PinMuxController() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Pin mux controller destroyed");
}

Result<void> PinMuxController::initialize(const Configuration& config) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Pin mux controller already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing pin mux controller");
    
    // Initialize pin mappings and default configuration
    initialize_pin_mappings();
    initialize_default_configuration();
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Pin mux controller initialized successfully");
    
    return {};
}

Result<void> PinMuxController::shutdown() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down pin mux controller");
    
    // Clear all configurations
    for (auto& config : pin_configurations_) {
        config = {};
    }
    
    peripheral_pin_assignments_.clear();
    
    initialized_ = false;
    COMPONENT_LOG_INFO("Pin mux controller shutdown completed");
    
    return {};
}

Result<void> PinMuxController::configure_pin(u8 pin_number, PinFunction function,
                                            u8 peripheral_instance, u8 peripheral_channel) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    if (!validate_pin_function(pin_number, function)) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Function not available for this pin"));
    }
    
    // Check for conflicts
    auto conflict_check = is_function_conflict(pin_number, function, peripheral_instance);
    if (!conflict_check) {
        return std::unexpected(conflict_check.error());
    }
    
    if (conflict_check.value()) {
        auto resolve_result = resolve_conflicts(pin_number, function);
        if (!resolve_result) {
            return std::unexpected(resolve_result.error());
        }
    }
    
    PinFunction old_function = pin_configurations_[pin_number].function;
    
    // Update pin configuration
    pin_configurations_[pin_number].pin_number = pin_number;
    pin_configurations_[pin_number].function = function;
    pin_configurations_[pin_number].peripheral_instance = peripheral_instance;
    pin_configurations_[pin_number].peripheral_channel = peripheral_channel;
    
    // Update peripheral routing
    update_peripheral_routing(pin_number, old_function, function);
    
    COMPONENT_LOG_DEBUG("Pin {} configured as {} (instance {}, channel {})",
                       pin_number, static_cast<u8>(function), peripheral_instance, peripheral_channel);
    
    return {};
}

Result<void> PinMuxController::set_pin_properties(u8 pin_number, PinDriveStrength drive_strength,
                                                  PinSlewRate slew_rate, bool schmitt_trigger, bool open_drain) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    pin_configurations_[pin_number].drive_strength = drive_strength;
    pin_configurations_[pin_number].slew_rate = slew_rate;
    pin_configurations_[pin_number].schmitt_trigger = schmitt_trigger;
    pin_configurations_[pin_number].open_drain = open_drain;
    
    COMPONENT_LOG_DEBUG("Pin {} properties updated: drive={}, slew={}, schmitt={}, open_drain={}",
                       pin_number, static_cast<u8>(drive_strength), static_cast<u8>(slew_rate),
                       schmitt_trigger, open_drain);
    
    return {};
}

Result<void> PinMuxController::set_pin_pull(u8 pin_number, bool pull_up, bool pull_down) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    if (pull_up && pull_down) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Cannot enable both pull-up and pull-down"));
    }
    
    pin_configurations_[pin_number].pull_up_enable = pull_up;
    pin_configurations_[pin_number].pull_down_enable = pull_down;
    
    COMPONENT_LOG_DEBUG("Pin {} pull configuration: up={}, down={}", pin_number, pull_up, pull_down);
    return {};
}

Result<void> PinMuxController::enable_pin_input(u8 pin_number, bool enable) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    pin_configurations_[pin_number].input_enable = enable;
    
    COMPONENT_LOG_DEBUG("Pin {} input {}", pin_number, enable ? "enabled" : "disabled");
    return {};
}

Result<void> PinMuxController::enable_pin_output(u8 pin_number, bool enable) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    pin_configurations_[pin_number].output_enable = enable;
    
    COMPONENT_LOG_DEBUG("Pin {} output {}", pin_number, enable ? "enabled" : "disabled");
    return {};
}

Result<PinConfiguration> PinMuxController::get_pin_configuration(u8 pin_number) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    return pin_configurations_[pin_number];
}

Result<std::vector<u8>> PinMuxController::get_pins_for_function(PinFunction function,
                                                               u8 peripheral_instance) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    std::vector<u8> pins;
    
    for (const auto& config : pin_configurations_) {
        if (config.function == function && 
            (peripheral_instance == 0xFF || config.peripheral_instance == peripheral_instance)) {
            pins.push_back(config.pin_number);
        }
    }
    
    return pins;
}

Result<std::vector<PinFunction>> PinMuxController::get_available_functions(u8 pin_number) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    if (pin_number >= MAX_PINS) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Invalid pin number"));
    }
    
    return pin_mappings_[pin_number].available_functions;
}

Result<bool> PinMuxController::is_function_conflict(u8 pin_number, PinFunction function,
                                                   u8 peripheral_instance) const {
    // Check if another pin is already using this peripheral function
    for (const auto& config : pin_configurations_) {
        if (config.pin_number != pin_number &&
            config.function == function &&
            config.peripheral_instance == peripheral_instance) {
            
            // Some functions can be shared (like GPIO), others cannot
            switch (function) {
                case PinFunction::GPIO:
                case PinFunction::ADC_IN:
                    // These can be shared
                    break;
                    
                default:
                    // Most peripheral functions are exclusive
                    return true;
            }
        }
    }
    
    return false;
}

Result<void> PinMuxController::resolve_conflicts(u8 pin_number, PinFunction new_function) {
    // Find conflicting pins and reset them to GPIO
    for (auto& config : pin_configurations_) {
        if (config.pin_number != pin_number &&
            config.function == new_function &&
            config.peripheral_instance == pin_configurations_[pin_number].peripheral_instance) {
            
            COMPONENT_LOG_WARN("Resolving conflict: resetting pin {} from {} to GPIO",
                              config.pin_number, static_cast<u8>(config.function));
            
            update_peripheral_routing(config.pin_number, config.function, PinFunction::GPIO);
            config.function = PinFunction::GPIO;
            config.peripheral_instance = 0;
            config.peripheral_channel = 0;
        }
    }
    
    return {};
}

Result<void> PinMuxController::load_default_configuration() {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    initialize_default_configuration();
    
    COMPONENT_LOG_INFO("Default pin configuration loaded");
    return {};
}

Result<void> PinMuxController::save_configuration_to_file(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        return std::unexpected(MAKE_ERROR(FILE_ERROR,
            "Cannot open file for writing"));
    }
    
    file << "# Pin Mux Configuration\n";
    file << "# Format: pin_number,function,peripheral_instance,peripheral_channel,drive_strength,slew_rate,input_enable,output_enable,pull_up,pull_down,schmitt_trigger,open_drain\n";
    
    for (const auto& config : pin_configurations_) {
        file << static_cast<u32>(config.pin_number) << ","
             << static_cast<u32>(config.function) << ","
             << static_cast<u32>(config.peripheral_instance) << ","
             << static_cast<u32>(config.peripheral_channel) << ","
             << static_cast<u32>(config.drive_strength) << ","
             << static_cast<u32>(config.slew_rate) << ","
             << config.input_enable << ","
             << config.output_enable << ","
             << config.pull_up_enable << ","
             << config.pull_down_enable << ","
             << config.schmitt_trigger << ","
             << config.open_drain << "\n";
    }
    
    file.close();
    
    COMPONENT_LOG_INFO("Pin configuration saved to {}", filename);
    return {};
}

Result<void> PinMuxController::load_configuration_from_file(const std::string& filename) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        return std::unexpected(MAKE_ERROR(FILE_ERROR,
            "Cannot open file for reading"));
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue; // Skip comments and empty lines
        }
        
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() != 12) {
            continue; // Skip malformed lines
        }
        
        try {
            u8 pin_number = static_cast<u8>(std::stoul(tokens[0]));
            if (pin_number >= MAX_PINS) {
                continue;
            }
            
            auto& config = pin_configurations_[pin_number];
            config.pin_number = pin_number;
            config.function = static_cast<PinFunction>(std::stoul(tokens[1]));
            config.peripheral_instance = static_cast<u8>(std::stoul(tokens[2]));
            config.peripheral_channel = static_cast<u8>(std::stoul(tokens[3]));
            config.drive_strength = static_cast<PinDriveStrength>(std::stoul(tokens[4]));
            config.slew_rate = static_cast<PinSlewRate>(std::stoul(tokens[5]));
            config.input_enable = (tokens[6] == "1");
            config.output_enable = (tokens[7] == "1");
            config.pull_up_enable = (tokens[8] == "1");
            config.pull_down_enable = (tokens[9] == "1");
            config.schmitt_trigger = (tokens[10] == "1");
            config.open_drain = (tokens[11] == "1");
            
        } catch (const std::exception&) {
            // Skip invalid lines
            continue;
        }
    }
    
    file.close();
    
    COMPONENT_LOG_INFO("Pin configuration loaded from {}", filename);
    return {};
}

Result<void> PinMuxController::handle_mmio_write(Address address, u32 value) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    Address offset = address - PIN_MUX_BASE_ADDR;
    
    if (offset >= MAX_PINS * 4) {
        return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Invalid pin mux register address"));
    }
    
    u8 pin_number = offset / 4;
    
    // Decode register value
    PinFunction function = static_cast<PinFunction>(value & 0xFF);
    u8 peripheral_instance = (value >> 8) & 0x0F;
    u8 peripheral_channel = (value >> 12) & 0x0F;
    PinDriveStrength drive_strength = static_cast<PinDriveStrength>((value >> 16) & 0x03);
    PinSlewRate slew_rate = static_cast<PinSlewRate>((value >> 18) & 0x01);
    bool input_enable = (value >> 19) & 0x01;
    bool output_enable = (value >> 20) & 0x01;
    bool pull_up = (value >> 21) & 0x01;
    bool pull_down = (value >> 22) & 0x01;
    bool schmitt_trigger = (value >> 23) & 0x01;
    bool open_drain = (value >> 24) & 0x01;
    
    // Apply configuration
    configure_pin(pin_number, function, peripheral_instance, peripheral_channel);
    set_pin_properties(pin_number, drive_strength, slew_rate, schmitt_trigger, open_drain);
    set_pin_pull(pin_number, pull_up, pull_down);
    enable_pin_input(pin_number, input_enable);
    enable_pin_output(pin_number, output_enable);
    
    return {};
}

Result<u32> PinMuxController::handle_mmio_read(Address address) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Pin mux controller not initialized"));
    }
    
    Address offset = address - PIN_MUX_BASE_ADDR;
    
    if (offset >= MAX_PINS * 4) {
        return std::unexpected(MAKE_ERROR(MEMORY_INVALID_ADDRESS,
            "Invalid pin mux register address"));
    }
    
    u8 pin_number = offset / 4;
    const auto& config = pin_configurations_[pin_number];
    
    // Encode register value
    u32 value = static_cast<u32>(config.function) |
                (static_cast<u32>(config.peripheral_instance) << 8) |
                (static_cast<u32>(config.peripheral_channel) << 12) |
                (static_cast<u32>(config.drive_strength) << 16) |
                (static_cast<u32>(config.slew_rate) << 18) |
                (config.input_enable ? (1U << 19) : 0) |
                (config.output_enable ? (1U << 20) : 0) |
                (config.pull_up_enable ? (1U << 21) : 0) |
                (config.pull_down_enable ? (1U << 22) : 0) |
                (config.schmitt_trigger ? (1U << 23) : 0) |
                (config.open_drain ? (1U << 24) : 0);
    
    return value;
}

void PinMuxController::initialize_pin_mappings() {
    // Initialize ESP32-P4 specific pin mappings
    for (u8 i = 0; i < MAX_PINS; ++i) {
        pin_mappings_[i].pin_number = i;
        pin_mappings_[i].function = PinFunction::GPIO;
        pin_mappings_[i].peripheral_instance = 0;
        pin_mappings_[i].peripheral_channel = 0;
        pin_mappings_[i].is_configurable = true;
        pin_mappings_[i].available_functions = {PinFunction::GPIO};
    }
    
    // Define specific pin capabilities based on ESP32-P4 pinout
    
    // UART pins
    std::vector<u8> uart_pins = {1, 2, 3, 4, 5, 6, 7, 8, 43, 44};
    for (u8 pin : uart_pins) {
        if (pin < MAX_PINS) {
            auto& funcs = pin_mappings_[pin].available_functions;
            funcs.insert(funcs.end(), {PinFunction::UART_TX, PinFunction::UART_RX,
                                     PinFunction::UART_RTS, PinFunction::UART_CTS});
        }
    }
    
    // SPI pins
    std::vector<u8> spi_pins = {9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    for (u8 pin : spi_pins) {
        if (pin < MAX_PINS) {
            auto& funcs = pin_mappings_[pin].available_functions;
            funcs.insert(funcs.end(), {PinFunction::SPI_CLK, PinFunction::SPI_MOSI,
                                     PinFunction::SPI_MISO, PinFunction::SPI_CS});
        }
    }
    
    // I2C pins
    std::vector<u8> i2c_pins = {21, 22, 23, 24};
    for (u8 pin : i2c_pins) {
        if (pin < MAX_PINS) {
            auto& funcs = pin_mappings_[pin].available_functions;
            funcs.insert(funcs.end(), {PinFunction::I2C_SDA, PinFunction::I2C_SCL});
        }
    }
    
    // PWM pins (most GPIO pins can be PWM)
    for (u8 i = 25; i < 45; ++i) {
        if (i < MAX_PINS) {
            auto& funcs = pin_mappings_[i].available_functions;
            funcs.insert(funcs.end(), {PinFunction::PWM_OUT, PinFunction::PWM_COMP});
        }
    }
    
    // ADC pins
    std::vector<u8> adc_pins = {45, 46, 47, 48, 49, 50, 51, 52};
    for (u8 pin : adc_pins) {
        if (pin < MAX_PINS) {
            auto& funcs = pin_mappings_[pin].available_functions;
            funcs.push_back(PinFunction::ADC_IN);
        }
    }
    
    // Touch pins
    pin_mappings_[53].available_functions.push_back(PinFunction::TOUCH_INT);
    pin_mappings_[54].available_functions.push_back(PinFunction::TOUCH_RST);
    
    // USB pins (fixed function)
    if (MAX_PINS > 55) {
        pin_mappings_[53].available_functions = {PinFunction::USB_DP};
        pin_mappings_[53].is_configurable = false;
        pin_mappings_[54].available_functions = {PinFunction::USB_DM};
        pin_mappings_[54].is_configurable = false;
    }
}

void PinMuxController::initialize_default_configuration() {
    // Set default pin configurations
    for (u8 i = 0; i < MAX_PINS; ++i) {
        auto& config = pin_configurations_[i];
        config.pin_number = i;
        config.function = PinFunction::GPIO;
        config.drive_strength = PinDriveStrength::MEDIUM;
        config.slew_rate = PinSlewRate::SLOW;
        config.input_enable = true;
        config.output_enable = false;
        config.pull_up_enable = false;
        config.pull_down_enable = false;
        config.schmitt_trigger = false;
        config.open_drain = false;
        config.peripheral_instance = 0;
        config.peripheral_channel = 0;
    }
    
    // Configure default peripherals
    // UART0 for debug console
    configure_pin(43, PinFunction::UART_TX, 0, 0);
    configure_pin(44, PinFunction::UART_RX, 0, 0);
    
    // I2C0 for touch controller
    configure_pin(21, PinFunction::I2C_SDA, 0, 0);
    configure_pin(22, PinFunction::I2C_SCL, 0, 0);
    
    // Touch controller pins
    configure_pin(53, PinFunction::TOUCH_INT, 0, 0);
    configure_pin(54, PinFunction::TOUCH_RST, 0, 0);
    
    // Some ADC pins
    for (u8 i = 45; i < 50; ++i) {
        configure_pin(i, PinFunction::ADC_IN, 0, i - 45);
    }
}

bool PinMuxController::validate_pin_function(u8 pin_number, PinFunction function) const {
    if (pin_number >= MAX_PINS) {
        return false;
    }
    
    const auto& available = pin_mappings_[pin_number].available_functions;
    return std::find(available.begin(), available.end(), function) != available.end();
}

void PinMuxController::update_peripheral_routing(u8 pin_number, PinFunction old_function, PinFunction new_function) {
    // Remove from old peripheral assignment
    if (old_function != PinFunction::GPIO && old_function != PinFunction::INVALID) {
        std::string old_key = std::to_string(static_cast<u8>(old_function)) + "_" +
                             std::to_string(pin_configurations_[pin_number].peripheral_instance);
        auto& old_pins = peripheral_pin_assignments_[old_key];
        old_pins.erase(std::remove(old_pins.begin(), old_pins.end(), pin_number), old_pins.end());
    }
    
    // Add to new peripheral assignment
    if (new_function != PinFunction::GPIO && new_function != PinFunction::INVALID) {
        std::string new_key = std::to_string(static_cast<u8>(new_function)) + "_" +
                             std::to_string(pin_configurations_[pin_number].peripheral_instance);
        peripheral_pin_assignments_[new_key].push_back(pin_number);
    }
}

void PinMuxController::dump_configuration() const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    COMPONENT_LOG_INFO("=== Pin Mux Configuration ===");
    COMPONENT_LOG_INFO("Initialized: {}", initialized_);
    
    if (!initialized_) {
        return;
    }
    
    // Group pins by function
    std::unordered_map<PinFunction, std::vector<u8>> function_pins;
    for (const auto& config : pin_configurations_) {
        function_pins[config.function].push_back(config.pin_number);
    }
    
    for (const auto& [function, pins] : function_pins) {
        if (!pins.empty()) {
            COMPONENT_LOG_INFO("Function {}: {} pins", static_cast<u8>(function), pins.size());
            
            for (u8 pin : pins) {
                const auto& config = pin_configurations_[pin];
                if (config.function != PinFunction::GPIO) {
                    COMPONENT_LOG_INFO("  Pin {}: instance={}, channel={}, drive={}, I/O={}/{}",
                                      pin, config.peripheral_instance, config.peripheral_channel,
                                      static_cast<u8>(config.drive_strength),
                                      config.input_enable ? "I" : "-",
                                      config.output_enable ? "O" : "-");
                }
            }
        }
    }
}

void PinMuxController::dump_pin_status(u8 pin_number) const {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    
    if (!initialized_ || pin_number >= MAX_PINS) {
        COMPONENT_LOG_ERROR("Invalid pin number or not initialized");
        return;
    }
    
    const auto& config = pin_configurations_[pin_number];
    const auto& mapping = pin_mappings_[pin_number];
    
    COMPONENT_LOG_INFO("=== Pin {} Status ===", pin_number);
    COMPONENT_LOG_INFO("Function: {} (instance {}, channel {})",
                      static_cast<u8>(config.function), config.peripheral_instance, config.peripheral_channel);
    COMPONENT_LOG_INFO("Properties: drive={}, slew={}, schmitt={}, open_drain={}",
                      static_cast<u8>(config.drive_strength), static_cast<u8>(config.slew_rate),
                      config.schmitt_trigger, config.open_drain);
    COMPONENT_LOG_INFO("I/O: input={}, output={}, pull_up={}, pull_down={}",
                      config.input_enable, config.output_enable,
                      config.pull_up_enable, config.pull_down_enable);
    COMPONENT_LOG_INFO("Configurable: {}", mapping.is_configurable);
    
    COMPONENT_LOG_INFO("Available functions:");
    for (auto func : mapping.available_functions) {
        COMPONENT_LOG_INFO("  {}", static_cast<u8>(func));
    }
}

}  // namespace m5tab5::emulator