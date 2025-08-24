#pragma once

#include "emulator/gui/control_panels.hpp"
#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"

#include <memory>
#include <vector>
#include <set>
#include <map>

namespace m5tab5::emulator::gui {

/**
 * @brief Memory inspector with hex editor and data visualization
 */
class MemoryInspector : public DockWidget {
public:
    enum class DataType {
        Uint8, Int8, Uint16, Int16, Uint32, Int32, 
        Uint64, Int64, Float, Double, ASCII, UTF8
    };

    enum class Endianness {
        Little, Big
    };

    struct MemoryRegion {
        u32 start_address;
        u32 end_address;
        std::string name;
        bool readable = true;
        bool writable = true;
        bool executable = false;
    };

    explicit MemoryInspector(MainWindow& parent);
    ~MemoryInspector() override = default;

    void render() override;
    void update() override;

    void goto_address(u32 address);
    void set_data_type(DataType type) { data_type_ = type; }
    void set_endianness(Endianness endian) { endianness_ = endian; }

private:
    // View state
    u32 current_address_ = 0x4FF00000; // Default to SRAM
    DataType data_type_ = DataType::Uint8;
    Endianness endianness_ = Endianness::Little;
    u32 bytes_per_row_ = 16;
    u32 visible_rows_ = 20;
    
    // Memory data
    std::vector<u8> memory_data_;
    std::vector<MemoryRegion> memory_regions_;
    
    // UI state
    bool show_ascii_ = true;
    bool show_addresses_ = true;
    bool highlight_changes_ = true;
    std::string goto_address_text_;
    std::string search_text_;
    std::vector<u32> search_results_;
    u32 current_search_index_ = 0;
    
    void render_toolbar();
    void render_address_bar();
    void render_hex_grid();
    void render_ascii_panel();
    void render_data_inspector();
    void render_goto_dialog();
    void render_search_panel();
    
    void load_memory_data();
    void handle_memory_edit(u32 address, u8 value);
    void handle_goto_address();
    void handle_search();
    void highlight_byte(u32 address, u32 color);
    
    std::string format_data_at_address(u32 address) const;
    u32 get_region_color(u32 address) const;
    bool is_address_valid(u32 address) const;
};

/**
 * @brief CPU register viewer and editor
 */
class RegisterViewer : public DockWidget {
public:
    enum class RegisterGroup {
        General,     // x0-x31 RISC-V general purpose
        FloatingPoint, // f0-f31 FPU registers
        Control,     // Control and Status Registers (CSR)
        Debug        // Debug registers
    };

    struct Register {
        std::string name;
        u32 value = 0;
        u32 previous_value = 0;
        bool changed = false;
        bool read_only = false;
        std::string description;
    };

    explicit RegisterViewer(MainWindow& parent);
    ~RegisterViewer() override = default;

    void render() override;
    void update() override;

private:
    RegisterGroup selected_group_ = RegisterGroup::General;
    std::map<RegisterGroup, std::vector<Register>> register_groups_;
    
    // UI state
    bool show_descriptions_ = false;
    bool highlight_changes_ = true;
    std::string filter_text_;
    
    void render_group_selector();
    void render_register_list();
    void render_register_item(Register& reg);
    void render_register_details();
    
    void update_registers();
    void handle_register_edit(const std::string& name, u32 new_value);
    void setup_register_groups();
    u32 get_change_color(const Register& reg) const;
    bool should_show_register(const Register& reg) const;
};

/**
 * @brief Disassembly viewer with breakpoint support
 */
class DisassemblyViewer : public DockWidget {
public:
    struct Instruction {
        u32 address;
        u32 opcode;
        std::string mnemonic;
        std::string operands;
        std::string comment;
        bool is_breakpoint = false;
        bool is_current_pc = false;
        u32 execution_count = 0;
    };

    explicit DisassemblyViewer(MainWindow& parent);
    ~DisassemblyViewer() override = default;

    void render() override;
    void update() override;

    void goto_address(u32 address);
    void set_pc(u32 pc) { current_pc_ = pc; }
    void add_breakpoint(u32 address);
    void remove_breakpoint(u32 address);
    void toggle_breakpoint(u32 address);

private:
    // View state
    u32 view_address_ = 0x40000000; // Default to Flash start
    u32 current_pc_ = 0;
    u32 visible_instructions_ = 30;
    
    // Instructions and breakpoints
    std::vector<Instruction> instructions_;
    std::set<u32> breakpoints_;
    
    // UI state
    bool follow_pc_ = true;
    bool show_addresses_ = true;
    bool show_opcodes_ = false;
    bool show_execution_count_ = false;
    std::string goto_address_text_;
    
    void render_toolbar();
    void render_instruction_list();
    void render_instruction_line(const Instruction& instr);
    void render_goto_dialog();
    void render_breakpoint_list();
    
    void load_instructions();
    void handle_instruction_click(u32 address);
    void handle_breakpoint_toggle(u32 address);
    void handle_goto_address();
    
    u32 get_instruction_color(const Instruction& instr) const;
    std::string format_instruction(const Instruction& instr) const;
    Instruction disassemble_at_address(u32 address) const;
};

/**
 * @brief Debug controls with step, continue, and breakpoint management
 */
class DebugControls : public DockWidget {
public:
    enum class ExecutionState {
        Running, Paused, Stepping, Stopped
    };

    explicit DebugControls(MainWindow& parent);
    ~DebugControls() override = default;

    void render() override;
    void update() override;

    void set_execution_state(ExecutionState state) { execution_state_ = state; }
    ExecutionState get_execution_state() const { return execution_state_; }

private:
    ExecutionState execution_state_ = ExecutionState::Stopped;
    
    // Debug session info
    u64 total_cycles_ = 0;
    u64 total_instructions_ = 0;
    u32 current_pc_ = 0;
    float execution_speed_ = 1.0f;
    
    // Stepping controls
    u32 step_count_ = 1;
    bool step_over_calls_ = false;
    
    void render_execution_controls();
    void render_stepping_controls();
    void render_breakpoint_controls();
    void render_session_info();
    void render_speed_control();
    
    // Debug actions
    void handle_start_debug();
    void handle_pause_debug();
    void handle_stop_debug();
    void handle_step_into();
    void handle_step_over();
    void handle_step_out();
    void handle_run_to_cursor();
    void handle_speed_change(float new_speed);
};

/**
 * @brief Call stack viewer for debugging function calls
 */
class CallStackViewer : public DockWidget {
public:
    struct StackFrame {
        u32 pc;
        u32 sp;
        std::string function_name;
        std::string file_name;
        u32 line_number = 0;
        std::map<std::string, u32> local_variables;
    };

    explicit CallStackViewer(MainWindow& parent);
    ~CallStackViewer() override = default;

    void render() override;
    void update() override;

    void set_stack_frames(const std::vector<StackFrame>& frames);

private:
    std::vector<StackFrame> stack_frames_;
    u32 selected_frame_ = 0;
    
    // UI state
    bool show_addresses_ = true;
    bool show_local_variables_ = false;
    
    void render_stack_list();
    void render_frame_details();
    void render_local_variables();
    
    void handle_frame_selection(u32 frame_index);
    std::string format_stack_frame(const StackFrame& frame) const;
    void update_stack_trace();
};

/**
 * @brief Variable watch window for monitoring values
 */
class WatchViewer : public DockWidget {
public:
    struct WatchVariable {
        std::string expression;
        std::string value;
        std::string type;
        u32 address = 0;
        bool valid = true;
        bool changed = false;
    };

    explicit WatchViewer(MainWindow& parent);
    ~WatchViewer() override = default;

    void render() override;
    void update() override;

    void add_watch(const std::string& expression);
    void remove_watch(u32 index);
    void clear_watches();

private:
    std::vector<WatchVariable> watch_variables_;
    std::string new_expression_;
    
    // UI state
    bool show_addresses_ = false;
    bool show_types_ = true;
    bool highlight_changes_ = true;
    
    void render_add_watch();
    void render_watch_list();
    void render_watch_item(WatchVariable& watch, u32 index);
    
    void handle_add_watch();
    void handle_remove_watch(u32 index);
    void handle_expression_edit(u32 index, const std::string& new_expr);
    void update_watch_values();
    
    WatchVariable evaluate_expression(const std::string& expression);
    u32 get_change_color(const WatchVariable& watch) const;
};

} // namespace m5tab5::emulator::gui