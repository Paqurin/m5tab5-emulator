#pragma once

/**
 * @file cpu_integration.hpp
 * @brief Integration layer between FreeRTOS scheduler and CPU execution engine
 * 
 * This header provides the interface for integrating the FreeRTOS task scheduler
 * with the RISC-V CPU execution engine, enabling real task execution on emulated cores.
 */

#include "emulator/core/types.hpp"
#include "emulator/utils/error.hpp"

namespace m5tab5::emulator {
    class DualCoreManager;
}

namespace m5tab5::emulator::freertos {
    class TaskScheduler;

/**
 * @brief Initialize CPU-FreeRTOS integration
 * 
 * This function sets up the integration between the CPU execution engine
 * and the FreeRTOS task scheduler, enabling tasks to actually execute
 * on the emulated RISC-V cores.
 * 
 * @param cpu_manager Reference to the dual-core manager
 * @param scheduler Reference to the task scheduler
 * @return Result indicating success or error
 */
Result<void> initialize_cpu_integration(DualCoreManager& cpu_manager, TaskScheduler& scheduler);

/**
 * @brief Start CPU-FreeRTOS integration
 * 
 * Begins the execution loops for both CPU cores, enabling tasks to run.
 * This should be called after the scheduler has been started.
 * 
 * @return Result indicating success or error
 */
Result<void> start_cpu_integration();

/**
 * @brief Stop CPU-FreeRTOS integration
 * 
 * Stops the execution loops, halting task execution.
 * 
 * @return Result indicating success or error
 */
Result<void> stop_cpu_integration();

/**
 * @brief Shutdown CPU-FreeRTOS integration
 * 
 * Cleans up all resources used by the integration layer.
 */
void shutdown_cpu_integration();

/**
 * @brief Check if CPU integration is running
 * 
 * @return True if the integration is active and tasks can execute
 */
bool is_cpu_integration_running();

} // namespace m5tab5::emulator::freertos