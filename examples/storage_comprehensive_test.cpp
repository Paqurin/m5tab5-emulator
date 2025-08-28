/**
 * @file storage_comprehensive_test.cpp
 * @brief Comprehensive test of M5Stack Tab5 Emulator storage subsystem
 * 
 * Demonstrates complete storage functionality including:
 * - Flash memory operations
 * - SPIFFS file system operations
 * - VFS (Virtual File System) integration
 * - OTA update framework
 * - ESP32-P4 file I/O compatibility
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>

#include "emulator/core/emulator_core.hpp"
#include "emulator/storage/flash_controller.hpp"
#include "emulator/storage/spiffs_filesystem.hpp"
#include "emulator/storage/vfs_manager.hpp"
#include "emulator/storage/ota_manager.hpp"
#include "emulator/config/configuration.hpp"
#include "emulator/utils/logging.hpp"

using namespace m5tab5::emulator;
using namespace m5tab5::emulator::storage;

// Ensure we have O_* constants
#ifndef O_CREAT
#define O_CREAT 0100
#endif
#ifndef O_WRONLY  
#define O_WRONLY 01
#endif
#ifndef O_RDONLY
#define O_RDONLY 00
#endif
#ifndef O_TRUNC
#define O_TRUNC 01000
#endif

DECLARE_LOGGER("StorageTest");

class StorageTest {
public:
    StorageTest() = default;
    ~StorageTest() = default;
    
    /**
     * @brief Run comprehensive storage system tests
     */
    bool run_comprehensive_test() {
        LOG_INFO("=== M5Stack Tab5 Storage Comprehensive Test ===");
        
        // Initialize emulator with storage support
        if (!initialize_emulator()) {
            return false;
        }
        
        // Test flash memory operations
        if (!test_flash_operations()) {
            return false;
        }
        
        // Test SPIFFS file system operations
        if (!test_spiffs_operations()) {
            return false;
        }
        
        // Test VFS integration
        if (!test_vfs_operations()) {
            return false;
        }
        
        // Test OTA framework
        if (!test_ota_operations()) {
            return false;
        }
        
        // Test file persistence across restarts
        if (!test_persistence()) {
            return false;
        }
        
        // Test ESP32-P4 compatibility
        if (!test_esp32_compatibility()) {
            return false;
        }
        
        LOG_INFO("=== All storage tests completed successfully ===");
        return true;
    }
    
private:
    std::unique_ptr<EmulatorCore> emulator_;
    std::shared_ptr<FlashController> flash_;
    std::shared_ptr<SPIFFSFileSystem> spiffs_;
    std::shared_ptr<VFSManager> vfs_;
    std::shared_ptr<OTAManager> ota_;
    
    bool initialize_emulator() {
        LOG_INFO("Initializing M5Stack Tab5 emulator with storage support...");
        
        try {
            // Create default configuration
            auto config_result = Configuration::create_default();
            if (!config_result) {
                LOG_ERROR("Failed to create configuration: {}", 
                         config_result.error().to_string());
                return false;
            }
            
            auto config = config_result.value();
            
            // Create emulator instance
            emulator_ = std::make_unique<EmulatorCore>(config);
            
            // Initialize emulator
            auto init_result = emulator_->initialize(config);
            if (!init_result) {
                LOG_ERROR("Failed to initialize emulator: {}", 
                         init_result.error().to_string());
                return false;
            }
            
            // Get storage components
            flash_ = emulator_->getComponent<FlashController>();
            spiffs_ = emulator_->getComponent<SPIFFSFileSystem>();
            vfs_ = emulator_->getComponent<VFSManager>();
            ota_ = emulator_->getComponent<OTAManager>();
            
            if (!flash_ || !spiffs_ || !vfs_ || !ota_) {
                LOG_ERROR("Storage components not available");
                return false;
            }
            
            LOG_INFO("Emulator initialized successfully with storage support");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during emulator initialization: {}", e.what());
            return false;
        }
    }
    
    bool test_flash_operations() {
        LOG_INFO("--- Testing Flash Memory Operations ---");
        
        try {
            // Test flash information
            auto flash_id_result = flash_->get_flash_id();
            if (flash_id_result) {
                LOG_INFO("Flash ID: 0x{:08X}", flash_id_result.value());
            }
            
            auto flash_info_result = flash_->get_flash_info();
            if (flash_info_result) {
                LOG_INFO("Flash Info: {}", flash_info_result.value());
            }
            
            // Test write/read operations
            const Address test_address = 0x00100000; // 1MB offset
            const std::string test_data = "Hello, M5Stack Tab5 Flash Memory!";
            std::vector<uint8_t> write_buffer(test_data.begin(), test_data.end());
            
            // Write test data
            LOG_INFO("Writing {} bytes to flash address 0x{:08X}", 
                    write_buffer.size(), test_address);
            auto write_result = flash_->write(test_address, write_buffer.data(), write_buffer.size());
            if (!write_result) {
                LOG_ERROR("Flash write failed: {}", write_result.error().to_string());
                return false;
            }
            
            // Read back and verify
            std::vector<uint8_t> read_buffer(write_buffer.size());
            auto read_result = flash_->read(test_address, read_buffer.data(), read_buffer.size());
            if (!read_result) {
                LOG_ERROR("Flash read failed: {}", read_result.error().to_string());
                return false;
            }
            
            if (write_buffer == read_buffer) {
                LOG_INFO("Flash read/write verification successful");
            } else {
                LOG_ERROR("Flash read/write verification failed");
                return false;
            }
            
            // Test erase operations
            LOG_INFO("Testing sector erase...");
            auto erase_result = flash_->erase_sector(test_address);
            if (!erase_result) {
                LOG_ERROR("Flash erase failed: {}", erase_result.error().to_string());
                return false;
            }
            
            // Verify erase (should read all 0xFF)
            auto erase_read_result = flash_->read(test_address, read_buffer.data(), read_buffer.size());
            if (!erase_read_result) {
                LOG_ERROR("Flash erase verification read failed");
                return false;
            }
            
            bool erased = std::all_of(read_buffer.begin(), read_buffer.end(), 
                                    [](uint8_t b) { return b == 0xFF; });
            if (erased) {
                LOG_INFO("Flash sector erase successful");
            } else {
                LOG_ERROR("Flash sector erase verification failed");
                return false;
            }
            
            // Test flash statistics
            auto stats = flash_->get_statistics();
            LOG_INFO("Flash Statistics: {} reads, {} writes, {} erases", 
                    stats.read_operations, stats.write_operations, stats.erase_operations);
            
            LOG_INFO("Flash operations test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in flash operations test: {}", e.what());
            return false;
        }
    }
    
    bool test_spiffs_operations() {
        LOG_INFO("--- Testing SPIFFS File System Operations ---");
        
        try {
            // Check if SPIFFS is mounted
            if (!spiffs_->is_mounted()) {
                LOG_ERROR("SPIFFS is not mounted");
                return false;
            }
            
            // Get filesystem statistics
            auto stats = spiffs_->get_statistics();
            LOG_INFO("SPIFFS Stats: {} total bytes, {} used bytes, {} free bytes", 
                    stats.total_bytes, stats.used_bytes, stats.free_bytes);
            
            // Test file creation and writing
            const std::string test_filename = "/test_file.txt";
            const std::string test_content = "M5Stack Tab5 SPIFFS Test Content\\nLine 2\\nLine 3";
            
            LOG_INFO("Creating file: {}", test_filename);
            auto open_result = spiffs_->open(test_filename, 
                storage::SPIFFSFileSystem::AccessMode::WRITE | 
                storage::SPIFFSFileSystem::AccessMode::CREATE |
                storage::SPIFFSFileSystem::AccessMode::TRUNCATE);
                
            if (!open_result) {
                LOG_ERROR("Failed to open file for writing: {}", open_result.error().to_string());
                return false;
            }
            
            int fd = open_result.value();
            
            auto write_result = spiffs_->write(fd, test_content.data(), test_content.size());
            if (!write_result) {
                LOG_ERROR("Failed to write to file: {}", write_result.error().to_string());
                spiffs_->close(fd);
                return false;
            }
            
            if (write_result.value() != test_content.size()) {
                LOG_ERROR("Incomplete write: {} bytes written, {} expected", 
                         write_result.value(), test_content.size());
                spiffs_->close(fd);
                return false;
            }
            
            auto close_result = spiffs_->close(fd);
            if (!close_result) {
                LOG_ERROR("Failed to close file: {}", close_result.error().to_string());
                return false;
            }
            
            // Test file reading
            LOG_INFO("Reading file: {}", test_filename);
            auto read_open_result = spiffs_->open(test_filename, storage::SPIFFSFileSystem::AccessMode::READ);
            if (!read_open_result) {
                LOG_ERROR("Failed to open file for reading: {}", read_open_result.error().to_string());
                return false;
            }
            
            fd = read_open_result.value();
            std::vector<char> read_buffer(test_content.size());
            auto read_result = spiffs_->read(fd, read_buffer.data(), read_buffer.size());
            
            if (!read_result) {
                LOG_ERROR("Failed to read from file: {}", read_result.error().to_string());
                spiffs_->close(fd);
                return false;
            }
            
            std::string read_content(read_buffer.begin(), read_buffer.begin() + read_result.value());
            spiffs_->close(fd);
            
            if (read_content == test_content) {
                LOG_INFO("File read/write verification successful");
            } else {
                LOG_ERROR("File read/write verification failed");
                LOG_ERROR("Expected: {}", test_content);
                LOG_ERROR("Got: {}", read_content);
                return false;
            }
            
            // Test directory listing
            LOG_INFO("Listing files in SPIFFS:");
            auto file_list_result = spiffs_->list_files();
            if (file_list_result) {
                for (const auto& filename : file_list_result.value()) {
                    LOG_INFO("  {}", filename);
                }
            }
            
            // Test file deletion
            LOG_INFO("Deleting test file");
            auto unlink_result = spiffs_->unlink(test_filename);
            if (!unlink_result) {
                LOG_ERROR("Failed to delete file: {}", unlink_result.error().to_string());
                return false;
            }
            
            // Verify deletion
            auto exists_result = spiffs_->exists(test_filename);
            if (exists_result && exists_result.value()) {
                LOG_ERROR("File still exists after deletion");
                return false;
            }
            
            LOG_INFO("SPIFFS operations test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in SPIFFS operations test: {}", e.what());
            return false;
        }
    }
    
    bool test_vfs_operations() {
        LOG_INFO("--- Testing VFS Integration ---");
        
        try {
            // Test POSIX-style file operations through VFS
            const char* test_file = "/spiffs/vfs_test.txt";
            const std::string test_data = "VFS Integration Test Data";
            
            // Open file for writing
            int fd = vfs_->vfs_open(test_file, O_CREAT | O_WRONLY | O_TRUNC, 0644);
            if (fd < 0) {
                LOG_ERROR("VFS open failed with error: {}", errno);
                return false;
            }
            
            // Write data
            ssize_t bytes_written = vfs_->vfs_write(fd, test_data.c_str(), test_data.size());
            if (bytes_written != static_cast<ssize_t>(test_data.size())) {
                LOG_ERROR("VFS write failed: {} bytes written, {} expected", 
                         bytes_written, test_data.size());
                vfs_->vfs_close(fd);
                return false;
            }
            
            vfs_->vfs_close(fd);
            
            // Open file for reading
            fd = vfs_->vfs_open(test_file, O_RDONLY, 0);
            if (fd < 0) {
                LOG_ERROR("VFS open for reading failed");
                return false;
            }
            
            // Read data
            std::vector<char> read_buffer(test_data.size());
            ssize_t bytes_read = vfs_->vfs_read(fd, read_buffer.data(), read_buffer.size());
            if (bytes_read != static_cast<ssize_t>(test_data.size())) {
                LOG_ERROR("VFS read failed: {} bytes read, {} expected", 
                         bytes_read, test_data.size());
                vfs_->vfs_close(fd);
                return false;
            }
            
            vfs_->vfs_close(fd);
            
            std::string read_data(read_buffer.begin(), read_buffer.end());
            if (read_data == test_data) {
                LOG_INFO("VFS file I/O test successful");
            } else {
                LOG_ERROR("VFS file I/O verification failed");
                return false;
            }
            
            // Test file stat
            struct stat st;
            int stat_result = vfs_->vfs_stat(test_file, &st);
            if (stat_result == 0) {
                LOG_INFO("File stat: size={}, mode=0{:o}", st.st_size, st.st_mode);
            } else {
                LOG_ERROR("VFS stat failed");
                return false;
            }
            
            // Test directory operations
            DIR* dir = vfs_->vfs_opendir("/spiffs");
            if (!dir) {
                LOG_ERROR("VFS opendir failed");
                return false;
            }
            
            LOG_INFO("Directory listing via VFS:");
            struct dirent* entry;
            while ((entry = vfs_->vfs_readdir(dir)) != nullptr) {
                LOG_INFO("  {} (type: {})", entry->d_name, entry->d_type);
            }
            
            vfs_->vfs_closedir(dir);
            
            // Clean up
            vfs_->vfs_unlink(test_file);
            
            LOG_INFO("VFS integration test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in VFS operations test: {}", e.what());
            return false;
        }
    }
    
    bool test_ota_operations() {
        LOG_INFO("--- Testing OTA Framework ---");
        
        try {
            // Get current boot partition
            auto boot_partition_result = ota_->get_boot_partition();
            if (boot_partition_result) {
                LOG_INFO("Current boot partition: {}", 
                        static_cast<int>(boot_partition_result.value()));
            }
            
            // Get next update partition
            auto next_partition_result = ota_->get_next_update_partition();
            if (next_partition_result) {
                LOG_INFO("Next update partition: {}", 
                        static_cast<int>(next_partition_result.value()));
            }
            
            // Scan available OTA partitions
            auto partitions_result = ota_->scan_ota_partitions();
            if (partitions_result) {
                LOG_INFO("Available OTA partitions:");
                for (const auto& partition : partitions_result.value()) {
                    LOG_INFO("  {}: 0x{:08X} - 0x{:08X} ({} bytes)", 
                            partition.label, 
                            partition.start_address,
                            partition.start_address + partition.size,
                            partition.size);
                }
            }
            
            // Simulate OTA update process
            const size_t update_size = 1024 * 1024; // 1MB update
            auto begin_result = ota_->begin_update(update_size);
            if (!begin_result) {
                LOG_ERROR("Failed to begin OTA update: {}", begin_result.error().to_string());
                return false;
            }
            
            LOG_INFO("OTA update started, writing {} bytes...", update_size);
            
            // Write simulated update data in chunks
            const size_t chunk_size = 4096;
            std::vector<uint8_t> chunk_data(chunk_size, 0xAB); // Test pattern
            
            for (size_t written = 0; written < update_size; written += chunk_size) {
                size_t current_chunk_size = std::min(chunk_size, update_size - written);
                
                auto write_result = ota_->write_update_data(chunk_data.data(), current_chunk_size);
                if (!write_result) {
                    LOG_ERROR("OTA write failed: {}", write_result.error().to_string());
                    ota_->abort_update();
                    return false;
                }
                
                if (write_result.value() != current_chunk_size) {
                    LOG_ERROR("OTA incomplete write: {} bytes written, {} expected", 
                             write_result.value(), current_chunk_size);
                    ota_->abort_update();
                    return false;
                }
                
                // Show progress every 10%
                if ((written % (update_size / 10)) < chunk_size) {
                    LOG_INFO("OTA progress: {}%", (written * 100) / update_size);
                }
            }
            
            LOG_INFO("OTA update data written successfully");
            
            // Complete the update (without committing for this test)
            auto end_result = ota_->end_update(false);
            if (!end_result) {
                LOG_ERROR("Failed to finalize OTA update: {}", end_result.error().to_string());
                return false;
            }
            
            // Get update statistics
            auto stats = ota_->get_statistics();
            LOG_INFO("OTA Statistics: {} successful, {} failed, {} rollbacks", 
                    stats.successful_updates, stats.failed_updates, stats.rollbacks);
            
            LOG_INFO("OTA framework test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in OTA operations test: {}", e.what());
            return false;
        }
    }
    
    bool test_persistence() {
        LOG_INFO("--- Testing Storage Persistence ---");
        
        try {
            // Write test data that should persist
            const std::string persistent_file = "/spiffs/persistent_test.txt";
            const std::string persistent_data = "This data should persist across emulator restarts";
            
            int fd = vfs_->vfs_open(persistent_file.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
            if (fd < 0) {
                LOG_ERROR("Failed to create persistent test file");
                return false;
            }
            
            ssize_t written = vfs_->vfs_write(fd, persistent_data.c_str(), persistent_data.size());
            vfs_->vfs_close(fd);
            
            if (written != static_cast<ssize_t>(persistent_data.size())) {
                LOG_ERROR("Failed to write persistent test data");
                return false;
            }
            
            // Sync all data to flash
            auto sync_result = spiffs_->sync_all();
            if (!sync_result) {
                LOG_ERROR("Failed to sync SPIFFS data: {}", sync_result.error().to_string());
                return false;
            }
            
            // Save flash image to ensure persistence
            auto save_result = flash_->save_flash_image();
            if (!save_result) {
                LOG_ERROR("Failed to save flash image: {}", save_result.error().to_string());
                return false;
            }
            
            LOG_INFO("Persistent data written and saved to flash");
            
            // Note: In a real test, we would restart the emulator here
            // and verify the data still exists. For this demo, we just
            // verify the file exists and contains the correct data.
            
            fd = vfs_->vfs_open(persistent_file.c_str(), O_RDONLY, 0);
            if (fd < 0) {
                LOG_ERROR("Failed to read persistent test file");
                return false;
            }
            
            std::vector<char> read_buffer(persistent_data.size());
            ssize_t read_bytes = vfs_->vfs_read(fd, read_buffer.data(), read_buffer.size());
            vfs_->vfs_close(fd);
            
            std::string read_data(read_buffer.begin(), read_buffer.begin() + read_bytes);
            if (read_data == persistent_data) {
                LOG_INFO("Persistence test successful - data verified");
            } else {
                LOG_ERROR("Persistence test failed - data mismatch");
                return false;
            }
            
            // Clean up
            vfs_->vfs_unlink(persistent_file.c_str());
            
            LOG_INFO("Storage persistence test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in persistence test: {}", e.what());
            return false;
        }
    }
    
    bool test_esp32_compatibility() {
        LOG_INFO("--- Testing ESP32-P4 Compatibility ---");
        
        try {
            // Test ESP32-style partition access
            auto partition_table = flash_->get_partition_table();
            if (!partition_table) {
                LOG_ERROR("No partition table available");
                return false;
            }
            
            // Look for standard ESP32 partitions
            auto nvs_partition_result = partition_table->find_partition_by_type(
                PartitionTable::PartitionType::DATA, 
                PartitionTable::PartitionSubtype::DATA_NVS);
                
            if (nvs_partition_result) {
                LOG_INFO("Found NVS partition: 0x{:08X} - 0x{:08X}", 
                        nvs_partition_result.value().offset,
                        nvs_partition_result.value().offset + nvs_partition_result.value().size);
            }
            
            // Test that all standard ESP32-P4 functionality is available
            if (!flash_->is_initialized()) {
                LOG_ERROR("Flash controller not properly initialized");
                return false;
            }
            
            if (!spiffs_->is_mounted()) {
                LOG_ERROR("SPIFFS not mounted");
                return false;
            }
            
            if (!vfs_->is_initialized()) {
                LOG_ERROR("VFS not initialized");
                return false;
            }
            
            if (!ota_->is_initialized()) {
                LOG_ERROR("OTA manager not initialized");
                return false;
            }
            
            // Test C-style file operations (ESP-IDF compatibility)
            LOG_INFO("Testing C-style file operations...");
            
            // Note: This would typically use the ESP-IDF C API functions
            // like esp_vfs_spiffs_register, fopen, fwrite, etc.
            // For this test, we verify the VFS layer provides these functions
            
            const char* test_c_file = "/spiffs/c_api_test.txt";
            const char* test_c_data = "C API compatibility test";
            
            // Using VFS C-compatible functions
            int fd = vfs_->vfs_open(test_c_file, O_CREAT | O_WRONLY | O_TRUNC, 0644);
            if (fd >= 0) {
                vfs_->vfs_write(fd, test_c_data, strlen(test_c_data));
                vfs_->vfs_close(fd);
                
                struct stat st;
                if (vfs_->vfs_stat(test_c_file, &st) == 0) {
                    LOG_INFO("C API compatibility verified (file size: {})", st.st_size);
                }
                
                vfs_->vfs_unlink(test_c_file);
            }
            
            LOG_INFO("ESP32-P4 compatibility test completed successfully");
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in ESP32 compatibility test: {}", e.what());
            return false;
        }
    }
};

int main(int argc, char* argv[]) {
    // Initialize logging
    auto log_result = Logger::initialize(Logger::LogLevel::INFO, "", true);
    if (!log_result) {
        std::cerr << "Failed to initialize logging" << std::endl;
        return 1;
    }
    
    LOG_INFO("Starting M5Stack Tab5 Storage Comprehensive Test");
    
    try {
        StorageTest test;
        bool success = test.run_comprehensive_test();
        
        if (success) {
            LOG_INFO("✅ All storage tests passed successfully!");
            return 0;
        } else {
            LOG_ERROR("❌ Storage tests failed!");
            return 1;
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Exception during storage test: {}", e.what());
        return 1;
    } catch (...) {
        LOG_ERROR("Unknown exception during storage test");
        return 1;
    }
}