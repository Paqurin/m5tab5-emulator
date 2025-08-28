#pragma once

#include "emulator/utils/types.hpp"
#include "emulator/utils/error.hpp"
#include "emulator/utils/logging.hpp"
#include "emulator/storage/spiffs_filesystem.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <dirent.h>
#include <sys/stat.h>

namespace m5tab5::emulator::storage {

/**
 * @brief Virtual File System (VFS) Manager for ESP-IDF Compatibility
 * 
 * Provides POSIX-compatible file system interface that bridges ESP-IDF VFS
 * calls to the underlying SPIFFS implementation. Supports:
 * - Standard POSIX file operations (fopen, fread, fwrite, fclose, etc.)
 * - Directory operations (opendir, readdir, closedir)
 * - File system mounting and unmounting
 * - Multiple file system instances on different partitions
 * - ESP-IDF compatible error codes and behavior
 */
class VFSManager {
public:
    // VFS operation function signatures matching ESP-IDF VFS interface
    using open_func_t = std::function<int(const char* path, int flags, int mode)>;
    using close_func_t = std::function<int(int fd)>;
    using read_func_t = std::function<ssize_t(int fd, void* dst, size_t size)>;
    using write_func_t = std::function<ssize_t(int fd, const void* data, size_t size)>;
    using lseek_func_t = std::function<off_t(int fd, off_t size, int mode)>;
    using stat_func_t = std::function<int(const char* path, struct stat* st)>;
    using unlink_func_t = std::function<int(const char* path)>;
    using rename_func_t = std::function<int(const char* src, const char* dst)>;
    using opendir_func_t = std::function<DIR*(const char* name)>;
    using closedir_func_t = std::function<int(DIR* pdir)>;
    using readdir_func_t = std::function<struct dirent*(DIR* pdir)>;
    using mkdir_func_t = std::function<int(const char* name, mode_t mode)>;
    using rmdir_func_t = std::function<int(const char* name)>;
    using truncate_func_t = std::function<int(const char* path, off_t length)>;
    using ftruncate_func_t = std::function<int(int fd, off_t length)>;
    using utime_func_t = std::function<int(const char* path, const struct utimbuf* times)>;
    
    // VFS filesystem interface
    struct VFSInterface {
        open_func_t open;
        close_func_t close;
        read_func_t read;
        write_func_t write;
        lseek_func_t lseek;
        stat_func_t stat;
        unlink_func_t unlink;
        rename_func_t rename;
        opendir_func_t opendir;
        closedir_func_t closedir;
        readdir_func_t readdir;
        mkdir_func_t mkdir;
        rmdir_func_t rmdir;
        truncate_func_t truncate;
        ftruncate_func_t ftruncate;
        utime_func_t utime;
        
        // Filesystem-specific data
        void* context = nullptr;
        std::string mount_point;
        std::string filesystem_type;
    };
    
    // Directory handle for VFS directory operations
    struct VFSDirectoryHandle {
        std::vector<SPIFFSFileSystem::DirectoryEntry> entries;
        size_t position = 0;
        std::string path;
        struct dirent current_dirent;
        bool valid = false;
    };
    
    // File descriptor mapping
    struct FileDescriptor {
        int local_fd;                    // File system specific FD
        std::string mount_point;         // Which filesystem
        std::string full_path;           // Original full path
        std::shared_ptr<VFSInterface> vfs; // VFS interface
        bool is_open = false;
    };
    
    VFSManager();
    ~VFSManager();
    
    // VFS lifecycle management
    Result<void> initialize();
    Result<void> shutdown();
    bool is_initialized() const { return initialized_; }
    
    // Mount point management
    Result<void> register_filesystem(const std::string& mount_point, 
                                   const std::string& filesystem_type,
                                   std::shared_ptr<VFSInterface> vfs_interface);
    Result<void> unregister_filesystem(const std::string& mount_point);
    Result<bool> is_filesystem_registered(const std::string& mount_point);
    
    // SPIFFS integration
    Result<void> mount_spiffs(const std::string& mount_point, 
                            std::shared_ptr<SPIFFSFileSystem> spiffs);
    Result<void> unmount_spiffs(const std::string& mount_point);
    
    // POSIX-compatible file operations
    int vfs_open(const char* path, int flags, int mode);
    int vfs_close(int fd);
    ssize_t vfs_read(int fd, void* dst, size_t size);
    ssize_t vfs_write(int fd, const void* data, size_t size);
    off_t vfs_lseek(int fd, off_t offset, int whence);
    int vfs_fstat(int fd, struct stat* st);
    int vfs_fsync(int fd);
    
    // POSIX-compatible file management
    int vfs_stat(const char* path, struct stat* st);
    int vfs_unlink(const char* path);
    int vfs_rename(const char* src, const char* dst);
    int vfs_truncate(const char* path, off_t length);
    int vfs_ftruncate(int fd, off_t length);
    int vfs_utime(const char* path, const struct utimbuf* times);
    int vfs_access(const char* path, int amode);
    
    // POSIX-compatible directory operations
    DIR* vfs_opendir(const char* name);
    int vfs_closedir(DIR* pdir);
    struct dirent* vfs_readdir(DIR* pdir);
    void vfs_rewinddir(DIR* pdir);
    long vfs_telldir(DIR* pdir);
    void vfs_seekdir(DIR* pdir, long offset);
    int vfs_mkdir(const char* name, mode_t mode);
    int vfs_rmdir(const char* name);
    
    // Filesystem information
    int vfs_statvfs(const char* path, struct statvfs* buf);
    
    // File descriptor management
    Result<std::vector<int>> get_open_files();
    Result<std::string> get_file_path(int fd);
    Result<std::string> get_mount_point(int fd);
    
    // Utility functions
    std::string normalize_path(const std::string& path);
    Result<std::pair<std::string, std::string>> split_path(const std::string& path);
    Result<std::shared_ptr<VFSInterface>> find_filesystem(const std::string& path);
    
    // Debug and diagnostics
    Result<void> dump_filesystem_registry();
    Result<void> dump_open_files();
    Result<std::vector<std::string>> get_mount_points();
    
    // Global VFS registration (for ESP-IDF compatibility)
    static int global_open(const char* path, int flags, int mode);
    static int global_close(int fd);
    static ssize_t global_read(int fd, void* dst, size_t size);
    static ssize_t global_write(int fd, const void* data, size_t size);
    static off_t global_lseek(int fd, off_t offset, int whence);
    static int global_stat(const char* path, struct stat* st);
    static int global_unlink(const char* path);
    static int global_rename(const char* src, const char* dst);
    static DIR* global_opendir(const char* name);
    static int global_closedir(DIR* pdir);
    static struct dirent* global_readdir(DIR* pdir);
    static int global_mkdir(const char* name, mode_t mode);
    static int global_rmdir(const char* name);
    
    // Global VFS manager instance
    static VFSManager& get_instance();
    
private:
    // Internal file descriptor management
    Result<int> allocate_global_fd(int local_fd, const std::string& mount_point, 
                                 const std::string& path, std::shared_ptr<VFSInterface> vfs);
    Result<void> free_global_fd(int global_fd);
    Result<FileDescriptor*> get_file_descriptor(int global_fd);
    
    // Path resolution
    Result<std::string> resolve_path(const std::string& path);
    Result<std::string> get_local_path(const std::string& full_path, const std::string& mount_point);
    bool is_absolute_path(const std::string& path);
    
    // Directory handle management
    Result<VFSDirectoryHandle*> get_directory_handle(DIR* pdir);
    Result<DIR*> create_directory_handle(const std::vector<SPIFFSFileSystem::DirectoryEntry>& entries,
                                       const std::string& path);
    Result<void> destroy_directory_handle(DIR* pdir);
    
    // Error handling
    int translate_error_code(const ErrorCode& error);
    void set_errno_from_error(const ErrorCode& error);
    
    // SPIFFS-specific VFS operations
    static int spiffs_open(void* context, const char* path, int flags, int mode);
    static int spiffs_close(void* context, int fd);
    static ssize_t spiffs_read(void* context, int fd, void* dst, size_t size);
    static ssize_t spiffs_write(void* context, int fd, const void* data, size_t size);
    static off_t spiffs_lseek(void* context, int fd, off_t offset, int whence);
    static int spiffs_stat(void* context, const char* path, struct stat* st);
    static int spiffs_unlink(void* context, const char* path);
    static int spiffs_rename(void* context, const char* src, const char* dst);
    static DIR* spiffs_opendir(void* context, const char* name);
    static int spiffs_closedir(void* context, DIR* pdir);
    static struct dirent* spiffs_readdir(void* context, DIR* pdir);
    static int spiffs_mkdir(void* context, const char* name, mode_t mode);
    static int spiffs_rmdir(void* context, const char* name);
    static int spiffs_truncate(void* context, const char* path, off_t length);
    static int spiffs_ftruncate(void* context, int fd, off_t length);
    static int spiffs_utime(void* context, const char* path, const struct utimbuf* times);
    
    // Filesystem registry
    std::unordered_map<std::string, std::shared_ptr<VFSInterface>> filesystem_registry_;
    mutable std::shared_mutex registry_mutex_;
    
    // File descriptor management
    std::unordered_map<int, std::unique_ptr<FileDescriptor>> global_file_descriptors_;
    std::atomic<int> next_global_fd_{1000};  // Start high to avoid conflicts
    mutable std::mutex fd_mutex_;
    
    // Directory handle management
    std::unordered_map<DIR*, std::unique_ptr<VFSDirectoryHandle>> directory_handles_;
    mutable std::mutex dir_mutex_;
    
    // State
    std::atomic<bool> initialized_{false};
    
    // Global instance for static functions
    static std::unique_ptr<VFSManager> global_instance_;
    static std::mutex global_instance_mutex_;
};

/**
 * @brief C-style file operations for ESP-IDF compatibility
 * 
 * These functions provide direct C interface compatibility with ESP-IDF
 * standard library replacements.
 */
extern "C" {
    // Standard C file operations
    FILE* emulator_fopen(const char* filename, const char* mode);
    int emulator_fclose(FILE* stream);
    size_t emulator_fread(void* ptr, size_t size, size_t nmemb, FILE* stream);
    size_t emulator_fwrite(const void* ptr, size_t size, size_t nmemb, FILE* stream);
    int emulator_fseek(FILE* stream, long offset, int whence);
    long emulator_ftell(FILE* stream);
    int emulator_fflush(FILE* stream);
    void emulator_rewind(FILE* stream);
    int emulator_feof(FILE* stream);
    int emulator_ferror(FILE* stream);
    
    // File system operations
    int emulator_remove(const char* filename);
    int emulator_rename(const char* oldname, const char* newname);
}

} // namespace m5tab5::emulator::storage