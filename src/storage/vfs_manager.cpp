#include "emulator/storage/vfs_manager.hpp"
#include "emulator/utils/logging.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>

namespace m5tab5::emulator::storage {

// Static members
std::unique_ptr<VFSManager> VFSManager::global_instance_;
std::mutex VFSManager::global_instance_mutex_;

VFSManager::VFSManager() {
    LOG_DEBUG("VFSManager created");
}

VFSManager::~VFSManager() {
    if (initialized_) {
        shutdown();
    }
}

Result<void> VFSManager::initialize() {
    if (initialized_) {
        return Result<void>{};
    }
    
    LOG_INFO("Initializing VFS Manager");
    
    // Clear any existing state
    filesystem_registry_.clear();
    global_file_descriptors_.clear();
    directory_handles_.clear();
    
    initialized_ = true;
    
    LOG_INFO("VFS Manager initialized successfully");
    return Result<void>{};
}

Result<void> VFSManager::shutdown() {
    if (!initialized_) {
        return Result<void>{};
    }
    
    LOG_INFO("Shutting down VFS Manager");
    
    // Close all open files
    {
        std::lock_guard lock(fd_mutex_);
        for (auto& [fd, descriptor] : global_file_descriptors_) {
            if (descriptor && descriptor->is_open && descriptor->vfs && descriptor->vfs->close) {
                LOG_WARN("Force closing file descriptor {} during shutdown", fd);
                descriptor->vfs->close(descriptor->local_fd);
            }
        }
        global_file_descriptors_.clear();
    }
    
    // Close all directory handles
    {
        std::lock_guard lock(dir_mutex_);
        directory_handles_.clear();
    }
    
    // Unregister all filesystems
    {
        std::unique_lock lock(registry_mutex_);
        filesystem_registry_.clear();
    }
    
    initialized_ = false;
    
    LOG_INFO("VFS Manager shut down successfully");
    return Result<void>{};
}

Result<void> VFSManager::register_filesystem(const std::string& mount_point, 
                                           const std::string& filesystem_type,
                                           std::shared_ptr<VFSInterface> vfs_interface) {
    if (!initialized_) {
        return error<void>(ErrorCode::VFS_NOT_INITIALIZED);
    }
    
    if (mount_point.empty() || !vfs_interface) {
        return error<void>(ErrorCode::VFS_INVALID_PARAMETER);
    }
    
    std::unique_lock lock(registry_mutex_);
    
    // Check if mount point already exists
    if (filesystem_registry_.find(mount_point) != filesystem_registry_.end()) {
        LOG_ERROR("Mount point already registered: {}", mount_point);
        return error<void>(ErrorCode::VFS_MOUNT_POINT_EXISTS);
    }
    
    // Setup VFS interface
    vfs_interface->mount_point = mount_point;
    vfs_interface->filesystem_type = filesystem_type;
    
    filesystem_registry_[mount_point] = vfs_interface;
    
    LOG_INFO("Registered filesystem '{}' at mount point '{}'", filesystem_type, mount_point);
    return Result<void>{};
}

Result<void> VFSManager::unregister_filesystem(const std::string& mount_point) {
    if (!initialized_) {
        return error<void>(ErrorCode::VFS_NOT_INITIALIZED);
    }
    
    std::unique_lock lock(registry_mutex_);
    
    auto it = filesystem_registry_.find(mount_point);
    if (it == filesystem_registry_.end()) {
        return error<void>(ErrorCode::VFS_MOUNT_POINT_NOT_FOUND);
    }
    
    // Close any open files on this filesystem
    {
        std::lock_guard fd_lock(fd_mutex_);
        auto fd_it = global_file_descriptors_.begin();
        while (fd_it != global_file_descriptors_.end()) {
            if (fd_it->second && fd_it->second->mount_point == mount_point) {
                if (fd_it->second->is_open && fd_it->second->vfs && fd_it->second->vfs->close) {
                    fd_it->second->vfs->close(fd_it->second->local_fd);
                }
                fd_it = global_file_descriptors_.erase(fd_it);
            } else {
                ++fd_it;
            }
        }
    }
    
    filesystem_registry_.erase(it);
    
    LOG_INFO("Unregistered filesystem at mount point: {}", mount_point);
    return Result<void>{};
}

Result<void> VFSManager::mount_spiffs(const std::string& mount_point, 
                                    std::shared_ptr<SPIFFSFileSystem> spiffs) {
    if (!spiffs) {
        return error<void>(ErrorCode::VFS_INVALID_PARAMETER);
    }
    
    LOG_INFO("Mounting SPIFFS at {}", mount_point);
    
    // Create VFS interface for SPIFFS
    auto vfs_interface = std::make_shared<VFSInterface>();
    vfs_interface->context = spiffs.get();
    
    // Set up function pointers
    vfs_interface->open = [](const char* path, int flags, int mode) {
        return VFSManager::spiffs_open(nullptr, path, flags, mode);
    };
    vfs_interface->close = [](int fd) {
        return VFSManager::spiffs_close(nullptr, fd);
    };
    vfs_interface->read = [](int fd, void* dst, size_t size) {
        return VFSManager::spiffs_read(nullptr, fd, dst, size);
    };
    vfs_interface->write = [](int fd, const void* data, size_t size) {
        return VFSManager::spiffs_write(nullptr, fd, data, size);
    };
    vfs_interface->lseek = [](int fd, off_t offset, int whence) {
        return VFSManager::spiffs_lseek(nullptr, fd, offset, whence);
    };
    vfs_interface->stat = [](const char* path, struct stat* st) {
        return VFSManager::spiffs_stat(nullptr, path, st);
    };
    vfs_interface->unlink = [](const char* path) {
        return VFSManager::spiffs_unlink(nullptr, path);
    };
    vfs_interface->rename = [](const char* src, const char* dst) {
        return VFSManager::spiffs_rename(nullptr, src, dst);
    };
    vfs_interface->opendir = [](const char* name) {
        return VFSManager::spiffs_opendir(nullptr, name);
    };
    vfs_interface->closedir = [](DIR* pdir) {
        return VFSManager::spiffs_closedir(nullptr, pdir);
    };
    vfs_interface->readdir = [](DIR* pdir) {
        return VFSManager::spiffs_readdir(nullptr, pdir);
    };
    vfs_interface->mkdir = [](const char* name, mode_t mode) {
        return VFSManager::spiffs_mkdir(nullptr, name, mode);
    };
    vfs_interface->rmdir = [](const char* name) {
        return VFSManager::spiffs_rmdir(nullptr, name);
    };
    vfs_interface->truncate = [](const char* path, off_t length) {
        return VFSManager::spiffs_truncate(nullptr, path, length);
    };
    vfs_interface->ftruncate = [](int fd, off_t length) {
        return VFSManager::spiffs_ftruncate(nullptr, fd, length);
    };
    
    return register_filesystem(mount_point, "spiffs", vfs_interface);
}

Result<void> VFSManager::unmount_spiffs(const std::string& mount_point) {
    LOG_INFO("Unmounting SPIFFS from {}", mount_point);
    return unregister_filesystem(mount_point);
}

int VFSManager::vfs_open(const char* path, int flags, int mode) {
    if (!initialized_ || !path) {
        errno = EINVAL;
        return -1;
    }
    
    LOG_DEBUG("VFS open: {} flags: 0x{:x} mode: 0x{:x}", path, flags, mode);
    
    auto split_result = split_path(path);
    if (!split_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto [mount_point, local_path] = split_result.value();
    
    auto vfs_result = find_filesystem(mount_point);
    if (!vfs_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto vfs = vfs_result.value();
    if (!vfs->open) {
        errno = ENOSYS;
        return -1;
    }
    
    // Call filesystem-specific open
    int local_fd = vfs->open(local_path.c_str(), flags, mode);
    if (local_fd < 0) {
        return -1;  // errno should be set by filesystem
    }
    
    // Allocate global file descriptor
    auto global_fd_result = allocate_global_fd(local_fd, mount_point, path, vfs);
    if (!global_fd_result.has_value()) {
        // Close the local fd since we couldn't allocate global one
        if (vfs->close) {
            vfs->close(local_fd);
        }
        errno = EMFILE;
        return -1;
    }
    
    LOG_DEBUG("VFS open successful: {} -> global_fd: {}, local_fd: {}", 
             path, global_fd_result.value(), local_fd);
    
    return global_fd_result.value();
}

int VFSManager::vfs_close(int fd) {
    if (!initialized_) {
        errno = EBADF;
        return -1;
    }
    
    LOG_DEBUG("VFS close: fd {}", fd);
    
    auto descriptor_result = get_file_descriptor(fd);
    if (!descriptor_result.has_value()) {
        errno = EBADF;
        return -1;
    }
    
    auto descriptor = descriptor_result.value();
    int result = 0;
    
    if (descriptor->is_open && descriptor->vfs && descriptor->vfs->close) {
        result = descriptor->vfs->close(descriptor->local_fd);
    }
    
    // Free the global file descriptor
    auto free_result = free_global_fd(fd);
    if (!free_result.has_value()) {
        LOG_WARN("Failed to free global file descriptor: {}", fd);
    }
    
    return result;
}

ssize_t VFSManager::vfs_read(int fd, void* dst, size_t size) {
    if (!initialized_ || !dst) {
        errno = EINVAL;
        return -1;
    }
    
    auto descriptor_result = get_file_descriptor(fd);
    if (!descriptor_result.has_value()) {
        errno = EBADF;
        return -1;
    }
    
    auto descriptor = descriptor_result.value();
    if (!descriptor->is_open || !descriptor->vfs || !descriptor->vfs->read) {
        errno = EBADF;
        return -1;
    }
    
    return descriptor->vfs->read(descriptor->local_fd, dst, size);
}

ssize_t VFSManager::vfs_write(int fd, const void* data, size_t size) {
    if (!initialized_ || !data) {
        errno = EINVAL;
        return -1;
    }
    
    auto descriptor_result = get_file_descriptor(fd);
    if (!descriptor_result.has_value()) {
        errno = EBADF;
        return -1;
    }
    
    auto descriptor = descriptor_result.value();
    if (!descriptor->is_open || !descriptor->vfs || !descriptor->vfs->write) {
        errno = EBADF;
        return -1;
    }
    
    return descriptor->vfs->write(descriptor->local_fd, data, size);
}

off_t VFSManager::vfs_lseek(int fd, off_t offset, int whence) {
    if (!initialized_) {
        errno = EBADF;
        return -1;
    }
    
    auto descriptor_result = get_file_descriptor(fd);
    if (!descriptor_result.has_value()) {
        errno = EBADF;
        return -1;
    }
    
    auto descriptor = descriptor_result.value();
    if (!descriptor->is_open || !descriptor->vfs || !descriptor->vfs->lseek) {
        errno = EBADF;
        return -1;
    }
    
    return descriptor->vfs->lseek(descriptor->local_fd, offset, whence);
}

int VFSManager::vfs_stat(const char* path, struct stat* st) {
    if (!initialized_ || !path || !st) {
        errno = EINVAL;
        return -1;
    }
    
    auto split_result = split_path(path);
    if (!split_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto [mount_point, local_path] = split_result.value();
    
    auto vfs_result = find_filesystem(mount_point);
    if (!vfs_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto vfs = vfs_result.value();
    if (!vfs->stat) {
        errno = ENOSYS;
        return -1;
    }
    
    return vfs->stat(local_path.c_str(), st);
}

int VFSManager::vfs_unlink(const char* path) {
    if (!initialized_ || !path) {
        errno = EINVAL;
        return -1;
    }
    
    auto split_result = split_path(path);
    if (!split_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto [mount_point, local_path] = split_result.value();
    
    auto vfs_result = find_filesystem(mount_point);
    if (!vfs_result.has_value()) {
        errno = ENOENT;
        return -1;
    }
    
    auto vfs = vfs_result.value();
    if (!vfs->unlink) {
        errno = ENOSYS;
        return -1;
    }
    
    return vfs->unlink(local_path.c_str());
}

DIR* VFSManager::vfs_opendir(const char* name) {
    if (!initialized_ || !name) {
        errno = EINVAL;
        return nullptr;
    }
    
    LOG_DEBUG("VFS opendir: {}", name);
    
    auto split_result = split_path(name);
    if (!split_result.has_value()) {
        errno = ENOENT;
        return nullptr;
    }
    
    auto [mount_point, local_path] = split_result.value();
    
    auto vfs_result = find_filesystem(mount_point);
    if (!vfs_result.has_value()) {
        errno = ENOENT;
        return nullptr;
    }
    
    auto vfs = vfs_result.value();
    if (!vfs->opendir) {
        errno = ENOSYS;
        return nullptr;
    }
    
    return vfs->opendir(local_path.c_str());
}

int VFSManager::vfs_closedir(DIR* pdir) {
    if (!initialized_ || !pdir) {
        errno = EINVAL;
        return -1;
    }
    
    auto handle_result = get_directory_handle(pdir);
    if (!handle_result.has_value()) {
        errno = EBADF;
        return -1;
    }
    
    return destroy_directory_handle(pdir).has_value() ? 0 : -1;
}

struct dirent* VFSManager::vfs_readdir(DIR* pdir) {
    if (!initialized_ || !pdir) {
        errno = EINVAL;
        return nullptr;
    }
    
    auto handle_result = get_directory_handle(pdir);
    if (!handle_result.has_value()) {
        errno = EBADF;
        return nullptr;
    }
    
    auto handle = handle_result.value();
    
    if (handle->position >= handle->entries.size()) {
        return nullptr;  // End of directory
    }
    
    auto& entry = handle->entries[handle->position++];
    
    // Fill in dirent structure
    handle->current_dirent.d_ino = entry.object_id;
    handle->current_dirent.d_type = (entry.type == SPIFFSFileSystem::ObjectType::FILE) ? DT_REG : DT_DIR;
    strncpy(handle->current_dirent.d_name, entry.name.c_str(), sizeof(handle->current_dirent.d_name) - 1);
    handle->current_dirent.d_name[sizeof(handle->current_dirent.d_name) - 1] = '\0';
    
    return &handle->current_dirent;
}

std::string VFSManager::normalize_path(const std::string& path) {
    if (path.empty()) {
        return "/";
    }
    
    std::string normalized = path;
    
    // Ensure path starts with '/'
    if (normalized[0] != '/') {
        normalized = "/" + normalized;
    }
    
    // Remove multiple consecutive slashes
    std::string result;
    bool last_was_slash = false;
    for (char c : normalized) {
        if (c == '/') {
            if (!last_was_slash) {
                result += c;
                last_was_slash = true;
            }
        } else {
            result += c;
            last_was_slash = false;
        }
    }
    
    // Remove trailing slash unless it's the root
    if (result.length() > 1 && result.back() == '/') {
        result.pop_back();
    }
    
    return result;
}

Result<std::pair<std::string, std::string>> VFSManager::split_path(const std::string& path) {
    std::string normalized = normalize_path(path);
    
    // Find longest matching mount point
    std::string best_mount_point;
    
    {
        std::shared_lock lock(registry_mutex_);
        for (const auto& [mount_point, vfs] : filesystem_registry_) {
            if (normalized.starts_with(mount_point) && mount_point.length() > best_mount_point.length()) {
                best_mount_point = mount_point;
            }
        }
    }
    
    if (best_mount_point.empty()) {
        return error<std::pair<std::string, std::string>>(ErrorCode::VFS_MOUNT_POINT_NOT_FOUND);
    }
    
    std::string local_path = normalized.substr(best_mount_point.length());
    if (local_path.empty() || local_path[0] != '/') {
        local_path = "/" + local_path;
    }
    
    return std::make_pair(best_mount_point, local_path);
}

Result<std::shared_ptr<VFSManager::VFSInterface>> VFSManager::find_filesystem(const std::string& path) {
    std::shared_lock lock(registry_mutex_);
    
    auto it = filesystem_registry_.find(path);
    if (it == filesystem_registry_.end()) {
        return error<std::shared_ptr<VFSInterface>>(ErrorCode::VFS_MOUNT_POINT_NOT_FOUND);
    }
    
    return it->second;
}

Result<int> VFSManager::allocate_global_fd(int local_fd, const std::string& mount_point, 
                                         const std::string& path, std::shared_ptr<VFSInterface> vfs) {
    std::lock_guard lock(fd_mutex_);
    
    int global_fd = next_global_fd_++;
    
    auto descriptor = std::make_unique<FileDescriptor>();
    descriptor->local_fd = local_fd;
    descriptor->mount_point = mount_point;
    descriptor->full_path = path;
    descriptor->vfs = vfs;
    descriptor->is_open = true;
    
    global_file_descriptors_[global_fd] = std::move(descriptor);
    
    return global_fd;
}

Result<void> VFSManager::free_global_fd(int global_fd) {
    std::lock_guard lock(fd_mutex_);
    
    auto it = global_file_descriptors_.find(global_fd);
    if (it == global_file_descriptors_.end()) {
        return error<void>(ErrorCode::VFS_INVALID_FD);
    }
    
    global_file_descriptors_.erase(it);
    return Result<void>{};
}

Result<VFSManager::FileDescriptor*> VFSManager::get_file_descriptor(int global_fd) {
    std::lock_guard lock(fd_mutex_);
    
    auto it = global_file_descriptors_.find(global_fd);
    if (it == global_file_descriptors_.end()) {
        return error<FileDescriptor*>(ErrorCode::VFS_INVALID_FD);
    }
    
    return it->second.get();
}

Result<VFSManager::VFSDirectoryHandle*> VFSManager::get_directory_handle(DIR* pdir) {
    std::lock_guard lock(dir_mutex_);
    
    auto it = directory_handles_.find(pdir);
    if (it == directory_handles_.end()) {
        return error<VFSDirectoryHandle*>(ErrorCode::VFS_INVALID_DIR);
    }
    
    return it->second.get();
}

Result<DIR*> VFSManager::create_directory_handle(const std::vector<SPIFFSFileSystem::DirectoryEntry>& entries,
                                                const std::string& path) {
    std::lock_guard lock(dir_mutex_);
    
    auto handle = std::make_unique<VFSDirectoryHandle>();
    handle->entries = entries;
    handle->position = 0;
    handle->path = path;
    handle->valid = true;
    
    DIR* pdir = reinterpret_cast<DIR*>(handle.get());
    directory_handles_[pdir] = std::move(handle);
    
    return pdir;
}

Result<void> VFSManager::destroy_directory_handle(DIR* pdir) {
    std::lock_guard lock(dir_mutex_);
    
    auto it = directory_handles_.find(pdir);
    if (it == directory_handles_.end()) {
        return error<void>(ErrorCode::VFS_INVALID_DIR);
    }
    
    directory_handles_.erase(it);
    return Result<void>{};
}

// Global VFS functions
VFSManager& VFSManager::get_instance() {
    std::lock_guard lock(global_instance_mutex_);
    if (!global_instance_) {
        global_instance_ = std::make_unique<VFSManager>();
        global_instance_->initialize();
    }
    return *global_instance_;
}

int VFSManager::global_open(const char* path, int flags, int mode) {
    return get_instance().vfs_open(path, flags, mode);
}

int VFSManager::global_close(int fd) {
    return get_instance().vfs_close(fd);
}

ssize_t VFSManager::global_read(int fd, void* dst, size_t size) {
    return get_instance().vfs_read(fd, dst, size);
}

ssize_t VFSManager::global_write(int fd, const void* data, size_t size) {
    return get_instance().vfs_write(fd, data, size);
}

off_t VFSManager::global_lseek(int fd, off_t offset, int whence) {
    return get_instance().vfs_lseek(fd, offset, whence);
}

int VFSManager::global_stat(const char* path, struct stat* st) {
    return get_instance().vfs_stat(path, st);
}

int VFSManager::global_unlink(const char* path) {
    return get_instance().vfs_unlink(path);
}

DIR* VFSManager::global_opendir(const char* name) {
    return get_instance().vfs_opendir(name);
}

int VFSManager::global_closedir(DIR* pdir) {
    return get_instance().vfs_closedir(pdir);
}

struct dirent* VFSManager::global_readdir(DIR* pdir) {
    return get_instance().vfs_readdir(pdir);
}

// SPIFFS-specific VFS operation implementations
int VFSManager::spiffs_open(void* context, const char* path, int flags, int mode) {
    // This would be implemented to bridge to SPIFFS filesystem
    // For now, return error to indicate not implemented
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_close(void* context, int fd) {
    errno = ENOSYS;
    return -1;
}

ssize_t VFSManager::spiffs_read(void* context, int fd, void* dst, size_t size) {
    errno = ENOSYS;
    return -1;
}

ssize_t VFSManager::spiffs_write(void* context, int fd, const void* data, size_t size) {
    errno = ENOSYS;
    return -1;
}

off_t VFSManager::spiffs_lseek(void* context, int fd, off_t offset, int whence) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_stat(void* context, const char* path, struct stat* st) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_unlink(void* context, const char* path) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_rename(void* context, const char* src, const char* dst) {
    errno = ENOSYS;
    return -1;
}

DIR* VFSManager::spiffs_opendir(void* context, const char* name) {
    errno = ENOSYS;
    return nullptr;
}

int VFSManager::spiffs_closedir(void* context, DIR* pdir) {
    errno = ENOSYS;
    return -1;
}

struct dirent* VFSManager::spiffs_readdir(void* context, DIR* pdir) {
    errno = ENOSYS;
    return nullptr;
}

int VFSManager::spiffs_mkdir(void* context, const char* name, mode_t mode) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_rmdir(void* context, const char* name) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_truncate(void* context, const char* path, off_t length) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_ftruncate(void* context, int fd, off_t length) {
    errno = ENOSYS;
    return -1;
}

int VFSManager::spiffs_utime(void* context, const char* path, const struct utimbuf* times) {
    errno = ENOSYS;
    return -1;
}

// Additional stub methods
Result<bool> VFSManager::is_filesystem_registered(const std::string& mount_point) {
    std::shared_lock lock(registry_mutex_);
    return filesystem_registry_.find(mount_point) != filesystem_registry_.end();
}

int VFSManager::vfs_fstat(int fd, struct stat* st) { errno = ENOSYS; return -1; }
int VFSManager::vfs_fsync(int fd) { errno = ENOSYS; return -1; }
int VFSManager::vfs_rename(const char* src, const char* dst) { errno = ENOSYS; return -1; }
int VFSManager::vfs_truncate(const char* path, off_t length) { errno = ENOSYS; return -1; }
int VFSManager::vfs_ftruncate(int fd, off_t length) { errno = ENOSYS; return -1; }
int VFSManager::vfs_utime(const char* path, const struct utimbuf* times) { errno = ENOSYS; return -1; }
int VFSManager::vfs_access(const char* path, int amode) { errno = ENOSYS; return -1; }
void VFSManager::vfs_rewinddir(DIR* pdir) {}
long VFSManager::vfs_telldir(DIR* pdir) { return 0; }
void VFSManager::vfs_seekdir(DIR* pdir, long offset) {}
int VFSManager::vfs_mkdir(const char* name, mode_t mode) { errno = ENOSYS; return -1; }
int VFSManager::vfs_rmdir(const char* name) { errno = ENOSYS; return -1; }
int VFSManager::vfs_statvfs(const char* path, struct statvfs* buf) { errno = ENOSYS; return -1; }

} // namespace m5tab5::emulator::storage

// C-style file operations implementation
extern "C" {

FILE* emulator_fopen(const char* filename, const char* mode) {
    // Not implemented in this initial version
    return nullptr;
}

int emulator_fclose(FILE* stream) {
    return -1;
}

size_t emulator_fread(void* ptr, size_t size, size_t nmemb, FILE* stream) {
    return 0;
}

size_t emulator_fwrite(const void* ptr, size_t size, size_t nmemb, FILE* stream) {
    return 0;
}

int emulator_fseek(FILE* stream, long offset, int whence) {
    return -1;
}

long emulator_ftell(FILE* stream) {
    return -1;
}

int emulator_fflush(FILE* stream) {
    return -1;
}

void emulator_rewind(FILE* stream) {
}

int emulator_feof(FILE* stream) {
    return 1;
}

int emulator_ferror(FILE* stream) {
    return 1;
}

int emulator_remove(const char* filename) {
    return -1;
}

int emulator_rename(const char* oldname, const char* newname) {
    return -1;
}

} // extern "C"