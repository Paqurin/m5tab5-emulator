#include "emulator/graphics/framebuffer.hpp"
#include "emulator/utils/logging.hpp"
#include <cstring>
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("Framebuffer");

Framebuffer::Framebuffer()
    : initialized_(false),
      width_(0),
      height_(0),
      pixel_format_(PixelFormat::RGB888),
      bytes_per_pixel_(0),
      front_buffer_(nullptr),
      back_buffer_(nullptr),
      current_buffer_(0) {
    COMPONENT_LOG_DEBUG("Framebuffer created");
}

Framebuffer::~Framebuffer() {
    if (initialized_) {
        shutdown();
    }
    COMPONENT_LOG_DEBUG("Framebuffer destroyed");
}

Result<void> Framebuffer::initialize(u32 width, u32 height, PixelFormat format, bool double_buffered) {
    if (initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_ALREADY_RUNNING,
            "Framebuffer already initialized"));
    }
    
    COMPONENT_LOG_INFO("Initializing framebuffer: {}x{} format={} double_buffered={}",
                      width, height, static_cast<int>(format), double_buffered);
    
    width_ = width;
    height_ = height;
    pixel_format_ = format;
    double_buffered_ = double_buffered;
    
    // Calculate bytes per pixel based on format
    bytes_per_pixel_ = get_bytes_per_pixel(format);
    if (bytes_per_pixel_ == 0) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Unsupported pixel format"));
    }
    
    // Calculate buffer size
    buffer_size_ = width_ * height_ * bytes_per_pixel_;
    
    // Allocate front buffer
    front_buffer_ = std::make_unique<u8[]>(buffer_size_);
    std::memset(front_buffer_.get(), 0, buffer_size_);
    
    // Allocate back buffer if double buffered
    if (double_buffered_) {
        back_buffer_ = std::make_unique<u8[]>(buffer_size_);
        std::memset(back_buffer_.get(), 0, buffer_size_);
        current_buffer_ = 0; // Start with back buffer for drawing
    }
    
    // Initialize dirty tracking
    dirty_regions_.clear();
    mark_entire_buffer_dirty();
    
    // Reset statistics
    statistics_ = {};
    
    initialized_ = true;
    COMPONENT_LOG_INFO("Framebuffer initialized successfully");
    
    return {};
}

Result<void> Framebuffer::shutdown() {
    if (!initialized_) {
        return {};
    }
    
    COMPONENT_LOG_INFO("Shutting down framebuffer");
    
    // Free buffers
    front_buffer_.reset();
    back_buffer_.reset();
    
    dirty_regions_.clear();
    
    initialized_ = false;
    COMPONENT_LOG_INFO("Framebuffer shutdown completed");
    
    return {};
}

Result<void> Framebuffer::clear(u32 color) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    u8* buffer = get_current_draw_buffer();
    fill_buffer_with_color(buffer, color);
    
    mark_entire_buffer_dirty();
    statistics_.clear_operations++;
    
    return {};
}

Result<void> Framebuffer::set_pixel(u32 x, u32 y, u32 color) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    if (x >= width_ || y >= height_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Pixel coordinates out of bounds"));
    }
    
    u8* buffer = get_current_draw_buffer();
    RETURN_IF_ERROR(write_pixel_to_buffer(buffer, x, y, color));
    
    mark_pixel_dirty(x, y);
    statistics_.pixels_written++;
    
    return {};
}

Result<u32> Framebuffer::get_pixel(u32 x, u32 y) const {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    if (x >= width_ || y >= height_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Pixel coordinates out of bounds"));
    }
    
    const u8* buffer = get_current_read_buffer();
    return read_pixel_from_buffer(buffer, x, y);
}

Result<void> Framebuffer::blit(const u8* source_data, u32 source_width, u32 source_height,
                               u32 dest_x, u32 dest_y, PixelFormat source_format) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    if (!source_data) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Source data is null"));
    }
    
    u8* dest_buffer = get_current_draw_buffer();
    
    // Perform format conversion if necessary
    if (source_format == pixel_format_) {
        // Same format - direct copy
        RETURN_IF_ERROR(blit_same_format(source_data, source_width, source_height,
                                        dest_buffer, dest_x, dest_y));
    } else {
        // Different format - convert pixels
        RETURN_IF_ERROR(blit_with_conversion(source_data, source_width, source_height,
                                           source_format, dest_buffer, dest_x, dest_y));
    }
    
    // Mark affected region as dirty
    mark_region_dirty(dest_x, dest_y, source_width, source_height);
    statistics_.blit_operations++;
    
    return {};
}

Result<void> Framebuffer::copy_region(u32 src_x, u32 src_y, u32 width, u32 height,
                                      u32 dest_x, u32 dest_y) {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    // Validate coordinates
    if (src_x + width > width_ || src_y + height > height_ ||
        dest_x + width > width_ || dest_y + height > height_) {
        return std::unexpected(MAKE_ERROR(INVALID_PARAMETER,
            "Region extends beyond framebuffer bounds"));
    }
    
    u8* buffer = get_current_draw_buffer();
    
    // Handle overlapping regions
    if (src_y < dest_y || (src_y == dest_y && src_x < dest_x)) {
        // Copy from bottom-right to top-left to avoid corruption
        for (i32 y = static_cast<i32>(height) - 1; y >= 0; --y) {
            for (i32 x = static_cast<i32>(width) - 1; x >= 0; --x) {
                u32 src_pixel_x = src_x + static_cast<u32>(x);
                u32 src_pixel_y = src_y + static_cast<u32>(y);
                u32 dest_pixel_x = dest_x + static_cast<u32>(x);
                u32 dest_pixel_y = dest_y + static_cast<u32>(y);
                
                u32 color = read_pixel_from_buffer(buffer, src_pixel_x, src_pixel_y);
                RETURN_IF_ERROR(write_pixel_to_buffer(buffer, dest_pixel_x, dest_pixel_y, color));
            }
        }
    } else {
        // Copy from top-left to bottom-right
        for (u32 y = 0; y < height; ++y) {
            for (u32 x = 0; x < width; ++x) {
                u32 src_pixel_x = src_x + x;
                u32 src_pixel_y = src_y + y;
                u32 dest_pixel_x = dest_x + x;
                u32 dest_pixel_y = dest_y + y;
                
                u32 color = read_pixel_from_buffer(buffer, src_pixel_x, src_pixel_y);
                RETURN_IF_ERROR(write_pixel_to_buffer(buffer, dest_pixel_x, dest_pixel_y, color));
            }
        }
    }
    
    mark_region_dirty(dest_x, dest_y, width, height);
    statistics_.copy_operations++;
    
    return {};
}

Result<void> Framebuffer::swap_buffers() {
    if (!initialized_) {
        return std::unexpected(MAKE_ERROR(SYSTEM_NOT_INITIALIZED,
            "Framebuffer not initialized"));
    }
    
    if (!double_buffered_) {
        return {}; // No-op for single buffered
    }
    
    // Swap front and back buffers
    std::swap(front_buffer_, back_buffer_);
    current_buffer_ = 1 - current_buffer_;
    
    // Clear dirty regions after swap
    dirty_regions_.clear();
    
    statistics_.buffer_swaps++;
    
    COMPONENT_LOG_TRACE("Framebuffer buffers swapped");
    return {};
}

std::vector<DirtyRegion> Framebuffer::get_dirty_regions() const {
    return dirty_regions_;
}

Result<void> Framebuffer::clear_dirty_regions() {
    dirty_regions_.clear();
    return {};
}

u32 Framebuffer::get_width() const {
    return width_;
}

u32 Framebuffer::get_height() const {
    return height_;
}

PixelFormat Framebuffer::get_pixel_format() const {
    return pixel_format_;
}

bool Framebuffer::is_double_buffered() const {
    return double_buffered_;
}

const u8* Framebuffer::get_front_buffer() const {
    return front_buffer_.get();
}

const u8* Framebuffer::get_back_buffer() const {
    return back_buffer_.get();
}

size_t Framebuffer::get_buffer_size() const {
    return buffer_size_;
}

const FramebufferStatistics& Framebuffer::get_statistics() const {
    return statistics_;
}

void Framebuffer::clear_statistics() {
    statistics_ = {};
}

u8 Framebuffer::get_bytes_per_pixel(PixelFormat format) const {
    switch (format) {
        case PixelFormat::RGB565:
            return 2;
        case PixelFormat::RGB888:
            return 3;
        case PixelFormat::RGBA8888:
            return 4;
        case PixelFormat::GRAYSCALE:
            return 1;
        default:
            return 0;
    }
}

u8* Framebuffer::get_current_draw_buffer() const {
    if (!double_buffered_) {
        return front_buffer_.get();
    }
    
    return (current_buffer_ == 0) ? back_buffer_.get() : front_buffer_.get();
}

const u8* Framebuffer::get_current_read_buffer() const {
    return front_buffer_.get();
}

void Framebuffer::fill_buffer_with_color(u8* buffer, u32 color) {
    size_t pixel_count = width_ * height_;
    
    switch (pixel_format_) {
        case PixelFormat::RGB565: {
            u16 rgb565 = convert_rgb888_to_rgb565(color);
            u16* pixels = reinterpret_cast<u16*>(buffer);
            for (size_t i = 0; i < pixel_count; ++i) {
                pixels[i] = rgb565;
            }
            break;
        }
        
        case PixelFormat::RGB888: {
            u8 r = (color >> 16) & 0xFF;
            u8 g = (color >> 8) & 0xFF;
            u8 b = color & 0xFF;
            
            for (size_t i = 0; i < pixel_count; ++i) {
                buffer[i * 3 + 0] = r;
                buffer[i * 3 + 1] = g;
                buffer[i * 3 + 2] = b;
            }
            break;
        }
        
        case PixelFormat::RGBA8888: {
            u32* pixels = reinterpret_cast<u32*>(buffer);
            u32 rgba = color | 0xFF000000; // Add alpha
            for (size_t i = 0; i < pixel_count; ++i) {
                pixels[i] = rgba;
            }
            break;
        }
        
        case PixelFormat::GRAYSCALE: {
            u8 gray = convert_rgb888_to_grayscale(color);
            std::memset(buffer, gray, pixel_count);
            break;
        }
    }
}

Result<void> Framebuffer::write_pixel_to_buffer(u8* buffer, u32 x, u32 y, u32 color) {
    size_t offset = (y * width_ + x) * bytes_per_pixel_;
    
    switch (pixel_format_) {
        case PixelFormat::RGB565: {
            u16 rgb565 = convert_rgb888_to_rgb565(color);
            *reinterpret_cast<u16*>(buffer + offset) = rgb565;
            break;
        }
        
        case PixelFormat::RGB888: {
            buffer[offset + 0] = (color >> 16) & 0xFF; // R
            buffer[offset + 1] = (color >> 8) & 0xFF;  // G
            buffer[offset + 2] = color & 0xFF;         // B
            break;
        }
        
        case PixelFormat::RGBA8888: {
            *reinterpret_cast<u32*>(buffer + offset) = color | 0xFF000000; // Add alpha
            break;
        }
        
        case PixelFormat::GRAYSCALE: {
            buffer[offset] = convert_rgb888_to_grayscale(color);
            break;
        }
        
        default:
            return std::unexpected(MAKE_ERROR(NOT_IMPLEMENTED,
                "Unsupported pixel format for writing"));
    }
    
    return {};
}

u32 Framebuffer::read_pixel_from_buffer(const u8* buffer, u32 x, u32 y) const {
    size_t offset = (y * width_ + x) * bytes_per_pixel_;
    
    switch (pixel_format_) {
        case PixelFormat::RGB565: {
            u16 rgb565 = *reinterpret_cast<const u16*>(buffer + offset);
            return convert_rgb565_to_rgb888(rgb565);
        }
        
        case PixelFormat::RGB888: {
            return (static_cast<u32>(buffer[offset + 0]) << 16) |  // R
                   (static_cast<u32>(buffer[offset + 1]) << 8) |   // G
                   static_cast<u32>(buffer[offset + 2]);           // B
        }
        
        case PixelFormat::RGBA8888: {
            u32 rgba = *reinterpret_cast<const u32*>(buffer + offset);
            return rgba & 0x00FFFFFF; // Remove alpha
        }
        
        case PixelFormat::GRAYSCALE: {
            u8 gray = buffer[offset];
            return (static_cast<u32>(gray) << 16) |
                   (static_cast<u32>(gray) << 8) |
                   static_cast<u32>(gray);
        }
        
        default:
            return 0;
    }
}

void Framebuffer::mark_pixel_dirty(u32 x, u32 y) {
    mark_region_dirty(x, y, 1, 1);
}

void Framebuffer::mark_region_dirty(u32 x, u32 y, u32 width, u32 height) {
    DirtyRegion region{x, y, width, height};
    
    // Try to merge with existing dirty regions
    bool merged = false;
    for (auto& existing : dirty_regions_) {
        if (can_merge_regions(existing, region)) {
            existing = merge_regions(existing, region);
            merged = true;
            break;
        }
    }
    
    if (!merged) {
        dirty_regions_.push_back(region);
    }
    
    // Limit number of dirty regions to prevent excessive overhead
    if (dirty_regions_.size() > MAX_DIRTY_REGIONS) {
        mark_entire_buffer_dirty();
    }
}

void Framebuffer::mark_entire_buffer_dirty() {
    dirty_regions_.clear();
    dirty_regions_.push_back({0, 0, width_, height_});
}

bool Framebuffer::can_merge_regions(const DirtyRegion& a, const DirtyRegion& b) const {
    // Check if regions overlap or are adjacent
    u32 a_right = a.x + a.width;
    u32 a_bottom = a.y + a.height;
    u32 b_right = b.x + b.width;
    u32 b_bottom = b.y + b.height;
    
    return !(a.x > b_right || b.x > a_right || a.y > b_bottom || b.y > a_bottom);
}

DirtyRegion Framebuffer::merge_regions(const DirtyRegion& a, const DirtyRegion& b) const {
    u32 min_x = std::min(a.x, b.x);
    u32 min_y = std::min(a.y, b.y);
    u32 max_x = std::max(a.x + a.width, b.x + b.width);
    u32 max_y = std::max(a.y + a.height, b.y + b.height);
    
    return {min_x, min_y, max_x - min_x, max_y - min_y};
}

Result<void> Framebuffer::blit_same_format(const u8* source, u32 source_width, u32 source_height,
                                          u8* dest, u32 dest_x, u32 dest_y) {
    for (u32 y = 0; y < source_height; ++y) {
        for (u32 x = 0; x < source_width; ++x) {
            u32 dest_pixel_x = dest_x + x;
            u32 dest_pixel_y = dest_y + y;
            
            if (dest_pixel_x < width_ && dest_pixel_y < height_) {
                size_t src_offset = (y * source_width + x) * bytes_per_pixel_;
                size_t dest_offset = (dest_pixel_y * width_ + dest_pixel_x) * bytes_per_pixel_;
                
                std::memcpy(dest + dest_offset, source + src_offset, bytes_per_pixel_);
            }
        }
    }
    
    return {};
}

Result<void> Framebuffer::blit_with_conversion(const u8* source, u32 source_width, u32 source_height,
                                              PixelFormat source_format, u8* dest, u32 dest_x, u32 dest_y) {
    u8 source_bpp = get_bytes_per_pixel(source_format);
    
    for (u32 y = 0; y < source_height; ++y) {
        for (u32 x = 0; x < source_width; ++x) {
            u32 dest_pixel_x = dest_x + x;
            u32 dest_pixel_y = dest_y + y;
            
            if (dest_pixel_x < width_ && dest_pixel_y < height_) {
                // Read pixel from source in its format
                size_t src_offset = (y * source_width + x) * source_bpp;
                u32 color = convert_pixel_format(source + src_offset, source_format, PixelFormat::RGB888);
                
                // Write pixel to destination in framebuffer format
                RETURN_IF_ERROR(write_pixel_to_buffer(dest, dest_pixel_x, dest_pixel_y, color));
            }
        }
    }
    
    return {};
}

u32 Framebuffer::convert_pixel_format(const u8* pixel_data, PixelFormat from, PixelFormat to) const {
    // For now, convert everything through RGB888
    u32 rgb888;
    
    switch (from) {
        case PixelFormat::RGB565: {
            u16 rgb565 = *reinterpret_cast<const u16*>(pixel_data);
            rgb888 = convert_rgb565_to_rgb888(rgb565);
            break;
        }
        case PixelFormat::RGB888:
            rgb888 = (static_cast<u32>(pixel_data[0]) << 16) |
                     (static_cast<u32>(pixel_data[1]) << 8) |
                     static_cast<u32>(pixel_data[2]);
            break;
        case PixelFormat::RGBA8888: {
            u32 rgba = *reinterpret_cast<const u32*>(pixel_data);
            rgb888 = rgba & 0x00FFFFFF;
            break;
        }
        case PixelFormat::GRAYSCALE: {
            u8 gray = pixel_data[0];
            rgb888 = (static_cast<u32>(gray) << 16) |
                     (static_cast<u32>(gray) << 8) |
                     static_cast<u32>(gray);
            break;
        }
        default:
            rgb888 = 0;
            break;
    }
    
    // For this implementation, we always return RGB888
    // In a full implementation, we would convert to the target format
    return rgb888;
}

u16 Framebuffer::convert_rgb888_to_rgb565(u32 rgb888) const {
    u8 r = (rgb888 >> 16) & 0xFF;
    u8 g = (rgb888 >> 8) & 0xFF;
    u8 b = rgb888 & 0xFF;
    
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

u32 Framebuffer::convert_rgb565_to_rgb888(u16 rgb565) const {
    u8 r = ((rgb565 >> 11) & 0x1F) << 3;
    u8 g = ((rgb565 >> 5) & 0x3F) << 2;
    u8 b = (rgb565 & 0x1F) << 3;
    
    return (static_cast<u32>(r) << 16) |
           (static_cast<u32>(g) << 8) |
           static_cast<u32>(b);
}

u8 Framebuffer::convert_rgb888_to_grayscale(u32 rgb888) const {
    u8 r = (rgb888 >> 16) & 0xFF;
    u8 g = (rgb888 >> 8) & 0xFF;
    u8 b = rgb888 & 0xFF;
    
    // Use standard luminance formula
    return static_cast<u8>(0.299f * r + 0.587f * g + 0.114f * b);
}

}  // namespace m5tab5::emulator