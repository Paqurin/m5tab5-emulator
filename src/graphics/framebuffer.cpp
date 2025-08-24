#include "emulator/graphics/framebuffer.hpp"
#include "emulator/utils/logging.hpp"
#include <cstring>
#include <algorithm>

namespace m5tab5::emulator {

DECLARE_LOGGER("Framebuffer");

// Color format conversion methods
uint16_t Framebuffer::Color::toRGB565() const {
    return static_cast<uint16_t>(
        ((r >> 3) << 11) |
        ((g >> 2) << 5) |
        (b >> 3)
    );
}

uint32_t Framebuffer::Color::toRGB888() const {
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

uint32_t Framebuffer::Color::toARGB8888() const {
    return (static_cast<uint32_t>(a) << 24) |
           (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

Framebuffer::Color Framebuffer::Color::fromRGB565(uint16_t value) {
    return Color(
        static_cast<uint8_t>((value >> 11) << 3),  // Red
        static_cast<uint8_t>(((value >> 5) & 0x3F) << 2), // Green  
        static_cast<uint8_t>((value & 0x1F) << 3), // Blue
        255 // Full alpha
    );
}

Framebuffer::Color Framebuffer::Color::fromRGB888(uint32_t value) {
    return Color(
        static_cast<uint8_t>((value >> 16) & 0xFF), // Red
        static_cast<uint8_t>((value >> 8) & 0xFF),  // Green
        static_cast<uint8_t>(value & 0xFF),         // Blue
        255 // Full alpha
    );
}

Framebuffer::Color Framebuffer::Color::fromARGB8888(uint32_t value) {
    return Color(
        static_cast<uint8_t>((value >> 16) & 0xFF), // Red
        static_cast<uint8_t>((value >> 8) & 0xFF),  // Green
        static_cast<uint8_t>(value & 0xFF),         // Blue
        static_cast<uint8_t>((value >> 24) & 0xFF)  // Alpha
    );
}

// Rectangle methods
bool Framebuffer::Rectangle::contains(uint32_t px, uint32_t py) const {
    return px >= x && px < (x + width) && py >= y && py < (y + height);
}

Framebuffer::Rectangle Framebuffer::Rectangle::intersect(const Rectangle& other) const {
    uint32_t left = std::max(x, other.x);
    uint32_t top = std::max(y, other.y);
    uint32_t right = std::min(x + width, other.x + other.width);
    uint32_t bottom = std::min(y + height, other.y + other.height);
    
    if (left >= right || top >= bottom) {
        return Rectangle(); // Empty rectangle
    }
    
    return Rectangle(left, top, right - left, bottom - top);
}

// Framebuffer implementation
Framebuffer::Framebuffer(uint32_t width, uint32_t height, PixelFormat format)
    : width_(width),
      height_(height),
      pixel_format_(format),
      stride_(width * getBytesPerPixel()),
      buffer_size_(height * stride_) {
    
    COMPONENT_LOG_DEBUG("Framebuffer created: {}x{} format={}", width, height, static_cast<int>(format));
    
    // Allocate buffers
    front_buffer_.resize(buffer_size_);
    back_buffer_.resize(buffer_size_);
    
    // Clear buffers
    std::memset(front_buffer_.data(), 0, buffer_size_);
    std::memset(back_buffer_.data(), 0, buffer_size_);
    
    initialized_.store(true);
}

Framebuffer::~Framebuffer() {
    COMPONENT_LOG_DEBUG("Framebuffer destroyed");
}

size_t Framebuffer::getBytesPerPixel() const {
    switch (pixel_format_) {
        case PixelFormat::RGB565:
            return 2;
        case PixelFormat::RGB888:
            return 3;
        case PixelFormat::ARGB8888:
            return 4;
        case PixelFormat::MONO:
            return 1;
        default:
            return 4; // Default to 32-bit
    }
}

EmulatorError Framebuffer::setPixel(uint32_t x, uint32_t y, const Color& color) {
    if (!initialized_.load()) {
        return EmulatorError::InvalidOperation;
    }
    
    if (x >= width_ || y >= height_) {
        return EmulatorError::InvalidAddress;
    }
    
    uint8_t* buffer = static_cast<uint8_t*>(getBackBuffer());
    size_t offset = y * stride_ + x * getBytesPerPixel();
    
    switch (pixel_format_) {
        case PixelFormat::RGB565: {
            uint16_t* pixel = reinterpret_cast<uint16_t*>(buffer + offset);
            *pixel = color.toRGB565();
            break;
        }
        case PixelFormat::RGB888: {
            buffer[offset] = color.r;
            buffer[offset + 1] = color.g;
            buffer[offset + 2] = color.b;
            break;
        }
        case PixelFormat::ARGB8888: {
            uint32_t* pixel = reinterpret_cast<uint32_t*>(buffer + offset);
            *pixel = color.toARGB8888();
            break;
        }
        case PixelFormat::MONO: {
            // Convert to grayscale (simple average)
            uint8_t gray = static_cast<uint8_t>((color.r + color.g + color.b) / 3);
            buffer[offset] = gray > 128 ? 255 : 0; // Threshold to monochrome
            break;
        }
    }
    
    return EmulatorError::Success;
}

Framebuffer::Color Framebuffer::getPixel(uint32_t x, uint32_t y) const {
    if (!initialized_.load() || x >= width_ || y >= height_) {
        return Color(); // Return black for invalid pixels
    }
    
    const uint8_t* buffer = static_cast<const uint8_t*>(getBuffer());
    size_t offset = y * stride_ + x * getBytesPerPixel();
    
    switch (pixel_format_) {
        case PixelFormat::RGB565: {
            const uint16_t* pixel = reinterpret_cast<const uint16_t*>(buffer + offset);
            return Color::fromRGB565(*pixel);
        }
        case PixelFormat::RGB888: {
            return Color(buffer[offset], buffer[offset + 1], buffer[offset + 2]);
        }
        case PixelFormat::ARGB8888: {
            const uint32_t* pixel = reinterpret_cast<const uint32_t*>(buffer + offset);
            return Color::fromARGB8888(*pixel);
        }
        case PixelFormat::MONO: {
            uint8_t gray = buffer[offset];
            return Color(gray, gray, gray);
        }
        default:
            return Color();
    }
}

EmulatorError Framebuffer::clear(const Color& color) {
    if (!initialized_.load()) {
        return EmulatorError::InvalidOperation;
    }
    
    // Fill the back buffer with the specified color
    for (uint32_t y = 0; y < height_; ++y) {
        for (uint32_t x = 0; x < width_; ++x) {
            setPixel(x, y, color);
        }
    }
    
    // Mark entire framebuffer as dirty
    markDirty(Rectangle(0, 0, width_, height_));
    
    return EmulatorError::Success;
}

EmulatorError Framebuffer::fillRect(const Rectangle& rect, const Color& color) {
    if (!initialized_.load()) {
        return EmulatorError::InvalidOperation;
    }
    
    // Clip rectangle to framebuffer bounds
    uint32_t x_start = std::min(rect.x, width_);
    uint32_t y_start = std::min(rect.y, height_);
    uint32_t x_end = std::min(rect.x + rect.width, width_);
    uint32_t y_end = std::min(rect.y + rect.height, height_);
    
    if (x_start >= x_end || y_start >= y_end) {
        return EmulatorError::Success; // Nothing to fill
    }
    
    // Fill the rectangular region
    for (uint32_t y = y_start; y < y_end; ++y) {
        for (uint32_t x = x_start; x < x_end; ++x) {
            if (auto error = setPixel(x, y, color); error != EmulatorError::Success) {
                return error;
            }
        }
    }
    
    // Mark the region as dirty
    markDirty(Rectangle(x_start, y_start, x_end - x_start, y_end - y_start));
    
    return EmulatorError::Success;
}

EmulatorError Framebuffer::swapBuffers() {
    if (!initialized_.load()) {
        return EmulatorError::InvalidOperation;
    }
    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    front_buffer_.swap(back_buffer_);
    
    return EmulatorError::Success;
}

void Framebuffer::markDirty(const Rectangle& region) {
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    
    if (dirty_region_.isEmpty()) {
        dirty_region_ = region;
    } else {
        // Expand dirty region to include new area
        uint32_t left = std::min(dirty_region_.x, region.x);
        uint32_t top = std::min(dirty_region_.y, region.y);
        uint32_t right = std::max(dirty_region_.x + dirty_region_.width, region.x + region.width);
        uint32_t bottom = std::max(dirty_region_.y + dirty_region_.height, region.y + region.height);
        
        dirty_region_ = Rectangle(left, top, right - left, bottom - top);
    }
}

void Framebuffer::markClean() {
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    dirty_region_ = Rectangle(); // Empty rectangle
}

EmulatorError Framebuffer::copyRegion(uint32_t src_x, uint32_t src_y, uint32_t dest_x, uint32_t dest_y,
                                     uint32_t width, uint32_t height) {
    if (!initialized_.load()) {
        return EmulatorError::InvalidOperation;
    }
    
    // Stub implementation - would copy rectangular region
    COMPONENT_LOG_DEBUG("Copy region: ({},{}) to ({},{}) size={}x{}", 
                       src_x, src_y, dest_x, dest_y, width, height);
    
    return EmulatorError::Success;
}

}  // namespace m5tab5::emulator