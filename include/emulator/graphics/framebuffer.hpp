#pragma once

#include "emulator/core/types.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

namespace m5tab5::emulator {

/**
 * @brief Framebuffer management for the emulated display
 * 
 * Handles:
 * - Multiple pixel formats (RGB565, RGB888, ARGB8888)
 * - Double buffering for smooth updates
 * - Color space conversions
 * - Dirty region tracking for optimization
 * - DMA integration
 */
class Framebuffer {
public:
    enum class PixelFormat {
        RGB565,     // 16-bit color
        RGB888,     // 24-bit color
        ARGB8888,   // 32-bit color with alpha
        MONO        // 1-bit monochrome
    };

    struct Color {
        uint8_t r, g, b, a;
        
        Color() : r(0), g(0), b(0), a(255) {}
        Color(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha = 255)
            : r(red), g(green), b(blue), a(alpha) {}
        
        // Convert to different formats
        uint16_t toRGB565() const;
        uint32_t toRGB888() const;
        uint32_t toARGB8888() const;
        
        // Create from different formats
        static Color fromRGB565(uint16_t value);
        static Color fromRGB888(uint32_t value);
        static Color fromARGB8888(uint32_t value);
    };

    struct Rectangle {
        uint32_t x, y, width, height;
        
        Rectangle() : x(0), y(0), width(0), height(0) {}
        Rectangle(uint32_t x_, uint32_t y_, uint32_t w, uint32_t h)
            : x(x_), y(y_), width(w), height(h) {}
        
        bool contains(uint32_t px, uint32_t py) const;
        Rectangle intersect(const Rectangle& other) const;
        bool isEmpty() const { return width == 0 || height == 0; }
    };

    explicit Framebuffer(uint32_t width, uint32_t height, PixelFormat format);
    ~Framebuffer();

    // Basic properties
    uint32_t getWidth() const { return width_; }
    uint32_t getHeight() const { return height_; }
    PixelFormat getPixelFormat() const { return pixel_format_; }
    size_t getBytesPerPixel() const;
    size_t getStride() const { return stride_; }
    size_t getTotalSize() const { return buffer_size_; }

    // Buffer access
    void* getBuffer() { return front_buffer_.data(); }
    const void* getBuffer() const { return front_buffer_.data(); }
    void* getBackBuffer() { return back_buffer_.data(); }

    // Pixel operations
    EmulatorError setPixel(uint32_t x, uint32_t y, const Color& color);
    Color getPixel(uint32_t x, uint32_t y) const;
    
    EmulatorError fillRect(const Rectangle& rect, const Color& color);
    EmulatorError clear(const Color& color = Color(0, 0, 0));

    // Buffer management
    EmulatorError swapBuffers();
    EmulatorError copyFromMemory(Address src_address, const Rectangle& region);
    EmulatorError copyToMemory(Address dst_address, const Rectangle& region) const;

    // Format conversion
    EmulatorError convertFormat(PixelFormat new_format);
    EmulatorError convertFromBuffer(const void* src_buffer, PixelFormat src_format,
                                   const Rectangle& region);

    // Dirty region tracking
    void markDirty(const Rectangle& region);
    void markClean();
    Rectangle getDirtyRegion() const { return dirty_region_; }
    bool isDirty() const { return !dirty_region_.isEmpty(); }

    // DMA operations
    EmulatorError startDMATransfer(Address src_address, const Rectangle& region);
    bool isDMAActive() const { return dma_active_; }
    EmulatorError waitDMAComplete();

    // Debug and utilities
    EmulatorError saveToFile(const std::string& filename) const;
    EmulatorError loadFromFile(const std::string& filename);
    std::vector<uint8_t> encodeAsPNG() const;
    
    // Statistics
    struct FramebufferStatistics {
        uint64_t pixel_writes = 0;
        uint64_t pixel_reads = 0;
        uint64_t buffer_swaps = 0;
        uint64_t dma_transfers = 0;
        uint64_t format_conversions = 0;
    };
    
    const FramebufferStatistics& getStatistics() const { return stats_; }
    void resetStatistics() { stats_ = {}; }

private:
    // Internal pixel operations
    void writePixelRaw(uint32_t x, uint32_t y, const void* pixel_data);
    void readPixelRaw(uint32_t x, uint32_t y, void* pixel_data) const;
    
    size_t getPixelOffset(uint32_t x, uint32_t y) const;
    bool isValidCoordinate(uint32_t x, uint32_t y) const;

    // Format-specific operations
    void setPixelRGB565(uint32_t x, uint32_t y, const Color& color);
    void setPixelRGB888(uint32_t x, uint32_t y, const Color& color);
    void setPixelARGB8888(uint32_t x, uint32_t y, const Color& color);
    void setPixelMono(uint32_t x, uint32_t y, const Color& color);

    Color getPixelRGB565(uint32_t x, uint32_t y) const;
    Color getPixelRGB888(uint32_t x, uint32_t y) const;
    Color getPixelARGB8888(uint32_t x, uint32_t y) const;
    Color getPixelMono(uint32_t x, uint32_t y) const;

    // DMA simulation
    void dmaWorker();

    // Configuration
    uint32_t width_;
    uint32_t height_;
    PixelFormat pixel_format_;
    size_t stride_;
    size_t buffer_size_;

    // Buffer storage (double buffered)
    std::vector<uint8_t> front_buffer_;
    std::vector<uint8_t> back_buffer_;
    mutable std::mutex buffer_mutex_;

    // Dirty region tracking
    Rectangle dirty_region_;
    mutable std::mutex dirty_mutex_;

    // DMA state
    std::atomic<bool> dma_active_{false};
    Address dma_source_ = 0;
    Rectangle dma_region_;
    std::thread dma_thread_;

    // Statistics
    mutable FramebufferStatistics stats_;
    mutable std::mutex stats_mutex_;
};

} // namespace m5tab5::emulator