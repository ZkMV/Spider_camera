/*
 * frame_buffer.cpp
 *
 * Frame buffer utilities for SpiderCamera (v0.2.4)
 * Handles RAW10 unpacking and buffer conversions
 */

#include "frame_buffer.hpp" // <--- v0.2.4: Додано заголовок
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace SpiderCameraUtils {

/**
 * @brief Unpacks RAW10_CSI2P format to uint16 array
 * * RAW10_CSI2P packing: 5 bytes → 4 pixels (10-bit each)
 * Byte layout: [P0_9:2][P1_9:2][P2_9:2][P3_9:2][P3_1:0 P2_1:0 P1_1:0 P0_1:0]
 * * @param packed Pointer to packed RAW10 data
 * @param unpacked Pointer to output uint16 buffer (must be pre-allocated)
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void unpack_raw10_csi2p(const uint8_t* packed, 
                        uint16_t* unpacked, 
                        size_t width, 
                        size_t height) {
    if (!packed || !unpacked) {
        throw std::invalid_argument("Null pointer passed to unpack_raw10_csi2p");
    }

    size_t pixel_count = width * height;
    // size_t packed_size = (pixel_count * 10) / 8;  // 10 bits per pixel
    // Примітка: змінна packed_size не використовується, тому я її закоментував,
    // щоб уникнути попередження компілятора, яке ми бачили раніше.

    // Process 4 pixels at a time (5 bytes)
    for (size_t i = 0; i < pixel_count; i += 4) {
        size_t byte_idx = (i * 5) / 4;

        // Extract 4 pixels from 5 bytes
        unpacked[i + 0] = (static_cast<uint16_t>(packed[byte_idx + 0]) << 2) | 
                         ((packed[byte_idx + 4] >> 0) & 0x3);
        
        unpacked[i + 1] = (static_cast<uint16_t>(packed[byte_idx + 1]) << 2) | 
                         ((packed[byte_idx + 4] >> 2) & 0x3);
        
        unpacked[i + 2] = (static_cast<uint16_t>(packed[byte_idx + 2]) << 2) | 
                         ((packed[byte_idx + 4] >> 4) & 0x3);
        
        unpacked[i + 3] = (static_cast<uint16_t>(packed[byte_idx + 3]) << 2) | 
                         ((packed[byte_idx + 4] >> 6) & 0x3);
    }
}

/**
 * @brief Calculate expected packed buffer size for RAW10 format
 * * @param width Frame width
 * @param height Frame height
 * @return Size in bytes
 */
size_t calculate_raw10_packed_size(size_t width, size_t height) {
    size_t pixel_count = width * height;
    // 10 bits per pixel, rounded up to nearest byte
    return (pixel_count * 10 + 7) / 8;
}

/**
 * @brief Validate buffer size matches expected RAW10 format
 * * @param buffer_size Actual buffer size
 * @param width Expected width
 * @param height Expected height
 * @return true if size matches
 */
bool validate_raw10_buffer_size(size_t buffer_size, size_t width, size_t height) {
    size_t expected = calculate_raw10_packed_size(width, height);
    return buffer_size >= expected;
}

} // namespace SpiderCameraUtils
