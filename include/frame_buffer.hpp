/*
 * frame_buffer.hpp
 *
 * Header file for frame buffer utilities (v0.2.4)
 * Declares RAW10 unpacking functions.
 */

#ifndef FRAME_BUFFER_HPP
#define FRAME_BUFFER_HPP

#include <cstdint>
#include <cstddef>

namespace SpiderCameraUtils {

/**
 * @brief Unpacks RAW10_CSI2P format to uint16 array
 * * RAW10_CSI2P packing: 5 bytes → 4 pixels (10-bit each)
 * * @param packed Pointer to packed RAW10 data
 * @param unpacked Pointer to output uint16 buffer (must be pre-allocated)
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void unpack_raw10_csi2p(const uint8_t* packed, 
                        uint16_t* unpacked, 
                        size_t width, 
                        size_t height);

} // namespace SpiderCameraUtils

#endif // FRAME_BUFFER_HPP
