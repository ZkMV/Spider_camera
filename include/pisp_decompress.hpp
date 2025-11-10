/*
 * pisp_decompress.hpp
 *
 * Header for PISP_COMP1 (8-bit compressed RAW) decompressor.
 * Ported from libcamera source code for SpiderCamera.
 */

#ifndef PISP_DECOMPRESS_HPP
#define PISP_DECOMPRESS_HPP

#include <cstdint>
#include <cstddef>

/**
 * @brief Decompresses a PISP_COMP1 (8-bit) buffer into a 16-bit Bayer buffer.
 * * @param output_buffer Pointer to the destination (uint16_t) buffer.
 * Must be pre-allocated to (width * height * 2) bytes.
 * @param input_buffer Pointer to the source (uint8_t) PISP_COMP1 buffer.
 * @param width The width of the image in pixels.
 * @param height The height of the image in pixels.
 */
void decompress_pisp(uint16_t *output_buffer, 
                     const uint8_t *input_buffer, 
                     size_t width, 
                     size_t height);

#endif // PISP_DECOMPRESS_HPP
