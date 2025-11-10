/*
 * pisp_decompress_lut.hpp
 *
 * Lookup table for PISP_COMP1 decompression.
 * Ported from libcamera source code (pisp_decompress_lut.h).
 */

#ifndef PISP_DECOMPRESS_LUT_HPP
#define PISP_DECOMPRESS_LUT_HPP

#include <cstdint>

/*
 * This is the combined lookup table for PISP decompression.
 * It maps 8-bit compressed values back to 10-bit sensor values,
 * stored within a 16-bit container.
 */
extern const uint16_t lut_combined_decompress[256];

#endif // PISP_DECOMPRESS_LUT_HPP
