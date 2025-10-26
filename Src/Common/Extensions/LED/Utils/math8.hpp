#pragma once

#include <algorithm>
#include <cstdint>

namespace LED {

/// Add one byte to another, saturating at 0xFF
/// @param i first byte to add
/// @param j second byte to add
/// @returns the sum of i + j, capped at 0xFF
static inline uint8_t qadd8(uint8_t _i, uint8_t _j)
{
    uint32_t t = _i + _j;
    t = std::min<uint32_t>(t, 255);
    return t;
}

/// Subtract one byte from another, saturating at 0x00
/// @param i byte to subtract from
/// @param j byte to subtract
/// @returns i - j with a floor of 0
static inline uint8_t qsub8(uint8_t _i, uint8_t _j)
{
    int t = _i - _j;
    t = std::max(t, 0);
    return t;
}

/// 8x8 bit multiplication with 8-bit result, saturating at 0xFF.
/// @param i first byte to multiply
/// @param j second byte to multiply
/// @returns the product of i * j, capping at 0xFF
static inline uint8_t qmul8(uint8_t _i, uint8_t _j)
{
    uint32_t p = (uint32_t)_i * (uint32_t)_j;
    p = std::min<uint32_t>(p, 255);
    return p;
}

} // namespace LED
