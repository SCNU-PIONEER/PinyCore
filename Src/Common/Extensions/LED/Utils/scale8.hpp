#pragma once

#include <sys/cdefs.h>
#include <cstdint>

namespace LED {

/// Scale three one-byte values by a fourth one, which is treated as
/// the numerator of a fraction whose demominator is 256.
///
/// In other words, it computes r,g,b * (scale / 256), ensuring
/// that non-zero values passed in remain non-zero, no matter how low the scale
/// argument.
///
/// @warning This function always modifies its arguments in place!
/// @param r first value to scale
/// @param g second value to scale
/// @param b third value to scale
/// @param scale scale factor, in n/256 units
static __always_inline void nscale8x3Video(uint8_t &_r, uint8_t &_g, uint8_t &_b, uint8_t _scale)
{
    uint8_t nonzeroscale = (_scale != 0) ? 1 : 0;
    _r = (_r == 0) ? 0 : (((int)_r * (int)_scale) >> 8) + nonzeroscale;
    _g = (_g == 0) ? 0 : (((int)_g * (int)_scale) >> 8) + nonzeroscale;
    _b = (_b == 0) ? 0 : (((int)_b * (int)_scale) >> 8) + nonzeroscale;
}

/// Scale three one-byte values by a fourth one, which is treated as
/// the numerator of a fraction whose demominator is 256.
///
/// In other words, it computes r,g,b * (scale / 256)
///
/// @warning This function always modifies its arguments in place!
/// @param r first value to scale
/// @param g second value to scale
/// @param b third value to scale
/// @param scale scale factor, in n/256 units
static __always_inline void nscale8x3(uint8_t &_r, uint8_t &_g, uint8_t &_b, uint8_t _scale)
{
    _r = ((int)_r * (int)(_scale)) >> 8;
    _g = ((int)_g * (int)(_scale)) >> 8;
    _b = ((int)_b * (int)(_scale)) >> 8;
}

/// Scale one byte by a second one, which is treated as
/// the numerator of a fraction whose denominator is 256.
///
/// In other words, it computes i * (scale / 256)
/// @param i input value to scale
/// @param scale scale factor, in n/256 units
/// @returns scaled value
/// @note Takes 4 clocks on AVR with MUL, 2 clocks on ARM
static __always_inline uint8_t scale8(uint8_t _i, uint8_t _scale) { return ((uint16_t)_i * (uint16_t)_scale) >> 8; }

/// Scale a 16-bit unsigned value by an 16-bit value, which is treated
/// as the numerator of a fraction whose denominator is 65536.
/// In other words, it computes i * (scale / 65536)
/// @param i input value to scale
/// @param scale scale factor, in n/65536 units
/// @returns scaled value
static __always_inline uint16_t scale16(uint16_t _i, uint16_t _scale)
{
    return ((uint32_t)(_i) * (uint32_t)(_scale)) / 65536;
}

/// Linear interpolation between two unsigned 8-bit values,
/// with 8-bit fraction
static __always_inline uint8_t lerp8by8(uint8_t _a, uint8_t _b, uint8_t _frac)
{
    uint8_t result;
    if (_b > _a) {
        uint8_t delta = _b - _a;
        uint8_t scaled = scale8(delta, _frac);
        result = _a + scaled;
    } else {
        uint8_t delta = _a - _b;
        uint8_t scaled = scale8(delta, _frac);
        result = _a - scaled;
    }
    return result;
}

/// Linear interpolation between two unsigned 16-bit values,
/// with 16-bit fraction
static __always_inline uint16_t lerp16by16(uint16_t _a, uint16_t _b, uint16_t _frac)
{
    uint16_t result;
    if (_b > _a) {
        uint16_t delta = _b - _a;
        uint16_t scaled = scale16(delta, _frac);
        result = _a + scaled;
    } else {
        uint16_t delta = _a - _b;
        uint16_t scaled = scale16(delta, _frac);
        result = _a - scaled;
    }
    return result;
}

} // namespace LED
