#pragma once

#include <cstdint>
#include <bit>

namespace COMM {

static constexpr uint8_t FP16_SIGN_BITS = 1;
static constexpr uint8_t FP16_EXP_BITS = 5;
static constexpr uint8_t FP16_MANT_BITS = 10;
static constexpr uint8_t FP16_MAX_EXP = (0x1F - 1);

static int16_t convertToInt16(float _f)
{
    const uint32_t fBits = std::bit_cast<uint32_t>(_f);
    const uint32_t sign = (fBits >> 31) & 0x1;
    int32_t exp = static_cast<int32_t>((fBits >> 23) & 0xFF) - 127;
    if (exp >= 16) { // 2^(15+1) = 65536 overflow
        return static_cast<int16_t>((sign << 15) | 0x7BFF); // sign + max
    }
    // construct sign bit
    uint16_t hBits = (sign << 15);
    // construct exponent and mantissa bits
    exp += 15; // bias
    if (exp <= 0) {
        // double underflow, return zero
        return 0;
    } else if (exp >= FP16_MAX_EXP) {
        // really big number, return inf
        hBits |= (FP16_MAX_EXP << FP16_MANT_BITS) | 0x3FF;
    } else {
        // normal number
        const uint32_t mantissa = (fBits & 0x007FFFFF) >> (23 - FP16_MANT_BITS);
        hBits |= (exp << FP16_MANT_BITS) | (mantissa & 0x3FF);
    }
    return static_cast<int16_t>(hBits);
}

static float convertToFloat(int16_t _u)
{
    const uint16_t hBits = static_cast<uint16_t>(_u);
    if (hBits == 0) {
        return 0.0f;
    }
    const uint32_t sign = (hBits >> 15) & 0x1;
    const uint32_t exp16 = (hBits >> FP16_MANT_BITS) &
                           ((1U << FP16_EXP_BITS) - 1);
    const uint32_t mantissa = hBits & ((1U << FP16_MANT_BITS) - 1);

    // inf or NaN
    if (exp16 == FP16_MAX_EXP && mantissa == 0x3FF) {
        // map as ±65504.0
        const uint32_t exp32 = 0x8E;            // 142 - 127 = 15
        const uint32_t mantissa32 = 0x007FE000; // 1.9990234375
        const uint32_t fBits = (sign << 31) | (exp32 << 23) | mantissa32;
        return std::bit_cast<float>(fBits);
    }
    // normal number
    const int32_t exp32 = static_cast<int32_t>(exp16) - 15 + 127;
    const uint32_t mantissa32 = mantissa << (23 - FP16_MANT_BITS);
    const uint32_t fBits = (sign << 31) | (static_cast<uint32_t>(exp32) << 23) |
                           mantissa32;
    return std::bit_cast<float>(fBits);
}

} // namespace COMM
