#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#ifndef PI
#define PI std::numbers::pi_v<float>
#endif // !PI

namespace PINYMOTOR {

static inline float getMinorArc(float _cur, float _ref, float _range)
{
    // float temp =
    //         std::fmod(1.5f * _range, _range) -
    //         (0.5f *
    //          _range); // 实际值：1.5 * range 略小于精确值 → fmod 结果略小于 0.5 * range

    float rslt = std::fmod(((_cur) - (_ref) + ((_range) * 1.5f)), (_range)) -
                 ((_range) * 0.5f);
    return rslt;
}

static inline float getMinorArc(float _cur, float _ref)
{
    return getMinorArc(_cur, _ref, 2 * PI);
}

static inline float clampArc(float _ang, float _min, float _max, float _range)
{
    float rslt = _ang;
    const float arc = getMinorArc(_min, _max, _range);
    if (arc < 0) {
        const float ang1 = getMinorArc(_ang, _min, _range);
        if (ang1 <= 0)
            rslt = _min;
        else {
            const float ang2 = getMinorArc(_ang, _max, _range);
            if (ang2 >= 0)
                rslt = _max;
        }
    } else {
        const float ang2 = getMinorArc(_ang, _max, _range);
        if (ang2 <= 0)
            rslt = _max;
        else {
            const float ang1 = getMinorArc(_ang, _min, _range);
            if (ang1 >= 0)
                rslt = _min;
        }
    }
    return rslt;
}

static inline float clampArc(float _ang, float _min, float _max)
{
    return clampArc(_ang, _min, _max, 2 * PI);
}

static inline float rangeMap(float _scale, float _min, float _max)
{
    const float period = _max - _min;
    if (period <= 0) {
        return _min;
    }
    float offset = std::fmod(_scale - _min, period);
    if (offset < 0) {
        offset += period;
    }
    float rslt = offset + _min;
    return rslt;
}

static inline float rangeMap(float _scale)
{
    return rangeMap(_scale, 0, 2 * PI);
}

static inline float uint2float(int _xInt, float _xMin, float _xMax, int _bits)
{
    /// converts unsigned int to float, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    return (((float)_xInt) * span / ((float)((1 << _bits) - 1))) + offset;
}

static inline uint16_t float2uint(float _xInt, float _xMin, float _xMax,
                                  int _bits)
{
    /// Converts a float to an unsigned int, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    uint16_t rawSet =
            (uint16_t)((_xInt - offset) * ((float)((1 << _bits) - 1)) / span);
    return rawSet;
}
static inline float rad2deg(float _rad) { return _rad * 180.f / PI; }

static inline float deg2rad(float _deg) { return _deg * PI / 180.f; }

static inline float rpm2radps(float _rpm) { return _rpm * PI / 30.f; }

static inline float radps2rpm(float _radps)
{
    return _radps * 60.f / (2.f * PI);
}

template <typename T>
inline T clamp(const T &_value, const T &_min, const T &_max)
{
    return std::min(std::max(_value, _min), _max);
}

} // namespace PINYMOTOR
