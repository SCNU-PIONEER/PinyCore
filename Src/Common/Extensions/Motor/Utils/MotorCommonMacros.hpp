#pragma once

#include <cmath>
#include <cstdint>

#ifndef PI
#define PI 3.14159265358979323846f
#endif // !PI

namespace PINYMOTOR {

static inline float getMinorArc(float _cur, float _ref, float _range)
{
    return (fmodf(((_cur) - (_ref) + (_range) * 1.5f), (_range)) -
            (_range) / 2.f);
}

static inline float rangeMap(float _scale, float _min, float _max)
{
    return ((_min) > (_max)) ?
                   (_scale) :
                   (_min) + fmodf(((_scale) - (_min)), ((_max) - (_min)));
}

static inline float uint2float(int _xInt, float _xMin, float _xMax, int _bits)
{
    /// converts unsigned int to float, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    return ((float)_xInt) * span / ((float)((1 << _bits) - 1)) + offset;
}

static inline uint16_t float2uint(float _x, float _xMin, float _xMax, int _bits)
{
    /// Converts a float to an unsigned int, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    uint16_t raw_set =
            (uint16_t)((_x - offset) * ((float)((1 << _bits) - 1)) / span);
    return raw_set;
}
static inline float rad2deg(float _rad) { return _rad * 180.f / PI; }

static inline float deg2rad(float _deg) { return _deg * PI / 180.f; }

static inline float rpm2radps(float _rpm) { return _rpm * PI / 30.f; }

static inline float radps2rpm(float _radps)
{
    return _radps * 60.f / (2.f * PI);
}

template <typename T> inline T clamp(const T &value, const T &min, const T &max)
{
    return (min > max)   ? value :
           (value < min) ? min :
           (value > max) ? max :
                           value;
}

};
