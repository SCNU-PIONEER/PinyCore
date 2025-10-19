#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#ifndef PI
#define PI std::numbers::pi_v<float>
#endif // !PI

namespace PINYMOTOR {

static float torq2volt(float _val)
{
    // foo function
    return _val;
}

/// @brief get the minor arc between two angles
/// @param _startAngle the start of the range
/// @param _endAngle the end of the range
/// @param _range the range of the angle
/// @return the minor arc from _startAngle to _endAngle in [-range/2, range/2]
static inline float getMinorArc(float _endAngle, float _startAngle,
                                float _range = 2.f * PI)
{
    float rslt = std::fmod(((_endAngle) - (_startAngle) + ((_range) * 1.5f)),
                           (_range)) -
                 ((_range) * 0.5f);
    return rslt;
}

/// @brief clamp an angle to a range
/// @param _angle the angle to clamp
/// @param _startAngle the start of the range
/// @param _endAngle the end of the range
/// @param _range the range of the angle
/// @return the clamped angle in [_startAngle, _endAngle]
/// @note if the angle is outside the range, it will be clamped to the nearest edge of the range
/// @note [-3/PI, 3/PI] is a minor arc range and [3/PI, -3/PI] is a major arc range
static float clampArc(float _angle, float _startAngle, float _endAngle,
                      float _range = 2.f * PI)
{
    auto normalize = [_range](float _a) {
        _a = std::fmodf(_a, _range);
        return (_a < 0 ? _a + _range : _a);
    };

    float a = normalize(_angle);
    float start = normalize(_startAngle);
    float end = normalize(_endAngle);

    if (start <= end) {
        // minor arc
        if (a >= start && a <= end)
            return _angle;
        float distToStart = std::fmin(a - start, start + _range - a);
        float distToEnd = std::fmin(end - a, a + _range - end);
        return distToStart < distToEnd ? _startAngle : _endAngle;
    } else {
        // major arc
        if (a >= start || a <= end)
            return _angle;
        float distToStart = std::fmin(start - a, a + _range - start);
        float distToEnd = std::fmin(a - end, end + _range - a);
        return distToStart < distToEnd ? _startAngle : _endAngle;
    }
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
