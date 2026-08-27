#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#ifndef PI
#define PI std::numbers::pi_v<float>
static constexpr float TWO_PI = 2.f * PI;
#endif // !PI

namespace PINYMOTOR {

static float torq2volt(float _val)
{
    // foo function
    return _val;
}

/**
 * @brief Calculate the minor arc between two angles
 * 
 * This function computes the shortest angular distance (minor arc) between
 * two given angles within a specified range. The result is always returned
 * as a value within [-range/2, range/2].
 * 
 * @param _startAngle The starting angle of the range
 * @param _endAngle The ending angle of the range
 * @param _range The total range of the angle (typically 2*PI for full circle)
 * @return The minor arc from _startAngle to _endAngle, guaranteed to be
 *         within [-range/2, range/2]
 * 
 * @note The function always returns the shortest path between the two angles.
 *       For example, for angles 350° and 10°, it will return 20° rather than
 *       340°.
 * 
 * @warning The _range parameter should be positive and typically 2*PI for
 *          circular measurements. Using other values may produce unexpected
 *          results.
 * 
 * @example
 * float start = 350.0f * PI/180.0f;  // 350 degrees or -10 degrees in radians
 * float end = 10.0f * PI/180.0f;     // 10 degrees in radians
 * float arc = getMinorArc(start, end, 2*PI);
 * // arc will be approximately +20 degrees in radians (0.349f)
 */

static __always_inline float getMinorArc(float _endAngle, float _startAngle, float _range = 2.f * PI)
{
    float rslt = std::fmod(((_endAngle) - (_startAngle) + ((_range) * 1.5f)), (_range)) - ((_range) * 0.5f);
    return rslt;
}

/**
 * @brief Clamp an angle to specified range
 * 
 * This function restricts the input angle value within the specified angle range.
 * If the angle exceeds the range, it will be clamped to the nearest boundary
 * of the range.
 * 
 * @param _angle The angle value to be clamped
 * @param _startAngle The start value of the angle range
 * @param _endAngle The end value of the angle range
 * @param _range The range value of the angle
 * @return The clamped angle value within [_startAngle, _endAngle] range
 * 
 * @note Range definitions:
 *       - [-3/PI, 3/PI] represents a minor arc range
 *       - [3/PI, -3/PI] represents a major arc range
 * 
 * @warning Input parameters should ensure that _startAngle and _endAngle 
 *          form a valid angle range
 * 
 * @example
 * float angle = 4.0f;
 * float result = clampAngle(angle, -3.0f/PI, 3.0f/PI, 6.0f/PI);
 * // result will be clamped within [-3/PI, 3/PI] range
 */

static float clampArc(float _angle, float _startAngle, float _endAngle, float _range = 2.f * PI)
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

static __always_inline float rangeMap(float _scale, float _min, float _max)
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

static __always_inline float rangeMap(float _scale) { return rangeMap(_scale, 0, 2 * PI); }

static __always_inline float uint2float(int _xInt, float _xMin, float _xMax, int _bits)
{
    /// converts unsigned int to float, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    return (((float)_xInt) * span / ((float)((1 << _bits) - 1))) + offset;
}

static __always_inline uint16_t float2uint(float _xInt, float _xMin, float _xMax, int _bits)
{
    /// Converts a float to an unsigned int, given range and number of _bits ///
    float span = _xMax - _xMin;
    float offset = _xMin;
    uint16_t rawSet = (uint16_t)((_xInt - offset) * ((float)((1 << _bits) - 1)) / span);
    return rawSet;
}
static __always_inline float rad2deg(float _rad) { return _rad * 180.f / PI; }

static __always_inline float deg2rad(float _deg) { return _deg * PI / 180.f; }

static __always_inline float rpm2radps(float _rpm) { return _rpm * PI / 30.f; }

static __always_inline float radps2rpm(float _radps) { return _radps * 60.f / (2.f * PI); }

template <typename T> __always_inline T clamp(const T &_value, const T &_min, const T &_max)
{
    return std::min(std::max(_value, _min), _max);
}

} // namespace PINYMOTOR
