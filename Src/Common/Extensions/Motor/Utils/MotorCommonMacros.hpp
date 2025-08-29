#pragma once

#include <cmath>
#include <cstdint>

#ifndef PI
#define PI 3.14159265358979323846f
#endif // !PI

#define CH_NUM   4
#define YAW_CH   0
#define PITCH_CH 1

namespace PINYMOTOR {

#define LIMIT_MIN_MAX(x, M1, M2)                                          \
    (x) = ((M1) < (M2)) ?                                                 \
                  (((x) <= (M1)) ? (M1) : (((x) >= (M2)) ? (M2) : (x))) : \
                  (((x) <= (M2)) ? (M2) : (((x) >= (M1)) ? (M1) : (x))) //copy
// You don't need to worry about the size of the input, just use it
#define IS_WITHIN_RANGE(x, M1, M2)                      \
    ({                                                  \
        typeof(x) _x = (x);                             \
        typeof(M1) _m1 = (M1);                          \
        typeof(M2) _m2 = (M2);                          \
        typeof(_m1) _min_val = (_m1 < _m2) ? _m1 : _m2; \
        typeof(_m2) _max_val = (_m1 < _m2) ? _m2 : _m1; \
        (_x >= _min_val && _x <= _max_val);             \
    }) //copy


static inline float getMinorArc(float _cur, float _ref, float _range)
{
    return (fmodf(((_cur) - (_ref) + ((_range) * 1.5f)), (_range)) -
            ((_range) / 2.f));
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
    return (_min > _max)   ? _value :
           (_value < _min) ? _min :
           (_value > _max) ? _max :
                             _value;
}

static inline float s_curve_acc(float _know, float _kref, float _accTime,
                                short _ch)
{
    // 显式初始化静态数组，解决动态初始化警告
    static int sTime[CH_NUM] = { 0 };     // 各通道的时间计数器
    static float temp[CH_NUM] = { 0.0f }; // 各通道的当前值缓存
    static short sFlag[CH_NUM] = { 0 };   // 各通道的状态标志

    // 将静态变量改为局部变量，避免跨调用干扰
    float tRatio; // 时间比例
    float deltaK; // 目标差值

    // 如果当前值发生变化，重置状态
    if (temp[_ch] != _know) {
        sFlag[_ch] = 0;
        temp[_ch] = _know;
        sTime[_ch] = 0;
    }

    if (sFlag[_ch] == 0) {
        sTime[_ch]++; // 递增时间计数器

        tRatio = static_cast<float>(sTime[_ch]) / _accTime;
        deltaK = _kref - _know;

        // S曲线的前半段（加速阶段）
        if (static_cast<float>(sTime[_ch]) <= _accTime / 2) {
            return (deltaK * (2.0f * tRatio * tRatio)) + _know;
        }
        // S曲线的后半段（减速阶段）
        else if (static_cast<float>(sTime[_ch]) < _accTime) {
            tRatio = tRatio - 1.0f;
            return (deltaK * (1.0f - 2.0f * tRatio * tRatio)) + _know;
        }
        // 达到目标值，切换状态
        else {
            sFlag[_ch] = 1;
            sTime[_ch] = 0;
            return _kref;
        }
    }
    // 已达到目标值，保持输出
    else if (sFlag[_ch] == 1) {
        return _kref;
    }

    return _kref;
}
} // namespace PINYMOTOR
