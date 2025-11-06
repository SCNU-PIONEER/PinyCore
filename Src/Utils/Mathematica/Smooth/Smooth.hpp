#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace SMOOTH {

// 数值平滑跟踪 跟踪步长为acc
static inline void slopeFollowing(float *_exp, float *_cur, float _acc)
{
    if (*_exp > *_cur) {
        *_cur = *_cur + _acc;
        *_cur = std::min(*_cur, *_exp);
    } else if (*_exp < *_cur) {
        *_cur = *_cur - _acc;
        *_cur = std::max(*_cur, *_exp);
    }
}
// 二次函数计算速度曲线
static float sCurve(float _vMax, float _cnt, float _tAccCnt)
{
    float cntcnt;
    if (_cnt < 0.0f) {
        _cnt = -_cnt;
        _vMax = -_vMax;
    }
    cntcnt = _cnt / _tAccCnt;
    if (_cnt < _tAccCnt / 2.0f) {
        return 2.0f * _vMax * (cntcnt * cntcnt);
    } else if (_cnt < _tAccCnt) {
        cntcnt = cntcnt - 1.0f;
        return _vMax * (1.0f - 2.0f * (cntcnt * cntcnt));
    } else
        return _vMax;
}
}; // namespace SMOOTH
