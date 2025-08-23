#pragma once
#include "dsp/fast_math_functions.h"

static inline float minorArc(float _ref, float _cur)
{
    return std::fmod(_ref - _cur + (3.f * PI), 2.f * PI) - PI;
}