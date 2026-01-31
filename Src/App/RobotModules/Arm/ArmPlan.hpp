#pragma once
#include <array>
#include <cstdint>
#include <cmath>

namespace ARM {

using Joint10 = std::array<float, 10>;

struct Joint10D_s {
    Joint10 joints;
    float _delay;

    constexpr Joint10D_s(const Joint10 &_joints = { 0.0f }, float _delay = 0.0f) : joints(_joints), _delay(_delay) {}
};

struct JointRoute_s {
    Joint10D_s *pose;
    uint8_t target_point;
    uint16_t point_cnt = 0;
};

inline Joint10 operator-(const Joint10 &_joints1, const Joint10 &_joints2)
{
    Joint10 tmp;
    for (int i = 0; i < 10; i++)
        tmp[i] = _joints1[i] - _joints2[i];
    return tmp;
}

inline float AbsMaxOf10(const Joint10 &_joints)
{
    float max = -1.0f;
    for (uint8_t i = 0; i < 10; ++i) {
        float val = std::fabs(_joints[i]);
        max = std::max(max, val);
    }
    return max;
}
} // namespace ARM
