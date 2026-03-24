#pragma once
#include <cstdint>
#include "ArmKinematic.hpp"

namespace ARM {

struct Joint_s {
    Joint joints;
    float _delay;

    constexpr Joint_s(const Joint &_joints = { 0.0f }, float _delay = 0.0f) : joints(_joints), _delay(_delay) {}
};

struct JointRoute_s {
    Joint_s *pose;
    uint8_t target_point;
    uint16_t point_cnt = 0;
};

inline Joint operator-(const Joint &_joints1, const Joint &_joints2)
{
    Joint tmp;
    for (int i = 0; i < 6; i++)
        tmp[i] = _joints1[i] - _joints2[i];
    return tmp;
}

} // namespace ARM
