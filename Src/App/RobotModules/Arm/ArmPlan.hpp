#pragma once
#include <cstdint>
#include "ArmKinematic.hpp"
#include "MsgImpl.hpp"

namespace ARM {

struct Joint_s {
    Joint joints;
    float _delay;
    ARM::ClampState_e clampState;

    constexpr Joint_s(const Joint &_joints = { 0.0f }, float _delay = 0.0f,
                      ARM::ClampState_e _clampState = ARM::ClampState_e::NONE)
            : joints(_joints), _delay(_delay), clampState(_clampState)
    {
    }
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

inline Joint_s test[2] = {
    { { -0.03f, -0.80f, 0.67f, 0.12f, -1.9f, 0.23f }, 0.f, ARM::ClampState_e::NONE },
    { { -0.90f, -0.81f, -0.20f, 0.92f, -1.9f, 0.23f }, 0.f, ARM::ClampState_e::NONE },
};
inline JointRoute_s test1 = { .pose = test, .target_point = 2, .point_cnt = 0 };

} // namespace ARM
