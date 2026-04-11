#pragma once
#include <cstdint>
#include "ArmKinematic.hpp"
#include "MsgImpl.hpp"

namespace ARM {

struct Joint_s {
    Joint joints;
    float _delay;
    ARM::ClampState_e clampState;
    bool _isBinStucked = false;

    constexpr Joint_s(const Joint &_joints = { 0.0f }, float _delay = 0.0f,
                      ARM::ClampState_e _clampState = ARM::ClampState_e::NONE, bool _isBinStucked = false)
            : joints(_joints), _delay(_delay), clampState(_clampState), _isBinStucked(_isBinStucked)
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
//黑色
// inline Joint_s test[6] = {
//     { { -0.03f, -0.80f, 0.67f, 0.12f, -1.9f, 0.23f }, 5000.f, ARM::ClampState_e::CLAMP, false },
//     { { -0.966f, -0.561f, -0.348f, 1.150f, -2.276f, -0.160f }, 1000.f, ARM::ClampState_e::CLAMP, false },
//     { { -0.935f, -0.773f, -0.357f, 0.967f, -1.945f, -0.054f }, 1000.f, ARM::ClampState_e::RELEASE, false },
//     { { -1.115f, -0.781f, -0.404f, 1.012f, -2.131f, 0.223f }, 1000.f, ARM::ClampState_e::RELEASE, false },
//     { { -1.115f, -0.781f, 0.793f, 1.012f, -2.131f, 0.223f }, 1000.f, ARM::ClampState_e::RELEASE, false },
//     { { -0.391f, -1.061f, 0.501f, 0.378f, -0.251f, 0.113f }, 5000.f, ARM::ClampState_e::RELEASE, false },
// };
//正——转——到——放——退——转——正
inline Joint_s storage[7] = {
    { { 0.f, 0.187f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::NONE, false },
    { { 2.489f, 0.148f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::NONE, false },
    { { 2.96f, 0.510f, 1.01f, -0.436f, -0.981f, 1.637f }, 0.f, ARM::ClampState_e::NONE, false },
    { { 2.937f, 0.583f, 0.988f, -0.507f, -0.855f, 1.767f }, 0.f, ARM::ClampState_e::RELEASE, false },
    { { 2.934f, 0.494f, 0.58f, -0.505f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::RELEASE, true },
    { { 2.489f, 0.148f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::RELEASE, true },
    { { 0.f, 0.187f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::RELEASE, true },
};
//正——转——过渡——到——取——拿——转——正
inline Joint_s takeOut[8] = {
    { { 0.f, 0.187f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::NONE, true },
    { { 2.489f, 0.148f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::NONE, true },
    { { 2.96f, 0.397f, 0.811f, -0.359f, -0.784f, 1.406f }, 0.f, ARM::ClampState_e::NONE, false },
    { { 2.96f, 0.612f, 1.002f, -0.389f, -0.797f, 1.539f }, 0.f, ARM::ClampState_e::NONE, false },
    { { 2.947f, 0.607f, 1.00f, -0.367f, -0.776f, 1.603f }, 0.f, ARM::ClampState_e::CLAMP, false },
    { { 2.889f, 0.416f, 1.001f, -0.510f, -0.656f, 1.530f }, 0.f, ARM::ClampState_e::CLAMP, false },
    { { 2.489f, 0.148f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::CLAMP, false },
    { { 0.f, 0.187f, 0.725f, 0.332f, -0.848f, 1.820f }, 0.f, ARM::ClampState_e::CLAMP, false },
};

inline JointRoute_s storageRoute = { .pose = storage, .target_point = 7, .point_cnt = 0 };
inline JointRoute_s takeOutRoute = { .pose = takeOut, .target_point = 8, .point_cnt = 0 };

} // namespace ARM
