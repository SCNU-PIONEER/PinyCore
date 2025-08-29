#pragma once

#include <cstdint>
#include <cmath>
#include "Pump.hpp"


class Joint7D {
public:
    Joint7D() = default;

    Joint7D(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,
            float _j7)
            : j{ _j1, _j2, _j3, _j4, _j5, _j6, _j7 }
    {
    }
    Joint7D(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,
            float _j7, uint32_t _delay)
            : j{
                _j1, _j2, _j3, _j4, _j5, _j6, _j7,
            }
    {
    }

    float j[7];
    uint32_t delay = 0;

    friend Joint7D operator-(const Joint7D &_joints1, const Joint7D &_joints2);
};

struct JointRoute_s {
    const Joint7D *pose;
    const uint8_t point;
    const PUMP::State_e *pump;
};

struct Route_s {
    char *goal;
    void (*set_goal)(const Joint7D *_route); //TODO:BUG
};


extern const JointRoute_s silverLeftRoute; //TODO:BUG
extern const Joint7D silverLeftStorage[7]; //  TODO:BUG


// 工具函数
inline float AbsMaxOf7(const Joint7D &_joints)
{
    float max = -1.0f;
    for (uint8_t i = 0; i < 7; ++i) {
        float val = std::fabs(_joints.j[i]);
        max = std::max(max, val);
    }
    return max;
}

inline float AbsMaxOf6(const Joint7D &_joints)
{
    float max = -1.0f;
    for (uint8_t i = 0; i < 6; ++i) {
        float val = std::fabs(_joints.j[i]);
        max = std::max(max, val);
    }
    return max;
}

inline float AbsMaxOf5(const Joint7D &_joints)
{
    float max = -1.0f;
    for (uint8_t i = 1; i < 6; ++i) {
        float val = std::fabs(_joints.j[i]);
        max = std::max(max, val);
    }
    return max;
}
