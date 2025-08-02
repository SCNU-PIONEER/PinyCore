/*
 * @File         : ArmKinematic.hpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 16:37:36
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */
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
            : j{ _j1, _j2, _j3, _j4, _j5, _j6, _j7,  }
    {
    }

    float j[7];
    uint32_t delay = 0;

    friend Joint7D operator-(const Joint7D &_joints1,
                               const Joint7D &_joints2);
};

struct JointRoute_s {
    const Joint7D *pose;
    uint8_t point;
    const Pump_s *pump;
};

typedef struct {
    char *goal;
    void (*set_goal)(const Joint7D *_route);
} route_s;


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

// 路径点与路线声明
// extern const Joint7D gold_left[];

// extern const Joint7D gold_middle[];
// extern const Joint7D gold_right[];
// extern const Joint7D silver_left[1];
// extern const Joint7D silver_left_storge[7];

// extern const JointRoute_s silver_left_test;
// extern const JointRoute_s silver_left_route;