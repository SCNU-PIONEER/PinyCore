#include "./Kinematic.hpp"
#include "Matrix.hpp"

using namespace ARM;

#ifndef TEST
#include "dsp/fast_math_functions.h"

float Kinematic::cosf(float _x) { return arm_cos_f32(_x); }

float Kinematic::sinf(float _y) { return arm_sin_f32(_y); }

float Kinematic::atan2f(float _y, float _x)
{
    float atan = 0;
    arm_atan2_f32(_y, _x, &atan);
    return atan;
}

float Kinematic::sqrtf(const float _x)
{
    float sqrt = 0;
    arm_sqrt_f32(_x, &sqrt);
    return sqrt;
}

#endif


Kinematic::Kinematic()
        : dh({ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // theta
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // a
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // alpha
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }) // d
{
}

Kinematic::Kinematic(const MDH &_mdh) : dh(_mdh) {}

array<float, 9> Kinematic::eulerAngle2Rotation(const array<float, 3> &_euler)
{
    array<float, 9> rot;

    float ca = cosf(_euler[2]);
    float cb = cosf(_euler[1]);
    float cc = cosf(_euler[0]);
    float sa = sinf(_euler[2]);
    float sb = sinf(_euler[1]);
    float sc = sinf(_euler[0]);

    rot[0] = ca * cb;
    rot[1] = (ca * sb * sc) - (sa * cc);
    rot[2] = (ca * sb * cc) + (sa * sc);
    rot[3] = sa * cb;
    rot[4] = (sa * sb * sc) + (ca * cc);
    rot[5] = (sa * sb * cc) - (ca * sc);
    rot[6] = -sb;
    rot[7] = cb * sc;
    rot[8] = cb * cc;

    return rot;
}

array<float, 3> Kinematic::rotation2euler(const array<float, 9> &_rot)
{
    array<float, 3> euler;

    /* 奇异点 */
    if (fabsf(_rot[6]) >= 0.9999f) {
        if (_rot[6] < 0) {
            // Pitch = +90° (万向节锁)
            euler[0] = 0.0f;
            euler[1] = (float)M_PI_2;
            euler[2] = atan2f(-_rot[5], _rot[4]);
        } else {
            // Pitch = -90° (万向节锁)
            euler[0] = 0.0f;
            euler[1] = -(float)M_PI_2;
            euler[2] = -atan2f(_rot[5], _rot[4]);
        }
    } else {
        // 非奇异情况 (ZYX顺序: Roll-Pitch-Yaw)
        float denominator = sqrtf((_rot[0] * _rot[0]) + (_rot[3] * _rot[3]));
        if (fabsf(denominator) < 1e-6f)
            denominator = 1e-6f;
        //Pitch
        euler[1] = atan2f(-_rot[6], denominator);
        float cosPitch = cosf(euler[1]);
        if (fabsf(cosPitch) < 1e-6f)
            cosPitch = 1e-6f;
        euler[0] = atan2f(_rot[7] / cosPitch, _rot[8] / cosPitch); //Roll
        euler[2] = atan2f(_rot[3] / cosPitch, _rot[0] / cosPitch); //Yaw
    }

    return euler;
}

/*
 * brief: get homogeneous transformation matrix in MDH
 * param: _a: link length
 *        _alpha: link twist
 *        _d: link offest
 *        _theta: joint angle theta
 *
 * return: Matirx<4,4> homogeneous transformation matrix
 *       [ct   , -st  , 0  , a    ],
 *       [st*ca, ct*ca, -sa, -d*sa],
 *       [st*sa, ct*sa, ca , d*ca ],
 *       [0    , 0    , 0  , 1    ]
*/
Matrix<4, 4> Kinematic::dhTransform(float _a, float _alpha, float _d, float _theta)
{
    float ct = cosf(_theta), st = sinf(_theta);
    float ca = cosf(_alpha), sa = sinf(_alpha);

    return Matrix<4, 4>{
        { ct, -st, 0, _a }, { st * ca, ct * ca, -sa, -_d * sa }, { st * sa, ct * sa, ca, _d * ca }, { 0, 0, 0, 1.0f }
    };
}


Matrix<4, 4> Kinematic::fkine2T(array<float, 6> _theta)
{
    auto t = Matrix<4, 4>::eye();

    for (int i = 0; i < 6; ++i) {
        auto theta = _theta[i] + dh.theta[i];
        auto ti = dhTransform(dh.a[i], dh.alpha[i], dh.d[i], theta);
        t = t * ti;
    }
    return t;
}

array<Matrix<4, 4>, 7> Kinematic::fkine2Ts(array<float, 6> _theta)
{
    auto t = Matrix<4, 4>::eye();
    array<Matrix<4, 4>, 7> ts{};

    // 基座变换（单位矩阵）
    ts[0] = t;
    for (int i = 0; i < 6; ++i) {
        auto theta = _theta[i] + dh.theta[i];
        auto ti = dhTransform(dh.a[i], dh.alpha[i], dh.d[i], theta);
        t = t * ti;
        ts[i + 1] = t;
    }
    return ts;
}

array<float, 3> Kinematic::fkine2euler(array<float, 6> _theta)
{
    array<float, 9> rot;

    auto t = fkine2T(_theta);

    rot[0] = t(0, 0);
    rot[1] = t(0, 1);
    rot[2] = t(0, 2);
    rot[3] = t(1, 0);
    rot[4] = t(1, 1);
    rot[5] = t(1, 2);
    rot[6] = t(2, 0);
    rot[7] = t(2, 1);
    rot[8] = t(2, 2);

    auto euler = rotation2euler(rot);
    return euler;
}
