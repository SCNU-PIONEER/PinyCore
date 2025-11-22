#include "./Kinematic.hpp"
#include "Matrix.hpp"
#include <cmath>
#include <cstdint>

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
#else

#include <cstdio>
template <int Rows, int Cols> void printfMatrix(const Matrix<Rows, Cols> &mat)
{
    for (int i = 0; i < Rows; i++) {
        printf("[");
        for (int j = 0; j < Cols; j++) {
            printf("%10.6f", mat(i, j));
            if (j < Cols - 1)
                printf(", ");
        }
        printf("]\n");
    }
    printf("\n");
}

#endif


Kinematic::Kinematic()
        : dh({ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // theta
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // a
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }, // alpha
             { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }) // d
{
}

Kinematic::Kinematic(const MDH &_mdh) : dh(_mdh)
{
    if (dh.a[2] == 0 || dh.a[3] == 0) {
        // LOG::error("Arm", "dh param error");
    }
}

Matrix<3, 1> Kinematic::t2p(Matrix<4, 4> _t) { return _t.block<3, 1>(0, 3); }

Matrix<3, 3> Kinematic::t2r(Matrix<4, 4> _t) { return _t.block<3, 3>(0, 0); }

Matrix<4, 4> Kinematic::p2t(Matrix<3, 1> _p)
{
    // T=[I,P;0,1]
    float t[16] = { 1, 0, 0, _p(0, 0), 0, 1, 0, _p(1, 0), 0, 0, 1, _p(2, 0), 0, 0, 0, 1 };
    return Matrix<4, 4>(t);
}

Matrix<4, 4> Kinematic::r2t(Matrix<3, 3> _r)
{
    Matrix<4, 4> t = { { _r(0, 0), _r(0, 1), _r(0, 2), 0 },
                       { _r(1, 0), _r(1, 1), _r(1, 2), 0 },
                       { _r(2, 0), _r(2, 1), _r(2, 2), 0 },
                       { 0, 0, 0, 1 } };
    return t;
}

Matrix<4, 4> Kinematic::rp2t(Matrix<3, 3> _r, array<float, 3> _p)
{
    Matrix<4, 4> t = { { _r(0, 0), _r(0, 1), _r(0, 2), _p[0] },
                       { _r(1, 0), _r(1, 1), _r(1, 2), _p[1] },
                       { _r(2, 0), _r(2, 1), _r(2, 2), _p[2] },
                       { 0, 0, 0, 1 } };
    return t;
}

Matrix<3, 3> Kinematic::hat(Matrix<3, 1> _vec)
{
    float hat[9] = { 0, -_vec(2, 0), _vec(1, 0), _vec(2, 0), 0, -_vec(0, 0), -_vec(1, 0), _vec(0, 0), 0 };
    return Matrix<3, 3>(hat);
}

Matrix<3, 1> Kinematic::cross(Matrix<3, 1> _vec1, Matrix<3, 1> _vec2) { return hat(_vec1) * _vec2; }

Matrix<3, 3> Kinematic::euler2Rotation(const array<float, 3> &_rpy)
{
    Matrix<3, 3> rot;

    float ca = cosf(_rpy[2]);
    float cb = cosf(_rpy[1]);
    float cc = cosf(_rpy[0]);
    float sa = sinf(_rpy[2]);
    float sb = sinf(_rpy[1]);
    float sc = sinf(_rpy[0]);

    rot(0, 0) = ca * cb;
    rot(0, 1) = (ca * sb * sc) - (sa * cc);
    rot(0, 2) = (ca * sb * cc) + (sa * sc);
    rot(1, 0) = sa * cb;
    rot(1, 1) = (sa * sb * sc) + (ca * cc);
    rot(1, 2) = (sa * sb * cc) - (ca * sc);
    rot(2, 0) = -sb;
    rot(2, 1) = cb * sc;
    rot(2, 2) = cb * cc;

    return rot;
}

array<float, 3> Kinematic::rotation2euler(const Matrix<3, 3> &_rot)
{
    array<float, 3> euler;

    /* 奇异点 */
    if (fabsf(_rot(2, 0)) >= 0.9999f) {
        if (_rot(2, 0) < 0) {
            // Pitch = +90° (万向节锁)
            euler[0] = 0.0f;
            euler[1] = (float)M_PI_2;
            euler[2] = atan2f(-_rot(1, 2), _rot(1, 1));
        } else {
            // Pitch = -90° (万向节锁)
            euler[0] = 0.0f;
            euler[1] = -(float)M_PI_2;
            euler[2] = -atan2f(_rot(1, 2), _rot(1, 1));
        }
    } else {
        // 非奇异情况 (ZYX顺序: Roll-Pitch-Yaw)
        float denominator = sqrtf((_rot(0, 0) * _rot(0, 0)) + (_rot(1, 0) * _rot(1, 0)));
        if (fabsf(denominator) < 1e-6f)
            denominator = 1e-6f;
        // Pitch
        euler[1] = atan2f(-_rot(2, 0), denominator);
        float cosPitch = cosf(euler[1]);
        if (fabsf(cosPitch) < 1e-6f)
            cosPitch = 1e-6f;
        euler[0] = atan2f(_rot(2, 1) / cosPitch, _rot(2, 2) / cosPitch); // Roll
        euler[2] = atan2f(_rot(1, 0) / cosPitch, _rot(0, 0) / cosPitch); // Yaw
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
// Matrix<4, 4> Kinematic::dhTransform(const float _a, const float _alpha, const float _d, const float _theta)
// {
//     float ct = cosf(_theta), st = sinf(_theta);
//     float ca = cosf(_alpha), sa = sinf(_alpha);
//     return Matrix<4, 4>{
//         { ct, -st, 0, _a }, { st * ca, ct * ca, -sa, -_d * sa }, { st * sa, ct * sa, ca, _d * ca }, { 0, 0, 0, 1.0f }
//     };
// }

/*
 * brief: get homogeneous transformation matrix in SDH
 * param: _a: link length
 *        _alpha: link twist
 *        _d: link offest
 *        _theta: joint angle theta
 *
 * return: Matirx<4,4> homogeneous transformation matrix
 *       [ct, -st*ca, st*sa , a*ct ],
 *       [st, ct*ca , -ct*sa, a*st ],
 *       [0 , sa    , ca    , d    ],
 *       [0 , 0     , 0     , 1    ]
*/
Matrix<4, 4> Kinematic::dhTransform(const float _a, const float _alpha, const float _d, const float _theta)
{
    float ct = cosf(_theta), st = sinf(_theta);
    float ca = cosf(_alpha), sa = sinf(_alpha);

    return Matrix<4, 4>{
        { ct, -st * ca, st * sa, _a * ct }, { st, ct * ca, -ct * sa, _a * st }, { 0, sa, ca, _d }, { 0, 0, 0, 1.0f }
    };
}


Matrix<4, 4> Kinematic::fkine2T(const array<float, 6> &_theta)
{
    auto t = Matrix<4, 4>::eye();

    for (int i = 0; i < 6; ++i) {
        // auto theta = _theta[i] + dh.theta[i];
        auto ti = dhTransform(dh.a[i], dh.alpha[i], dh.d[i], _theta[i]);
        t = t * ti;
    }
    return t;
}

array<Matrix<4, 4>, 7> Kinematic::fkine2Ts(const array<float, 6> &_theta)
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

array<float, 3> Kinematic::fkine2euler(const array<float, 6> &_theta)
{
    auto t = fkine2T(_theta);
    Matrix<3, 3> rot = t2r(t);
    auto euler = rotation2euler(rot);
    return euler;
}


Matrix<6, 6> Kinematic::jacob0(const array<float, 6> &_q)
{
    Matrix<3, 1> p = t2p(fkine2T(_q));    // p_e
    Matrix<4, 4> t = Matrix<4, 4>::eye(); // T_i-1
    Matrix<3, 1> z;                       // 关节 i 相对于基座（坐标系 0）的 z 轴单位向量
    Matrix<3, 1> r;                       // 末端执行器相对于坐标系 i 的位置
    Matrix<3, 1> jl;                      // 关节速度与线速度关联起来的部分
    Matrix<3, 1> ja;                      // 关节速度与角速度关联起来的部分
    Matrix<6, 6> j;                       //return

    // revolute joint: J_pi = z-1x(p_e-p_i-1), J_oi = z-1
    for (int i = 0; i < 6; i++) {
        // MDH
        // auto theta = _q[i] + dh.theta[i];
        // t = t * dhTransform(dh.a[i], dh.alpha[i], dh.d[i], theta);

        z = t.block<3, 1>(0, 2);
        r = t2p(t);

        // SDH
        auto theta = _q[i] + dh.theta[i];
        t = t * dhTransform(dh.a[i], dh.alpha[i], dh.d[i], theta);

        jl = cross(p - r, z);
        ja = z;

        j(0, i) = jl(0, 0);
        j(1, i) = jl(1, 0);
        j(2, i) = jl(2, 0);
        j(3, i) = ja(0, 0);
        j(4, i) = ja(1, 0);
        j(5, i) = ja(2, 0);
    }
    return j;
}

/**
 * @brief Inverse Kinematic
 * @param[in] target t
 * @param[in] current pose
 *
 * @refer：
 * Villalobos, J., Sanchez, I. Y., & Martell, F. (2022). 
 * Singularity Analysis and Complete Methods to Compute the Inverse Kinematics 
 * for a 6-DOF UR/TM-Type Robot. Robotics, 11(6), 137.
 * https://doi.org/10.3390/robotics11060137
 *
 * Algorithm 2: compute only one complete set of angles by FSM
 */
array<float, 6> Kinematic::ikine(const Matrix<4, 4> &_t, const array<float, 6> &_pose)
{
    //NOLINTBEGIN
    array<float, 6> target;

    auto minAngle = [](const float currentAngle, const float angle1, const float angle2) -> float {
        auto angularDistance = [](const float a, const float b) -> float {
            float diff = fabs(a - b);
            return (diff <= M_PI) ? diff : (2 * M_PI - diff);
        };
        float diff1 = angularDistance(angle1, currentAngle);
        float diff2 = angularDistance(angle2, currentAngle);
        return (diff1 < diff2) ? angle1 : angle2;
    };

    float r11 = _t(0, 0);
    float r12 = _t(0, 1);
    float r13 = _t(0, 2);
    float r21 = _t(1, 0);
    float r22 = _t(1, 1);
    float r23 = _t(1, 2);
    float r31 = _t(2, 0);
    float r32 = _t(2, 1);
    float r33 = _t(2, 2);

    float px = _t(0, 3);
    float py = _t(1, 3);
    float pz = _t(2, 3);

    float d1 = dh.d[0];
    float d2 = dh.d[1];
    float d3 = dh.d[2];
    float d4 = dh.d[3];
    float d5 = dh.d[4];
    float d6 = dh.d[5];

    float a2 = dh.a[1];
    float a3 = dh.a[2];

    // theta 1
    float A = py - d6 * r23;
    float B = px - d6 * r13;

    float sqrt1 = B * B + A * A - d4 * d4;

    if (sqrt1 < 0)
        return _pose;

    float theta1_1 = atan2f(sqrtf(sqrt1), d4) + atan2f(B, -A);
    float theta1_2 = -atan2f(sqrtf(sqrt1), d4) + atan2f(B, -A);

    target[0] = minAngle(_pose[0], theta1_1, theta1_2);

    // theta 5
    float c1 = cosf(target[0]);
    float s1 = sinf(target[0]);

    float D = c1 * r22 - s1 * r12;
    float E = s1 * r11 - c1 * r21;
    if (fabsf(D) < 1e-6)
        D = 0.0;
    if (abs(E) < 1e-6)
        E = 0.0;

    float theta5_1 = atan2f(sqrtf(E * E + D * D), s1 * r13 - c1 * r23);
    float theta5_2 = atan2f(-sqrtf(E * E + D * D), s1 * r13 - c1 * r23);

    target[4] = minAngle(_pose[4], theta5_1, theta5_2);

    // Singularities
    if (fabsf(sinf(target[4])) < 1e-12f)
        return _pose;

    // theta 6
    float s5 = sinf(target[4]);
    target[5] = atan2f(D / s5, E / s5);

    // theta 3
    float c5 = cosf(target[4]);
    float s6 = sinf(target[5]);
    float c6 = cosf(target[5]);
    float C = c1 * r11 + s1 * r21;
    float F = c5 * c6;

    float theta234 = atan2f(((r31 * F) - (s6 * C)), ((F * C) + (s6 * r31)));

    float s234 = sinf(theta234);
    float c234 = cosf(theta234);

    float Kc = (c1 * px) + (s1 * py) - (s234 * d5) + (c234 * s5 * d6);
    float Ks = pz - d1 + (c234 * d5) + (s234 * s5 * d6);

    float c3 = ((Ks * Ks) + (Kc * Kc) - (a2 * a2) - (a3 * a3)) / (2 * a2 * a3);

    float s3 = sqrtf(1 - (c3 * c3));
    if (abs(s3) < 1e-6)
        s3 = 0.0;

    float theta3_1 = atan2f(s3, c3);
    float theta3_2 = atan2f(-s3, c3);

    target[2] = minAngle(_pose[2], theta3_1, theta3_2);

    // Singularities
    if (fabsf(sinf(target[2])) < 1e-12f)
        return _pose;

    // theta 2
    target[1] = atan2f(Ks, Kc) - atan2f(s3 * dh.a[2], c3 * dh.a[2] + dh.a[1]);

    // theta 4
    target[3] = theta234 - target[1] - target[2];

    // map angles to [-pi, pi]
    for (uint8_t i : { 0, 1, 2, 3, 4, 5 }) {
        if (!std::isfinite(target[i])) {
            target = _pose; // just as same
            return target;
        }

        const double y = fmod(target[i], 2.0 * M_PI);
        if (y > M_PI) {
            target[i] = y - 2.0 * M_PI;
        } else if (y < -M_PI) {
            target[i] = y + 2.0 * M_PI;
        }
    }

    return target;
    //NOLINTEND
}

array<float, 6> Kinematic::ikinePose(const array<float, 3> &_xyz, const std::array<float, 3> &_rpy,
                                     const array<float, 6> &_pose)
{
    auto rot = euler2Rotation(_rpy);
    auto t = rp2t(rot, _xyz);

    return ikine(t, _pose);
}

array<float, 6> Kinematic::ikineWithTcp(const Matrix<4, 4> &_t, const Matrix<4, 4> &_tcp, const array<float, 6> &_pose)
{
    return ikine(_t * _tcp.inv(), _pose);
}
