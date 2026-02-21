#pragma once

#include <array>
#include "Matrix.hpp"

namespace ARM {

using std::array;

using Joint = array<float, 6>;
Joint operator-(const Joint &_joint1, const Joint &_joint2);

class MDH {
public:
    MDH(const array<float, 6> &_theta, const array<float, 6> &_a, const array<float, 6> &_alpha,
        const array<float, 6> &_d, const array<float, 6> &_offset)
            : theta(_theta), a(_a), alpha(_alpha), d(_d), offset(_offset) {};

    const array<float, 6> theta;  // joint angle theta  (rad)
    const array<float, 6> a;      // link length        (m)
    const array<float, 6> alpha;  // link twist         (rad)
    const array<float, 6> d;      // link offest        (m)
    const array<float, 6> offset; // joint angle offest (rad)
};

struct RPY_s {
    float roll, yaw, pitch; // euler angle (rad)
};

struct XYZ_s {
    float x, y, z; // xyz (m)
};

struct Pose_s {
    RPY_s rpy;
    XYZ_s xyz;
};

class Kinematic {
public:
    Kinematic(); //just for test
    Kinematic(const MDH &_mdh);

    // utils functions
    static Matrix<3, 1> t2p(const Matrix<4, 4> _t);
    static Matrix<3, 3> t2r(const Matrix<4, 4> _t);
    static Matrix<4, 4> p2t(const Matrix<3, 1> _p);
    static Matrix<4, 4> r2t(const Matrix<3, 3> _r);
    static Matrix<4, 4> rp2t(const Matrix<3, 3> _r, const array<float, 3> _p);
    static Matrix<3, 3> hat(const Matrix<3, 1> _vec);
    static Matrix<3, 1> cross(const Matrix<3, 1> _vec1, const Matrix<3, 1> _vec2);
    static Matrix<3, 3> euler2Rotation(const array<float, 3> &_euler);
    static array<float, 3> rotation2euler(const Matrix<3, 3> &_rotationM);
    static Matrix<4, 4> dhTransform(const float _a, const float _alpha, const float _d, const float _theta);

    static float absMax(const Joint &_joints)
    {
        float max = -1.0f;
        for (uint8_t i = 0; i < 6; ++i) {
            float val = std::fabs(_joints[i]);
            max = std::max(max, val);
        }
        return max;
    }

    // kinematic functions
    Matrix<4, 4> fkine2T(const array<float, 6> &_theta); // benchmark: 76.496338us
    array<Matrix<4, 4>, 7> fkine2Ts(const array<float, 6> &_theta);
    array<float, 3> fkine2euler(const array<float, 6> &_theta);
    Pose_s fkine2pose(const array<float, 6> &_theta);
    Matrix<6, 6> jacob0(const array<float, 6> &_q);
    Joint ikine(const Matrix<4, 4> &_t, const array<float, 6> &_pose); // benchmark: 35.487488us
    Joint ikinePose(const array<float, 3> &_xyz, const array<float, 3> &_rpy, const array<float, 6> &_pose);
    Joint ikineWithTcp(const Matrix<4, 4> &_t, const Matrix<4, 4> &_tcp, const array<float, 6> &_pose);

    // TODO: dynamic functions
    void gravityCompensation(const array<float, 6> &_pos);
    void frictionCompensation(const array<float, 6> &_v);

private:
#ifndef TEST
    static inline float sqrtf(const float _x);
    static inline float cosf(float _x);
    static inline float sinf(float _y);
    static inline float atan2f(float _y, float _x);
#endif


private:
    // kinematic params
    MDH dh;

    // dynamic params
    array<float, 6> mass;    // link mass (kg)
    array<float, 3> gravity; // world gravity
    Matrix<6, 3> centerMass; // center of Mass (m)
    Matrix<6, 9> I;          // link inertia matrix
    array<float, 6> B;       // link viscous friction
    array<float, 6> Tc;      // link coulomb friction
    array<float, 6> Jm;      // motor inertia
};

} // namespace ARM
