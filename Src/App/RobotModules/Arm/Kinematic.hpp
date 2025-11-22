#pragma once

#include <array>
#include "Matrix.hpp"

namespace ARM {

using std::array;

class MDH {
public:
    MDH(const array<float, 6> &_theta, const array<float, 6> &_a, const array<float, 6> &_alpha,
        const array<float, 6> &_d)
            : theta(_theta), a(_a), alpha(_alpha), d(_d) {};

private:
    friend class Kinematic;

    array<float, 6> theta; // joint angle theta
    array<float, 6> a;     // link length
    array<float, 6> alpha; // link twist
    array<float, 6> d;     // link offest
};

class Kinematic {
public:
    Kinematic(); //just for test
    Kinematic(const MDH &_mdh);

    // utils functions
    Matrix<3, 1> t2p(Matrix<4, 4> _t);
    Matrix<4, 4> p2t(Matrix<3, 1> _p);
    Matrix<3, 3> hat(Matrix<3, 1> _vec);
    Matrix<3, 1> cross(Matrix<3, 1> _vec1, Matrix<3, 1> _vec2);
    static array<float, 9> eulerAngle2Rotation(const array<float, 3> &_euler);
    static array<float, 3> rotation2euler(const array<float, 9> &_rotationM);
    static Matrix<4, 4> dhTransform(const float _a, const float _alpha, const float _d, const float _theta);

    // kinematic functions
    Matrix<4, 4> fkine2T(const array<float, 6> &_theta);
    array<Matrix<4, 4>, 7> fkine2Ts(const array<float, 6> &_theta);
    array<float, 3> fkine2euler(const array<float, 6> &_theta);
    Matrix<6, 6> jacob0(const array<float, 6> &_q);
    array<float, 6> ikine(const Matrix<4, 4> &_t, const array<float, 6> &_pose);
    array<float, 6> ikineWithTcp(const Matrix<4, 4> &_t, const Matrix<4, 4> &_tcp, const array<float, 6> &_pose);

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
    array<float, 6> mass;    // link mass
    array<float, 3> gravity; // world gravity
    Matrix<6, 3> I;          // link inertia matrix
    array<float, 6> B;       // link viscous friction
    array<float, 6> Tc;      // link coulomb friction
    array<float, 6> Jm;      // motor inertia
};

} // namespace ARM
