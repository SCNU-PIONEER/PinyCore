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

    static array<float, 9> eulerAngle2Rotation(const array<float, 3> &_euler);
    static array<float, 3> rotation2euler(const array<float, 9> &_rotationM);
    static Matrix<4, 4> dhTransform(float _a, float _alpha, float _d, float _theta);

    Matrix<4, 4> fkine2T(array<float, 6> _theta);
    array<Matrix<4, 4>, 7> fkine2Ts(array<float, 6> _theta);
    array<float, 3> fkine2euler(array<float, 6> _theta);

private:
#ifndef TEST
    static inline float sqrtf(const float _x);
    static inline float cosf(float _x);
    static inline float sinf(float _y);
    static inline float atan2f(float _y, float _x);
#endif


private:
    MDH dh;

    // no use
    array<float, 6> mass;
    array<float, 3> gravity;
    Matrix<6, 3> localCom;
};

} // namespace ARM
