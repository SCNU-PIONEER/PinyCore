#pragma once

#include "Matrix.hpp"


namespace CHASSIS::BIPEDAL {

struct LegState_s {
    // basic
    float phi1, phi4;   // raw angle
    float dphi1, dphi4; // raw angular velocity
    float tau1, tau4;   // raw torque

    // derived
    float phi2, phi3; // derived angle

    // derived's derivative
    float tp;          // end's tangential torqe
    float fd;          // end's radial torqe
    float vp, vd;      // end's tangential and radial velocity
    float phi0, dphi0; // end's angle
    float L0;          // end's distance

    // other
    float ap, ad, ax;     // end's acceleration
    float vpPrev, vdPrev; // end's previous velocity
};

struct VMCConfig_s {
    const float L1;
    const float L2;
    const float L3;
    const float L4;
    const float L5;
};

/**
 * @note plot
 *  axis:
 *  ↑ y
 *  |             5 C
 *  ·——-> x     L2  /·\  L3
 *                /   \
 *           B 2 /     \ 3 D
 *           L1  \_____/  L4
 *           A 1   L5   4 E
 * 
 * @note The definition of angle refers to
 *        https://zhuanlan.zhihu.com/p/613007726
 */
class VMC {
public:
    VMC(const VMCConfig_s &_config);
    ~VMC() = default;

    void update(LegState_s &_curLegState, float _dt);

    void inverse(LegState_s &_curLegState);

    Matrix<2, 1> forwardTorq(float _tp, float _fd);
    Matrix<2, 1> inverseTorq(float _tau1, float _tau4);
    Matrix<2, 1> forwardVel(float _vp, float _vd);
    Matrix<2, 1> inverseVel(float _dphi1, float _dphi4);

private:
    // Pole length
    float L1_;
    float L2_;
    float L3_;
    float L4_;
    float L5_;

    // TF matrix
    Matrix<2, 2> J_T_; // Jacobian Trans
    Matrix<2, 2> J_;   // Jacobian : T1,T4->Fx,Fy
    Matrix<2, 2> R_;   // Rotation matrix : Polar->Cartesian
    Matrix<2, 2> M_;   // Translation matrix : Force->Torque

    Matrix<2, 2> J_T_inv_; // Inverse of J_T
    Matrix<2, 2> J_inv_;   // Inverse of J
    Matrix<2, 2> R_inv_;   // Inverse of R
    Matrix<2, 2> M_inv_;   // Inverse of M

    Matrix<2, 2> invFast(Matrix<2, 2> &_mat) const;
};
} // namespace CHASSIS::BIPEDAL
