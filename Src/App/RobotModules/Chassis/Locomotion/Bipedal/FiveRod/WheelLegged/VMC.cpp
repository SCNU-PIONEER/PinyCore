#include "VMC.hpp"

#include <cmath>
#include "dsp/fast_math_functions.h"

using namespace CHASSIS::BIPEDAL;

VMC::VMC(const VMCConfig_s &_config)
        : L1_(_config.L1)
        , L2_(_config.L2)
        , L3_(_config.L3)
        , L4_(_config.L4)
        , L5_(_config.L5)
{
}

void VMC::update(LegState_s &_legState, float _dt)
{
    // temporary I
    float sin1 = arm_sin_f32(_legState.phi1);
    float cos1 = arm_cos_f32(_legState.phi1);
    float sin4 = arm_sin_f32(_legState.phi4);
    float cos4 = arm_cos_f32(_legState.phi4);

    // calc phi2 and phi3
    float xA, yA, xB, yB, xD, yD, xE, yE;
    xA = -L5_ / 2.f;
    yA = 0.f;
    xE = L5_ / 2.f;
    yE = 0.f;
    xB = xA + L1_ * cos1;
    yB = yA + L1_ * sin1;
    xD = xE + L4_ * cos4;
    yD = yE + L4_ * sin4;

    float xDB, yDB, lDB;
    xDB = xB - xD;
    yDB = yB - yD;
    arm_sqrt_f32(((xDB * xDB) + (yDB * yDB)), &lDB); // TODO: optimize sqrt

    float cosBeta, beta, alpha;
    // k = abs((L3_ * L3_) - (L2_ * L2_ + lDB * lDB)) / (2.f * L2_ * lDB);
    cosBeta = ((L2_ * L2_ + lDB * lDB) - (L3_ * L3_)) / (2.f * L2_ * lDB);
    beta = acos(cosBeta);
    // arm_atan2_f32(yDB, xDB, &addTheta);
    alpha = atan(yDB / xDB); // TODO: should I use atan2?

    _legState.phi2 = beta + alpha;
    _legState.phi3 = PI - beta + alpha;

    // temporary II
    float sin2 = arm_sin_f32(_legState.phi2);
    float cos2 = arm_cos_f32(_legState.phi2);
    float sin3 = arm_sin_f32(_legState.phi3);
    float cos3 = arm_cos_f32(_legState.phi3);

    // calc phi0 and l0
    float xC, yC, phiC;
    xC = L2_ * cos2 + L1_ * cos1 - L5_ / 2.f;
    yC = L2_ * sin2 + L1_ * sin1;

    arm_atan2_f32(yC, xC, &phiC);
    _legState.phi0 = phiC - PI / 2.f;
    arm_sqrt_f32(((xC * xC) + (yC * yC)), &_legState.L0);

    // assemble J Matrix
    float sin21 = arm_sin_f32(_legState.phi1 - _legState.phi2);
    float sin43 = arm_sin_f32(_legState.phi3 - _legState.phi4);
    float sin32 = arm_sin_f32(_legState.phi2 - _legState.phi3);
    J_[0][0] = +L1_ * sin3 * sin21 / sin32;
    J_[0][1] = +L4_ * sin2 * sin43 / sin32;
    J_[1][0] = -L1_ * cos3 * sin21 / sin32;
    J_[1][1] = -L4_ * cos2 * sin43 / sin32;
    J_inv_ = J_.inv();
    J_T_[0][0] = +L1_ * sin3 * sin21 / sin32;
    J_T_[0][1] = -L1_ * cos3 * sin21 / sin32;
    J_T_[1][0] = +L4_ * sin2 * sin43 / sin32;
    J_T_[1][1] = -L4_ * cos2 * sin43 / sin32;
    J_T_inv_ = J_T_.inv();

    // assemble R Matrix
    R_[0][0] = -arm_sin_f32(phiC);
    R_[0][1] = +arm_cos_f32(phiC);
    R_[1][0] = +arm_cos_f32(phiC);
    R_[1][1] = +arm_sin_f32(phiC);
    R_inv_ = R_.inv();

    // assemble M Matrix
    M_[0][0] = 1.f / _legState.L0;
    M_[0][1] = 0.f;
    M_[1][0] = 0.f;
    M_[1][1] = 1.f;
    M_inv_ = M_.inv();

    // update derivatives
    _legState.vpPrev = _legState.vp;
    _legState.vdPrev = _legState.vd;

    Matrix<2, 1> curTqFd = inverseTorq(_legState.tau1, _legState.tau4);
    _legState.tp = curTqFd[0][0];
    _legState.fd = curTqFd[1][0];

    Matrix<2, 1> curVqVd = inverseVel(_legState.dphi1, _legState.dphi4);
    _legState.vp = curVqVd[0][0];
    _legState.vd = curVqVd[1][0];

    _legState.dphi0 = _legState.vp / _legState.L0;

    _legState.ap = (_legState.vp - _legState.vpPrev) / _dt;
    _legState.ad = (_legState.vd - _legState.vdPrev) / _dt;
    _legState.ax = arm_cos_f32(_legState.phi0) * _legState.ap +
                   arm_sin_f32(_legState.phi0) * _legState.ad;
}

Matrix<2, 1> VMC::forwardTorq(float _tp, float _fd)
{
    float input[2] = { _tp, _fd };
    Matrix<2, 1> inputVector(input);
    return J_T_ * R_ * M_ * inputVector; // get tau1 and tau4
}

Matrix<2, 1> VMC::inverseTorq(float _tau1, float _tau4)
{
    float input[2] = { _tau1, _tau4 };
    Matrix<2, 1> inputVector(input);
    return J_T_inv_ * R_inv_ * M_inv_ * inputVector; // get tp and fd
}

Matrix<2, 1> VMC::forwardVel(float _vp, float _vd)
{
    float input[2] = { _vp, _vd };
    Matrix<2, 1> inputVector(input);
    return J_inv_ * R_inv_ * inputVector; // get dphi1 and dphi4
}

Matrix<2, 1> VMC::inverseVel(float _dphi1, float _dphi4)
{
    float input[2] = { _dphi1, _dphi4 };
    Matrix<2, 1> inputVector(input);
    return J_ * R_ * inputVector; // get vp and vd
}

Matrix<2, 2> VMC::invFast(Matrix<2, 2> &_mat) const
{
    float det = (_mat[0][0] * _mat[1][1]) - (_mat[0][1] * _mat[1][0]);
    Matrix<2, 2> res;
    res[0][0] = _mat[1][1] / det;
    res[0][1] = -_mat[0][1] / det;
    res[1][0] = -_mat[1][0] / det;
    res[1][1] = _mat[0][0] / det;
    return res;
}
