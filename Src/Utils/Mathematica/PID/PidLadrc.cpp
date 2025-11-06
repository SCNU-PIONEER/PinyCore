#include "PidLadrc.hpp"
#include <algorithm>

LadrcPid::LadrcPid(float _wo, float _wc, float _b0, float _zMax, float _outMax,
                   float _dt)
        : kp_(_wc * _wc)
        , kd_(2 * _wc)
        , dt_(_dt)
        , outMax_(_outMax)
        , zMax_(_zMax)
        , beta1_(3.f * _wo)
        , beta2_(3.f * _wo * _wo)
        , beta3_(_wo * _wo * _wo)
        , b0_(_b0)
{
    reset();
}

void LadrcPid::reset() { z1_ = z2_ = z3_ = 0; }

float LadrcPid::calc(float _ref, float _cur)
{
    float u0 = (kp_ * (_ref - z1_)) - (kd_ * z2_);
    float u = (u0 - z3_) / b0_;
    u = std::clamp(u, -outMax_, outMax_);
    updateLeso(_cur, u);
    return u;
}

void LadrcPid::updateLeso(float _y, float _u)
{
    float err = _y - z1_;
    float dz1 = z2_ + (beta1_ * err);
    float dz2 = z3_ + (beta2_ * err) + (b0_ * _u);
    float dz3 = beta3_ * err;
    z1_ += dz1 * dt_;
    z2_ += dz2 * dt_;
    z3_ += dz3 * dt_;
    z1_ = std::clamp(z1_, -zMax_, zMax_);
    z2_ = std::clamp(z2_, -zMax_, zMax_);
    z3_ = std::clamp(z3_, -zMax_, zMax_);
}
