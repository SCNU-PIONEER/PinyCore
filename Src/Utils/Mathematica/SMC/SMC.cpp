#include "SMC.hpp"
#include <utility>

using namespace SMC;

Smc::Smc(float _c, float _dt) : c(_c), dt(_dt), err1(0.f), err2(0.f), s(0.f), sDot(0.f), u(0.f) {}

float Smc::smcSimplePosCalc(float _posErr)
{
    float err1Prev = std::exchange(err1, _posErr);
    err2 = (err1 - err1Prev) / dt;
    s = (c * err1) + err2;
    sDot = reachingLaw(s);
    u = (c * err2) - sDot;

    return u;
}

ConstantReachSmc::ConstantReachSmc(float _c, float _epsilon, float _dt) : Smc(_c, _dt), epsilon(_epsilon) {}

ExponReachSmc::ExponReachSmc(float _c, float _epsilon, float _k, float _dt) : Smc(_c, _dt), epsilon(_epsilon), k(_k) {}

PowerReachSmc::PowerReachSmc(float _c, float _epsilon, float _alpha, float _dt)
        : Smc(_c, _dt), epsilon(_epsilon), alpha(_alpha)
{
}
