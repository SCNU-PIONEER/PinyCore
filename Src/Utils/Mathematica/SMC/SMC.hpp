#pragma once

#include <cmath>
#include <cstring>

namespace SMC {

class Smc {
public:
    Smc(float _c, float _dt);
    virtual ~Smc() = default;

    float smcSimplePosCalc(float _posErr);

    virtual float reachingLaw(float _s);

protected:
    float c;
    float dt;
    float err1;
    float err2;
    float s;
    float sDot;
    float u;
    float sigmoid(float _s, float _m = 10.f) { return 2.f * ((1.f / (1.f + expf(-_m * _s))) - 0.5f); }
    float integralX1;
};

class ConstantReachSmc final : public Smc {
public:
    ConstantReachSmc(float _c, float _epsilon, float _dt);
    ~ConstantReachSmc() = default;

    float reachingLaw(float _s) override { return -epsilon * sigmoid(_s); };

private:
    float epsilon;
};

class ExponReachSmc final : public Smc {
public:
    ExponReachSmc(float _c, float _epsilon, float _k, float _dt);
    ~ExponReachSmc() = default;

    float reachingLaw(float _s) override { return -(epsilon * sigmoid(_s)) - (k * _s); };

private:
    float epsilon;
    float k;
};

class PowerReachSmc final : public Smc {
public:
    PowerReachSmc(float _c, float _k, float _alpha, float _dt);
    ~PowerReachSmc() = default;

    float reachingLaw(float _s) override { return -k * powf(fabsf(_s), alpha) * sigmoid(_s); };

private:
    float k;
    float alpha; // 0 < alpha < 1
};


} // namespace SMC