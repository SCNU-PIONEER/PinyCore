#pragma once

#include "Pid.hpp"
#include <array>
#include <numbers>

class FuzzyPid : public PID {
public:
    using RuleTable = std::array<std::array<int, 7>, 7>;

    FuzzyPid(float _kp, float _ki, float _kd, float _dt, float _iMax,
             float _outMax, float _deadband = 0.f,
             float _errorScale = std::numbers::pi_v<float>,
             float _errorRateScale = 2.5f * std::numbers::pi_v<float>);

    void reset() final;

    float calc(float _ref, float _cur) final;

protected:
    float iOut;
    float err[2];

private:
    float errorScale;
    float errorRateScale;

    float kp;
    float ki;
    float kd;
    float dt;
    float iMax;
    float outMax;
    float deadband;

    float fuzzyInference(float _err, float _errRate,
                         const RuleTable &_rule) const;

    float membershipCalc(float _val, float _ref) const;
};
