#include "PidFuzzy.hpp"

#include <algorithm>
#include <cmath>

// fuzzy domain hierarchical constant values, corresponding to linguistic variables
// NB, NM, NS, ZO, PS, PM, PB

// NOLINTBEGIN
inline static constexpr int NB_ = -3;
inline static constexpr int NM_ = -2;
inline static constexpr int NS_ = -1;
inline static constexpr int ZO_ = 0;
inline static constexpr int PS_ = 1;
inline static constexpr int PM_ = 2;
inline static constexpr int PB_ = 3;
// NOLINTEND

// fuzzy rule table
inline static constexpr FuzzyPid::RuleTable DELTA_KP_RULE = { {
        //NB   NM   NS   ZO   PS   PM   PB   (de →)
        { { PB_, PB_, PM_, PM_, PS_, ZO_, ZO_ } }, // NB (e ↓)
        { { PB_, PB_, PM_, PS_, PS_, ZO_, NS_ } }, // NM
        { { PM_, PM_, PM_, PS_, ZO_, NS_, NS_ } }, // NS
        { { PM_, PM_, PS_, ZO_, NS_, NM_, NM_ } }, // ZO
        { { PS_, PS_, ZO_, NS_, NS_, NM_, NM_ } }, // PS
        { { PS_, ZO_, NS_, NM_, NM_, NM_, NB_ } }, // PM
        { { ZO_, ZO_, NM_, NM_, NM_, NB_, NB_ } }  // PB
} };

inline static constexpr FuzzyPid::RuleTable DELTA_KI_RULE = { {
        //NB   NM   NS   ZO   PS   PM   PB   (de →)
        { { NB_, NB_, NM_, NM_, NS_, ZO_, ZO_ } }, // NB (e ↓)
        { { NB_, NB_, NM_, NS_, NS_, ZO_, ZO_ } }, // NM
        { { NB_, NM_, NS_, NS_, ZO_, PS_, PS_ } }, // NS
        { { NM_, NM_, NS_, ZO_, PS_, PM_, PM_ } }, // ZO
        { { NM_, NS_, ZO_, PS_, PS_, PM_, PB_ } }, // PS
        { { ZO_, ZO_, PS_, PS_, PM_, PB_, PB_ } }, // PM
        { { ZO_, ZO_, PS_, PM_, PM_, PB_, PB_ } }  // PB
} };

inline static constexpr FuzzyPid::RuleTable DELTA_KD_RULE = { {
        //NB   NM   NS   ZO   PS   PM   PB   (de →)
        { { PS_, NS_, NB_, NB_, NB_, NM_, PS_ } }, // NB (e ↓)
        { { PS_, NS_, NB_, NM_, NM_, NS_, ZO_ } }, // NM
        { { ZO_, NS_, NM_, NM_, NS_, NS_, ZO_ } }, // NS
        { { ZO_, NS_, NS_, NS_, NS_, NS_, ZO_ } }, // ZO
        { { ZO_, ZO_, ZO_, ZO_, ZO_, ZO_, ZO_ } }, // PS
        { { PB_, NS_, PS_, PS_, PS_, PS_, PB_ } }, // PM
        { { PB_, PM_, PM_, PM_, PS_, PS_, PB_ } }  // PB
} };

// fuzzy membership function parameters
inline static constexpr float MEMBERSHIPS_PARAMS[7][3] = {
    { -3, -3, -2 }, // NB
    { -2, -2, -1 }, // NM
    { -1, -1, 0 },  // NS
    { -1, 0, 1 },   // ZO
    { 0, 1, 1 },    // PS
    { 1, 2, 2 },    // PM
    { 2, 3, 3 }     // PB
};

FuzzyPid::FuzzyPid(float _kp, float _ki, float _kd, float _dt, float _iMax,
                   float _outMax, float _deadband, float _errorScale,
                   float _errorRateScale)
        : errorScale(_errorScale)
        , errorRateScale(_errorRateScale)
        , kp(_kp)
        , ki(_ki)
        , kd(_kd)
        , dt(_dt)
        , iMax(_iMax)
        , outMax(_outMax)
        , deadband(_deadband)
{
    reset();
}

float FuzzyPid::membershipCalc(float _val, float _ref) const
{
    const auto &params = MEMBERSHIPS_PARAMS[static_cast<int>(_ref)];

    // triangular membership function
    if (_val <= params[0] || _val >= params[2])
        return 0.0f;
    if (_val <= params[1])
        return (_val - params[0]) / (params[1] - params[0]);
    return (params[2] - _val) / (params[2] - params[1]);
}

float FuzzyPid::fuzzyInference(float _err, float _errRate,
                               const RuleTable &_rules) const
{
    // normalize
    float e = _err / errorScale * 3.f;
    float ec = _errRate / errorRateScale * 3.f;

    // limit
    e = std::fabs(e) > 3.0f ? (e > 0.0f ? 3.0f : -3.0f) : e;
    ec = std::fabs(ec) > 3.0f ? (ec > 0.0f ? 3.0f : -3.0f) : ec;

    // calc membership
    std::array<float, 7> eDegrees, ecDegrees;
    for (int i = 0; i < 7; ++i) {
        eDegrees[i] = membershipCalc(e, static_cast<float>(i));
        ecDegrees[i] = membershipCalc(ec, static_cast<float>(i));
    }

    // rule inference
    float fuzzyRule = 0.f, weightSum = 0.f;
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            // calc weight
            const float w = eDegrees[i] * ecDegrees[j];
            if (w <= 0.0f)
                continue;
            // update fuzzy rule
            fuzzyRule += w * static_cast<float>(_rules[i][j]);
            weightSum += w;
        }
    }

    // no zero
    if (weightSum == 0.0f)
        weightSum = 1e-6f;

    // average
    fuzzyRule /= weightSum;

    return fuzzyRule / 3.f;
}

void FuzzyPid::reset()
{
    iOut = 0.0f;
    err[0] = err[1] = 0.0f;
}

float FuzzyPid::calc(float _ref, float _cur)
{
    err[1] = err[0];
    err[0] = _ref - _cur;

    if (err[0] > deadband) {
        err[0] -= deadband;
    } else if (err[0] < -deadband) {
        err[0] += deadband;
    } else {
        if (_ref < deadband && _ref > -deadband &&
            (err[1] < -deadband || err[1] > deadband)) {
            iOut = 0.0f;
        } else
            return 0.0f;
    }

    float errRate = (err[0] - err[1]) / dt;

    float deltaKp = fuzzyInference(err[0], errRate, DELTA_KP_RULE) * kp / 2.f;
    float deltaKi = fuzzyInference(err[0], errRate, DELTA_KI_RULE) * ki / 2.f;
    float deltaKd = fuzzyInference(err[0], errRate, DELTA_KD_RULE) * kd / 2.f;

    float fuzzyKp = kp + deltaKp;
    float fuzzyKi = ki + deltaKi;
    float fuzzyKd = kd + deltaKd;

    iOut += fuzzyKi * (err[0] + err[1]) / 2.f * dt;
    iOut = std::clamp(iOut, -iMax, iMax);

    return std::clamp((fuzzyKp * err[0]) + iOut + (fuzzyKd * errRate), -outMax,
                      outMax);
}