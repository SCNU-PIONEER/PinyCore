#include "PidBasic.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

IncrementalPid::IncrementalPid(float _kp, float _ki, float _kd, float _outMax,
                               float _deadband)
        : kp(_kp), ki(_ki), kd(_kd), outMax(_outMax), deadband(_deadband)
{
    /* Derived coefficient A0 */
    A0 = kp + ki + kd;

    /* Derived coefficient A1 */
    A1 = (-kp) - ((float_t)2.0f * kd);

    /* Derived coefficient A2 */
    A2 = kd;

    /* Reset state to zero, The size will be always 3 samples */
    memset(state, 0, 3U * sizeof(float_t));
}

float IncrementalPid::calc(float _ref, float _cur)
{
    float_t delta = _ref - _cur;

    /* Check deadband */
    if (fabs(delta) <= this->deadband) {
        delta = 0.f;
    }

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    float_t out = (A0 * delta) + (A1 * state[0]) + (A2 * state[1]) + (state[2]);
    out = std::clamp(out, -outMax, outMax);

    /* Update state */
    state[1] = state[0];
    state[0] = delta;
    state[2] = out;

    /* return to application */
    return out;
}

void IncrementalPid::reset()
{
    /* Reset state to zero, The size will be always 3 samples */
    memset(state, 0, 3U * sizeof(float_t));
}


PositonalPid::PositonalPid(float _kp, float _ki, float _kd, float _dt,
                           float _iMax, float _outMax, float _deadband)
        : iOut(0.0f)
        , kp(_kp)
        , ki(_ki)
        , kd(_kd)
        , dt(_dt)
        , iMax(_iMax)
        , outMax(_outMax)
        , deadband(_deadband)
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
}

float PositonalPid::calc(float _ref, float _cur)
{
    err[1] = err[0];
    err[0] = _ref - _cur;
    if (fabsf(err[0]) <= deadband) {
        return 0.0f;
    }
    iOut += ki * err[0] * dt;
    iOut = std::clamp(iOut, -iMax, iMax);
    return std::clamp((kp * err[0]) + iOut + (kd * (err[0] - err[1]) / dt),
                      -outMax, outMax);
}

void PositonalPid::reset()
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
    iOut = 0.0f;
}
