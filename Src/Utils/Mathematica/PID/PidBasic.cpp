#include "PidBasic.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

incrementalPid::incrementalPid(float _Kp, float _Ki, float _Kd, float _outMax,
                               float _deadband)
        : Kp(_Kp), Ki(_Ki), Kd(_Kd), outMax(_outMax), deadband(_deadband)
{
    /* Derived coefficient A0 */
    A0 = Kp + Ki + Kd;

    /* Derived coefficient A1 */
    A1 = (-Kp) - ((float_t)2.0f * Kd);

    /* Derived coefficient A2 */
    A2 = Kd;

    /* Reset state to zero, The size will be always 3 samples */
    memset(state, 0, 3U * sizeof(float_t));
}

float incrementalPid::calc(float ref, float cur)
{
    float_t delta = ref - cur;

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

void incrementalPid::reset()
{
    /* Reset state to zero, The size will be always 3 samples */
    memset(state, 0, 3U * sizeof(float_t));
}


positonalPid::positonalPid(float _Kp, float _Ki, float _Kd, float _dt,
                           float _iMax, float _outMax, float _deadband)
        : kp(_Kp), ki(_Ki), kd(_Kd), dt(_dt), iMax(_iMax), outMax(_outMax)
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
    iOut = 0.0f;
}

float positonalPid::calc(float ref, float cur)
{
    err[1] = err[0];
    err[0] = ref - cur;
    if (fabs(err[0]) <= deadband) {
        return 0.0f;
    }
    iOut += ki * err[0] * dt;
    iOut = std::clamp(iOut, -iMax, iMax);
    return std::clamp((kp * err[0] + iOut + kd * (err[0] - err[1]) / dt),
                      -outMax, outMax);
}

void positonalPid::reset()
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
    iOut = 0.0f;
}
