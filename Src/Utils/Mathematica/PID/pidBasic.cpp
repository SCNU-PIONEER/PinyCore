#include "pidBasic.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>


incrementalPid::incrementalPid(incrementalPid_s &_pid) : pid_(_pid)
{
    /* Derived coefficient A0 */
    A0 = pid_.Kp + pid_.Ki + pid_.Kd;

    /* Derived coefficient A1 */
    A1 = (-pid_.Kp) - ((float_t)2.0f * pid_.Kd);

    /* Derived coefficient A2 */
    A2 = pid_.Kd;

    /* Reset state to zero, The size will be always 3 samples */
    memset(state, 0, 3U * sizeof(float_t));
}

float incrementalPid::calc(float ref, float cur)
{
    float_t delta = ref - cur;

    /* Check deadband */
    if (fabs(delta) <= this->pid_.deadband) {
        delta = 0.f;
    }

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    float_t out = (A0 * delta) + (A1 * state[0]) + (A2 * state[1]) + (state[2]);
    out = std::clamp(out, -pid_.outMax, pid_.outMax);

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


positonalPid::positonalPid(positonalPid_s &_pid) : pid_(_pid)
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
    iOut = 0.0f;
}

float positonalPid::calc(float ref, float cur)
{
    err[1] = err[0];
    err[0] = ref - cur;
    if (fabs(err[0]) <= pid_.deadband) {
        return 0.0f;
    }
    iOut += pid_.ki * err[0] * pid_.dt;
    iOut = std::clamp(iOut, -pid_.iMax, pid_.iMax);
    return std::clamp(
            (pid_.kp * err[0] + iOut + pid_.kd * (err[0] - err[1]) / pid_.dt),
            -pid_.outMax, pid_.outMax);
}

void positonalPid::reset()
{
    /* Reset state to zero */
    memset(err, 0, 2U * sizeof(float_t));
    iOut = 0.0f;
}
