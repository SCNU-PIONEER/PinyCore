#include "pidBasic.hpp"

void PIDBasic::init(float _kp, float _ki, float _kd, float _i_max, float _out_max,
               float _deadband)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
    i_max = _i_max;
    out_max = _out_max;
    k_deadband = _deadband;
}

void PIDBasic::reset()
{
    err[1] = err[0] = 0.0f;
    p_out = 0.0f;
    i_out = 0.0f;
    d_out = 0.0f;
}

float PIDBasic::calc(float ref, float cur)
{
    float output = 0;
    err[1] = err[0];
    err[0] = ref - cur;
    if (ki == 0.f)
        i_out = 0.0f;

    p_out = kp * err[0];
    i_out += ki * err[0];
    d_out = kd * (err[0] - err[1]);
    limitMinMax(i_out, -i_max, i_max);

    output = p_out + i_out + d_out;
    limitMinMax(output, -out_max, out_max);
    return output;
}

float PIDBasic::calcDeadband(float ref, float cur)
{
    float output;
    err[1] = err[0];

    err[0] = ref - cur;
    if (err[0] > k_deadband) {
        err[0] -= k_deadband;
    } else if (err[0] < -k_deadband) {
        err[0] += k_deadband;
    } else {
        if (ref < k_deadband && ref > -k_deadband &&
            (err[1] < -k_deadband || err[1] > k_deadband)) {
            i_out = 0.0f;
        }
        if (deadband_zero_output)
            return 0.0f;
    }

    p_out = kp * err[0];
    i_out += ki * err[0];
    d_out = kd * (err[0] - err[1]);
    limitMinMax(i_out, -i_max, i_max);

    output = p_out + i_out + d_out;
    limitMinMax(output, -out_max, out_max);
    return output;
}
