#pragma once

#include "pid.hpp"


class PidBasic : public Pid {
public:
    /**
    * @brief  init pid parameter
    */
    void init(float kp, float ki, float kd, float i_max, float out_max,
              float deadband);

    /**
    * @brief  pid reset
    */
    void reset() override;

    /**
    * @brief  pid calculation
    */
    float calc(float ref, float cur) override;

    /**
    * @brief  pid calculation with deadband
    */
    float calcDeadband(float ref, float cur);

protected:
    template <typename T> T limitMinMax(T x, T min_val, T max_val)
    {
        return (min_val < max_val) ?
                       (x <= min_val ? min_val : (x >= max_val ? max_val : x)) :
                       (x <= max_val ? max_val : (x >= min_val ? min_val : x));
    }

private:
    float err[2];

    float kp, ki, kd;
    float i_max, out_max;
    float p_out, i_out, d_out;

    float k_deadband;
    int deadband_zero_output;
};
