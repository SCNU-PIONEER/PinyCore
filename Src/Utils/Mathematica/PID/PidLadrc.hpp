#pragma once

#include "Pid.hpp"

class LadrcPid : public PID {
public:
    /**
     * @brief Construct a new Ladrc Pid object
     * 
     * @param _wo observation bandwidth
     * @param _wc control bandwidth
     * @param _b0 disturbance rejection control gain
     * @param _zMax state limiting
     * @param _outMax output limiting
     * @param _dt 
     *
     * @note The larger _b0 is, the worse the anti-disturbance effect will be. 
     *       Usually, the approximate value of the controlled object model is taken, 
     *       such as the gain from the control variable to the output in the motor transfer function
     */
    LadrcPid(float _wo, float _wc, float _b0, float _zMax, float _outMax,
             float _dt);

    void reset() final;
    float calc(float _ref, float _cur) final;

private:
    float kp_;
    float kd_;

    float dt_ = 0.001f;

    float outMax_;

    // leso
    float z1_;
    float z2_;
    float z3_;
    float zMax_;

    float beta1_;
    float beta2_;
    float beta3_;
    float b0_;

    void updateLeso(float _y, float _u);
};
