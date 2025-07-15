#pragma once

#include <cmath>
#include "Pid.hpp"
#include "Wheel.hpp"


union WheelsState_u {
    struct {
        float M_RF; ///< motor of the right front 0
        float M_LF; ///< motor of the left front 1
        float M_LB; ///< motor of the left back 2
        float M_RB; ///< motor of the right back 3
    };
    float motors[4];
};

class Mecanum : public Wheel {
public:
    Mecanum();
    Mecanum(float diameter, float kxyFront, float kxyBack);

    void stop() override;
    void update() override;
    void ctrl(ChassisState_s _refState) override;

protected:
    void forward(WheelsState_u _refState);
    WheelsState_u reverse(ChassisState_s _refState);
    void iir3speed(WheelsState_u _rawSpeed);

private:
    float diameter = 0.1525;
    float circumference = (M_PI * diameter);
    float kxyFront = 0.354;
    float kxyBack = 0.354;

    PID *wheelPID_[4];
    WheelsState_u currentWheels;

    //TODO: add motor;
};
