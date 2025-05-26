#pragma once

#include <cmath>
#include "pid.hpp"
#include "wheel.hpp"


struct WheelsState_s {
    float M_RF; ///< motor of the right front
    float M_LF; ///< motor of the left front
    float M_LB; ///< motor of the left back
    float M_RB; ///< motor of the right back
};

class Mecanum : public Wheel {
public:
    Mecanum();
    Mecanum(float diameter, float kxyFront, float kxyBack);

    void stop() override;
    void update() override;
    void ctrl(ChassisState_s _refState) override;

protected:
    void forward(WheelsState_s _refState);
    WheelsState_s reverse(ChassisState_s _refState);
    void iir3speed(WheelsState_s _rawSpeed);

private:
    float diameter = 0.1525;
    float circumference = (M_PI * diameter);
    float kxyFront = 0.354;
    float kxyBack = 0.354;

    Pid *wheelPid_[4];
    WheelsState_s currentWheels;

    //TODO: add motor;
};
