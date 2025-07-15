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

enum class mode_e { OMNI_X = 0, OMNI_H = 1 };

class Omni : public Wheel {
public:
    Omni();

    Omni(float diameter, float kxyFront, float kxyBack);

    void stop() override;
    void update() override;

    void forward(WheelsState_u _refState);
    WheelsState_u reverse(ChassisState_s _refState);
    void iir3speed(WheelsState_u _rawSpeed);
    void ctrl(ChassisState_s _refState) override;

private:
    float diameter = 0.1525;
    float circumference = (M_PI * diameter);
    float kxyFront = 0.354;
    float kxyBack = 0.354;

    mode_e mode = mode_e::OMNI_X;

    PID *wheelPID_[4];
    WheelsState_u currentWheels;

    //TODO: add motor;
};
