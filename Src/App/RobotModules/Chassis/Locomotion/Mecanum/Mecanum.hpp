#pragma once

#include <cmath>
#include "Locomotion.hpp"

namespace CHASSIS {

union WheelsSpeed_u {
    struct {
        float M_RF; // motor of the right front
        float M_LF; // motor of the left front
        float M_LB; // motor of the left back
        float M_RB; // motor of the right back
    };
    float _[4]; // rpm
};

class Mecanum : public Locomotion {
public:
    static constexpr float W_DIAMETER = 0.1525f;
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float FRONT_R = 0.354f;
    static constexpr float BACK_R = 0.354f;
    Mecanum();

    void stop() override;
    void enter() override;
    void update() override;
    void ctrl(const Speed_u &_refSpeed) override;

protected:
    /*
     * @brief: calculate the speed of chassis
     * @param: _wSpeed: speed of each wheel
     * @return: speed of the chassis in m/s and rad/s
    */
    Speed_u forward(const WheelsSpeed_u &_wSpeed);

    /*
     * @brief: calculate the speed of each wheel
     * @param: _speed: speed of the chassis in m/s and rad/s
     * @return: speed of each wheel
    */
    WheelsSpeed_u reverse(const Speed_u &_speed);

private:
    PINYMOTOR::IMotor *motor_[4];
    WheelsSpeed_u wSpeed_;
};

} //namespace CHASSIS
