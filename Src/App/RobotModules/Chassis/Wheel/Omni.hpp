#pragma once

#include <cmath>
#include "Wheel.hpp"

namespace CHASSIS {

union wheelsSpeed_u {
    struct {
        float M_RF; ///< motor of the right front 0
        float M_LF; ///< motor of the left front 1
        float M_LB; ///< motor of the left back 2
        float M_RB; ///< motor of the right back 3
    };
    float _[4]; // rpm
};

enum class mode_e { OMNI_X = 0, OMNI_H = 1 };

class Omni : public Wheel {
public:
    Omni();

    Omni(float diameter, float kxyFront, float kxyBack);

    void stop() override;
    void update() override;
    void ctrl(const speed_u &_speed) override;

protected:
    /*
     * @brief: calculate the speed of chassis
     * @param: _wSpeed: speed of each wheel
     * @return: speed of the chassis in m/s and rad/s
    */
    speed_u forward(const wheelsSpeed_u &_wSpeed);

    /*
     * @brief: calculate the speed of each wheel
     * @param: _speed: speed of the chassis in m/s and rad/s
     * @return: speed of each wheel
    */
    wheelsSpeed_u reverse(const speed_u &_speed);

private:
    float diameter = 0.1525;
    float circumference = (M_PI * diameter);
    float kxyFront = 0.354;
    float kxyBack = 0.354;

    mode_e mode = mode_e::OMNI_X;

    PINYMOTOR::IMotor *motor[4];
    wheelsSpeed_u wSpeed_;
};

}
