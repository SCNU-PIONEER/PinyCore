#pragma once

#include <cmath>
#include "Wheel.hpp"
#include <cstdint>

namespace CHASSIS {

union WheelsSpeed_u {
    struct {
        float M_RF; ///< motor of the right front 0
        float M_LF; ///< motor of the left front 1
        float M_LB; ///< motor of the left back 2
        float M_RB; ///< motor of the right back 3
    };
    float _[4]; // rpm
};

enum class Mode_e : uint8_t { OMNI_X = 0, OMNI_H = 1 };

class Omni : public Wheel {
public:
    static constexpr float W_DIAMETER = 0.1525f;
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float FRONT_R = 0.354f;
    static constexpr float BACK_R = 0.354f;

    Omni();

    void stop() override;
    void update() override;
    void ctrl(const Speed_u &_speed) override;

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
    Mode_e mode = Mode_e::OMNI_X;

    PINYMOTOR::IMotor *motor[4];
    WheelsSpeed_u wSpeed_;
};

} // namespace CHASSIS
