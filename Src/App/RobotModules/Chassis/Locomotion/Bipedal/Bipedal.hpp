#pragma once
#include "Locomotion.hpp"

#include "WheelLegged.hpp"
#include "BalanceController.hpp"
#include "BalanceControllerEX.hpp"

namespace CHASSIS::BIPEDAL {

enum class Posture_e : uint8_t {
    NONE, // Balanced
    TUMBLE,
    SIT
};

class Bipedal {
public:
    Bipedal();

    void stop();
    void enter();
    void update();

    void setL(float _ll, float _lr);
    void setL(float _l);

    void setRoll(float _roll);
    void setPitch(float _pitch);
    void setYaw(float _yaw);

    void ctrl(States_u _ref);
    void ctrl(StatesEX_u _ref);
    void ctrl(float _vx);

private:
    PINYMOTOR::IMotor *motor_[6];

    BalanceController balCtrlL;
    BalanceController balCtrlR;
    BalanceControllerEX balCtrlEX;

    WheelLegged legL{ motor_[1], motor_[2], motor_[0] };
    WheelLegged legR{ motor_[4], motor_[5], motor_[3] };

    float roll_;
    float pitch_;
    float yaw_;
};

} // namespace CHASSIS::BIPEDAL
