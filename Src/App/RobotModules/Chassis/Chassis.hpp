#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 18.f;
    static constexpr float MAX_VY_SPEED = 18.f;
    static constexpr float MAX_WZ_SPEED = 30.f;
    static constexpr float MOUSE_CAL = 200.f;

    ChassisMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg;

    float deltaYaw = 0.f;

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
};
