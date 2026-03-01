#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 3.f; // m/s
    static constexpr float MAX_VY_SPEED = 3.f; // m/s
    static constexpr float MAX_WZ_SPEED = 5.f; // rad/s

    ChassisMsg_s msg = {};

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
};
