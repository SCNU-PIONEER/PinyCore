#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN, SPIN };
}
class Chassis {
public:
    ChassisMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg;

    float deltaYaw = 0.f;

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
};
