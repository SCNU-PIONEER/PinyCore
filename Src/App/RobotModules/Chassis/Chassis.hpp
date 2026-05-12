#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"


namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 0, LAUNCH, RUN, TANK, SPIN, TEST };
}
class Chassis {
public:
    ChassisMsg_s msg = {};

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
};
