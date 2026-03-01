#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

namespace GIMBAL {
inline const char *tag = "gimbal";
}

class Gimbal {
public:
    GimbalMsg_s msg = {};

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;
};
