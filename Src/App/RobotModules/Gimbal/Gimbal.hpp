#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
inline const char *tag = "gimbal";
}

class Gimbal {
public:
    GimbalMsg_s msg = {};

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;
};
