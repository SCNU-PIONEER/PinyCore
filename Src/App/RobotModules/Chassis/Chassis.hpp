#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}
class Chassis {
public:
    ChassisMsg_s msg = {};

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
};
