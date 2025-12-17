#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

#include "MotorCommonMacros.hpp"

#include "dsp/fast_math_functions.h"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN, TEST };
}
class Chassis {
public:
    Chassis() : gimbalYawSub_(new Subscriber<GimbalYawMsg_s>(1, &TopicRouter::instance().gimbalYawTopic, &gimbalYawMsg))
    {
    }

    ChassisMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg;

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;
    Subscriber<GimbalYawMsg_s> *gimbalYawSub_;
};