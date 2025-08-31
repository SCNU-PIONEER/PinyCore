#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

#include "MotorCommonMacros.hpp"

#include "dsp/fast_math_functions.h"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}
class Chassis {
public:
    Chassis()
            : gimbalYawSub_(new Subscriber<GimbalYawMsg_s>(
                      1, &TopicRouter::instance().gimbalYawTopic,
                      &gimbalYawMsg))
    {
    }

    void loadDeltaYaw()
    {
        // Master Board method
        if (gimbalYawSub_->receive()) {
            deltaYaw = PINYMOTOR::getMinorArc(gimbalYawMsg.followYaw,
                                              gimbalYawMsg.chassisYaw);
        }
        // TODO: Slave Board method
        // gimbalYawMsg.chassisYaw came from Chassis Board's ins
        cosYaw_ = arm_cos_f32(deltaYaw);
        sinYaw_ = arm_sin_f32(deltaYaw);
    };

    ChassisMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg;

    float deltaYaw = 0.f;

protected:
    StateFactory<CHASSIS::FSMState_e> stateFactory_;

    Subscriber<GimbalYawMsg_s> *gimbalYawSub_;

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
};