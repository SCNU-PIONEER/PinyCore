#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"


class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

    Chassis()
            : deltaYawSub_(new Subscriber<DeltaYawMsg_s>(
                      1, &TopicRouter::instance().deltaYawTopic, &deltaYawMsg_))
    {
    }

    void loadDeltaYaw()
    {
        // Master Board method
        if (deltaYawSub_->receive()) {
            deltaYaw_ = deltaYawMsg_.deltaYaw;
        }
        // TODO: Slave Board method
    };

    ChassisMsg_s msg = {};

    StateFactory stateFactory_;

    DeltaYawMsg_s deltaYawMsg_;
    Subscriber<DeltaYawMsg_s> *deltaYawSub_;

    float deltaYaw_ = 0.f;
};
