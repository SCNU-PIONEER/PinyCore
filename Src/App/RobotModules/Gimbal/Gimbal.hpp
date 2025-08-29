#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

#include "INS.hpp"

class Gimbal {
public:
    // some constant
    Gimbal()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_))
            , deltaYawPub_(new Publisher<DeltaYawMsg_s>(
                      &TopicRouter::instance().deltaYawTopic, &deltaYawMsg_))
    {
    }

    GimbalMsg_s msg = {};

    const INS_SYS::INSData_s &ins() const { return insData_; }

    float endYawAng() const { return endYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

protected:
    StateFactory stateFactory_;

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    // Master send deltaYaw to chassis
    DeltaYawMsg_s deltaYawMsg_{};
    Publisher<DeltaYawMsg_s> *deltaYawPub_;

    float endYawAng_ = 0.f;        // end's ang in earth
    float constDeltaYawArc_ = 0.f; // end to base's arc
    bool isConfirmYawZero_ = false;

    float baseYawAng_ = 0.f; // J0's ang in earth
};
