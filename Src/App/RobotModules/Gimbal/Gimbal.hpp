#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}

class Gimbal {
public:
    Gimbal()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_))
            , gimbalYawPub_(new Publisher<GimbalYawMsg_s>(
                      &TopicRouter::instance().gimbalYawTopic, &gimbalYawMsg))
    {
    }

    const INS_SYS::INSData_s &ins() const { return insData_; }

    float endYawAng() const { return endYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

    GimbalMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg{};

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    // Master send deltaYaw to chassis
    Publisher<GimbalYawMsg_s> *gimbalYawPub_;

    float endYawAng_ = 0.f;        // end's ang in earth
    float constDeltaYawArc_ = 0.f; // end to base's arc
    bool isConfirmYawZero_ = false;

    float baseYawAng_ = 0.f; // J0's ang in earth
};
