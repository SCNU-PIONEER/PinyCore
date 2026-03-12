#pragma once

#include "INS.hpp"
#include "FSMState.hpp"
#include "MsgImpl.hpp"
#include "Topic.hpp"
#include "TopicRouter.hpp"

namespace GIMBAL {
static constexpr char TAG[] = "gimbal";
}

class Gimbal {
public:
    Gimbal() : insSub_(new Subscriber<INS_SYS::INSData_s>(4, &insTopic, &insData_)) {};
    GimbalMsg_s msg = {};

    const INS_SYS::INSData_s &ins() const { return insData_; }
    Subscriber<INS_SYS::INSData_s> *insSub_;

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;
    INS_SYS::INSData_s insData_ = {};
};
