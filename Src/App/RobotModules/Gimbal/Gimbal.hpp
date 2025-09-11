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
    {
    }

    const INS_SYS::INSData_s &ins() const { return insData_; }

    GimbalMsg_s msg = {};

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;
};
