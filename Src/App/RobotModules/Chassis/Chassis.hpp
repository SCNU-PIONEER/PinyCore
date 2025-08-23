#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"

class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

    StateFactory stateFactory;

    LOG::Logger &log = LOG::Logger::instance();

    ChassisMsg_s msg = {};
};
