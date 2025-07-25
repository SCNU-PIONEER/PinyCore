#pragma once

#include "Locomotion.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, START, RUN };
}

class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

    Chassis(CHASSIS::Locomotion *_wheel);

    void update(void *_param);

    StateFactory stateFactory;

    LOG::Logger &log = LOG::Logger::instance();

    chassisMsg msg = {};

    CHASSIS::Locomotion *wheel;
};
