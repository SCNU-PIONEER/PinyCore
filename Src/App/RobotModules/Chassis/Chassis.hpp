#pragma once

#include "Wheel.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"

class Chassis {
protected:
    static constexpr float S_CURVE_VX_ACC = 1.4f;
    static constexpr float S_CURVE_VY_ACC = 2.2f;
    static constexpr float S_CURVE_WZ_ACC = 2.8f;
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

public:
    Chassis(Wheel *_wheel);

    void update(void *_param);

    StateFactory stateFactory_;

    LOG::Logger &log = LOG::Logger::instance();

    chassisMsg msg_;

    Wheel *wheel_;
};
