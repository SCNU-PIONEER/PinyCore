#pragma once

#include "cmd.hpp"
#include "FSMState.hpp"
#include "stm_log.hpp"


const float S_CURVE_VX_ACC = 1.4f;
const float S_CURVE_VY_ACC = 2.2f;
const float S_CURVE_WZ_ACC = 2.8f;

const float MAX_VX_SPEED = 2.f;
const float MAX_VY_SPEED = 2.f;
const float MAX_WZ_SPEED = 3.0f;

const float MOUSE_CAL = 200.0f;

class Chassis : public IObserver {
public:
    Chassis();

    void update(Msg& _msg);
    void task();


    StateFactory stateFactory_;

    LOG::Logger &log = LOG::Logger::instance();

    Msg msg;

};
