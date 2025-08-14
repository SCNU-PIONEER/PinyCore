#pragma once

#include "Head.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN, MOTOR_TEST };

union States_u {
    struct {
        float pos;
        float vel;
    };
    float _[2];
};
} // namespace GIMBAL

class Gimbal {
public:
    static constexpr float MAX_WPITCH_SPEED = 3.14f;
    static constexpr float MAX_WYAW_SPEED = 3.14f;

    Gimbal(GIMBAL::Head *_head);

    void update(void *_param);

    StateFactory stateFactory;

    GIMBAL::Head *head;

    gimbalMsg msg = {};

protected:
    LOG::Logger &log = LOG::Logger::instance();
};
