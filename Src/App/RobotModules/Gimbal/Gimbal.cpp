#include "Gimbal.hpp"

#include "GimbalStopState.hpp"
// #include "GimbalRunState.hpp"
// #include "GimbalLaunchState.hpp"

// #include "GimbalMotorTestState.hpp"

using namespace GIMBAL;

Gimbal::Gimbal(Head *_head) : head(_head)
{
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::STOP),
                          std::make_unique<StopState>(this));

    // stateFactory.addState(static_cast<uint8_t>(FSMState_e::LAUNCH),
    //                       std::make_unique<LaunchState>(this));

    // stateFactory.addState(static_cast<uint8_t>(FSMState_e::RUN),
    //                       std::make_unique<RunState>(this));

    // stateFactory.addState(static_cast<uint8_t>(FSMState_e::MOTOR_TEST),
    //                       std::make_unique<MotorTestState>(this));
    stateFactory.init(
            stateFactory.getNextState(static_cast<uint8_t>(FSMState_e::STOP)));
}

void Gimbal::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };
    head->update();
    stateFactory.update();
}
