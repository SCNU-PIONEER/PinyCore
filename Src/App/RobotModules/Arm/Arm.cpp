#include <memory>

#include "./Arm.hpp"
#include "./ArmMotor.hpp"
#include "StmLog.hpp"
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "Cmd.hpp"

using namespace ARM;

Arm::Arm()
{
    /* FSM */
    stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(*this));
    stateFactory_.addState(FSMState_e::NORMAL, std::make_unique<NormalState>(*this));
    stateFactory_.init(stateFactory_.getNextState(FSMState_e::STOP));

    LOG::info("ARM", "register");

    motors.stop();
}

void Arm::update(MsgBus_s *_param)
{
    if (xQueueReceive(_param->armQueue, &msg, 0) == pdTRUE) {
        uint32_t i = 0;
    }
    motors.update();
    stateFactory_.update();
}
