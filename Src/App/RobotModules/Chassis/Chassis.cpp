#include "Chassis.hpp"
#include "ChassisRunState.hpp"
#include "ChassisStopState.hpp"
#include <memory>

using namespace CHASSIS;

Chassis::Chassis(Locomotion *_wheel) : wheel(_wheel)
{
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::RUN),
                          std::make_unique<RunState>(this));
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::STOP),
                          std::make_unique<StopState>(this));
    stateFactory.init(
            stateFactory.getNextState(static_cast<uint8_t>(FSMState_e::STOP)));
}

void Chassis::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
        pdTRUE) {
    };
    stateFactory.update();
}
