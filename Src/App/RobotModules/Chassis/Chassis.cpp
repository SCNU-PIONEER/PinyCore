#include "Chassis.hpp"
#include "ChassisRunState.hpp"
#include "ChassisStopState.hpp"
#include <memory>

using namespace CHASSIS;

Chassis::Chassis(Wheel *_wheel)
{
    wheel_ = _wheel;
    memset(&msg_, 0, sizeof(chassisMsg));
    stateFactory_.addState(static_cast<uint8_t>(fsmState_e::RUN),
                           std::make_unique<RunState>(this));
    stateFactory_.addState(static_cast<uint8_t>(fsmState_e::STOP),
                           std::make_unique<StopState>(this));
    stateFactory_.init(
            stateFactory_.getNextState(static_cast<uint8_t>(fsmState_e::STOP)));
}

void Chassis::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg_, 0) ==
        pdTRUE) {
        log.info(LOCATION, "chassis", "chassis update");
    };
    stateFactory_.update();
}
