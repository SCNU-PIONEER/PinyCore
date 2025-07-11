#include "Chassis.hpp"
#include "ChassisRunState.hpp"
#include "ChassisStopState.hpp"
#include <memory>

Chassis::Chassis(Wheel *_wheel)
{
    wheel_ = _wheel;
    stateFactory_.addState("ChassisRun",
                           std::make_unique<ChassisRunState>(this));
    stateFactory_.addState("ChassisStop",
                           std::make_unique<ChassisStopState>(this));
    stateFactory_.init(stateFactory_.getNextState("ChassisStop"));
}

void Chassis::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg_, 10) ==
        pdTRUE) {
        log.info(LOCATION, "chassis", "chassis update");
    };
    stateFactory_.update();
}
