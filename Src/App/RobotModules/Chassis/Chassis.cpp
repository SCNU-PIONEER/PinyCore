#include "Chassis.hpp"
#include "RunState.hpp"
#include "StopState.hpp"

Chassis::Chassis(Wheel *_wheel)
{
    wheel_ = _wheel;
    stateFactory_.addState("run", std::make_unique<RunState>(this));
    stateFactory_.addState("stop", std::make_unique<StopState>(this));
    stateFactory_.init(stateFactory_.getNextState("stop"));
}

void Chassis::update(Msg &_msg)
{
    msg_ = _msg;
    stateFactory_.update();
}
