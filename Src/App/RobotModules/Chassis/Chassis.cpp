#include "Chassis.hpp"
#include "chassisRunState.hpp"
#include "chassisStopState.hpp"

Chassis::Chassis(Wheel *_wheel)
{
    wheel_ = _wheel;
    stateFactory_.addState("run", std::make_unique<RunState>(this));
    stateFactory_.addState("stop", std::make_unique<StopState>(this));
    stateFactory_.init(stateFactory_.getNextState("stop"));
}

void Chassis::getMsg(Msg &_msg) { msg_ = static_cast<chassisMsg &>(_msg); }

void Chassis::update() { stateFactory_.update(); }
