#include "Chassis.hpp"
#include "pidBasic.hpp"
#include "RunState.hpp"
#include "StopState.hpp"

Chassis::Chassis()
{
    stateFactory_.addState("run", std::make_unique<RunState>(this));
    stateFactory_.addState("stop", std::make_unique<StopState>(this));
    stateFactory_.init(stateFactory_.getNextState("stop"));
}

void Chassis::update(Msg& _msg)
{
    msg = _msg;
    stateFactory_.update();
}
