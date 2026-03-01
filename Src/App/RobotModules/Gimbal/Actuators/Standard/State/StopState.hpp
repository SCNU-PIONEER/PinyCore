#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
namespace GIMBAL {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Standard &_gimbal) : FSMState<FSMState_e>(FSMState_e::STOP), gimbal_(_gimbal) {}

    void enter() final { LOG::info("GimbalStop", "enter"); }

    void run() final { gimbal_.stop(); }

    void exit() final { LOG::info("GimbalStop", "exit"); }

    FSMState_e checkChange() final
    {
        using enum FSMState_e;
        switch (gimbal_.msg.state) {
        case RUN:
            return RUN;
        default:
            return STOP;
        }
    }

private:
    Standard &gimbal_;
};

} // namespace GIMBAL
