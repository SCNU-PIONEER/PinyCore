#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {
class StopState : public FSMState<FSMState_e> {
public:
    StopState(Swerve &_chassis)
            : FSMState<FSMState_e>(FSMState_e::STOP), chassis_(_chassis) {};

    void enter() final
    {
        chassis_.stop();
        LOG::info("ChassisStop", " enter");
    }

    void run() final { chassis_.stop(); }

    void exit() final { LOG::info("ChassisStop", " exit"); }


    FSMState_e checkChange() final
    {
        using enum FSMState_e;
        switch (chassis_.msg.state) {
        case RUN:
            return RUN;
        default:
            return STOP;
        }
    }

private:
    Swerve &chassis_;
};
} // namespace CHASSIS
