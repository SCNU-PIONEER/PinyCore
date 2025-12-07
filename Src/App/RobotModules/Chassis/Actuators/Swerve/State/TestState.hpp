#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class TestState : public FSMState<FSMState_e> {
public:
    TestState(Swerve *_chassis)
            : FSMState<FSMState_e>(FSMState_e::TEST), chassis_(_chassis)
    {
    }

    void enter() final { LOG::info("ChassisTest", " enter"); }

    void run() final
    {
        Speed_u ref;

        ref.vx = 0.5f;
        chassis_->ctrl(ref);
    }

    void exit() final { LOG::info("ChassisTest", " exit"); }


    FSMState_e checkChange() final { return FSMState_e::TEST; }

private:
    Swerve *chassis_;
};
} // namespace CHASSIS