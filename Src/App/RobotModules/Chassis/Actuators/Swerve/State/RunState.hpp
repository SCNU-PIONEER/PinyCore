#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class RunState : public FSMState<FSMState_e> {
public:
    RunState(Swerve *_chassis)
            : FSMState<FSMState_e>(FSMState_e::RUN), chassis_(_chassis) {};

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        Speed_u ref;

        ref.vx = chassis_->msg.vx;
        ref.vy = chassis_->msg.vy;
        // ref.wz = wzAngPid_.calc(0, PINYMOTOR::getMinorArc(chassis_->msg.yaw,
        //                                                   chassis_->yaw()));
        ref.wz = 0;
        chassis_->ctrl(ref);
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::STOP;
    }

private:
    Swerve *chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
};
} // namespace CHASSIS
