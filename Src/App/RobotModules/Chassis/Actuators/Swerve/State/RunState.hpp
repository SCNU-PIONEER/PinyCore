#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS::SWERVE {
class RunState : public FSMState {
public:
    RunState(Swerve *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    }

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        SWERVE::Speed_u ref;

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


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == FSMState_e::RUN)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Swerve *chassis_;
    PositonalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
};
} // namespace CHASSIS::SWERVE