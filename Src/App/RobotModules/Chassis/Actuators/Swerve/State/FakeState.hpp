#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {
//让你的舵轮像云台一样旋转
class FakeState : public FSMState<FSMState_e> {
public:
    FakeState(Swerve *_chassis) : FSMState<FSMState_e>(FSMState_e::FAKE), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisFake", " enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.vx = chassis_->msg.vx;
        ref.vy = chassis_->msg.vy;
        ref.wz = chassis_->msg.wz;
        chassis_->fakeGimbalCtrl(ref.vy);
    }

    void exit() final { LOG::info("ChassisFake", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || !chassis_->isGimbalReset)
            return FSMState_e::STOP;

        if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::FAKE;
    }

private:
    Swerve *chassis_;
};
} // namespace CHASSIS