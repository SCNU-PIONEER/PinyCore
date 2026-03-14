#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

namespace CHASSIS {
class TANKState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;

public:
    TANKState(Swerve *_chassis) : FSMState<FSMState_e>(FSMState_e::TANK), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisTANK", " enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.Speed.vx = std::fmax(chassis_->getSpeed().Speed.vx - ACC,
                                 std::fmin(chassis_->msg.vx, chassis_->getSpeed().Speed.vx + ACC));

        ref.Speed.vy = std::fmax(chassis_->getSpeed().Speed.vy - ACC,
                                 std::fmin(chassis_->msg.vy, chassis_->getSpeed().Speed.vy + ACC));
        ref.Speed.wz = std::fmax(chassis_->getSpeed().Speed.wz - ACC,
                                 std::fmin(chassis_->msg.wz, chassis_->getSpeed().Speed.wz + ACC));
        chassis_->ctrl(ref);
    }

    void exit() final
    {
        chassis_->isGimbalReset = false;
        LOG::info("ChassisTANK", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || !chassis_->isGimbalReset)
            return FSMState_e::STOP;

        if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else
            return FSMState_e::TANK;
    }

private:
    Swerve *chassis_;
};
} // namespace CHASSIS