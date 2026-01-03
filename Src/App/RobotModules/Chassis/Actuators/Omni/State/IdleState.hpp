#pragma once
#include "Omni.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"

#include "MotorCommonMacros.hpp"

static PositionalPid idleFollowVelPid{ 7.f, 1.f, 0.10f, 0.001f, 0.1f, 6.f, 0.0f };

namespace CHASSIS {

class IdleState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;

public:
    IdleState(Omni *_chassis) : FSMState<FSMState_e>(FSMState_e::IDLE), chassis_(_chassis), wzAngPid_(&idleFollowVelPid)
    {
    }

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.vx = std::fmax(chassis_->getSpeed().vx - ACC, std::fmin(chassis_->msg.vx, chassis_->getSpeed().vx + ACC));

        ref.vy = std::fmax(chassis_->getSpeed().vy - ACC, std::fmin(chassis_->msg.vy, chassis_->getSpeed().vy + ACC));
        float targetYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        ref.wz = wzAngPid_->calc(0, PINYMOTOR::getMinorArc(chassis_->deltaYaw, targetYaw));
        // ref.wz = 0;
        chassis_->ctrl(ref);
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else
            return FSMState_e::IDLE;
    }

private:
    Omni *chassis_;
    PositionalPid *wzAngPid_;
};
} // namespace CHASSIS
