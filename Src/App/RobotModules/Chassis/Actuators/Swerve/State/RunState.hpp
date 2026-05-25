#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

static PositionalPid gWzAP{ 15.f, 0.3f, 0.001f, 0.001f, 0.2f, CHASSIS::Swerve::MAX_WZ_SPEED, 0.12f };
static float spinWz;

namespace CHASSIS {
class RunState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.25f;

public:
    RunState(Swerve *_chassis) : FSMState<FSMState_e>(FSMState_e::RUN), chassis_(_chassis), wzAngPid_(&gWzAP) {}

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
#if !APP_USE_COMM //无头骑士
        ref.wz = chassis_->msg.wz;
#else
        float targetYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        ref.wz = wzAngPid_->calc(0, PINYMOTOR::getMinorArc(targetYaw, chassis_->deltaYaw));
        spinWz = ref.wz;

        // ref.wz = 0.f;
#endif
        chassis_->ctrl(ref);

        // Speed_u ref;
        // ref.vx = chassis_->msg.vx;
        // ref.vy = chassis_->msg.vy;
        // ref.wz = chassis_->msg.wz;
        // chassis_->fakeGimbalCtrl(ref.vy);
    }

    void exit() final
    {
        wzAngPid_->reset();
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || !chassis_->isGimbalReset)
            return FSMState_e::STOP;

        if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else
            return FSMState_e::RUN;
    }

private:
    Swerve *chassis_;
    PositionalPid *wzAngPid_;
};
} // namespace CHASSIS