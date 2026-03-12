#pragma once
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

static PositionalPid gWzAP{ 10.f, 0.3f, 0.001f, 0.001f, 0.2f, CHASSIS::Swerve::MAX_WZ_SPEED, 0.1f };
static float spinWz;

namespace CHASSIS {
class RunState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;

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
        ref.Speed.vx = std::fmax(chassis_->getSpeed().Speed.vx - ACC,
                                 std::fmin(chassis_->msg.vx, chassis_->getSpeed().Speed.vx + ACC));

        ref.Speed.vy = std::fmax(chassis_->getSpeed().Speed.vy - ACC,
                                 std::fmin(chassis_->msg.vy, chassis_->getSpeed().Speed.vy + ACC));
        float targetYaw = (fabs(chassis_->deltaYaw) > PI / 2.f) ? PI : 0.f;
        ref.Speed.wz = wzAngPid_->calc(0, PINYMOTOR::getMinorArc(targetYaw, chassis_->deltaYaw));

        spinWz = ref.Speed.wz;
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
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else
            return FSMState_e::STOP;
    }

private:
    Swerve *chassis_;
    PositionalPid *wzAngPid_;
};
} // namespace CHASSIS