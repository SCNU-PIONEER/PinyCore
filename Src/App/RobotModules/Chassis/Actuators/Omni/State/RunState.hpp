#pragma once
#include "Omni.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

#include "PidBasic.hpp"

static PositionalPid gWzAP{ 0.6f,   0.3f, 0.01f,
                            0.001f, 0.2f, CHASSIS::Omni::MAX_WZ_SPEED,
                            0.f };

namespace CHASSIS {

class RunState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;

public:
    RunState(Omni *_chassis)
            : FSMState<FSMState_e>(FSMState_e::RUN)
            , chassis_(_chassis)
            , wzAngPid_(&gWzAP)
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

        ref.vx = std::fmax(chassis_->getSpeed().vx - ACC,
                           std::fmin(chassis_->msg.vx,
                                     chassis_->getSpeed().vx + ACC));

        ref.vy = std::fmax(chassis_->getSpeed().vy - ACC,
                           std::fmin(chassis_->msg.vy,
                                     chassis_->getSpeed().vy + ACC));

        ref.wz = wzAngPid_->calc(0, chassis_->deltaYaw);
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
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::RUN;
    }

private:
    Omni *chassis_;
    PositionalPid *wzAngPid_;
};
} // namespace CHASSIS
