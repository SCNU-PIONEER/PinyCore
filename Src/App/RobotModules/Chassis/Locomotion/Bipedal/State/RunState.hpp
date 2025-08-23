#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "CircleFunc.h"

#include "PidBasic.hpp"

namespace CHASSIS::BIPEDAL {
class RunState : public FSMState {
public:
    RunState(Bipedal *_chassis) : chassis_(_chassis)
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
        // WL
        // WLController::States_u ref{};
        // ref.ds = chassis_->msg.vx;
        // chassis_->ctrlBal(ref, minorArc(chassis_->msg.yaw,
        //                                           chassis_->yaw()) /
        //                                0.001f);

        // WBR
        WBRController::States_u ref{};
        ref.ds = chassis_->msg.vx;
        ref.yaw = chassis_->msg.yaw;
        ref.dyaw =
                wzAngPid_.calc(0, minorArc(chassis_->msg.yaw, chassis_->yaw()));
        chassis_->ctrlBal(ref);
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisRun", " exit");
    }


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || chassis_->isTumble())
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == FSMState_e::RUN)
            return static_cast<uint8_t>(FSMState_e::RUN);
        // TODO: the other state
        else
            return 0;
    }

private:
    Bipedal *chassis_;
    PositonalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
};

} // namespace CHASSIS::BIPEDAL
