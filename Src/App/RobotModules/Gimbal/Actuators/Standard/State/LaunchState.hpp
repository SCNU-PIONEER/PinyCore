#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"
#include "MotorCommonMacros.hpp"
#include "LKMotorRS485.hpp"

float delta = 0.f;
namespace GIMBAL {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr char TAG[] = "LaunchState";

public:
    LaunchState(Standard &_gimbal) : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal) {}
    void enter() final
    {
        // 零点设置
        // auto motor = (PINYMOTOR::LKMOTOR::LKMotorRS485 *)gimbal_.motor();
        // motor->setZeroPos();
        gimbal_.isLaunched = false;
        gimbal_.enter();
        LOG::info(TAG, "enter");
    }

    void run() final
    {
        if (std::fabs(gimbal_.motor()->posNorm()) < 0.1f) {
            gimbal_.isLaunched = true;
        } else {
            delta = -PINYMOTOR::getMinorArc(0, gimbal_.motor()->posNorm());
            // printf("launching: delta: %.2f\n", delta);
            gimbal_.ctrl(delta);
        }
    }

    void exit() final { LOG::info(TAG, "exit"); }

    FSMState_e checkChange() final
    {
        using enum FSMState_e;
        if (gimbal_.isLaunched)
            return FSMState_e::RUN;
         if (gimbal_.msg.state == FSMState_e::STOP) {
            gimbal_.isLaunched = false;
            return FSMState_e::STOP;
        } else
            return LAUNCH;
    }

private:
    Standard &gimbal_;
};

} // namespace GIMBAL
