#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"
#include "MotorCommonMacros.hpp"
// #include "LKMotorRS485.hpp"

namespace GIMBAL {
class RunState : public FSMState<FSMState_e> {
    static constexpr char TAG[] = "RunState";

public:
    RunState(Standard &_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal) {}

    void enter() final
    {
        // 零点设置
        // auto motor = (PINYMOTOR::LKMOTOR::LKMotorRS485 *)gimbal_.motor();
        // motor->setZeroPos();
        targetAng = gimbal_.ins().yaw;
        LOG::info(TAG, "enter");
    }

    void run() final
    {
        targetAng += gimbal_.msg.yaw;
        targetAng = PINYMOTOR::rangeMap(targetAng, -std::numbers::pi_v<float>, std::numbers::pi_v<float>);
        float insYawDelArc = -PINYMOTOR::getMinorArc(targetAng, gimbal_.ins().yaw);
        gimbal_.ctrl(insYawDelArc);
    }

    void exit() final
    {
        targetAng = gimbal_.motor()->posNorm();
        gimbal_.isLaunched = false;

        LOG::info(TAG, "exit");
    }

    FSMState_e checkChange() final
    {
        using enum FSMState_e;
        switch (gimbal_.msg.state) {
        case STOP:
            return STOP;
        default:
            return RUN;
        }
    }

private:
    Standard &gimbal_;
    float targetAng;
};

} // namespace GIMBAL
