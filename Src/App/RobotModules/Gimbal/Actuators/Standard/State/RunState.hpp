#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"
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
        // targetAng = gimbal_.motor()->posNorm();
        LOG::info(TAG, "enter");
    }

    bool change() final
    {
        /* 上控复位 */
        if (!isEnabled) {
            gimbal_.enter();
            isEnabled = true;
        } else {
            float yawAng = gimbal_.motor()->posNorm();
            if (std::fabs(yawAng) < 0.01f) {
                gimbal_.isLaunched = true;
                targetAng = gimbal_.motor()->posNorm();
            } else {
                gimbal_.ctrl(0, 2.f);
                gimbal_.isLaunched = false;
            }
        }
        return gimbal_.isLaunched;
    }

    void run() final
    {
        targetAng += gimbal_.msg.yaw;
        gimbal_.ctrl(targetAng);
    }

    void exit() final
    {
        targetAng = gimbal_.motor()->posNorm();
        isEnabled = false;
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
    bool isEnabled = false;
};

} // namespace GIMBAL
