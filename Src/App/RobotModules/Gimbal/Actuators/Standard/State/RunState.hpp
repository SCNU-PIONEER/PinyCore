#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
public:
    RunState(Standard &_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal) {}

    void enter() final
    {
        targetAng = gimbal_.motor()->posNorm();
        gimbal_.enter();
        LOG::info(tag, "enter");
    }

    bool change() final
    {
        /* 上控复位 */
        float yawAng = gimbal_.motor()->posNorm();
        if (std::fabs(yawAng) < 0.1f) {
            return true;
        } else {
            gimbal_.ctrl(0);
            return false;
        }
    }

    void run() final
    {
        targetAng += gimbal_.msg.yaw;
        gimbal_.ctrl(targetAng);
    }

    void exit() final { LOG::info(tag, "exit"); }

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
