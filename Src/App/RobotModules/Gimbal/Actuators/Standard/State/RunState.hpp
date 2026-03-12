#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"
#include "MotorCommonMacros.hpp"
#include "LKMotorRS485.hpp"


float yaw;
float targetang;

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
            if (std::fabs(yawAng) < 0.02f) {
                gimbal_.isLaunched = true;
                targetAng = gimbal_.ins().yaw;
            } else {
                gimbal_.ctrl(0, 3.f);
                gimbal_.isLaunched = false;
            }
        }
        return gimbal_.isLaunched;
        // return true;
    }

    void run() final
    {
        static uint16_t cnt = 0;
        cnt++;
        yaw = gimbal_.ins().yaw;
        targetAng += gimbal_.msg.yaw;
        targetAng = PINYMOTOR::rangeMap(targetAng, -std::numbers::pi_v<float>, std::numbers::pi_v<float>);
        targetang = targetAng;
        float insYawDelArc = PINYMOTOR::getMinorArc(targetAng, gimbal_.ins().yaw);
        float pos = gimbal_.motor()->pos() - insYawDelArc;

        if (cnt % 100 == 0) {
            cnt = 0;
            printf("motor:%.2f,target:%.2f,ins:%.2f,insDel:%.2f,pos:%.2f\n", gimbal_.motor()->pos(), targetAng,
                   gimbal_.ins().yaw, insYawDelArc, pos);
        }

        gimbal_.ctrl(pos);
    }

    void exit() final
    {
        targetAng = gimbal_.motor()->posNorm();
        isEnabled = false;
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
    bool isEnabled = false;
};

} // namespace GIMBAL
