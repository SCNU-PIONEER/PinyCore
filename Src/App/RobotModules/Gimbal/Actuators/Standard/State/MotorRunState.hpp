#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

namespace GIMBAL::STANDARD {

class MotorRunState : public FSMState {
    static constexpr float PITCH_MIN_ANG = 0.f;
    static constexpr float PITCH_MAX_ANG = 0.f;

public:
    MotorRunState(Standard *_gimbal) : gimbal_(_gimbal)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    }

    void enter() final
    {
        cmdMotorPitch_.pos = gimbal_->pitch()->posNorm();
        cmdMotorYaw_.pos = gimbal_->yaw()->posNorm();

        gimbal_->enter();

        LOG::info("GimbalMotorRun", "enter");
    }

    void run() final
    {
        // cmdMotorPitch_.pos = PINYMOTOR::clampArc(gimbal_->msg.pitch,
        //                                          PITCH_MIN_ANG, PITCH_MAX_ANG);
        cmdMotorYaw_.pos = gimbal_->msg.yaw;

        cmdMotorPitch_.pos =
                PINYMOTOR::rangeMap(cmdMotorPitch_.pos, 0, 2 * M_PI);
        cmdMotorYaw_.pos = PINYMOTOR::rangeMap(cmdMotorYaw_.pos, 0, 2 * M_PI);

        gimbal_->pitch()->cmdPos(cmdMotorPitch_.pos);
        gimbal_->yaw()->cmdPos(cmdMotorYaw_.pos);
    }

    void exit() final { LOG::info("GimbalMotorRun", "exit"); }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Standard *gimbal_;

    States_u cmdMotorPitch_;
    States_u cmdMotorYaw_;
};
} // namespace GIMBAL::STANDARD