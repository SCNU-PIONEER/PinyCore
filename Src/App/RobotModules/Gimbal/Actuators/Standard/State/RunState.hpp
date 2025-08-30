#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

namespace GIMBAL::STANDARD {

class RunState : public FSMState {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    RunState(Standard *_gimbal) : gimbal_(_gimbal)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    }

    void enter() final
    {
        // cmdImuPitch_.pos = gimbal_->ins().pitch;
        // cmdImuYaw_.pos = gimbal_->ins().yaw;

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        cmdImuPitch_.pos = PINYMOTOR::clampArc(
                gimbal_->msg.pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        cmdImuYaw_.pos = gimbal_->msg.yaw;

        float insPitchDelArc =
                PINYMOTOR::getMinorArc(cmdImuPitch_.pos, gimbal_->ins().pitch);
        float insYawDelArc =
                PINYMOTOR::getMinorArc(cmdImuYaw_.pos, gimbal_->endYawAng());

        cmdMotorPitch_.pos = PINYMOTOR::rangeMap(
                gimbal_->pitch()->posNorm() + (Standard::PITCH_REVERSE ?
                                                       -insPitchDelArc :
                                                       insPitchDelArc),
                0, 2 * M_PI);
        cmdMotorYaw_.pos = PINYMOTOR::rangeMap(
                gimbal_->yaw()->posNorm() +
                        (Standard::YAW_REVERSE ? -insYawDelArc : insYawDelArc),
                0, 2 * M_PI);

        gimbal_->pitch()->cmdPos(cmdMotorPitch_.pos);
        gimbal_->yaw()->cmdPos(cmdMotorYaw_.pos);
    }

    void exit() final { LOG::info("GimbalRun", "exit"); }

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

    States_u cmdImuPitch_;
    States_u cmdImuYaw_;

    States_u cmdMotorPitch_;
    States_u cmdMotorYaw_;
};
} // namespace GIMBAL::STANDARD
