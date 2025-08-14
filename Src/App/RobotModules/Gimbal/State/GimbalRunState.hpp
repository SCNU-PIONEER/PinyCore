#pragma once

#include "Gimbal.hpp"
#include "FSMState.hpp"
#include "MotorCommonMacros.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class RunState : public FSMState {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    RunState(Gimbal *_gimbal) : gimbal_(_gimbal), head_(_gimbal->head)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    }

    void enter() final
    {
        cmdImuPitch_.pos = head_->ins().pitch;
        cmdImuYaw_.pos = head_->ins().yaw;

        head_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        cmdImuPitch_.pos = PINYMOTOR::clamp(
                cmdImuPitch_.pos + (gimbal_->msg.wPitch * 0.001f),
                IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        cmdImuYaw_.pos += gimbal_->msg.wYaw * 0.001f;

        float insPitchDelArc =
                PINYMOTOR::getMinorArc(cmdImuPitch_.pos, head_->ins().pitch);
        float insYawDelArc =
                PINYMOTOR::getMinorArc(cmdImuYaw_.pos, head_->ins().yaw);

        cmdMotorPitch_.pos = PINYMOTOR::rangeMap(
                head_->joint(0)->posNorm() + insPitchDelArc, 0, 2 * M_PI);
        cmdMotorYaw_.pos = PINYMOTOR::rangeMap(
                head_->joint(1)->posNorm() + insYawDelArc, 0, 2 * M_PI);

        head_->joint(0)->cmdPos(cmdMotorPitch_.pos);
        head_->joint(1)->cmdPos(cmdMotorYaw_.pos);
    }

    void exit() final { LOG::info("GimbalRun", "exit"); }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (gimbal_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Gimbal *gimbal_;
    Head *head_;

    States_u cmdImuPitch_;
    States_u cmdImuYaw_;

    States_u cmdMotorPitch_;
    States_u cmdMotorYaw_;
};
} // namespace GIMBAL
