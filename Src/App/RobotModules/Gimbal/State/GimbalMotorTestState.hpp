#pragma once

#include "Gimbal.hpp"
#include "FSMState.hpp"
#include "MotorCommonMacros.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class MotorTestState : public FSMState {
    static constexpr float PITCH_MIN_ANG = 0;
    static constexpr float PITCH_MAX_ANG = 0;

public:
    MotorTestState(Gimbal *_gimbal) : gimbal_(_gimbal), head_(gimbal_->head)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::MOTOR_TEST));
    }

    void enter() final
    {
        cmdMotorPitch_.pos = head_->joint(0)->posNorm();
        cmdMotorYaw_.pos = head_->joint(1)->posNorm();

        head_->enter();

        LOG::info("GimbalMotorTest", "enter");
    }

    void run() final
    {
        cmdMotorPitch_.pos += gimbal_->msg.wPitch * 0.001f;
        cmdMotorYaw_.pos += gimbal_->msg.wYaw * 0.001f;

        cmdMotorPitch_.pos =
                PINYMOTOR::rangeMap(cmdMotorPitch_.pos, 0, 2 * M_PI);
        cmdMotorYaw_.pos = PINYMOTOR::rangeMap(cmdMotorYaw_.pos, 0, 2 * M_PI);

        head_->joint(0)->cmdPos(cmdMotorPitch_.pos, -1, PITCH_MIN_ANG,
                                PITCH_MAX_ANG);
        head_->joint(1)->cmdPos(cmdMotorYaw_.pos, -1, 0, 0);
    }

    void exit() final { LOG::info("GimbalMotorTest", "exit"); }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (gimbal_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::MOTOR_TEST);
        else
            return 0;
    }

private:
    Gimbal *gimbal_;
    Head *head_;

    States_u cmdMotorPitch_;
    States_u cmdMotorYaw_;
};
} // namespace GIMBAL
