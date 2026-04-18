#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "MsgBase.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cmath>

static PositionalPid gRunPosPid[3] = {
    PositionalPid(15.f, 10.f, 0.5f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(25.f, 0.f, 0.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(15.f, 10.f, 3.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
//PositionalPid(30.f, 20.f, 5.f, 0.001f, 3.f, 25.f, 0.f),4310
static PositionalPid gRunVelPid[3] = {
    PositionalPid(2.5f, 0.f, 0.005f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.55f, 0.f, 0.001f, 1.f / 1000.f, 0.f, 7.f, 0.f),
    PositionalPid(1.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};
//PositionalPid(0.5f, 0.5f, 0.f, 0.001f, 0.f, 7.f, 0.f),4310

static PositionalPid wBYawFollowPid = {
    PositionalPid(45.f, 0.1f, 0.001f, 0.001f, 0.5f, GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.1f),
};
float debugBYaw;

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MAX_ANG = 0.436618f; // 25°
    static constexpr float IMU_PITCH_MIN_ANG = -0.6012f;  // -35°

    static constexpr float SECOND_YAW_MAX_ANG = 0.1309f; //±15°

public:
    RunState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i] = &gRunPosPid[i];
            runVelPid_._[i] = &gRunVelPid[i];
        }
        wBYawFollowPid_ = &wBYawFollowPid;
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
        }
        wBYaw_ = 0.f;
        expBigYaw_ = cIA.FirstYaw;
        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);

        eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.SecondYaw = 0.f; //run模式下小yaw不改变方向

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);

        //大yaw跟随小yaw
        // eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        // eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);

        // eIA.SecondYaw = rangeConversion(eIA.SecondYaw);
        // eIA.SecondYaw += gimbal_->msg.wYaw * 0.001f;

        // if (fabsf(rangeConversion(cMA.SecondYaw)) < 0.1f) {
        //     wBYaw_ = 0.f;
        //     eIA.FirstYaw = expBigYaw_;
        // } else {
        //     wBYaw_ = wBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, cMA.SecondYaw));

        //     if (gimbal_->msg.wYaw != 0.f) {
        //         wBYaw_ = PINYMOTOR::clamp(wBYaw_, -fabsf(gimbal_->msg.wYaw), fabsf(gimbal_->msg.wYaw));
        //     } else {
        //         wBYaw_ = PINYMOTOR::clamp(wBYaw_, -0.1f, 0.1f);
        //     }

        //     eIA.FirstYaw += wBYaw_ * 0.001f;
        //     expBigYaw_ = cIA.FirstYaw; //重载当前大yaw角度
        // }
        // debugBYaw = wBYaw_;

        gimbal_->ctrl(eIA, runPosPid_, runVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i]->reset();
            runVelPid_._[i]->reset();
        }
        gimbal_->debug.gimbalState = gimbal_->msg.state;
        LOG::info("GimbalRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::NAV)
            return FSMState_e::NAV;
        else
            return FSMState_e::RUN;
    };

private:
    DualStage *gimbal_;
    float wBYaw_ = 0.f;
    float expBigYaw_ = 0.f;

    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;

    PositionalPid *wBYawFollowPid_;
    DualStage_u<PositionalPid *> runPosPid_;
    DualStage_u<PositionalPid *> runVelPid_;
};
} // namespace GIMBAL
