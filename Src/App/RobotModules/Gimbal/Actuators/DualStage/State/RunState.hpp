#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gRunPosPid[3] = {
    PositionalPid(10.f, 0.001f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f),
    PositionalPid(20.f, 0.0f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f),
    PositionalPid(14.f, 0.0f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f)
};
static PositionalPid gRunVelPid[3] = {
    PositionalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
    PositionalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
    PositionalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f)
};
namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    RunState(DualStage *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i] = &gRunPosPid[i];
            runVelPid_._[i] = &gRunVelPid[i];
        }
    }

    void enter() final
    {
        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        DualStage_u<float> delArc;

        eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.SecondYaw = 0.f; //run模式下小yaw不改变方向

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG,
                                        IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = gimbal_->rangeConversion(eIA.FirstYaw);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA.Pitch, cIA.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA.FirstYaw, cIA.FirstYaw);

        eMA.Pitch = PINYMOTOR::rangeMap(cMA.Pitch + delArc.Pitch);
        eMA.FirstYaw = PINYMOTOR::rangeMap(cMA.FirstYaw + delArc.FirstYaw);
        eMA.SecondYaw = 0.f;

        for (uint8_t i = 0; i < 3; i++) {
            gimbal_->ctrl(gimbal_->motors()->_[i],
                          gimbal_->gimbalFilter()->_[i], runPosPid_._[i],
                          runVelPid_._[i], eMA._[i]);
        }
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i]->reset();
            runVelPid_._[i]->reset();
        }

        LOG::info("GimbalRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::RUN;
    }

private:
    DualStage *gimbal_;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStagePtr_u<PositionalPid> runPosPid_;
    DualStagePtr_u<PositionalPid> runVelPid_;
};
} // namespace GIMBAL
