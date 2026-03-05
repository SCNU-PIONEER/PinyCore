#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gRunPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(30.f, 40.f, 3.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gRunVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(4.f, 0.f, 0.03f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.3f, 0.0f, 0.001f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
};

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    static constexpr float SECOND_YAW_MAX_ANG = 0.1309f; //±15°

public:
    RunState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i] = &gRunPosPid[i];
            runVelPid_._[i] = &gRunVelPid[i];
        }
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
            eMA._[i] = cMA._[i];
        }
        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        DualStage_s<float> delArc = {};
        eIA_s.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA_s.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA_s.SecondYaw = 0.f; //run模式下小yaw不改变方向

        if (gimbal_->isVisionAngOver_) {
            eIA_s.Pitch = vision->v2eData().targetPitch;
            eIA_s.FirstYaw = vision->v2eData().targetYaw;
        }

        eIA_s.Pitch = PINYMOTOR::clampArc(eIA_s.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA_s.FirstYaw = rangeConversion(eIA_s.FirstYaw);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA_s.Pitch, cIA_s.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA_s.FirstYaw, cIA_s.FirstYaw);

        eMA_s.Pitch = PINYMOTOR::rangeMap(cMA_s.Pitch + delArc.Pitch);
        eMA_s.FirstYaw = PINYMOTOR::rangeMap(cMA_s.FirstYaw + delArc.FirstYaw);
        eMA_s.SecondYaw = 0.f;

        gimbal_->ctrl(eMA, runPosPid_, runVelPid_);
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
        else if (gimbal_->msg.state == FSMState_e::PATROL)
            return FSMState_e::PATROL;
        else if (gimbal_->msg.state == FSMState_e::AUTO || vision->v2eData().control)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::NAV)
            return FSMState_e::NAV;
        else
            return FSMState_e::STOP;
    };

private:
    DualStage *gimbal_;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_s<float> &cMA_s = this->gimbal_->curMotorAng_.Gim_s;

    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_s<float> &cIA_s = this->gimbal_->curImuAng_.Gim_s;

    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_s<float> &eMA_s = this->gimbal_->expMotorAng_.Gim_s;

    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;
    DualStage_s<float> &eIA_s = this->gimbal_->expImuAng_.Gim_s;

    DualStage_u<PositionalPid *> runPosPid_;
    DualStage_u<PositionalPid *> runVelPid_;
};
} // namespace GIMBAL
