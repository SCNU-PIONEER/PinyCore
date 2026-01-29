#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gNavPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(60.f, 100.f, 0.5f, 0.001f, 3.f, 7.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gNavVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.8f, 0.f, 0.05f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.3f, 0.0f, 0.001f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
};

namespace GIMBAL {

class NavState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    static constexpr float SECOND_YAW_MAX_ANG = 0.1309f; //±15°

public:
    NavState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::NAV), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            navPosPid_._[i] = &gNavPosPid[i];
            navVelPid_._[i] = &gNavVelPid[i];
        }
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
            eMA._[i] = cMA._[i];
        }
        gimbal_->enter();

        LOG::info("GimbalNav", "enter");
    }

    void run() final
    {
        DualStage_u<float> delArc = {};

        eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.SecondYaw = 0.f; //run模式下小yaw不改变方向

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA.Pitch, cIA.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA.FirstYaw, cIA.FirstYaw);

        eMA.Pitch = PINYMOTOR::rangeMap(cMA.Pitch + delArc.Pitch);
        eMA.FirstYaw = PINYMOTOR::rangeMap(cMA.FirstYaw + delArc.FirstYaw);
        eMA.SecondYaw = 0.f;

        gimbal_->ctrl(eMA, navPosPid_, navVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            navPosPid_._[i]->reset();
            navVelPid_._[i]->reset();
        }

        LOG::info("GimbalNav", "exit");
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
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStagePtr_u<PositionalPid> navPosPid_;
    DualStagePtr_u<PositionalPid> navVelPid_;
};
} // namespace GIMBAL
