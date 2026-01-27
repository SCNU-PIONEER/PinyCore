#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(100.f, 600.f, 0.5f, 0.001f, 3.f, 7.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(1.f, 0.001f, 0.05f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.3f, 0.0f, 0.001f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
};

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    static constexpr float SECOND_YAW_MAX_ANG = 0.2f;
    static constexpr float SECOND_YAW_MIN_ANG = -0.15f;

public:
    AutoState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i] = &gAutoPosPid[i];
            autoVelPid_._[i] = &gAutoVelPid[i];
        }
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
            eMA._[i] = cMA._[i];
        }
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }
    //auto下视觉进行控制,此时只控小yaw,转动范围不应该超出小yaw范围，
    void run() final
    {
        DualStage_u<float> delArc = {};

        eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA.FirstYaw = 0.f; //auto模式下大yaw不改变方向
        eIA.SecondYaw += gimbal_->msg.wYaw * 0.001f;

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);
        eIA.SecondYaw = PINYMOTOR::clampArc(eIA.SecondYaw, SECOND_YAW_MIN_ANG, SECOND_YAW_MAX_ANG);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA.Pitch, cIA.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA.FirstYaw, cIA.FirstYaw);
        delArc.SecondYaw = PINYMOTOR::getMinorArc(eIA.SecondYaw, cIA.SecondYaw);

        eMA.Pitch = PINYMOTOR::rangeMap(cMA.Pitch + delArc.Pitch);
        eMA.FirstYaw = 0.f;
        eMA.SecondYaw = PINYMOTOR::rangeMap(cMA.SecondYaw + delArc.SecondYaw);

        gimbal_->ctrl(eMA, autoPosPid_, autoVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i]->reset();
            autoVelPid_._[i]->reset();
        }

        LOG::info("GimbalAuto", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (gimbal_->msg.state == FSMState_e::PATROL)
            return FSMState_e::PATROL;
        else if (gimbal_->msg.state == FSMState_e::AUTO)
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

    DualStagePtr_u<PositionalPid> autoPosPid_;
    DualStagePtr_u<PositionalPid> autoVelPid_;
};
} // namespace GIMBAL
