#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "Vision.hpp"

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(30.f, 40.f, 3.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(4.f, 0.f, 0.03f, 0.001f, 0.f, 7.f, 0.f),
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
        visionExpBigYaw_ = eIA_s.FirstYaw; //进入auto状态时，钉住大yaw角度
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }
    //auto下视觉进行控制,此时只控小yaw,转动范围不应该超出小yaw范围，
    void run() final
    {
        DualStage_s<float> delArc = {};

        deltaVisionAng_ = PINYMOTOR::getMinorArc(vision->v2eData().yaw, cMA_s.SecondYaw);
        isVisionAngOver_ = deltaVisionAng_ >= SECOND_YAW_MAX_ANG || deltaVisionAng_ <= SECOND_YAW_MIN_ANG;

        if (isVisionAngOver_) {
            eIA_s.Pitch = vision->v2eData().pitch;
            eIA_s.SecondYaw = 0.f;
            eIA_s.FirstYaw = vision->v2eData().yaw;
        } else {
            eIA_s.Pitch = vision->v2eData().pitch;
            eIA_s.SecondYaw = vision->v2eData().yaw;
            eIA_s.FirstYaw = visionExpBigYaw_;
        }

        eIA_s.Pitch = PINYMOTOR::clampArc(eIA_s.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA_s.FirstYaw = rangeConversion(eIA_s.FirstYaw);
        eIA_s.SecondYaw = PINYMOTOR::clampArc(eIA_s.SecondYaw, SECOND_YAW_MIN_ANG, SECOND_YAW_MAX_ANG);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA_s.Pitch, cIA_s.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA_s.FirstYaw, cIA_s.FirstYaw);
        delArc.SecondYaw = PINYMOTOR::getMinorArc(eIA_s.SecondYaw, cIA_s.SecondYaw);

        eMA_s.Pitch = PINYMOTOR::rangeMap(cMA_s.Pitch + delArc.Pitch);
        eMA_s.FirstYaw = PINYMOTOR::rangeMap(cMA_s.FirstYaw + delArc.FirstYaw);
        eMA_s.SecondYaw = PINYMOTOR::rangeMap(cMA_s.SecondYaw + delArc.SecondYaw);

        if (isVisionAngOver_) {
            eMA_s.SecondYaw = 0.f;
        }

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

        if (gimbal_->msg.state == FSMState_e::NAV && !vision->v2eData().control)
            return FSMState_e::NAV;
        else if (gimbal_->msg.state == FSMState_e::RUN && !vision->v2eData().control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    };

private:
    DualStage *gimbal_;
    float visionExpBigYaw_;        //视觉进行自瞄，钉住大yaw角度
    bool isVisionAngOver_ = false; //视觉角度是否超出小yaw控制范围
    float deltaVisionAng_;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_s<float> &cMA_s = this->gimbal_->curMotorAng_.Gim_s;

    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_s<float> &cIA_s = this->gimbal_->curImuAng_.Gim_s;

    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_s<float> &eMA_s = this->gimbal_->expMotorAng_.Gim_s;

    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;
    DualStage_s<float> &eIA_s = this->gimbal_->expImuAng_.Gim_s;

    DualStage_u<PositionalPid *> autoPosPid_;
    DualStage_u<PositionalPid *> autoVelPid_;
};
} // namespace GIMBAL
