#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "Vision.hpp"
#include <cmath>

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(15.f, 10.f, 0.5f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(30.f, 20.f, 5.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(100.f, 10.f, 2.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
//PositionalPid(200.f, 20.f, 5.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(2.5f, 0.f, 0.005f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.5f, 0.5f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(10.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};
//PositionalPid(20.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),

static PositionalPid wBYFollowPid = {
    PositionalPid(10.f, 0.3f, 0.001f, 0.001f, 0.5f, GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.01f),
};
float debugWBYaw;

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_RESET_TIME = 300; // 0.3s

    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    static constexpr float SECOND_YAW_MAX_ANG = 0.22f; // 13
    static constexpr float SECOND_YAW_MIN_ANG = -0.22f;

public:
    AutoState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i] = &gAutoPosPid[i];
            autoVelPid_._[i] = &gAutoVelPid[i];
        }
        wBYawFollowPid_ = &wBYFollowPid;
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
            eMA._[i] = cMA._[i];
        }
        visionExpBigYaw_ = cIA_s.FirstYaw; //进入auto状态时，钉住大yaw角度

        gimbal_->debug.gimbalState = FSMState_e::AUTO;
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }
    //auto下视觉进行控制,此时只控小yaw,转动范围不应该超出小yaw范围，
    void run() final
    {
        DualStage_s<float> delArc = {};

        eIA_s.Pitch = gimbal_->vmsg.pitch;
        eIA_s.SecondYaw = gimbal_->vmsg.yaw;

        delArc.SecondYaw = PINYMOTOR::getMinorArc(eIA_s.SecondYaw, cIA_s.SecondYaw);
        eMA_s.SecondYaw = PINYMOTOR::rangeMap(cMA_s.SecondYaw + delArc.SecondYaw);

        deltaVisionAng_ = PINYMOTOR::getMinorArc(eMA_s.SecondYaw, cMA_s.SecondYaw);
        //超量程为正
        gimbal_->isVisionAngOver_ = (deltaVisionAng_ > SECOND_YAW_MAX_ANG) || (deltaVisionAng_ < SECOND_YAW_MIN_ANG);

        if (gimbal_->isVisionAngOver_) {
            float deltaYawAng = PINYMOTOR::getMinorArc(gimbal_->vmsg.yaw, cIA_s.FirstYaw);
            wBYaw_ = wBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, deltaYawAng));
            // wBYaw_ = 0.18 * 2.f; //0.18rad/s,10°
            eIA_s.FirstYaw += wBYaw_ * 0.001f;
            visionExpBigYaw_ = cIA_s.FirstYaw; //重载当前大yaw角度

            debugWBYaw = wBYaw_;
        } else {
            eIA_s.FirstYaw = visionExpBigYaw_;
        }

        eIA_s.Pitch = PINYMOTOR::clampArc(eIA_s.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA_s.FirstYaw = rangeConversion(eIA_s.FirstYaw);

        delArc.Pitch = PINYMOTOR::getMinorArc(eIA_s.Pitch, cIA_s.Pitch);
        delArc.FirstYaw = PINYMOTOR::getMinorArc(eIA_s.FirstYaw, cIA_s.FirstYaw);

        eMA_s.Pitch = PINYMOTOR::rangeMap(cMA_s.Pitch + delArc.Pitch);
        eMA_s.FirstYaw = PINYMOTOR::rangeMap(cMA_s.FirstYaw + delArc.FirstYaw);
        eMA_s.SecondYaw = PINYMOTOR::clampArc(eMA_s.SecondYaw, SECOND_YAW_MIN_ANG, SECOND_YAW_MAX_ANG);

        gimbal_->ctrl(eMA, autoPosPid_, autoVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i]->reset();
            autoVelPid_._[i]->reset();
        }
        gimbal_->debug.gimbalState = gimbal_->msg.state;
        LOG::info("GimbalAuto", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::NAV && !gimbal_->vmsg.control)
            return FSMState_e::NAV;
        else if (gimbal_->msg.state == FSMState_e::RUN && !gimbal_->vmsg.control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    };

private:
    DualStage *gimbal_;
    float visionExpBigYaw_; //视觉进行自瞄，钉住大yaw角度
    float wBYaw_;
    float deltaVisionAng_;
    // bool isSecondYawReset_ = false;
    // uint16_t sYResetTime_ = 0;


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
    PositionalPid *wBYawFollowPid_;
};
} // namespace GIMBAL
