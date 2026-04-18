#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cmath>

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(15.f, 10.f, 0.5f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(25.f, 0.f, 0.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(100.f, 10.f, 2.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
//    PositionalPid(30.f, 20.f, 5.f, 0.001f, 3.f, 25.f, 0.f),4310
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(2.5f, 0.f, 0.005f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.55f, 0.f, 0.001f, 1.f / 1000.f, 0.f, 7.f, 0.f),
    PositionalPid(10.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};
//    PositionalPid(0.5f, 0.5f, 0.f, 0.001f, 0.f, 7.f, 0.f),4310

static PositionalPid wBYFollowPid = {
    PositionalPid(25.f, 0.1f, 0.004f, 0.001f, 0.5f, GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.1f),
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
        visionExpBigYaw_ = cIA.FirstYaw; //进入auto状态时，钉住大yaw角度

        gimbal_->debug.gimbalState = FSMState_e::AUTO;
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }
    //auto下视觉进行控制,此时只控小yaw,转动范围不应该超出小yaw范围，
    void run() final
    {
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);
        eIA.SecondYaw = rangeConversion(eIA.SecondYaw);

        eIA.Pitch = gimbal_->v2eData.pitch;
        eIA.SecondYaw = gimbal_->v2eData.yaw;

        // delArc.SecondYaw = PINYMOTOR::getMinorArc(eIA.SecondYaw, cIA.SecondYaw);
        // eMA.SecondYaw = PINYMOTOR::rangeMap(cMA.SecondYaw + delArc.SecondYaw);

        // deltaVisionAng_ = PINYMOTOR::getMinorArc(eMA.SecondYaw, cMA.SecondYaw);
        // //超量程为正
        // gimbal_->isVisionAngOver_ = (deltaVisionAng_ > SECOND_YAW_MAX_ANG) || (deltaVisionAng_ < SECOND_YAW_MIN_ANG);

        // if (gimbal_->isVisionAngOver_) {
        //     float deltaYawAng = PINYMOTOR::getMinorArc(gimbal_->v2eData.yaw, cIA.FirstYaw);
        //     wBYaw_ = wBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, deltaYawAng));
        //     // wBYaw_ = 0.18 * 2.f; //0.18rad/s,10°
        //     eIA.FirstYaw += wBYaw_ * 0.001f;
        //     visionExpBigYaw_ = cIA.FirstYaw; //重载当前大yaw角度

        //     debugWBYaw = wBYaw_;
        // } else {
        //     eIA.FirstYaw = visionExpBigYaw_;
        // }

        if (fabsf(rangeConversion(cMA.SecondYaw)) < 0.1f) {
            wBYaw_ = 0.f;
            eIA.FirstYaw = visionExpBigYaw_;
        } else {
            wBYaw_ = wBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, cMA.SecondYaw));
            wBYaw_ = PINYMOTOR::clamp(wBYaw_, -3.f, 3.f);
            eIA.FirstYaw += wBYaw_ * 0.001f;
            visionExpBigYaw_ = cIA.FirstYaw; //重载当前大yaw角度
        }
        debugWBYaw = wBYaw_;

        gimbal_->ctrl(eIA, autoPosPid_, autoVelPid_);
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

        if (gimbal_->msg.state == FSMState_e::NAV && !gimbal_->v2eData.control)
            return FSMState_e::NAV;
        else if (gimbal_->msg.state == FSMState_e::RUN && !gimbal_->v2eData.control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    };

private:
    DualStage *gimbal_;
    float visionExpBigYaw_; //视觉进行自瞄，钉住大yaw角度
    float wBYaw_;
    float deltaVisionAng_;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStage_u<PositionalPid *> autoPosPid_;
    DualStage_u<PositionalPid *> autoVelPid_;
    PositionalPid *wBYawFollowPid_;
};
} // namespace GIMBAL
