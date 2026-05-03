#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cmath>

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(60.f, 10.f, 2.f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(15.f, 0.f, 0.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(50.f, 8.f, 1.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
//    PositionalPid(30.f, 20.f, 5.f, 0.001f, 3.f, 25.f, 0.f),4310
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(2.f, 0.f, 0.05f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.45f, 0.f, 0.001f, 1.f / 1000.f, 0.f, 7.f, 0.f),
    PositionalPid(8.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};
//    PositionalPid(0.5f, 0.5f, 0.f, 0.001f, 0.f, 7.f, 0.f),4310

static PositionalPid wSlowBYawFollowPid = {
    PositionalPid(3.f, 0.f, 0.004f, 0.001f, 0.5f, GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.1f),
};
static PositionalPid wFastBYawFollowPid = {
    PositionalPid(45.f, 0.1f, 0.01f, 0.001f, 0.5f, GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.1f),
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
        wSlowBYawFollowPid_ = &wSlowBYawFollowPid;
        wFastBYawFollowPid_ = &wFastBYawFollowPid;
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
        eIA.Pitch = gimbal_->v2eData.pitch;
        eIA.SecondYaw = gimbal_->v2eData.yaw;

        if (fabsf(rangeConversion(cMA.SecondYaw)) < 0.1f) {
            wBYaw_ = 0.f;
            eIA.FirstYaw = visionExpBigYaw_;
        } else {
            if (isSmallAng_) {
                wBYaw_ = wFastBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, cMA.SecondYaw));
            } else {
                wBYaw_ = wSlowBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, cMA.SecondYaw));
            }

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

    bool &isSmallAng_ = gimbal_->d2eData.isSmallAng; //是否是副相机角度
    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStage_u<PositionalPid *> autoPosPid_;
    DualStage_u<PositionalPid *> autoVelPid_;
    PositionalPid *wSlowBYawFollowPid_;
    PositionalPid *wFastBYawFollowPid_;
};
} // namespace GIMBAL
