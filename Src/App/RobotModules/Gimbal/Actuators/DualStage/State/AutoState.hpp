#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cmath>

static PositionalPid gAutoPosPid[3] = {
    PositionalPid(45.f, 15.f, 1.7f, 0.001f, 30.f, 80.f, 0.f),
    PositionalPid(15.f, 0.f, 2.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(35.f, 10.f, 0.5f, 1.f / 1000.f, 60.f, 80.f, 0.f),
};
static PositionalPid gAutoVelPid[3] = {
    PositionalPid(2.f, 0.f, 0.05f, 0.001f, 2.f, 5.f, 0.f),
    PositionalPid(0.55f, 0.f, 0.0f, 1.f / 1000.f, 0.f, 7.f, 0.f),
    PositionalPid(100.f, 30.f, 1.f, 1.f / 1000.f, 5.f, 5.f, 0.f),
};

// static PositionalPid gAutoPosPid[3] = {
//     PositionalPid(100.f, 50.f, 1.5f, 0.001f, 30.f, 80.f, 0.f),
//     PositionalPid(15.f, 0.f, 2.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
//     PositionalPid(30.f, 15.f, 1.f, 1.f / 1000.f, 30.f, 80.f, 0.f),
// };
// static PositionalPid gAutoVelPid[3] = {
//     PositionalPid(2.5f, 0.f, 0.05f, 0.001f, 2.f, 5.f, 0.f),
//     PositionalPid(0.55f, 0.f, 0.0f, 1.f / 1000.f, 0.f, 7.f, 0.f),
//     PositionalPid(80.f, 40.f, 1.f, 1.f / 1000.f, 2.f, 5.f, 0.f),
// };

static PositionalPid gBufferPosPid[3] = {
    PositionalPid(25.f, 10.f, 0.05f, 0.001f, 30.f, 80.f, 0.f),
    PositionalPid(15.f, 0.f, 0.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(30.f, 0.f, 3.f, 1.f / 1000.f, 30.f, 80.f, 0.f),
};
static PositionalPid gBufferVelPid[3] = {
    PositionalPid(2.f, 0.f, 0.05f, 0.001f, 1.5f, 3.5f, 0.f),
    PositionalPid(0.45f, 0.f, 0.001f, 1.f / 1000.f, 3.f, 7.f, 0.f),
    PositionalPid(100.f, 0.0f, 2.f, 1.f / 1000.f, 1.5f, 3.5f, 0.f),
};

static PositionalPid wSlowBYawFollowPid = { PositionalPid(5.f, 0.f, 0.004f, 0.001f, 0.5f,
                                                          GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.1f) };
static PositionalPid wFastBYawFollowPid = { PositionalPid(20.f, 0.5f, 0.01f, 0.001f, 0.5f,
                                                          GIMBAL::DualStage::MAX_BYAW_FOLLOW_SPEED, 0.05f) };

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_RESET_TIME = 300; // 0.3s

    static constexpr float IMU_PITCH_MAX_ANG = 0.53f;  //
    static constexpr float IMU_PITCH_MIN_ANG = -0.58f; //

    static constexpr float SECOND_YAW_MAX_ANG = 0.22f; // 13
    static constexpr float SECOND_YAW_MIN_ANG = -0.22f;


public:
    AutoState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i] = &gAutoPosPid[i];
            autoVelPid_._[i] = &gAutoVelPid[i];

            bufferPosPid_._[i] = &gBufferPosPid[i];
            bufferVelPid_._[i] = &gBufferVelPid[i];
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
        gimbal_->msg.wYaw = 0;

        gimbal_->debug.gimbalState = FSMState_e::AUTO;
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }
    //auto下视觉进行控制,此时只控小yaw,转动范围不应该超出小yaw范围，
    void run() final
    {
        float wBYaw = 0.f;

        eIA.Pitch = gimbal_->v2eData.pitch;
        eIA.SecondYaw = gimbal_->v2eData.yaw;

        if (isSmallAng_) {
            wBYaw = wFastBYawFollowPid_->calc(0, -PINYMOTOR::getMinorArc(gimbal_->v2eData.yaw, cIA.FirstYaw));
            wBYaw = PINYMOTOR::clamp(wBYaw, -5.f, 5.f);
            eIA.FirstYaw += wBYaw * 0.001f;

            eMA = gimbal_->ctrl(eIA);
            eMA.SecondYaw = 0;

            visionExpBigYaw_ = cIA.FirstYaw;
        } else {
            if (fabsf(rangeConversion(cMA.SecondYaw)) < 0.1f) {
                wBYaw = 0.f;
                eIA.FirstYaw = visionExpBigYaw_;
            } else {
                wBYaw = wSlowBYawFollowPid_->calc(0, PINYMOTOR::getMinorArc(0, cMA.SecondYaw));
                wBYaw = PINYMOTOR::clamp(wBYaw, -5.f, 5.f);
                eIA.FirstYaw += wBYaw * 0.001f;

                visionExpBigYaw_ = cIA.FirstYaw; //重载当前大yaw角度
            }
            eMA = gimbal_->ctrl(eIA);
        }
        gimbal_->debug.followByaw = wBYaw;

        if (gimbal_->d2eData.bufferMode != 0) {
            gimbal_->ctrlMotor(eMA, bufferPosPid_, bufferVelPid_);
        } else {
            gimbal_->ctrlMotor(eMA, autoPosPid_, autoVelPid_);
        }
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            autoPosPid_._[i]->reset();
            autoVelPid_._[i]->reset();
            bufferPosPid_._[i]->reset();
            bufferVelPid_._[i]->reset();
        }
        wFastBYawFollowPid_->reset();
        wSlowBYawFollowPid_->reset();
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

    bool &isSmallAng_ = gimbal_->d2eData.isSmallAng; //是否是副相机角度
    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStage_u<PositionalPid *> autoPosPid_, bufferPosPid_;
    DualStage_u<PositionalPid *> autoVelPid_, bufferVelPid_;
    PositionalPid *wSlowBYawFollowPid_, *wFastBYawFollowPid_;
};
} // namespace GIMBAL
