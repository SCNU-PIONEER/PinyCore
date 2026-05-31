#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "Smooth.hpp"
#include "StmLog.hpp"
#include "dsp/fast_math_functions.h"
#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cmath>

static PositionalPid gNavPosPid[3] = {
    PositionalPid(45.f, 30.f, 1.f, 0.001f, 30.f, 80.f, 0.f),
    PositionalPid(10.f, 5.f, 2.f, 1.f / 1000.f, 60.f, 110.f, 0.f),
    PositionalPid(45.f, 10.f, 0.5f, 1.f / 1000.f, 30.f, 80.f, 0.f),
};
//    PositionalPid(60.f, 80.f, 3.f, 0.001f, 3.f, 25.f, 0.f),4310
static PositionalPid gNavVelPid[3] = {
    PositionalPid(0.5f, 0.f, 0.01f, 0.001f, 1.5f, 3.5f, 0.f),
    PositionalPid(2.f, 0.f, 0.0f, 1.f / 1000.f, 0.f, 7.f, 0.f),
    PositionalPid(3.f, 0.0f, 0.1f, 1.f / 1000.f, 1.5f, 3.5f, 0.f),
};
//    PositionalPid(0.5f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),4310

namespace GIMBAL {

class NavState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MAX_ANG = 0.53f;  //
    static constexpr float IMU_PITCH_MIN_ANG = -0.05f; //

    static constexpr float SECOND_YAW_MAX_ANG = 0.22f; // 13°

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
        eIA.Pitch = cIA.Pitch;
        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.SecondYaw = cIA.SecondYaw;

        eIA.Pitch = PINYMOTOR::clamp(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);

        eMA = gimbal_->ctrl(eIA);
        if (gimbal_->d2eData.isBumpArea) {
            for (auto &i : eMA._) {
                i = 0;
            }
        } else {
            //sin(Ωt),巡检模式下小yaw,pitch使用电机系控制
            if (patrolCnt + 5 >= MAX_PAT_CNT)
                maxPatCount = 0;
            else if (patrolCnt - 5 <= 0)
                maxPatCount = MAX_PAT_CNT;
            SMOOTH::slopeFollowing(&maxPatCount, &patrolCnt, 1.f);

            eMA.Pitch = PINYMOTOR::rangeMap(((amplitude * cos(omiga * patrolCnt)) + amplitudeBias));
            eMA.SecondYaw = PINYMOTOR::rangeMap(SECOND_YAW_MAX_ANG * sin(2.f * omiga * patrolCnt));
        }
        gimbal_->ctrlMotor(eMA, navPosPid_, navVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            navPosPid_._[i]->reset();
            navVelPid_._[i]->reset();
        }
        gimbal_->debug.gimbalState = gimbal_->msg.state;
        LOG::info("GimbalNav", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (gimbal_->msg.state == FSMState_e::NAV && gimbal_->v2eData.control)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::RUN && gimbal_->v2eData.control)
            return FSMState_e::AUTO; //平时调试用
        else
            return FSMState_e::NAV;
    };

private:
    DualStage *gimbal_;

    static constexpr float PAT_FREQ = 1000.f; //hz,代码运行频率
    static constexpr float PERIOD = 1.2f;     //s
    static constexpr float MAX_PAT_CNT = PERIOD * PAT_FREQ;
    float patrolCnt = 0, maxPatCount = 0;
    float omiga = 2 * PI / PERIOD / PAT_FREQ; //rad/s
    float amplitude = ((IMU_PITCH_MAX_ANG - IMU_PITCH_MIN_ANG) / 2.f);
    float amplitudeBias = ((IMU_PITCH_MAX_ANG + IMU_PITCH_MIN_ANG) / 2.f);

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStage_u<PositionalPid *> navPosPid_;
    DualStage_u<PositionalPid *> navVelPid_;
};
} // namespace GIMBAL
