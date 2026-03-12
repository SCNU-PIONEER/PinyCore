#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "Smooth.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gNavPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(60.f, 80.f, 3.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gNavVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.5f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
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

        eIA_s.Pitch = cIA_s.Pitch;
        eIA_s.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA_s.SecondYaw = cIA_s.SecondYaw;

        eIA_s.Pitch = PINYMOTOR::clampArc(eIA_s.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA_s.FirstYaw = rangeConversion(eIA_s.FirstYaw);

        for (uint8_t i = 0; i < 3; i++) {
            delArc._[i] = PINYMOTOR::getMinorArc(eIA._[i], cIA._[i]);
            eMA._[i] = PINYMOTOR::rangeMap(cMA._[i] + delArc._[i]);
        }

        //sin(Ωt),巡检模式下小yaw,pitch使用电机系控制
        if (patrolCnt + 5 >= MAX_PAT_CNT)
            maxPatCount = 0;
        else if (patrolCnt - 5 <= 0)
            maxPatCount = MAX_PAT_CNT;
        SMOOTH::slopeFollowing(&maxPatCount, &patrolCnt, 1.f);

        eMA_s.Pitch = PINYMOTOR::rangeMap((0.2f * cos(omiga * patrolCnt)) + 0.1f);
        eMA_s.SecondYaw = PINYMOTOR::rangeMap(SECOND_YAW_MAX_ANG * sin(omiga * patrolCnt));

        // eMA_s.Pitch = 0.f;
        // eMA_s.SecondYaw = 0.f;

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

        if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (gimbal_->msg.state == FSMState_e::NAV && vision->v2eData().control)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::RUN && vision->v2eData().control)
            return FSMState_e::AUTO; //平时调试用
        else
            return FSMState_e::NAV;
    };

private:
    DualStage *gimbal_;

    static constexpr float PAT_FREQ = 1000.f; //hz,代码运行频率
    static constexpr float PERIOD = 1;        //s
    static constexpr float MAX_PAT_CNT = PERIOD * PAT_FREQ;
    float patrolCnt = 0;
    float maxPatCount = 0;
    float omiga = 2 * PI / PERIOD / PAT_FREQ; //rad/s

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_s<float> &cMA_s = this->gimbal_->curMotorAng_.Gim_s;

    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_s<float> &cIA_s = this->gimbal_->curImuAng_.Gim_s;

    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_s<float> &eMA_s = this->gimbal_->expMotorAng_.Gim_s;

    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;
    DualStage_s<float> &eIA_s = this->gimbal_->expImuAng_.Gim_s;

    DualStage_u<PositionalPid *> navPosPid_;
    DualStage_u<PositionalPid *> navVelPid_;
};
} // namespace GIMBAL
