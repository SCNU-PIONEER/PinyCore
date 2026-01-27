#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Smooth.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

static PositionalPid gPatPosPid[3] = {
    PositionalPid(15.f, 65.f, 0.25f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(60.f, 100.f, 0.5f, 0.001f, 3.f, 7.f, 0.f),
    PositionalPid(25.f, 5.f, 0.05f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gPatVelPid[3] = {
    PositionalPid(0.4f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.8f, 0.f, 0.05f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.6f, 0.0f, 0.001f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
};
namespace GIMBAL {

class PatrolState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    // static constexpr float SECOND_YAW_MAX_ANG = 0.1309f; //±15°
    static constexpr float SECOND_YAW_MAX_ANG = 0.15f; //±17°

public:
    PatrolState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::PATROL), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            patPosPid_._[i] = &gPatPosPid[i];
            patVelPid_._[i] = &gPatVelPid[i];
        }
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
            eMA._[i] = cMA._[i];
        }
        patrolCnt = 0;
        gimbal_->enter();

        LOG::info("GimbalPatrol", "enter");
    }

    void run() final
    {
        DualStage_u<float> delArc = {};

        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.Pitch = cIA.Pitch;
        eIA.SecondYaw = cIA.SecondYaw;

        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);

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

        eMA.Pitch = PINYMOTOR::rangeMap((0.4f * cos(omiga * patrolCnt)) - 0.1f);
        eMA.SecondYaw = PINYMOTOR::rangeMap(SECOND_YAW_MAX_ANG * sin(omiga * patrolCnt));

        gimbal_->ctrl(eMA, patPosPid_, patVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            patPosPid_._[i]->reset();
            patVelPid_._[i]->reset();
        }

        LOG::info("GimbalPatrol", "exit");
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
    }

private:
    DualStage *gimbal_;

    static constexpr float PAT_FREQ = 1000.f; //hz,代码运行频率
    static constexpr float PERIOD = 1;        //s
    static constexpr float MAX_PAT_CNT = PERIOD * PAT_FREQ;
    float patrolCnt = 0;
    float maxPatCount = 0;
    float omiga = 2 * PI / PERIOD / PAT_FREQ; //rad/s

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStagePtr_u<PositionalPid> patPosPid_;
    DualStagePtr_u<PositionalPid> patVelPid_;
};
} // namespace GIMBAL
