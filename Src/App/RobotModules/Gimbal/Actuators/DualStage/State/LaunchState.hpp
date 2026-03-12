#pragma once

#include "DualStage.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "IIR.hpp"
#include <cstdint>

static PositionalPid gLauPosPid[3] = {
    PositionalPid(40.f, 20.f, 1.f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(4.f, 1.f, 0.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(15.f, 10.f, 3.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
static PositionalPid gLauVelPid[3] = {
    PositionalPid(0.4f, 0.4f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.5f, 0.1f, 0.f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.4f, 0.0f, 0.03f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};

namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_LAUNCH_TIME = 500; // 0.5s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

    static constexpr float PITCH_LAUNCH_STEP = 0.6f;
    static constexpr float FIRST_YAW_LAUNCH_STEP = 0.5f;
    static constexpr float SECOND_YAW_LAUNCH_STEP = 0.05f;

public:
    LaunchState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            lauPosPid_._[i] = &gLauPosPid[i];
            lauVelPid_._[i] = &gLauVelPid[i];
        }
        launchStep.Gim_s.Pitch = PITCH_LAUNCH_STEP;
        launchStep.Gim_s.FirstYaw = FIRST_YAW_LAUNCH_STEP;
        launchStep.Gim_s.SecondYaw = SECOND_YAW_LAUNCH_STEP;
    }

    void enter() final
    {
        launchTotalTime_ = 0;
        launchFinishTime_ = 0;

        gimbal_->enter();
        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        // determine if the target position (0) has been reached
        launchTotalTime_++;

        if (std::fabs(PINYMOTOR::getMinorArc(0, cMA_s.Pitch)) < PITCH_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(0, cMA_s.FirstYaw)) < YAW_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(0, cMA_s.SecondYaw)) < YAW_LAUNCH_ANG_DEADBAND) {
            launchFinishTime_++;
        } else {
            launchFinishTime_ = 0;
        }

        for (float &i : eMA._) {
            i = 0;
        }
        // ctrl (smooth)
        gimbal_->ctrlSmooth(eMA, launchStep, lauPosPid_, lauVelPid_);
    }

    void exit() final
    {
        finishLaunching_ = true;
        gimbal_->loadResetState(true);

        for (uint8_t i = 0; i < 3; i++) {
            lauPosPid_._[i]->reset();
            lauVelPid_._[i]->reset();
        }
        LOG::info("GimbalLaunch", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (launchFinishTime_ > MAX_LAUNCH_TIME)
            return FSMState_e::RUN;
        else
            return FSMState_e::LAUNCH;
    }

private:
    DualStage *gimbal_;

    DualStage_u<float> launchStep;

    DualStage_u<PositionalPid *> lauPosPid_;
    DualStage_u<PositionalPid *> lauVelPid_;

    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_s<float> &eMA_s = this->gimbal_->expMotorAng_.Gim_s;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_s<float> &cMA_s = this->gimbal_->curMotorAng_.Gim_s;

    uint16_t launchTotalTime_ = 0; // ms
    uint16_t launchFinishTime_ = 0;
    bool finishLaunching_ = false;
};

} // namespace GIMBAL
