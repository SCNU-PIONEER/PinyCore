#pragma once

#include "DualStage.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "IIR.hpp"
#include <cstdint>

static PositonalPid gLauPosPid[3] = {
    PositonalPid(130.f, 300.f, 30.f, 1.f / 500.f, 1.f, 120.f, 0.f),
    PositonalPid(80.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f),
    PositonalPid(80.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f)
};
static PositonalPid gLauVelPid[3] = {
    PositonalPid(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f),
    PositonalPid(0.035f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f),
    PositonalPid(0.035f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f)
};


namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_LAUNCH_TIME = 1000; // 3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

    static constexpr float PITCH_LAUNCH_STEP = 0.08f;
    static constexpr float FIRST_YAW_LAUNCH_STEP = 0.05f;
    static constexpr float SECOND_YAW_LAUNCH_STEP = 0.05f;


public:
    LaunchState(DualStage *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i <= 3; i++) {
            posPid_._[i] = &gLauPosPid[i];
            velPid_._[i] = &gLauVelPid[i];
        }
        launchStep.Pitch = PITCH_LAUNCH_STEP;
        launchStep.FirstYaw = FIRST_YAW_LAUNCH_STEP;
        launchStep.SecondYaw = SECOND_YAW_LAUNCH_STEP;
    }

    void enter() final
    {
        launchTotalTime_ = 0;
        launchFinishTime_ = 0;

        for (auto &i : cmdMotorAng_._) {
            i = 0.f;
        }

        gimbal_->enter();
        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        // determine if the target position (0) has been reached
        launchTotalTime_++;
        DualStage_u<float> gimbalAng;

        for (uint8_t i = 0; i < 3; i++) {
            gimbalAng._[i] = gimbal_->motors()->_[i]->posNorm();
        };

        if (std::fabs(PINYMOTOR::getMinorArc(0, gimbalAng.Pitch)) <
                    PITCH_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(0, gimbalAng.FirstYaw)) <
                    YAW_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(0, gimbalAng.SecondYaw)) <
                    YAW_LAUNCH_ANG_DEADBAND) {
            launchFinishTime_++;
        } else {
            launchFinishTime_ = 0;
        }

        // ctrl (smooth)
        for (uint8_t i = 0; i < 3; i++) {
            gimbal_->ctrl(gimbal_->motors()->_[i],
                          gimbal_->gimbalFilter()->_[i], posPid_._[i],
                          velPid_._[i], cmdMotorAng_._[i], launchStep._[i]);
        };
    }

    void exit() final
    {
        finishLaunching_ = true;
        gimbal_->gimbalYawMsg.isReset = true;

        for (uint8_t i = 0; i < 3; i++) {
            posPid_._[i]->reset();
            velPid_._[i]->reset();
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

    DualStagePtr_u<PositonalPid> posPid_;
    DualStagePtr_u<PositonalPid> velPid_;

    DualStage_u<float> cmdMotorAng_ = {}; //电机控制量

    uint16_t launchTotalTime_ = 0; // ms
    uint16_t launchFinishTime_ = 0;
    bool finishLaunching_ = false;
};

} // namespace GIMBAL
