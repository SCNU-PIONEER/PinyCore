#pragma once

#include "DualStage.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include <cstdint>

static PositionalPid gLauPosPid[3] = {
    PositionalPid(20.f, 20.f, 1.f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(8.f, 0.f, 1.f, 0.001f, 3.f, 25.f, 0.f),
    PositionalPid(15.f, 10.f, 3.f, 1.f / 1000.f, 1.2f, 5.f, 0.f),
};
static PositionalPid gLauVelPid[3] = {
    PositionalPid(0.8f, 0.4f, 0.f, 0.001f, 1.f, 3.5f, 0.f),
    PositionalPid(0.55f, 0.4f, 0.f, 0.001f, 2.f, 7.f, 0.f),
    PositionalPid(1.5f, 0.05f, 0.03f, 1.f / 1000.f, 1.5f, 3.5f, 0.f),
};
namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_LAUNCH_TIME = 300; // 0.3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.05f;
    static constexpr float FIRST_YAW_LAUNCH_ANG_DEADBAND = 0.08f;
    static constexpr float SECOND_YAW_LAUNCH_ANG_DEADBAND = 0.01f;

    static constexpr float PITCH_LAUNCH_STEP = 0.4f;
    static constexpr float FIRST_YAW_LAUNCH_STEP = 0.5f;
    static constexpr float SECOND_YAW_LAUNCH_STEP = 0.2f;

public:
    LaunchState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            lauPosPid_._[i] = &gLauPosPid[i];
            lauVelPid_._[i] = &gLauVelPid[i];
        }
        launchStep.Pitch = PITCH_LAUNCH_STEP;
        launchStep.FirstYaw = FIRST_YAW_LAUNCH_STEP;
        launchStep.SecondYaw = SECOND_YAW_LAUNCH_STEP;
    }

    void enter() final
    {
        launchTime_ = 0;
        totalLaunchTime_ = 0;
        gimbal_->isLaunched_ = false;
        for (float &i : eMA._) {
            i = 0;
        }
        gimbal_->enter();
        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        float targetAng = 0; //缓启动大yaw目标角度为0或pi,根据当前角度选择较近的一个
        //导航要求必须一个缓启动方向，因此暂不实现双方向缓启动
        // if (cMA.FirstYaw > PI / 2.f && cMA.FirstYaw < 3.f * PI / 2.f) {
        //     targetAng = PI;
        //     eMA.FirstYaw = PI;
        // } else
        //     targetAng = 0;

        if (launchTime_ < MAX_LAUNCH_TIME) {
            if (std::fabs(PINYMOTOR::getMinorArc(0, cMA.Pitch)) < PITCH_LAUNCH_ANG_DEADBAND &&
                std::fabs(PINYMOTOR::getMinorArc(targetAng, cMA.FirstYaw)) < FIRST_YAW_LAUNCH_ANG_DEADBAND &&
                std::fabs(PINYMOTOR::getMinorArc(0, cMA.SecondYaw)) < SECOND_YAW_LAUNCH_ANG_DEADBAND) {
                launchTime_++;
            }

            gimbal_->ctrlSmooth(eMA, launchStep, lauPosPid_, lauVelPid_);
        } else {
            gimbal_->isLaunched_ = true;
        }

        // if (totalLaunchTime_ < MAX_LAUNCH_TIME * 5) {
        //     totalLaunchTime_++;
        // } else {
        //     gimbal_->isLaunched_ = true;
        // }
    }

    void exit() final
    {
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
        else if (gimbal_->isLaunched_)
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
    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;

    uint16_t launchTime_ = 0;
    uint16_t totalLaunchTime_ = 0;
};

} // namespace GIMBAL
