#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

namespace GIMBAL::STANDARD {

class LaunchState : public FSMState {
    static constexpr uint16_t MAX_LAUNCH_TIME = 1000; // 3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

public:
    LaunchState(Standard *_gimbal) : gimbal_(_gimbal)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::LAUNCH));
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
        launchTotalTime_++;

        float pitchAng = gimbal_->pitch()->posNorm();
        float yawAng = gimbal_->yaw()->posNorm();

        if (std::fabs(PINYMOTOR::getMinorArc(0, pitchAng)) <
                    PITCH_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(0, yawAng)) <
                    YAW_LAUNCH_ANG_DEADBAND) {
            launchFinishTime_++;
        } else {
            launchFinishTime_ = 0;
        }

        gimbal_->pitch()->cmdPos(0, 5, 0, 0);
        gimbal_->yaw()->cmdPos(0, 5, 0, 0);
    }

    void exit() final
    {
        finishLaunching_ = true;

        LOG::info("GimbalLaunch", "exit");
    }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (launchFinishTime_ > MAX_LAUNCH_TIME)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Standard *gimbal_;

    uint16_t launchTotalTime_ = 0; // ms
    uint16_t launchFinishTime_ = 0;
    bool finishLaunching_ = false;
};

} // namespace GIMBAL::STANDARD
