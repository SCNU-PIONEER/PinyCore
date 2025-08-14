#pragma once

#include "Gimbal.hpp"
#include "FSMState.hpp"
#include "MotorCommonMacros.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class LaunchState : public FSMState {
    static constexpr uint16_t MAX_LAUNCH_TIME = 3000; // 3s

    static constexpr float PITCH_LAUNCH_ANG = 2.08033204f; // need to be tuned
    static constexpr float YAW_LAUNCH_ANG = 5.9817214f;    // need to be tuned

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

public:
    LaunchState(Gimbal *_gimbal) : gimbal_(_gimbal), head_(_gimbal->head)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::LAUNCH));
    }

    void enter() final
    {
        launchTotalTime_ = 0;
        launchFinishTime_ = 0;
        head_->enter();
        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        launchTotalTime_++;

        float pitchAng = head_->joint(0)->posNorm();
        float yawAng = head_->joint(0)->posNorm();

        if (std::fabs(PINYMOTOR::getMinorArc(PITCH_LAUNCH_ANG, pitchAng)) <
                    PITCH_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(YAW_LAUNCH_ANG, yawAng)) <
                    YAW_LAUNCH_ANG_DEADBAND) {
            launchFinishTime_++;
        }

        head_->joint(0)->cmdPos(PITCH_LAUNCH_ANG, 10, 0, 0);
        head_->joint(1)->cmdPos(YAW_LAUNCH_ANG, 10, 0, 0);
    }

    void exit() final
    {
        finishLaunching_ = true;

        head_->joint(0)->setZeroAng();
        LOG::info("Gimbal", "Zero pitch set to %f",
                  head_->joint(0)->data().zeroAng);
        head_->joint(1)->setZeroAng();
        LOG::info("Gimbal", "Zero yaw set to %f",
                  head_->joint(1)->data().zeroAng);

        // TODO: Reset INS Yaw to 0

        LOG::info("GimbalLaunch", "exit");
    }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (launchFinishTime_ > MAX_LAUNCH_TIME)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Gimbal *gimbal_;
    Head *head_;

    uint16_t launchTotalTime_ = 0; // ms
    uint16_t launchFinishTime_ = 0;
    bool finishLaunching_ = false;
};

} // namespace GIMBAL
