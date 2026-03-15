#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static volatile float launchPitchStep = 0.6f;
static volatile float launchYawStep = 0.5f;

static PositionalPid launchPitchAngPID(30.f, 100.f, 0.f, 0.001f, 3.f, 10.f, 0.f);
static PositionalPid launchPitchVelPID(0.7f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);
static PositionalPid launchYawAngPID(15.f, 30.f, 0.5f, 0.001f, 0.1f, 7.f, 0.f);
static PositionalPid launchYawVelPID(0.3f, 0.f, 0.0001f, 0.001f, 0.f, 2.23f, 0.f);

namespace GIMBAL {

class LaunchState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *pitchAngPid;
            PositionalPid *pitchVelPid;

            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[4];
    };

    static constexpr uint16_t MAX_LAUNCH_TIME = 300; // 0.3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

public:
    LaunchState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &launchPitchAngPID;
        pids_.pitchVelPid = &launchPitchVelPID;
        pids_.yawAngPid = &launchYawAngPID;
        pids_.yawVelPid = &launchYawVelPID;
    }

    void enter() final
    {
        launchTotalTime_ = 0;
        launchFinishTime_ = 0;

        // pitch target
        cmdMotorPitchAng_ = gimbal_->ins().pitch - gimbal_->pitch()->posNorm();

        // yaw target
        if (fabs(PINYMOTOR::getMinorArc(gimbal_->yaw()->posNorm(), 0)) > PI / 2.f) {
            cmdMotorYawAng_ = gimbal_->ins().yaw - gimbal_->yaw()->posNorm() + PI;
        } else {
            cmdMotorYawAng_ = gimbal_->ins().yaw - gimbal_->yaw()->posNorm();
        }

        gimbal_->enter();

        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        // determine if the target position (0) has been reached
        launchTotalTime_++;

        float pitchAng = gimbal_->pitch()->posNorm();
        float yawAng = gimbal_->yaw()->posNorm();

        float targetAng = 0;
        if (yawAng > PI / 2.f && yawAng < 3.f * PI / 2.f)
            targetAng = PI;
        else
            targetAng = 0;

        if (std::fabs(PINYMOTOR::getMinorArc(0, pitchAng)) < PITCH_LAUNCH_ANG_DEADBAND &&
            std::fabs(PINYMOTOR::getMinorArc(targetAng, yawAng)) < YAW_LAUNCH_ANG_DEADBAND) {
            launchFinishTime_++;
        } else {
            launchFinishTime_ = 0;
        }

        // ctrl (smooth)
        gimbal_->ctrlPitchSmooth(pids_.pitchAngPid, pids_.pitchVelPid, cmdMotorPitchAng_, launchPitchStep);
        gimbal_->ctrlYawSmooth(pids_.yawAngPid, pids_.yawVelPid, cmdMotorYawAng_, launchYawStep);
    }

    void exit() final
    {
        isLaunched_ = true;
        gimbal_->loadResetState(true);

        for (auto &i : pids_._) {
            i->reset();
        }

        LOG::info("GimbalLaunch", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (launchFinishTime_ > MAX_LAUNCH_TIME && gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (launchFinishTime_ > MAX_LAUNCH_TIME && gimbal_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else
            return FSMState_e::LAUNCH;
    }

private:
    Standard *gimbal_;

    Pids_u pids_;

    float cmdMotorPitchAng_;
    float cmdMotorYawAng_;

    uint16_t launchTotalTime_ = 0; // ms
    uint16_t launchFinishTime_ = 0;
    bool isLaunched_ = false;
};

} // namespace GIMBAL
