#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "IIR.hpp"

static volatile float launchPitchStep = 0.6f;
static volatile float launchYawStep = 0.5f;

static PositionalPid launchPitchAngPID(15.f, 0.f, 0.001f, 0.001f, 0.f, 5.f,
                                       0.f);
static PositionalPid launchPitchVelPID(0.2f, 0.f, 0.f, 0.001f, 0.f, 1.2f, 0.f);
static PositionalPid launchYawAngPID(10.f, 0.f, 0.f, 0.001f, 0.f, 10.f, 0.f);
static PositionalPid launchYawVelPID(0.2f, 0.f, 0.f, 0.001f, 0.f, 1.2f, 0.f);

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

    static constexpr uint16_t MAX_LAUNCH_TIME = 300; // 3s

    static constexpr float PITCH_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;

public:
    LaunchState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
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

        cmdMotorPitchAng_ = 0.f;
        cmdMotorYawAng_ = 0.f;

        gimbal_->enter();

        LOG::info("GimbalLaunch", "enter");
    }

    void run() final
    {
        // determine if the target position (0) has been reached
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

        // ctrl (smooth)
        gimbal_->ctrlPitchSmooth(pids_.pitchAngPid, pids_.pitchVelPid,
                                 cmdMotorPitchAng_, launchPitchStep);
        gimbal_->ctrlYawSmooth(pids_.yawAngPid, pids_.yawVelPid,
                               cmdMotorYawAng_, launchYawStep);
    }

    void exit() final
    {
        finishLaunching_ = true;
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
        else if (launchFinishTime_ > MAX_LAUNCH_TIME)
            return FSMState_e::RUN;
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
    bool finishLaunching_ = false;
};

} // namespace GIMBAL
