#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gMotPA(130.f, 300.f, 5.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositionalPid gMotPV(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 7.f, 0.f);
static PositionalPid gMotYA(50.f, 300.f, 5.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositionalPid gMotYV(0.01f, 0.0f, 0.f, 1.f / 500.f, 0.f, 0.1f, 0.f);

namespace GIMBAL {

class MotorRunState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *pitchAngPid;
            PositionalPid *pitchVelPid;

            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[4];
    };
    static constexpr float PITCH_MIN_ANG = 0.f;
    static constexpr float PITCH_MAX_ANG = 0.f;

public:
    MotorRunState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &gMotPA;
        pids_.pitchVelPid = &gMotPV;
        pids_.yawAngPid = &gMotYA;
        pids_.yawVelPid = &gMotYV;
    }

    void enter() final
    {
        cmdMotorPitchAng_ = gimbal_->pitch()->posNorm();
        cmdMotorYawAng_ = gimbal_->yaw()->posNorm();

        gimbal_->enter();

        LOG::info("GimbalMotorRun", "enter");
    }

    void run() final
    {
        // cmdMotorPitchAng_ = PINYMOTOR::clampArc(
        //         cmdMotorPitchAng_ + (gimbal_->msg.wPitch * 0.001f),
        //         PITCH_MIN_ANG, PITCH_MAX_ANG);
        cmdMotorPitchAng_ += gimbal_->msg.wPitch * 0.001f;
        cmdMotorYawAng_ += gimbal_->msg.wYaw * 0.001f;

        // joints' target pos
        cmdMotorPitchAng_ = PINYMOTOR::rangeMap(cmdMotorPitchAng_);
        cmdMotorYawAng_ = PINYMOTOR::rangeMap(cmdMotorYawAng_);

        // ctrl
        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid,
                           cmdMotorPitchAng_);
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdMotorYawAng_);
    }

    void exit() final
    {
        for (auto &i : pids_._) {
            i->reset();
        }

        LOG::info("GimbalMotorRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::RUN;
    }

private:
    Standard *gimbal_;

    Pids_u pids_;

    float cmdMotorPitchAng_;
    float cmdMotorYawAng_;
};
} // namespace GIMBAL