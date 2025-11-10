#pragma once

#include "LaunchState.hpp"
namespace GIMBAL {

class MotorRunState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[2];
    };

    static constexpr float PITCH_MIN_ANG = 0.f;
    static constexpr float PITCH_MAX_ANG = 0.f;

public:
    MotorRunState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        pids_.yawAngPid = &launchYawAngPID;
        pids_.yawVelPid = &launchYawVelPID;
    }

    void enter() final
    {
        cmdMotorYawAng_ = gimbal_->yaw()->posNorm();

        gimbal_->enter();

        LOG::info("GimbalMotorRun", "enter");
    }

    void run() final
    {
        cmdMotorYawAng_ += gimbal_->msg.yaw * 0.001f;

        // joints' target pos
        cmdMotorYawAng_ = PINYMOTOR::rangeMap(cmdMotorYawAng_);

        // ctrl
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
        using enum FSMState_e;
        switch (gimbal_->msg.state) {
        case STOP:
            return STOP;
        default:
            return RUN;
        }
    }

private:
    Standard *gimbal_;

    Pids_u pids_;

    float cmdMotorYawAng_;
};
} // namespace GIMBAL
