#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid autoPitchAngPID(40.f, 5.f, 0.f, 0.001f, 0.5f, 20.f, 0.f);
static PositionalPid autoPitchVelPID(0.9f, 0.f, 0.f, 0.001f, 0.f, 2.23f, 0.f);
static PositionalPid autoYawAngPID(20.f, 50.f, 0.1f, 0.001f, 0.1f, 30.f, 0.f);
static PositionalPid autoYawVelPID(0.6f, 0.f, 0.f, 0.001f, 0.f, 2.23f, 0.f);

namespace GIMBAL {

class AutoState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *pitchAngPid;
            PositionalPid *pitchVelPid;

            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[4];
    };

    static constexpr float IMU_PITCH_START_ANG = -0.26f;
    static constexpr float IMU_PITCH_END_ANG = 0.21f;

    static constexpr float IMU_YAW_START_ANG = -PI / 1.f;
    static constexpr float IMU_YAW_END_ANG = PI / 1.f;

public:
    AutoState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::AUTO), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &autoPitchAngPID;
        pids_.pitchVelPid = &autoPitchVelPID;
        pids_.yawAngPid = &autoYawAngPID;
        pids_.yawVelPid = &autoYawVelPID;
    }

    void enter() final
    {
        gimbal_->enter();

        LOG::info("GimbalAuto", "enter");
    }

    void run() final
    {
        cmdImuPitchAng_ = gimbal_->vision.data().targetPitch;
        cmdImuYawAng_ = gimbal_->vision.data().targetYaw;

        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_, IMU_PITCH_START_ANG, IMU_PITCH_END_ANG);

        // float rangeCenter = PINYMOTOR::getMinorArc(gimbal_->ins().yaw,
        //                                            gimbal_->yaw()->posNorm());
        // imuYawMinAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_START_ANG);
        // imuYawMaxAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_END_ANG);
        // cmdImuYawAng_ = PINYMOTOR::clampArc(cmdImuYawAng_, imuYawMinAng_,
        //                                     imuYawMaxAng_);

        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAng_);
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdImuYawAng_);
    }

    void exit() final
    {
        for (auto &i : pids_._) {
            i->reset();
        }
        LOG::info("GimbalAuto", "exit");
    }
    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::RUN || !gimbal_->vision.data().control)
            return FSMState_e::RUN;
        else
            return FSMState_e::AUTO;
    }

private:
    Standard *gimbal_;

    Pids_u pids_;

    // float imuYawMinAng_ = IMU_YAW_START_ANG;
    // float imuYawMaxAng_ = IMU_YAW_END_ANG;

    float cmdImuPitchAng_;

    float cmdImuYawAng_;
};

} // namespace GIMBAL
