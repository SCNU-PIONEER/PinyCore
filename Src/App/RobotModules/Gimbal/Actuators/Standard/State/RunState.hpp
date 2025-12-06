#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static volatile float pitchGimbalExpOffset = 0.f;
static volatile float yawGimbalExpOffset = 0.f;

static PositionalPid runPitchAngPID(40.f, 5.f, 0.f, 0.001f, 0.5f, 20.f, 0.f);
static PositionalPid runPitchVelPID(0.9f, 0.f, 0.f, 0.001f, 0.f, 2.23f, 0.f);
static PositionalPid runYawAngPID(20.f, 50.f, 0.1f, 0.001f, 0.1f, 30.f, 0.f);
static PositionalPid runYawVelPID(0.6f, 0.f, 0.f, 0.001f, 0.f, 2.23f, 0.f);

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *pitchAngPid;
            PositionalPid *pitchVelPid;

            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[4];
    };

    static constexpr float IMU_PITCH_START_ANG = -0.5f;
    static constexpr float IMU_PITCH_END_ANG = 0.3f;

    static constexpr float IMU_YAW_START_ANG = -PI / 1.f;
    static constexpr float IMU_YAW_END_ANG = PI / 1.f;

public:
    RunState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &runPitchAngPID;
        pids_.pitchVelPid = &runPitchVelPID;
        pids_.yawAngPid = &runYawAngPID;
        pids_.yawVelPid = &runYawVelPID;
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ins().yaw;

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        // update ctrl ang
        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_ + (gimbal_->msg.wPitch * 0.001f), IMU_PITCH_START_ANG,
                                              IMU_PITCH_END_ANG);
        cmdImuPitchAngWithOffset_ = cmdImuPitchAng_ + pitchGimbalExpOffset;
        cmdImuPitchAngWithOffset_ =
                PINYMOTOR::clampArc(cmdImuPitchAngWithOffset_, IMU_PITCH_START_ANG, IMU_PITCH_END_ANG);

        // update yaw range
        // float rangeCenter = PINYMOTOR::getMinorArc(gimbal_->ins().yaw,
        //                                            gimbal_->yaw()->posNorm());
        // imuYawMinAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_START_ANG);
        // imuYawMaxAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_END_ANG);
        // cmdImuYawAng_ = PINYMOTOR::clampArc(
        //         cmdImuYawAng_ + (gimbal_->msg.wYaw * 0.001f), imuYawMinAng_,
        //         imuYawMaxAng_);
        cmdImuYawAng_ = cmdImuYawAng_ + (gimbal_->msg.wYaw * 0.001f);
        cmdImuYawAngWithOffset_ = cmdImuYawAng_ + yawGimbalExpOffset;
        // cmdImuYawAngWithOffset_ = PINYMOTOR::clampArc(
        //         cmdImuYawAngWithOffset_, imuYawMinAng_, imuYawMaxAng_);

        gimbal_->ctrlPitch(pids_.pitchAngPid, pids_.pitchVelPid, cmdImuPitchAngWithOffset_);
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdImuYawAngWithOffset_);
    }

    void exit() final
    {
        for (auto &i : pids_._) {
            i->reset();
        }
        LOG::info("GimbalRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::AUTO && gimbal_->vision.data().control)
            return FSMState_e::AUTO;
        else
            return FSMState_e::RUN;
    }

private:
    Standard *gimbal_;

    Pids_u pids_;

    // float imuYawMinAng_ = IMU_YAW_START_ANG;
    // float imuYawMaxAng_ = IMU_YAW_END_ANG;

    float cmdImuPitchAng_;
    float cmdImuPitchAngWithOffset_;

    float cmdImuYawAng_;
    float cmdImuYawAngWithOffset_;

    // float cmdMotorPitchAng_;
    // float cmdMotorYawAng_;
};

} // namespace GIMBAL
