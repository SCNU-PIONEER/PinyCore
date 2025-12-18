#pragma once

#include "FSMState.hpp"
#include "Gimbal.hpp"
#include "Vision.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static volatile float navPitchGimbalExpOffset = 0.f;
static volatile float navYawGimbalExpOffset = 0.f;

static PositionalPid navPitchAngPID(80.f, 50.f, 0.001f, 0.001f, 1.f, 15.f, 0.f);
static PositionalPid navPitchVelPID(0.7f, 0.f, 0.f, 0.001f, 0.f, 7.f, 0.f);
static PositionalPid navYawAngPID(60.f, 30.f, 1.5f, 0.001f, 0.1f, 10.f, 0.f);
static PositionalPid navYawVelPID(0.5f, 0.f, 0.f, 0.001f, 0.f, 2.23f, 0.f);

namespace GIMBAL {

class NavState : public FSMState<FSMState_e> {
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
    NavState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::NAV), gimbal_(_gimbal)
    {
        pids_.pitchAngPid = &navPitchAngPID;
        pids_.pitchVelPid = &navPitchVelPID;
        pids_.yawAngPid = &navYawAngPID;
        pids_.yawVelPid = &navYawVelPID;
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ins().yaw;

        gimbal_->enter();

        LOG::info("GimbalNav", "enter");
    }

    void run() final
    {
        // update ctrl ang
        cmdImuPitchAng_ = PINYMOTOR::clampArc(cmdImuPitchAng_ + (gimbal_->msg.wPitch * 0.001f), IMU_PITCH_START_ANG,
                                              IMU_PITCH_END_ANG);
        cmdImuPitchAngWithOffset_ = cmdImuPitchAng_ + navPitchGimbalExpOffset;
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
        cmdImuYawAngWithOffset_ = cmdImuYawAng_ + navYawGimbalExpOffset;
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
        LOG::info("GimbalNav", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::PATROL && vision->visionToEData().control)
            return FSMState_e::PATROL;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return FSMState_e::NAV;
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
