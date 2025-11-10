#pragma once

#include "FSMState.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static volatile float yawGimbalExpOffset = 0.f;

static PositionalPid runYawAngPID(25.f, 30.f, 0.f, 0.001f, 0.1f, 10.f, 0.f);
static PositionalPid runYawVelPID(0.3f, 0.f, 0.f, 0.001f, 0.f, 1.2f, 0.f);

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PositionalPid *yawAngPid;
            PositionalPid *yawVelPid;
        };
        PID *_[2];
    };

    static constexpr float IMU_PITCH_START_ANG = -0.26f;
    static constexpr float IMU_PITCH_END_ANG = 0.21f;

    static constexpr float IMU_YAW_START_ANG = -PI / 1.f;
    static constexpr float IMU_YAW_END_ANG = PI / 1.f;

public:
    RunState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        pids_.yawAngPid = &runYawAngPID;
        pids_.yawVelPid = &runYawVelPID;
    }

    void enter() final
    {
        cmdImuYawAng_ = gimbal_->ins().yaw;

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        // update yaw range
        float rangeCenter = PINYMOTOR::getMinorArc(gimbal_->ins().yaw,
                                                   gimbal_->yaw()->posNorm());
        imuYawMinAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_START_ANG);
        imuYawMaxAng_ = PINYMOTOR::rangeMap(rangeCenter + IMU_YAW_END_ANG);

        // cmdImuYawAng_ = PINYMOTOR::clampArc(
        //         cmdImuYawAng_ + (gimbal_->msg.wYaw * 0.001f), imuYawMinAng_,
        //         imuYawMaxAng_);
        cmdImuYawAng_ = cmdImuYawAng_ + (gimbal_->msg.yaw * 0.001f);
        cmdImuYawAngWithOffset_ = cmdImuYawAng_ + yawGimbalExpOffset;
        // cmdImuYawAngWithOffset_ = PINYMOTOR::clampArc(
        //         cmdImuYawAngWithOffset_, imuYawMinAng_, imuYawMaxAng_);

        // calc ang with motor
        float insYawDelArc = PINYMOTOR::getMinorArc(cmdImuYawAngWithOffset_,
                                                    gimbal_->ins().yaw);

        // joints' target pos
        cmdMotorYawAng_ =
                PINYMOTOR::rangeMap(gimbal_->yaw()->posNorm() + insYawDelArc);

        // ctrl
        gimbal_->ctrlYaw(pids_.yawAngPid, pids_.yawVelPid, cmdMotorYawAng_);
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

    float imuYawMinAng_ = IMU_YAW_START_ANG;
    float imuYawMaxAng_ = IMU_YAW_END_ANG;


    float cmdImuYawAng_;
    float cmdImuYawAngWithOffset_;

    float cmdMotorYawAng_;
};
} // namespace GIMBAL
