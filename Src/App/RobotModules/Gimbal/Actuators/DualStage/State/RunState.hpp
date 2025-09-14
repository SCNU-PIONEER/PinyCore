#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositonalPid gRunPA(130.f, 300.f, 30.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gRunPV(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);
static PositonalPid gRunFYA(80.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gRunFYV(0.035f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);
static PositonalPid gRunSYA(80.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gRunSYV(0.035f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);

namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    RunState(DualStage *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        posPid_.Pitch = &gRunPA;
        velPid_.Pitch = &gRunPV;
        posPid_.FirstYaw = &gRunFYA;
        velPid_.FirstYaw = &gRunFYV;
        posPid_.SecondYaw = &gRunSYA;
        velPid_.SecondYaw = &gRunSYV;
    }

    void enter() final
    {
        cmdImuAng_.Pitch = gimbal_->ins().pitch;
        cmdImuAng_.FirstYaw = gimbal_->ctrlYawAng();
        // cmdImuAng_.SecondYaw = 0.f; //此版本小yaw在run模式下不动，并不是失能

        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        cmdImuAng_.Pitch = PINYMOTOR::clampArc(
                cmdImuAng_.Pitch + (gimbal_->msg.wPitch * 0.001f),
                IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);

        cmdImuAng_.FirstYaw += gimbal_->msg.wYaw * 0.001f;

        cmdImuAng_.FirstYaw = cmdImuAng_.FirstYaw > PI ?
                                      cmdImuAng_.FirstYaw - (2 * PI) :
                                      (cmdImuAng_.FirstYaw < -PI ?
                                               cmdImuAng_.FirstYaw + (2 * PI) :
                                               cmdImuAng_.FirstYaw);

        float insPitchDelArc =
                PINYMOTOR::getMinorArc(cmdImuAng_.Pitch, gimbal_->ins().pitch);

        float insYawDelArc = PINYMOTOR::getMinorArc(cmdImuAng_.FirstYaw,
                                                    gimbal_->ctrlYawAng());

        //joints' target pos
        cmdMotorAng_.Pitch = PINYMOTOR::rangeMap(
                gimbal_->motors()->Pitch->posNorm() +
                (DualStage::PITCH_REVERSE ? -insPitchDelArc : insPitchDelArc));
        cmdMotorAng_.FirstYaw = PINYMOTOR::rangeMap(
                gimbal_->motors()->FirstYaw->posNorm() +
                (DualStage::FIRST_YAW_REVERSE ? -insYawDelArc : insYawDelArc));

        cmdMotorAng_.SecondYaw = 0.f; //此版本小yaw在run模式下不动，并不是失能

        // ctrl
        for (uint8_t i = 0; i < 3; i++) {
            gimbal_->ctrl(gimbal_->motors()->_[i],
                          gimbal_->gimbalFilter()->_[i], posPid_._[i],
                          velPid_._[i], cmdMotorAng_._[i]);
        }
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            posPid_._[i]->reset();
            velPid_._[i]->reset();
        }

        LOG::info("GimbalRun", "exit");
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
    DualStage *gimbal_;

    DualStagePtr_u<PositonalPid> posPid_;
    DualStagePtr_u<PositonalPid> velPid_;

    DualStage_u<float> cmdImuAng_ = {};   //imu控制量
    DualStage_u<float> cmdMotorAng_ = {}; //电机控制量
};
} // namespace GIMBAL
