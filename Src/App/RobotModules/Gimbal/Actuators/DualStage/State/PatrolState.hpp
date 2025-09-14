#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositonalPid gPatPA(130.f, 300.f, 15.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gPatPV(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);
static PositonalPid gPatFYA(150.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gPatFYV(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);
static PositonalPid gPatSYA(150.f, 300.f, 25.f, 1.f / 500.f, 1.f, 120.f, 0.f);
static PositonalPid gPatSYV(0.025f, 0.0f, 0.f, 1.f / 500.f, 0.f, 1.2f, 0.f);


namespace GIMBAL {

class PatrolState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

public:
    PatrolState(DualStage *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::PATROL), gimbal_(_gimbal)
    {
        posPid_.Pitch = &gPatPA;
        velPid_.Pitch = &gPatPV;
        posPid_.FirstYaw = &gPatFYA;
        velPid_.FirstYaw = &gPatFYV;
        posPid_.SecondYaw = &gPatSYA;
        velPid_.SecondYaw = &gPatSYV;
    }

    void enter() final
    {
        cmdImuPitchAng_ = gimbal_->ins().pitch;
        cmdImuYawAng_ = gimbal_->ctrlYawAng();

        gimbal_->enter();

        LOG::info("GimbalPatrol", "enter");
    }

    void run() final {}

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            posPid_._[i]->reset();
            velPid_._[i]->reset();
        }

        LOG::info("GimbalPatrol", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (gimbal_->msg.state == FSMState_e::PATROL)
            return FSMState_e::PATROL;
        else
            return FSMState_e::RUN;
    }

private:
    DualStage *gimbal_;

    DualStagePtr_u<PositonalPid> posPid_;
    DualStagePtr_u<PositonalPid> velPid_;

    float cmdImuPitchAng_;
    float cmdImuYawAng_;
};
} // namespace GIMBAL
