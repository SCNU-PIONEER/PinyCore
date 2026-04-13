#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "Gimbal.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositionalPid gRunPosPid[3] = {
    PositionalPid(15.f, 10.f, 0.5f, 0.001f, 0.5f, 5.f, 0.f),
    PositionalPid(30.f, 0.f, 0.f, 1.f / 1000.f, 10.f, 110.f, 0.f),
    PositionalPid(50.f, 5.f, 1.f, 1.f / 1000.f, 1.2f, 120.f, 0.f),
};
//PositionalPid(30.f, 20.f, 5.f, 0.001f, 3.f, 25.f, 0.f),4310
static PositionalPid gRunVelPid[3] = {
    PositionalPid(2.5f, 0.f, 0.005f, 0.001f, 0.f, 7.f, 0.f),
    PositionalPid(0.5f, 0.f, 0.001f, 1.f / 1000.f, 0.f, 8.5f, 0.f),
    PositionalPid(5.f, 0.0f, 0.1f, 1.f / 1000.f, 0.f, 1.85f, 0.f),
};
//PositionalPid(0.5f, 0.5f, 0.f, 0.001f, 0.f, 7.f, 0.f),4310
namespace GIMBAL {

class RunState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.64578f; // -37°
    static constexpr float IMU_PITCH_MAX_ANG = 0.35f;     //20°

    static constexpr float SECOND_YAW_MAX_ANG = 0.1309f; //±15°

public:
    RunState(DualStage *_gimbal) : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i] = &gRunPosPid[i];
            runVelPid_._[i] = &gRunVelPid[i];
        }
    }

    void enter() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            eIA._[i] = cIA._[i];
        }
        gimbal_->enter();

        LOG::info("GimbalRun", "enter");
    }

    void run() final
    {
        eIA.Pitch += gimbal_->msg.wPitch * 0.001f;
        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.SecondYaw = 0.f; //run模式下小yaw不改变方向

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG, IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = rangeConversion(eIA.FirstYaw);

        gimbal_->ctrl(eIA, runPosPid_, runVelPid_);
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            runPosPid_._[i]->reset();
            runVelPid_._[i]->reset();
        }
        gimbal_->debug.gimbalState = gimbal_->msg.state;
        LOG::info("GimbalRun", "exit");
    }

    FSMState_e checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (gimbal_->msg.state == FSMState_e::NAV && gimbal_->v2eData.control)
            return FSMState_e::AUTO;
        else if (gimbal_->msg.state == FSMState_e::RUN && gimbal_->v2eData.control)
            return FSMState_e::AUTO; //平时调试用
        else if (gimbal_->msg.state == FSMState_e::NAV)
            return FSMState_e::NAV;
        else
            return FSMState_e::RUN;
    };

private:
    DualStage *gimbal_;

    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;


    DualStage_u<PositionalPid *> runPosPid_;
    DualStage_u<PositionalPid *> runVelPid_;
};
} // namespace GIMBAL
