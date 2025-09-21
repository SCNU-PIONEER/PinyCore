#pragma once

#include "FSMState.hpp"
#include "DualStage.hpp"
#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

static PositonalPid gPatPosPid[3] = {
    PositonalPid(10.f, 0.001f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f),
    PositonalPid(18.f, 0.0f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f),
    PositonalPid(15.f, 0.0f, 0.f, 1.f / 1000.f, 1.f, 120.f, 0.f)
};
static PositonalPid gPatVelPid[3] = {
    PositonalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
    PositonalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f),
    PositonalPid(0.01f, 0.0f, 0.f, 1.f / 1000.f, 0.f, 1.2f, 0.f)
};
namespace GIMBAL {

class PatrolState : public FSMState<FSMState_e> {
    static constexpr float IMU_PITCH_MIN_ANG = -0.5f;
    static constexpr float IMU_PITCH_MAX_ANG = 0.3f;

    static constexpr float SECOND_YAW_MAX_ANG = 0.2618f; //±15°
    static constexpr float SECOND_YAW_MIN_ANG = -0.2618f;

    static constexpr float PITCH_PATROL_SPEED = 0.05f;
    static constexpr float SECOND_YAW_PATROL_SPEED = 0.05f;

public:
    PatrolState(DualStage *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::PATROL), gimbal_(_gimbal)
    {
        for (uint8_t i = 0; i < 3; i++) {
            patPosPid_._[i] = &gPatPosPid[i];
            patVelPid_._[i] = &gPatVelPid[i];
        }
    }

    void enter() final
    {
        gimbal_->enter();

        LOG::info("GimbalPatrol", "enter");
    }

    void run() final
    {
        DualStage_u<float> delArc;

        eIA.FirstYaw += gimbal_->msg.wYaw * 0.001f;
        eIA.Pitch += pitchPatDir * PITCH_PATROL_SPEED * 0.001f;
        eIA.SecondYaw += secondYawPatDir * SECOND_YAW_PATROL_SPEED * 0.001f;

        if (eIA.Pitch > IMU_PITCH_MAX_ANG) {
            pitchPatDir = -1;
        } else if (eIA.Pitch < IMU_PITCH_MIN_ANG) {
            pitchPatDir = 1;
        }
        if (eIA.SecondYaw > SECOND_YAW_MAX_ANG) {
            secondYawPatDir = -1;
        } else if (eIA.SecondYaw < SECOND_YAW_MIN_ANG) {
            secondYawPatDir = 1;
        }

        eIA.Pitch = PINYMOTOR::clampArc(eIA.Pitch, IMU_PITCH_MIN_ANG,
                                        IMU_PITCH_MAX_ANG);
        eIA.FirstYaw = gimbal_->rangeConversion(eIA.FirstYaw);
        eIA.SecondYaw = PINYMOTOR::clampArc(eIA.SecondYaw, SECOND_YAW_MIN_ANG,
                                            SECOND_YAW_MAX_ANG);


        for (uint8_t i = 0; i < 3; i++) {
            delArc._[i] = PINYMOTOR::getMinorArc(eIA._[i], cIA._[i]);

            eMA._[i] = PINYMOTOR::rangeMap(cMA._[i] + delArc._[i]);

            gimbal_->ctrl(gimbal_->motors()->_[i],
                          gimbal_->gimbalFilter()->_[i], patPosPid_._[i],
                          patVelPid_._[i], eMA._[i]);
        }
    }

    void exit() final
    {
        for (uint8_t i = 0; i < 3; i++) {
            patPosPid_._[i]->reset();
            patVelPid_._[i]->reset();
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

    float pitchPatDir = 1;
    float secondYawPatDir = 1;

    DualStage_u<float> &cMA = this->gimbal_->curMotorAng_;
    DualStage_u<float> &cIA = this->gimbal_->curImuAng_;
    DualStage_u<float> &eMA = this->gimbal_->expMotorAng_;
    DualStage_u<float> &eIA = this->gimbal_->expImuAng_;

    DualStagePtr_u<PositonalPid> patPosPid_;
    DualStagePtr_u<PositonalPid> patVelPid_;
};
} // namespace GIMBAL
