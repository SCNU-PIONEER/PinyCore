#include "Gimbal.hpp"
#include "MotorCommonMacros.hpp"
#include "DualStage.hpp"

#include "PidBasic.hpp"

#include "GM6020.hpp"

#include "sdkconfig.h"
#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/PatrolState.hpp"
#include <cstdint>


extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

static FILTER::LPF gimbalFilterOutput[3]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
};

DualStage::DualStage()
{
    for (uint8_t i = 0; i < 3; i++) {
        gimbalFilterOutput_._[i] = &gimbalFilterOutput[i];
    }

    InitConfig_s pitchGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = PITCH_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = PITCH_REVERSE,
    };
    motors_.Pitch = new DJIMOTOR::GM6020("Pitch", pitchGM6020Config);
    motors_.Pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f",
              motors_.Pitch->data().zeroAng);

    InitConfig_s firstYawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = FIRST_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = FIRST_YAW_REVERSE,
    };
    motors_.FirstYaw = new DJIMOTOR::GM6020("Yaw", firstYawGM6020Config);
    motors_.FirstYaw->setZeroAng(FIRST_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Big yaw zeros has set to %f",
              motors_.FirstYaw->data().zeroAng);

    InitConfig_s secondYawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = SECOND_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = SECOND_YAW_REVERSE,
    };
    motors_.SecondYaw = new DJIMOTOR::GM6020("Yaw", secondYawGM6020Config);
    motors_.SecondYaw->setZeroAng(SECOND_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Small yaw zeros has set to %f",
              motors_.SecondYaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH,
                                 std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::PATROL,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));
}

void DualStage::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void DualStage::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void DualStage::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };
    this->insSub_->receive();

    updateCtrlYaw();
    updateBaseYaw();

    gimbalYawMsg.followYaw = ctrlYawAng_;
    gimbalYawMsg.chassisYaw = baseYawAng_;
    gimbalYawPub_->publish();

    this->stateFactory_.update();
}


void DualStage::updateCtrlYaw()
{
    if (this->isConfirmYawZero_) {
        this->ctrlYawAng_ =
                getMinorArc(this->firstYawAng_.yaw, -this->constDeltaYawArc_);
    } else {
        // confirm zero
        if (motors_.FirstYaw->posNorm() != 0.f) { // TODO: prefer to globalState
            this->constDeltaYawArc_ =
                    FIRST_YAW_REVERSE ?
                            getMinorArc(0.f, motors_.FirstYaw->posNorm()) :
                            getMinorArc(motors_.FirstYaw->posNorm(), 0.f);
            this->isConfirmYawZero_ = true;
        } else {
            this->isConfirmYawZero_ = false;
        }
    }
}

void DualStage::updateBaseYaw()
{
    // this->baseYawAng_ = getMinorArc(
    //         this->ctrlYawAng_, (motors_.firstYaw->posNorm() > PI) ?
    //                                    motors_.firstYaw->posNorm() - (2 * PI) :
    //                                    motors_.firstYaw->posNorm());
}

void DualStage::ctrl(PINYMOTOR::IMotor *_motor, FILTER::LPF *_filter,
                     PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, _motor->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, _motor->vel());
    _motor->cmdTorq(_filter->process(torq));
}

void DualStage::ctrl(PINYMOTOR::IMotor *_motor, FILTER::LPF *_filter,
                     PID *_angPID, PID *_velPID, float _pos, float _vel)
{
    float del = getMinorArc(_pos, _motor->posNorm());
    float vel = _angPID->calc(fmax(-_vel, fmin(del, _vel)), 0);
    float torq = _velPID->calc(vel, _motor->vel());
    _motor->cmdTorq(_filter->process(torq));
}
