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

    isGimbalResverse.Pitch = DualStage::PITCH_REVERSE;
    isGimbalResverse.FirstYaw = DualStage::FIRST_YAW_REVERSE;
    isGimbalResverse.SecondYaw = DualStage::SECOND_YAW_REVERSE;

    InitConfig_s pitchGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
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
        .offsetId = 1,
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
        .offsetId = 2,
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
                                 std::make_unique<PatrolState>(this));
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

    //ins -pi~pi
    curImuAng_.Pitch = insData_.pitch;
    curImuAng_.SecondYaw = insData_.yaw; //云台imu在小yaw上

    for (uint8_t i = 0; i < 3; i++) {
        curImuAng_._[i] = rangeConversion(curImuAng_._[i]); // 0~2*pi
        curMotorAng_._[i] = motors_._[i]->posNorm();        // 0~2*pi
    }

    curImuAng_.FirstYaw =
            rangeConversion(curImuAng_.SecondYaw - curMotorAng_.SecondYaw);

    compensateImuAngle();
    // updateBaseYaw();

    // gimbalYawMsg.followYaw = ctrlYawAng_;
    // gimbalYawMsg.chassisYaw = baseYawAng_;
    gimbalYawPub_->publish();

    this->stateFactory_.update();

    if (msg.state == FSMState_e::STOP) {
        for (uint8_t i = 0; i < 3; i++) {
            motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(0.f));
        }
    }
}

void DualStage::compensateImuAngle()
{
    float tempComAng;
    if (this->isConfirmYawZero_) {
        //控制大yaw
        this->curImuAng_.SecondYaw = rangeConversion(getMinorArc(
                this->curImuAng_.SecondYaw, -this->constDeltaYawArc_));
        this->curImuAng_.FirstYaw = rangeConversion(getMinorArc(
                this->curImuAng_.FirstYaw, -this->constDeltaYawArc_));

        //与gimbal类保持相同
        this->ctrlYawAng_ = this->curImuAng_.FirstYaw;
    } else {
        // confirm zero
        if (motors_.FirstYaw->posNorm() != 0.f &&
            motors_.SecondYaw->posNorm() != 0.f) {
            // TODO: prefer to globalState
            /*实际在获取上电后imu确定的坐标系与车体坐标系正方向的差角,当前imu在小yaw上.
            小yaw+大yaw=imu坐标系与车体坐标系正方向的差角*/
            tempComAng =
                    motors_.SecondYaw->posNorm() + motors_.FirstYaw->posNorm();
            //0~2*pi
            tempComAng = rangeConversion(tempComAng);
            this->constDeltaYawArc_ = FIRST_YAW_REVERSE ?
                                              getMinorArc(0.f, tempComAng) :
                                              getMinorArc(tempComAng, 0.f);
            this->isConfirmYawZero_ = true;
        } else {
            this->isConfirmYawZero_ = false;
        }
    }
}


void DualStage::updateBaseYaw() {}

void DualStage::ctrl(PINYMOTOR::IMotor *_motor, FILTER::LPF *_filter,
                     PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, _motor->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, _motor->vel());
    if (msg.state == FSMState_e::STOP) {
        torq = 0.f;
    }
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
