#include "Standard.hpp"

#include "PidBasic.hpp"

#include "GM6020.hpp"

#include "sdkconfig.h"
#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"

extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

Standard::Standard()
{
    InitConfig_s pitchGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = 500.0f,
        .posPID = new PositonalPid(50.f, 0.f, 0.f, 1.f / 500.f, 0.f, 35.f, 0.f),
        .velPID = new PositonalPid(0.03f, 0.05f, 0.f, 1.f / 500.f, 0.3f, 1.2f,
                                   0.f),
        .torqPID = nullptr,
        .isReverse = PITCH_REVERSE,
    };
    motors_.pitch = new DJIMOTOR::GM6020("Pitch", pitchGM6020Config);
    motors_.pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f",
              motors_.pitch->data().zeroAng);

    InitConfig_s yawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = 500.0f,
        .posPID =
                new PositonalPid(50.0f, 0.f, 0.f, 1.f / 500.f, 0.f, 35.f, 0.f),
        .velPID = new PositonalPid(0.03f, 0.05f, 0.000001f, 1.f / 500.f, 0.3f,
                                   1.2f, 0.f),
        .torqPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    motors_.yaw = new DJIMOTOR::GM6020("Yaw", yawGM6020Config);
    motors_.yaw->setZeroAng(YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Yaw zeros has set to %f", motors_.yaw->data().zeroAng);

    this->stateFactory_.addState(static_cast<uint8_t>(FSMState_e::RUN),
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(static_cast<uint8_t>(FSMState_e::LAUNCH),
                                 std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(static_cast<uint8_t>(FSMState_e::STOP),
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(
            static_cast<uint8_t>(FSMState_e::STOP)));
}

void Standard::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void Standard::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void Standard::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };
    this->insSub_->receive();

    updateEndYaw();
    updateBaseYaw();

    deltaYawMsg_.deltaYaw = PINYMOTOR::getMinorArc(endYawAng_, baseYawAng_);
    deltaYawPub_->publish();


    this->stateFactory_.update();
}

void Standard::updateEndYaw()
{
    if (this->isConfirmYawZero_) {
        this->endYawAng_ =
                getMinorArc(this->insData_.yaw, -this->constDeltaYawArc_);
    } else {
        // confirm zero
        if (motors_.yaw->posNorm() != 0.f) { // TODO: prefer to globalState
            this->constDeltaYawArc_ =
                    YAW_REVERSE ? getMinorArc(0.f, motors_.yaw->posNorm()) :
                                  getMinorArc(motors_.yaw->posNorm(), 0.f);
            this->isConfirmYawZero_ = true;
        } else {
            this->isConfirmYawZero_ = false;
        }
    }
}

void Standard::updateBaseYaw()
{
    this->baseYawAng_ = getMinorArc(this->endYawAng_,
                                    (motors_.yaw->posNorm() > PI) ?
                                            motors_.yaw->posNorm() - (2 * PI) :
                                            motors_.yaw->posNorm());
}

PINYMOTOR::IMotor *Standard::pitch() const { return motors_.pitch; }
PINYMOTOR::IMotor *Standard::yaw() const { return motors_.yaw; }
