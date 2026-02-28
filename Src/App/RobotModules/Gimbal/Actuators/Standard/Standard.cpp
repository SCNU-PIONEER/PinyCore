#include "Standard.hpp"
#include "Bsp.hpp"

#include "MG8016E_i6_RS485.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"

using namespace GIMBAL;
using namespace PINYMOTOR;

void ChassisCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN1, uid(), _buf, _len);
}

Standard::Standard()
{
    InitConfig_s yawConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&huart2),
        .comType = ComType_e::RS485,
        .workMode = WorkMode_e::MULTI_PDESVDES,
        .offsetId = 2,
        .txFreq = YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    motor_ = new LKMOTOR::MG8016Ei6RS485("Yaw", yawConfig);

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(*this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(*this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop() { motor_->cmd(PINYMOTOR::MotorCmdType_e::OFF); }

void Standard::enter() { motor_->cmd(PINYMOTOR::MotorCmdType_e::ON); }

void Standard::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };

    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) == pdTRUE) {
        chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    };
    chassisCtrlTxPacket_.setData().yaw = motor_->posNorm();

    if (motor_->globalState == PINYMOTOR::GlobalState_e::OFFLINE) {
        msg.state = GIMBAL::FSMState_e::STOP;
    }

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::motor() const { return motor_; }

void Standard::ctrl(float _pos, float _maxSpeed) { motor_->cmdPosVel(_pos, _maxSpeed); }

void Standard::loadResetState(bool _state) { chassisCtrlTxPacket_.setData().gimbalReset = _state; }
