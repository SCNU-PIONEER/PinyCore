#include "Standard.hpp"
#include "Bsp.hpp"

#include "MG8016E_i6_RS485.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"

#include "dsp/fast_math_functions.h"
#include "MotorCommonMacros.hpp"

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

#if APP_USE_COMM
    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) == pdTRUE) {
        if (cmsg.state == CHASSIS::FSMState_e::RUN) {
            if (isLaunched) {
                chassisCtrlTxPacket_.setData().state = cmsg.state;
            } else {
                chassisCtrlTxPacket_.setData().state = CHASSIS::FSMState_e::STOP;
            }
        }
        float deltaYaw = getMinorArc(motor_->posNorm(), 0.f);
        float cosYaw = 1.f, sinYaw = 0.f;
        if (fabsf(deltaYaw) > 0.12f) {
            cosYaw = arm_cos_f32(deltaYaw);
            sinYaw = arm_sin_f32(deltaYaw);
        }
        chassisCtrlTxPacket_.setData().vx = (cmsg.vx * cosYaw) - (cmsg.vy * sinYaw);
        chassisCtrlTxPacket_.setData().vy = (cmsg.vy * cosYaw) + (cmsg.vx * sinYaw);
    };
    chassisCtrlTxPacket_.setData().yaw = motor_->posNorm();
#endif

    /* 离线检测 */
    if (motor_->globalState == PINYMOTOR::GlobalState_e::OFFLINE) {
        msg.state = GIMBAL::FSMState_e::STOP;
    }

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::motor() const { return motor_; }

void Standard::ctrl(float _pos, float _maxSpeed) { motor_->cmdPosVel(_pos, _maxSpeed); }
