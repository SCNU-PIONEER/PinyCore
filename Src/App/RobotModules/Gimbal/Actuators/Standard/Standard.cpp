#include "Standard.hpp"
#include "Bsp.hpp"

#include "MG8016E_i6_RS485.hpp"

#include "./State/LaunchState.hpp"
#include "./State/StopState.hpp"
#include "./State/RunState.hpp"

#include "PidBasic.hpp"
#include "dsp/fast_math_functions.h"
#include "MotorCommonMacros.hpp"

using namespace GIMBAL;
using namespace PINYMOTOR;

static PositionalPid posPid(8.f, 0.04f, 0.f, 0.01f, 4.f, 4.f, 0.005f);
static PositionalPid speedPid(300.f, 0.f, 10.f, 0.01f, 0.f, 350.f, 0.f);

void ChassisCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

void UITxPacket::transmit(uint8_t *_buf, uint16_t _len) { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); }


Standard::Standard()
{
    InitConfig_s yawConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&huart2),
        .comType = ComType_e::RS485,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    motor_ = new LKMOTOR::MG8016Ei6RS485("Yaw", yawConfig);

    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(*this));
    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(*this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(*this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop()
{
    posPid.reset();
    speedPid.reset();
    motor_->cmd(PINYMOTOR::MotorCmdType_e::OFF);
}

void Standard::enter()
{
    posPid.reset();
    speedPid.reset();
    motor_->cmd(PINYMOTOR::MotorCmdType_e::ON);
}

void Standard::update(const MsgBus_s *_bus)
{
    this->insSub_->receive();

    xQueueReceive(_bus->gimbalQueue, &msg, 0);

    /* 离线检测 */
    if (motor_->globalState == PINYMOTOR::GlobalState_e::OFFLINE) {
        msg.state = GIMBAL::FSMState_e::STOP;
        LOG::error("Standard", "motor offline!");
    }

#if APP_USE_COMM
    ChassisMsg_s cmsg;
    if (xQueueReceive(_bus->chassisQueue, &cmsg, 0) == pdTRUE) {
        if (cmsg.state != CHASSIS::FSMState_e::STOP) {
            if (isLaunched) {
                txPacket_.setData().state = cmsg.state;
            } else {
                txPacket_.setData().state = CHASSIS::FSMState_e::STOP;
            }
        } else {
            txPacket_.setData().state = CHASSIS::FSMState_e::STOP;
        }
        float deltaYaw = getMinorArc(motor_->posNorm(), 0.f);
        float cosYaw = 1.f, sinYaw = 0.f;
        if (fabsf(deltaYaw) > 0.12f) {
            cosYaw = arm_cos_f32(deltaYaw);
            sinYaw = arm_sin_f32(deltaYaw);
        }
        txPacket_.setData().vx = (cmsg.vx * cosYaw) - (cmsg.vy * sinYaw);
        txPacket_.setData().vy = (cmsg.vy * cosYaw) + (cmsg.vx * sinYaw);
    };
    txPacket_.setData().deltaYaw = getMinorArc(motor_->posNorm(), 0.f);
    uiPacket_.setData().gimbalState = msg.state;
#endif

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::motor() const { return motor_; }

void Standard::ctrl(float _delta)
{
    const float targetSpeed = posPid.calc(0, _delta);
    const float torq = speedPid.calc(targetSpeed, insData_.body.gz);
    motor_->cmdTorq(torq);
}
