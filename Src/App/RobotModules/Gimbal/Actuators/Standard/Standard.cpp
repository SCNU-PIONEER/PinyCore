#include "Standard.hpp"
#include "Bsp_can.hpp"
#include "GM6020.hpp"
#include "MotorCommonMacros.hpp"
#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
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
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    yaw_ = new DJIMOTOR::GM6020("Yaw", yawConfig);
    yaw_->setZeroAng(YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Yaw zeros has set to %f", yaw_->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH,
                                 std::make_unique<LaunchState>(*this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(*this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop()
{
    outFilrYaw_.reset();

    yaw_->cmd(PINYMOTOR::MotorCmdType_e::OFF);
}

void Standard::enter() { yaw_->cmd(PINYMOTOR::MotorCmdType_e::ON); }

void Standard::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };
    this->insSub_->receive();

#ifdef APP_GIMBAL_MASTER
    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) ==
        pdTRUE) {
        chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    };
    chassisCtrlTxPacket_.setData().deltaYaw = yaw_->posNorm();

#endif

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::yaw() const { return yaw_; }

void Standard::ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos,
                             float _step)
{
    float delta = getMinorArc(_pos, yaw_->posNorm());
    delta = clamp(delta, -_step, _step);
    ctrlYaw(_angPID, _velPID, yaw_->posNorm() + delta);
}

void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, yaw_->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gz);
    yaw_->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::loadResetState(bool _state)
{
    chassisCtrlTxPacket_.setData().gimbalReset = _state;
}
