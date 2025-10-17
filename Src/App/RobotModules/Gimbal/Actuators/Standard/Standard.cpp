#include "Standard.hpp"

#include "PidBasic.hpp"

#include "GM6020.hpp"

#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"


extern canHandle HCAN1;
extern canHandle HCAN2;

using namespace GIMBAL;
using namespace PINYMOTOR;

void ChassisCtrlTxPacket::send(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN1, uid(), _buf, _len);
}

Standard::Standard()
{
#if !APP_USE_COMM
    // NOLINTBEGIN
    gimbalYawPub_ = new Publisher<GimbalYawMsg_s>(
            &TopicRouter::instance().gimbalYawTopic, &gimbalYawMsg_);
    // NOLINTEND
#endif

    InitConfig_s pitchGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = PITCH_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
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
        .txFreq = YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = YAW_REVERSE,
    };
    motors_.yaw = new DJIMOTOR::GM6020("Yaw", yawGM6020Config);
    motors_.yaw->setZeroAng(YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Yaw zeros has set to %f", motors_.yaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH,
                                 std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));
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

    updateCtrlYaw();

#ifdef APP_GIMBAL_MASTER
    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) ==
        pdTRUE) {
    };
    chassisCtrlTxPacket_.setData().vx = cmsg.vx;
    chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    chassisCtrlTxPacket_.setData().gimbalYaw = ctrlYawAng_;
#endif

#if !APP_USE_COMM
    updateBaseYaw();
    gimbalYawMsg_.followYaw = ctrlYawAng_;
    gimbalYawMsg_.chassisYaw = baseYawAng_;
    gimbalYawPub_->publish();
#endif

    this->stateFactory_.update();
}

void Standard::updateCtrlYaw()
{
    if (this->isConfirmYawZero_) {
        this->ctrlYawAng_ =
                getMinorArc(this->insData_.yaw, this->constDeltaYawArc_);
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
    this->baseYawAng_ = getMinorArc(this->ctrlYawAng_,
                                    (motors_.yaw->posNorm() > PI) ?
                                            motors_.yaw->posNorm() - (2 * PI) :
                                            motors_.yaw->posNorm());
}

PINYMOTOR::IMotor *Standard::pitch() const { return motors_.pitch; }
PINYMOTOR::IMotor *Standard::yaw() const { return motors_.yaw; }

void Standard::ctrlPitch(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, motors_.pitch->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, motors_.pitch->vel());
    motors_.pitch->cmdTorq(outFilrPitch.process(torq));
}
void Standard::ctrlPitch(PID *_angPID, PID *_velPID, float _pos, float _vel)
{
    float del = getMinorArc(_pos, motors_.pitch->posNorm());
    float vel = _angPID->calc(fmax(-_vel, fmin(del, _vel)), 0);
    float torq = _velPID->calc(vel, motors_.pitch->vel());
    motors_.pitch->cmdTorq(outFilrPitch.process(torq));
}

void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, motors_.yaw->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, motors_.yaw->vel());
    motors_.yaw->cmdTorq(outFilrYaw.process(torq));
}
void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _pos, float _vel)
{
    float del = getMinorArc(_pos, motors_.yaw->posNorm());
    float vel = _angPID->calc(fmax(-_vel, fmin(del, _vel)), 0);
    float torq = _velPID->calc(vel, motors_.yaw->vel());
    motors_.yaw->cmdTorq(outFilrYaw.process(torq));
}

void Standard::loadResetState(bool _state)
{
#if !APP_USE_COMM
    gimbalYawMsg_.isReset = _state;
#endif
#ifdef APP_GIMBAL_MASTER
    chassisCtrlTxPacket_.setData().gimbalReset = _state;
#endif
}
