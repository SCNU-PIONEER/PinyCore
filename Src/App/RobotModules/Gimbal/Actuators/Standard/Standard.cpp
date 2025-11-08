#include "Standard.hpp"

#include "PidBasic.hpp"

#include "GM6020.hpp"

#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"

using namespace GIMBAL;
using namespace PINYMOTOR;

Standard::Standard()
{
#if !APP_USE_COMM
    // NOLINTBEGIN
    chassisCtrlPub_ = new Publisher<ChassisCtrlMsg_s>(
            &TopicRouter::instance().chassisCtrlTopic, &chassisCtrlMsg_);
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
    outFilrYaw_.reset();
    outFilrPitch_.reset();

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

    chassisCtrlMsg_.deltaYaw = motors_.yaw->posNorm();
    chassisCtrlPub_->publish();

    this->stateFactory_.update();
}

PINYMOTOR::IMotor *Standard::pitch() const { return motors_.pitch; }
PINYMOTOR::IMotor *Standard::yaw() const { return motors_.yaw; }

void Standard::ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _pos,
                               float _step)
{
    float delta = getMinorArc(_pos, motors_.pitch->posNorm());
    delta = clamp(delta, -_step, _step);
    ctrlPitch(_angPID, _velPID, motors_.pitch->posNorm() + delta);
}

void Standard::ctrlPitch(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, motors_.pitch->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gy);
    motors_.pitch->cmdTorq(outFilrPitch_.process(torq));
}

void Standard::ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos,
                             float _step)
{
    float delta = getMinorArc(_pos, motors_.yaw->posNorm());
    delta = clamp(delta, -_step, _step);
    ctrlYaw(_angPID, _velPID, motors_.yaw->posNorm() + delta);
}

void Standard::ctrlYaw(PID *_angPID, PID *_velPID, float _pos)
{
    float del = getMinorArc(_pos, motors_.yaw->posNorm());
    float vel = _angPID->calc(del, 0);
    float torq = _velPID->calc(vel, insData_.body.gz);
    motors_.yaw->cmdTorq(outFilrYaw_.process(torq));
}

void Standard::loadResetState(bool _state) { chassisCtrlMsg_.isReset = _state; }
