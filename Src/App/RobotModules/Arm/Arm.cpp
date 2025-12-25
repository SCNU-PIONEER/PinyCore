#include <memory>

#include "./Arm.hpp"
#include "./ArmMotor.hpp"
#include "StmLog.hpp"
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "Cmd.hpp"
#include "PidBasic.hpp"

using namespace ARM;
using namespace PINYMOTOR;

static PositionalPid liftingPosPid = PositionalPid(15.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f);

static InitConfig_s m3508Config = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .workMode = WorkMode_e::QUAD_CURR,
    .offsetId = 4, //fix
    .txFreq = 100.0f,
    .posPID = &liftingPosPid,
    .velPID = new IncrementalPid(0.5f, 0.005f, 0.f, 4.f, 0.f),
};
Lifting::Lifting() : motor("Lifting", m3508Config) {}


Arm::Arm()
{
    /* FSM */
    stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(*this));
    stateFactory_.addState(FSMState_e::NORMAL, std::make_unique<NormalState>(*this));
    stateFactory_.init(stateFactory_.getNextState(FSMState_e::STOP));

    LOG::info("ARM", "register");

    motors.stop();
}

void Arm::update(MsgBus_s *_param)
{
    if (xQueueReceive(_param->armQueue, &msg, 0) == pdTRUE) {
        uint32_t i = 0;
    }
    motors.update();
    stateFactory_.update();
}

void OreBin::init()
{
    pwm1 = std::make_unique<Pwm>(&htim1, TIM_CHANNEL_1);
    pwm2 = std::make_unique<Pwm>(&htim1, TIM_CHANNEL_3);
    pwm1->start();
    pwm2->start();

    LOG::info("OreBin", "init");
}

void OreBin::loop(ArmMsg_s &_msg)
{
    if (_msg.isStucked) {
        pwm1->setDutyCycle(stuck_angle);
        pwm2->setDutyCycle(stuck_angle);
    } else if (!_msg.isStucked) {
        pwm1->setDutyCycle(release_angle);
        pwm2->setDutyCycle(release_angle);
    }
}
void OreBin::stop()
{
    LOG::info("OreBin", "stop");
    pwm1->stop();
    pwm2->stop();
}


void Lifting::lift() { motor.cmdPosVel(motor.pos() + 100.f, 2.f); }

void Lifting::enable() { motor.cmd(MotorCmdType_e::ON); }

void Lifting::stop() { motor.cmd(MotorCmdType_e::OFF); }
