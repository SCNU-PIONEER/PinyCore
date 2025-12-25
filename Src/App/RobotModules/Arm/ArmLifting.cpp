#include "ArmLifting.hpp"
#include "PidBasic.hpp"
#include "Cmd.hpp"

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

void Lifting::lift() { motor.cmdPosVel(motor.pos() + 100.f, 2.f); }

void Lifting::enable() { motor.cmd(MotorCmdType_e::ON); }

void Lifting::stop() { motor.cmd(MotorCmdType_e::OFF); }
