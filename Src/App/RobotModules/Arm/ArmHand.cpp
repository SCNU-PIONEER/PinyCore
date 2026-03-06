#include "ArmHand.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"

using namespace ARM;
using namespace PINYMOTOR;

using enum State_e;

static InitConfig_s hand1Config = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .workMode = WorkMode_e::QUAD_CURR,
    .offsetId = 1,
    .txFreq = 500.0f,
    .posPID = nullptr,
    .velPID = nullptr,
};

static InitConfig_s hand2Config = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .workMode = WorkMode_e::QUAD_CURR,
    .offsetId = 2,
    .txFreq = 500.0f,
    .posPID = nullptr,
    .velPID = nullptr,
};

Hand::Hand() : left("Hand1", hand2Config), right("Hand2", hand1Config) {}

State_e Hand::clamp()
{
    right.cmdTorq(-MOVE_TORQ);

    if (right.data().curr < -SAFE_CURRENT) {
        // 超过电流阈值，释放夹爪以防损坏
        release();
    } else if (right.data().curr < -CLAMP_CURRENT) {
        state = CLAMP;
    } else {
        state = MOVING;
    }
    return state;
}

void Hand::release()
{
    if (state == RELEASE) {
        right.cmdTorq(0);
        return;
    }
    if (right.data().curr > SAFE_CURRENT) {
        right.cmdTorq(0);
        state = RELEASE;
    }
    state = MOVING;
    right.cmdTorq(MOVE_TORQ);
}

void Hand::enable()
{
    left.cmd(MotorCmdType_e::ON);
    right.cmd(MotorCmdType_e::ON);
}

void Hand::stop()
{
    left.cmd(MotorCmdType_e::OFF);
    right.cmd(MotorCmdType_e::OFF);
}
