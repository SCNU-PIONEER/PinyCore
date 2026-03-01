#include "ArmHand.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"

using namespace ARM;
using namespace PINYMOTOR;

using enum State_e;

static InitConfig_s handConfig = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .workMode = WorkMode_e::QUAD_CURR,
    .offsetId = 1,
    .txFreq = 200.0f,
    .posPID = nullptr,
    .velPID = nullptr,
};

Hand::Hand() : motor("Hand", handConfig) {}

State_e Hand::clamp()
{
    motor.cmdTorq(-MOVE_CURRENT);

    if (motor.data().curr < -SAFE_CURRENT) {
        // 超过电流阈值，释放夹爪以防损坏
        release();
    } else if (motor.data().curr < -MOVE_CURRENT) {
        state = CLAMP;
    } else {
        state = MOVING;
    }
    return state;
}

void Hand::release()
{
    if (state == RELEASE) {
        motor.cmdTorq(0);
        return;
    }
    if (motor.data().curr > SAFE_CURRENT) {
        motor.cmdTorq(0);
        state = RELEASE;
    }
    state = MOVING;
    motor.cmdTorq(MOVE_CURRENT);
}

void Hand::enable() { motor.cmd(MotorCmdType_e::ON); }

void Hand::stop() { motor.cmd(MotorCmdType_e::OFF); }
