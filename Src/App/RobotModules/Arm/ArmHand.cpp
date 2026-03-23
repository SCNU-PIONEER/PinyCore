#include "ArmHand.hpp"
#include "Projdefs.hpp"
#include "StmLog.hpp"

using namespace ARM;
using namespace PINYMOTOR;

using enum State_e;

Hand::Hand(const char _name[16], InitConfig_s _config) : motor(_name, _config) {}

State_e Hand::clamp()
{
    motor.cmdTorq(-MOVE_TORQ);

    if (motor.data().curr < -SAFE_CURRENT) {
        // 超过电流阈值，释放夹爪以防损坏
        release();
    } else if (motor.data().curr < -CLAMP_CURRENT) {
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
        LOG::error("HAND", "Over current! Current: %.2f", motor.data().curr);
        state = RELEASE;
    }
    if (motor.data().tempture > 43.f) {
        motor.cmdTorq(0);
        LOG::error("HAND", "Over temperature! Temperature: %.2f", motor.data().tempture);
        state = FAULT;
    }
    state = MOVING;
    motor.cmdTorq(RELEASE_TORQ);
}

void Hand::enable()
{
    if (motor.globalState == GlobalState_e::OFFLINE) {
        LOG::error("HAND", "Motor is offline, cannot enable");
    }
    motor.cmd(MotorCmdType_e::ON);
}

void Hand::stop() { motor.cmd(MotorCmdType_e::OFF); }
