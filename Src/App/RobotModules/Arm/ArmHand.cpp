#include "ArmHand.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"

using namespace ARM;
using namespace PINYMOTOR;

using enum State_e;

static InitConfig_s handConfig = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
    .comType = ComType_e::FDCAN,
    .workMode = WorkMode_e::EMIT,
    .offsetId = 5,
    .txFreq = 100.0f,
    .posPID = nullptr,
    .velPID = nullptr,
};

Hand::Hand() : motor("Hand", handConfig), level(ClampLevel_e::ONE) {}

State_e Hand::clamp()
{
    using enum ClampLevel_e;
    switch (level) {
    case ONE:
        if (inRange(motor.data().ang, 1.8f, 2.2f)) {
            // level = ClampLevel_e::TWO;
            return FINISH;
        }

        motor.cmdMIT(5, 0.2, 0.04);

        if (motor.data().curr > 2.0f) {
            // 超过电流阈值，释放夹爪以防损坏
            release();
            return FAULT;
        } else {
            return MOVING;
        }
    case TWO:
        motor.cmdMIT(5, 0.5, 0.05);
        // 分别判断夹爪角度和电流是否在设定范围内
        if (inRange(motor.data().ang, 4.8f, 5.2f)) {
            return FINISH;
        } else if (motor.data().curr > 1.0f) {
            // 超过电流阈值，释放夹爪以防损坏
            release();
            return FINISH;
        } else {
            return MOVING;
        }
    case THREE:
    default:
        return FINISH;
    }
}

void Hand::release() { motor.cmdMIT(4, 0.2, 0.03); }

void Hand::enable() { motor.cmd(MotorCmdType_e::ON); }

void Hand::stop() { motor.cmd(MotorCmdType_e::OFF); }
