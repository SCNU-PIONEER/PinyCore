#include "PowerController.hpp"


PowerController::PowerController(ChassisType_e chassisType)
        : chassisType_(chassisType)
{
    const uint8_t motorNum = static_cast<uint8_t>(chassisType_);
    motorNum_ = motorNum;

    cmdPower.resize(motorNum);
    relPower.resize(motorNum);
    setIq.resize(motorNum);
    setPower.resize(motorNum);

    for (int i = 0; i < motorNum; i++) {
        cmdPower[i] = 0;
        relPower[i] = 0;
        setIq[i] = 0;
        setPower[i] = 0;
    }
}

void PowerController::refereeDataUpdate()
{
    // TODO: update from referee
}
