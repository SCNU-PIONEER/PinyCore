#include "PowerController.hpp"
#include <cstdint>

PowerController::PowerController(ChassisType_e _chassisType)
        : chassisType_(_chassisType)
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

void PowerController::dynamicPower(float _capVoltage)
{
    float bufferDP = std::clamp(energyPid_->calc(expPowerBuffer, powerBuffer),
                                -1.f, 1.f); //动态规划缓冲能量
    capRealRatio =
            (powf(_capVoltage, 2.f) - powf(VCAP_MIN, 2.f)) / VOLTAGE_RANGE;

    float ratioErr = capRealRatio - capCmdRatio;
    float capExpRatio =
            std::clamp(capCmdRatio + (ratioErr * bufferDP), 0.f, 1.f);

    offsetPower = powerPid_->calc(capExpRatio, capRealRatio);

    maxPower = std::clamp(limitPower - offsetPower, limitPower,
                          (_capVoltage * CAPCurrMax) + limitPower);
}

void PowerController::refereeDataUpdate()
{
    // TODO: update from referee
}
