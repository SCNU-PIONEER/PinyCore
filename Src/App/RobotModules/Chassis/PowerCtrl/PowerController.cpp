#include "PowerController.hpp"

PowerController::PowerController(ChassisType_e _chassisType, CAP *_cap) : cap_(_cap), chassisType_(_chassisType)
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

void PowerController::update(RefereeMsg_s _msg)
{
    float capFreq = 0.f;

    if (cap_ != nullptr) {
        CapData_s capData = cap_->getCapData();
        capFeedbackPower = capData.inputVoltage * capData.outputCurrent;
        capFreq = cap_->getRxFreq();
        dynamicPower(capData.capVoltage);
    }
    errorCheck(_msg.rxFreq, capFreq);
    updateReferee(_msg);
}

void PowerController::dynamicPower(float _capVoltage)
{
    float bufferDP = std::clamp(energyPid.calc(expPowerBuffer, powerBuffer), -1.f, 1.f);
    capRealRatio = (powf(_capVoltage, 2.f) - powf(VCAP_MIN, 2.f)) / VOLTAGE_RANGE;

    float ratioErr = capRealRatio - capCmdRatio;
    float capExpRatio = std::clamp(capCmdRatio + (ratioErr * bufferDP), 0.f, 1.f);

    offsetPower = powerPid.calc(capExpRatio, capRealRatio);

    maxPower = std::clamp(limitPower - offsetPower, limitPower, (_capVoltage * CAP_CURRENT_MAX) + limitPower);
}

void PowerController::updateReferee(RefereeMsg_s _msg)
{
#if POWERCTRL_USE_REFEREE
    powerBuffer = _msg.chassisPowerBuffer;
    limitPower = _msg.chassisPowerLimit;
#else
    powerBuffer = 60;
    limitPower = 60;
#endif
}

void PowerController::errorCheck(float _refereeFreq, float _capFreq)
{
    bool capError = _capFreq < 0.5f * CAP::DATA_RX_FREQ;
#if POWERCTRL_USE_REFEREE
    bool refereeError = _refereeFreq < 10.f;
#else
    bool refereeError = false;
#endif
    errorState_ = (capError && refereeError) ?
                          ErrorCode_e::ALL_DISCONNECT :
                          (capError ? ErrorCode_e::CAP_DISCONNECT :
                                      (refereeError ? ErrorCode_e::REFREEE_DISCONNECT : ErrorCode_e::NO_ERROR));
}
