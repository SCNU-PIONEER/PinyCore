#include "PowerController.hpp"

PowerController::PowerController(ChassisType_e _chassisType, CAP *_cap) : cap_(_cap), chassisType_(_chassisType)
{
    motorNum_ = static_cast<uint8_t>(chassisType_);

    cmdPower_.resize(motorNum_, 0);
    relPower_.resize(motorNum_, 0);
    setTorq_.resize(motorNum_, 0);
    setPower_.resize(motorNum_, 0);
}

void PowerController::update(const RefereeMsg_s &_msg)
{
    float capFreq = 0.f;

    if (cap_ != nullptr) {
        CapData_s capData = cap_->getCapData();
        capFeedbackPower_ = capData.inputVoltage * capData.outputCurrent;
        capFreq = cap_->getRxFreq();
        dynamicPower(capData.capVoltage);
    }
    errorCheck(_msg.rxFreq, capFreq);
    updateReferee(_msg);
}

void PowerController::dynamicPower(float _capVoltage)
{
    float bufferDP = std::clamp(energyPid_.calc(expPowerBuffer_, powerBuffer_), -1.f, 1.f);
    capRealRatio_ = (powf(_capVoltage, 2.f) - powf(VCAP_MIN, 2.f)) / VOLTAGE_RANGE;

    float ratioErr = capRealRatio_ - capCmdRatio_;
    float capExpRatio = std::clamp(capCmdRatio_ + (ratioErr * bufferDP), 0.f, 1.f);

    offsetPower_ = powerPid_.calc(capExpRatio, capRealRatio_);

    maxPower_ = std::clamp(limitPower_ - offsetPower_, limitPower_, (_capVoltage * CAP_CURRENT_MAX) + limitPower_);
}

void PowerController::updateReferee(const RefereeMsg_s &_msg)
{
#if POWERCTRL_USE_REFEREE
    powerBuffer_ = _msg.chassisPowerBuffer;
    limitPower_ = _msg.chassisPowerLimit;
#else
    powerBuffer_ = 60.f;
    limitPower_ = 50.f;
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
