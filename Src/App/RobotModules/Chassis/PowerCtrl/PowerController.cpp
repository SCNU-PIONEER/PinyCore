#include "PowerController.hpp"

PowerController::PowerController(ChassisType_e _chassisType, CAP *_cap) : cap_(_cap), chassisType_(_chassisType)
{
    motorNum_ = static_cast<uint8_t>(chassisType_);

    cmdPower_.resize(motorNum_, 0);
    fitPower_.resize(motorNum_, 0);
    setTorq_.resize(motorNum_, 0);
    setPower_.resize(motorNum_, 0);
}

void PowerController::update(const RefereeMsg_s &_msg)
{
    float capFreq = 0.f;
    if (cap_ != nullptr) {
        CapData_s capData = cap_->getCapData();
        chassisRealPower_ = realPowerFilter.process(capData.chassisPower);
        capRealRatio_ = capData.capEnergyRatio;

        float offsetPower = powerPid_.calc(capCmdRatio_, capRealRatio_);
        offsetPower = std::clamp(offsetPower, offsetPower, REMAIN_POWER);
        maxPower_ = static_cast<float>(_msg.chassisPowerLimit) - offsetPower;
        capFreq = cap_->getRxFreq();
    }
    errorCheck(_msg.rxFreq, capFreq);
}

void PowerController::errorCheck(float _refereeRxFreq, float _capFreq)
{
    (void)_refereeRxFreq;
    bool capError = _capFreq < 0.5f * CAP::DATA_RX_FREQ;
#if EXTENSION_REFEREE
    bool refereeError = _refereeRxFreq < 10.f;
#else
    bool refereeError = false;
#endif
    errorState_ = (capError && refereeError) ?
                          ErrorCode_e::ALL_DISCONNECT :
                          (capError ? ErrorCode_e::CAP_DISCONNECT :
                                      (refereeError ? ErrorCode_e::REFREEE_DISCONNECT : ErrorCode_e::NO_ERROR));
}
