#include "PowerController.hpp"

PowerController::PowerController(ChassisType_e _chassisType, SuperCap *_cap) : cap_(_cap), chassisType_(_chassisType)
{
    motorNum_ = static_cast<uint8_t>(chassisType_);

    cmdPower_.resize(motorNum_, 0);
    fitPower_.resize(motorNum_, 0);
    setTorq_.resize(motorNum_, 0);
    setPower_.resize(motorNum_, 0);
}

void PowerController::update(const RefereeMsg_s &_msg)
{
    if (cap_ != nullptr) {
        CapData_s capData = cap_->getCapData();
        chassisRealPower_ = realPowerFilter.process(capData.outputPower);
        capRealRatio_ = capData.capEnergyRatio;

        float offsetPower = powerPid_.calc(capCmdRatio_, capRealRatio_);
        offsetPower = std::clamp(offsetPower, offsetPower, REMAIN_POWER);
        maxPower_ = static_cast<float>(_msg.chassisPowerLimit) - offsetPower;
    }
}
