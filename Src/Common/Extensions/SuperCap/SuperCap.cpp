#include "SuperCap.hpp"
#include <cmath>

SuperCap::SuperCap(canHandle *_hcan, uint16_t _cmdId, uint16_t _dataId, float _txFreq)
{
    aux_.hcan = _hcan;
    aux_.cmdId = _cmdId;
    aux_.rxQueue = xQueueCreate(2, sizeof(aux_.rxBuf));
    Can::instance().registerCallback(_hcan, _dataId, [this](const uint8_t *_rxBuf) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(this->aux_.rxQueue, _rxBuf, &higherPriorityTaskWoken);
    });

    uint32_t periodMs = static_cast<uint32_t>(std::round(1000.0f / _txFreq));
    aux_.txPeriodTicks = pdMS_TO_TICKS(periodMs);

    capCmd_.capEnable = false;
    capCmd_.systemRestart = false;
    capCmd_.clearError = false;
    capCmd_.enChargeLimit = false;
}

void SuperCap::praseCapData(const uint8_t *_rxbuf)
{
    RawCapData_s rawCapData{ _rxbuf };

    memcpy(&capData_.capState, &rawCapData.statusCode, sizeof(CapState_s));
    capData_.outputPower = rawCapData.outputPower;
    capData_.capEnergyRatio = static_cast<float>(rawCapData.capEnergy) / CAP_ENERGY_MAX;
    capData_.vBat = static_cast<float>(rawCapData.vBat) / 65535.f * VOTAGE_MAX;
}

void SuperCap::computeRawCapCmd(RawCapCmd_s &_rawCmd) const
{
    _rawCmd.enableDCDC = capCmd_.capEnable;
    _rawCmd.systemRestart = capCmd_.systemRestart;
    _rawCmd.clearError = capCmd_.clearError;
    _rawCmd.enChargeLimit = capCmd_.enChargeLimit;
    _rawCmd.chargeRatioLimit = static_cast<uint8_t>(capCmd_.chargeRatioLimit * CAP_ENERGY_MAX);
    _rawCmd.useFeedback = 1;
    _rawCmd.chargePowerLimit = capCmd_.chargePowerLimit;
    _rawCmd.chargeEnergySlack = capCmd_.chargeEnergySlack;
    _rawCmd.reserved1 = 0;
    _rawCmd.reserved2 = 0;
}

uint8_t SuperCap::capDataSend()
{
    uint8_t ret = 0;
    if ((xTaskGetTickCount() - aux_.lastSendTick) >= aux_.txPeriodTicks) {
        RawCapCmd_s raw;
        computeRawCapCmd(raw);
        ret |= static_cast<uint8_t>(
                Can::instance().transmitData(aux_.hcan, aux_.cmdId, reinterpret_cast<uint8_t *>(&raw), sizeof(raw)));
        aux_.lastSendTick = xTaskGetTickCount();
    }
    return ret;
}

void SuperCap::enable() { capCmd_.capEnable = true; }
void SuperCap::disable() { capCmd_.capEnable = false; }
void SuperCap::limitCharge() { capCmd_.enChargeLimit = true; }
void SuperCap::unlimitCharge() { capCmd_.enChargeLimit = false; }
void SuperCap::setChargelimitRatio(float _ratio)
{
    _ratio = std::fmax(0.f, std::fmin(1.f, _ratio));
    capCmd_.chargeRatioLimit = _ratio;
}
void SuperCap::setClearErrorFlag(bool _flag) { capCmd_.clearError = _flag; }
void SuperCap::setSystemRestartFlag(bool _flag) { capCmd_.systemRestart = _flag; }
void SuperCap::setChargePowerLimit(uint16_t _chargePowerLimit) { capCmd_.chargePowerLimit = _chargePowerLimit; }
void SuperCap::setChargeEnergySlack(uint16_t _chargeEnergySlack) { capCmd_.chargeEnergySlack = _chargeEnergySlack; }

bool SuperCap::isOnline() const { return aux_.rxFreq > OFFLINE_FREQ_THRESHOLD; }

void SuperCap::task()
{
    if (xQueueReceive(aux_.rxQueue, aux_.rxBuf, 0) == pdTRUE) {
        aux_.rxCnt++;
        praseCapData(aux_.rxBuf);
    }

    capDataSend();
    rxFreqCalc();
}

void SuperCap::rxFreqCalc()
{
    uint32_t dt = (xTaskGetTickCount() - aux_.lastRecvTick);
    if (dt >= pdMS_TO_TICKS(1000)) {
        aux_.rxFreq = static_cast<float>(aux_.rxCnt) / (static_cast<float>(dt) / 1000.f);
        aux_.rxCnt = 0;
        aux_.lastRecvTick = xTaskGetTickCount();
    }
    state_ = isOnline() ? State_e::ONLINE : State_e::OFFLINE;
}
