#include "SuperCap.hpp"
#include <cstring>
#include "Referee.hpp"

CAP::CAP(canHandle *_hcan) : hcan_(_hcan), rxQueue_(xQueueCreate(2, sizeof(PINYMOTOR::RxBus_s::CANRxBuf_s<8>)))
{
    Can::instance().registerCallback(hcan_, CAP_DATA_ID, [this](const uint8_t *_rxBuf) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(this->rxQueue_, _rxBuf, &higherPriorityTaskWoken);
    });
}

void CAP::praseCapData(const uint8_t *_rxbuf)
{
    memcpy(&rawCapData_, _rxbuf, sizeof(RawCapData_s));

    memcpy(&capData_.capState, &rawCapData_.statusCode, sizeof(CapState_s));
    capData_.chassisPower = (static_cast<float>(rawCapData_.chassisPower) - 16384.f) / 64.f;
    capData_.refereePower = (static_cast<float>(rawCapData_.refereePower) - 16384.f) / 64.f;
    capData_.chassisPowerLimit = static_cast<float>(rawCapData_.chassisPowerLimit);
    capData_.capEnergyRatio = static_cast<float>(rawCapData_.capEnergy) / CAP_ENERGY_MAX;
}

uint8_t CAP::capDataSend(bool _capEnable, bool _systemRestart, bool _clearError, bool _enChargeLimit,
                         uint8_t _chargeRatioLimit)
{
    uint8_t ret = 0;
    capCmd_.enableDCDC = _capEnable;
    capCmd_.systemRestart = _systemRestart;
    capCmd_.clearError = _clearError;
    capCmd_.enChargeLimit = _enChargeLimit;
    capCmd_.chargeRatioLimit = _chargeRatioLimit;
#if EXTENSION_REFEREE
    capCmd_.powerLimit = referee->receiver.getRefereeData().gameRobotStatus.chassisPowerLimit;
    capCmd_.energyBuffer = referee->receiver.getRefereeData().powerHeatData.chassisPowerBuffer;
#else
    capCmd_.powerLimit = 60;
    capCmd_.energyBuffer = 60;
#endif
    capCmd_.reserved1 = 0;
    capCmd_.reserved2 = 0;

    if (checkSend()) {
        uint8_t txbuf[8] = {};
        memcpy(txbuf, &capCmd_, 8);

        ret = static_cast<uint8_t>(Can::instance().transmitData(hcan_, CAP_CMD_ID, txbuf, 8));
        lastSendTick_ = xTaskGetTickCount();
    }
    return ret;
}

void CAP::capTask(bool _capEnable, bool _systemRestart, bool _clearError, bool _enChargeLimit,
                  uint8_t _chargeRatioLimit)
{
    if (xQueueReceive(rxQueue_, &rxBuf_.data, 0) == pdTRUE) {
        rxCnt_++;
        praseCapData(rxBuf_.data);
    }

    capDataSend(_capEnable, _systemRestart, _clearError, _enChargeLimit, _chargeRatioLimit);
    rxFreqCalc();
}

bool CAP::checkSend() { return (xTaskGetTickCount() - lastSendTick_) >= pdMS_TO_TICKS(1000.f / CAP_TX_FREQ); }

void CAP::rxFreqCalc()
{
    static uint32_t lastTick = 0;
    if ((xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(1000)) {
        rxFreq_ = static_cast<float>(rxCnt_) / (static_cast<float>(xTaskGetTickCount() - lastTick) / 1000.f);
        rxCnt_ = 0;
        lastTick = xTaskGetTickCount();
    }
}
