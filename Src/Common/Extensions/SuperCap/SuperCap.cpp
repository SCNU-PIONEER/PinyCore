#include "SuperCap.hpp"
#include <cstring>

CAP::CAP(canHandle *_hcan)
        : hcan_(_hcan)
        , rxQueue_(xQueueCreate(2, sizeof(PINYMOTOR::RxBus_s::CANRxBuf_s<8>)))
{
    Can::instance().registerCallback(
            hcan_, CAP_DATA_ID, [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
}

void CAP::praseCapData(const uint8_t *_rxbuf)
{
    //TODO: 昭庆蜜汁换算magic number ,后续跟琪宝交流
    memcpy(&rawCapData_, _rxbuf, sizeof(RawCapData_s));

    capData_.inputVoltage = BATTERY_VOLTAGE +
                            static_cast<float>(rawCapData_.busVoltage) / 100.0f;
    capData_.capVoltage = static_cast<float>(rawCapData_.capVoltage) / 70.0f;
    capData_.inputCurrent =
            static_cast<float>(rawCapData_.inputCurrent) / 1000.0f;
    capData_.outputCurrent =
            capData_.inputCurrent -
            static_cast<float>(rawCapData_.chargeCurrent) / 1000.0f;
    capData_.powerSet = static_cast<float>(rawCapData_.setPower);

    capData_.CapEnableFlag = rawCapData_.CapEnableFlag;
    capData_.LowVoltageFlag = rawCapData_.LowVoltageFlag;
}

uint8_t CAP::capDataSend(float _capChargePower, bool _capEnableFlag,
                         bool _enableCharge, uint16_t _chassisPower)
{
    uint8_t ret = 0;

    capCmd_.chargePower = _capChargePower;
    capCmd_.EnableCAP = _capEnableFlag;
    capCmd_.EnableCharge = _enableCharge;
    capCmd_.chassisCmdPower = _chassisPower;

    if (checkSend()) {
        uint8_t txbuf[8] = {};
        memcpy(txbuf, &capCmd_, 8);

        ret = static_cast<uint8_t>(
                Can::instance().transmitData(hcan_, CAP_CMD_ID, txbuf, 8));
        lastSendTick_ = xTaskGetTickCount();
    }
    return ret;
}

void CAP::capTask(float _capChargePower, bool _capEnableFlag,
                  bool _enableCharge, uint16_t _chassisPower)
{
    if (xQueueReceive(rxQueue_, &rxBuf_.data, 0) == pdTRUE) {
        rxCnt_++;
        praseCapData(rxBuf_.data);
    }

    capDataSend(_capChargePower, _capEnableFlag, _enableCharge, _chassisPower);
    rxFreqCalc();
}

bool CAP::checkSend()
{
    return (xTaskGetTickCount() - lastSendTick_) >=
           pdMS_TO_TICKS(1000.f / CAP_TX_FREQ);
}

void CAP::rxFreqCalc()
{
    static uint32_t lastTick = 0;
    if ((xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(1000)) {
        rxFreq_ = static_cast<float>(rxCnt_) /
                  (static_cast<float>(xTaskGetTickCount() - lastTick) / 1000.f);
        rxCnt_ = 0;
        lastTick = xTaskGetTickCount();
    }
}
