#include "SuperCap.hpp"
#include "Bsp_can.hpp"
#include <cstring>


CAP::CAP()
{
    memset(&rawCapData_, 0, sizeof(RawCapData_s));
    memset(&capData_, 0, sizeof(CapData_s));
    memset(&capCmd_, 0, sizeof(CapCmd_s));
    this->registerCapCallback();
    capTxFreq_ = 1000.f;
}

void CAP::registerCapCallback()
{
    extern canHandle HCAN1;
    const uint8_t *_rxbuf;
    Can::instance().registerCallback(&HCAN1, CAPMasterID,
                                     [this](const uint8_t *_rxBuf) {
                                         this->praseCapData(_rxBuf);
                                     });
}

void CAP::praseCapData(const uint8_t *_rxbuf)
{
    memcpy(&rawCapData_, _rxbuf, sizeof(RawCapData_s));

    //get_data_refresh_freq(capData_.inputVoltage, cap_data_rfreq);
    //TODO: 翻译各个数据
    capData_.inputVoltage =
            24.f + static_cast<float>(rawCapData_.busVoltage) / 100.0f;
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
                         bool _EnableFeedforward, bool _limitPower)
{
    extern canHandle HCAN1;
    uint8_t ret = 0;
    if (checkSend()) {
        capCmd_.chargeCmdPower = _capChargePower;
        capCmd_.EnableCAP = _capEnableFlag;
        capCmd_.EnableFeedforward = _EnableFeedforward;
        capCmd_.chassisCmdPower = _limitPower;

        uint8_t _txbuf[8] = { 0 };
        memcpy(_txbuf, &capCmd_, sizeof(CapCmd_s));

        ret = static_cast<uint8_t>(
                Can::instance().transmitData(&HCAN1, CAPCmdID, _txbuf, 8));

        lastSendTick_ = xTaskGetTickCount();
    }
    return ret;
}

bool CAP::checkSend()
{
    return (xTaskGetTickCount() - lastSendTick_) >=
           pdMS_TO_TICKS(1000.f / capTxFreq_);
}