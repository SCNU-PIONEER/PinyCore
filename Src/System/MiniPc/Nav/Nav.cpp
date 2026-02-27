#include "Nav.hpp"
#include "Crc.hpp"
#include "MiniPcManager.hpp"
#include "Referee.hpp"
#include <cmath>
#include <cstdint>

using namespace MINIPC;

Nav::Nav(const EventGroupHandle_t &_event, uint32_t _eventBit)
{
    PcManager::instance().registerTransmitter([this]() { sendData(); });
    Usb::instance().registerCallback([this, _event, _eventBit](uint8_t *_buf, uint16_t _len) {
        {
            usbIdleCallback(_buf, _len, _event, _eventBit);
        };
    });
    memset(&navToEcDatas_, 0, NAV_DATA_LEN);
    memset(&ecToNavDatas_, 0, sizeof(EcToNav_s));
};

void Nav::usbIdleCallback(uint8_t *_buf, uint16_t _len, const EventGroupHandle_t &_event, uint32_t _eventBit)
{
    if (_buf[0] == NAV_UART_CTRL_SOF && _len == NAV_DATA_LEN && Verify_CRC16_Check_Sum(&_buf[0], NAV_DATA_LEN)) {
        memcpy(&rxBuffer_, _buf, NAV_DATA_LEN);
        rxCnt_++;
        xEventGroupSetBitsFromISR(_event, _eventBit, nullptr);
    } else {
        LOG::warn("Nav", "Invalid Nav data received via USB");
    }
};

void Nav::parseData() { memcpy(&navToEcDatas_, &rxBuffer_, NAV_DATA_LEN); }

void Nav::sendData()
{
    ecToNavDatas_.navHeader = NAV_UART_REQ_SOF;
    // ecToNavDatas_.gameProgress = referee->receiver.getRefereeData().gameStatus.gameProgress;
    // ecToNavDatas_.gameTime = referee->receiver.getRefereeData().gameStatus.stageRemainTime;
    // ecToNavDatas_.navHPstate = referee->receiver.getRefereeData().gameRobotHP.red4RobotHP;
    Append_CRC16_Check_Sum((uint8_t *)&ecToNavDatas_, sizeof(EcToNav_s));
    Usb::instance().send((uint8_t *)&ecToNavDatas_, sizeof(EcToNav_s));
};

void Nav::rxFreqCalc()
{
    static uint32_t lastTick = 0;
    if ((xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(1000)) {
        rxFreq_ = static_cast<float>(rxCnt_) / (static_cast<float>(xTaskGetTickCount() - lastTick) / 1000.f);
        rxCnt_ = 0;
        lastTick = xTaskGetTickCount();
    }
}
