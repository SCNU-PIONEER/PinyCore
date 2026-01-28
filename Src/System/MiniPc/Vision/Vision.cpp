#include "Vision.hpp"
#include "Bsp_usb.hpp"
#include "Crc.hpp"
#include "Referee.hpp"
#include <cmath>
#include "MiniPcManager.hpp"

using namespace MINIPC;

Vision::Vision()
{
    PcManager::instance().registerTransmitter([this]() { sendData(); });
    memset(&visionToEcDatas_, 0, VISION_DATA_LEN);
    memset(&ecToVisionDatas_, 0, sizeof(EcToVision_s));
};

void Vision::init(const EventGroupHandle_t &_event, uint32_t _eventBit)
{
    Usb::instance().registerCallback([this, _event, _eventBit](uint8_t *_buf, uint16_t _len) {
        {
            usbIdleCallback(_buf, _len, _event, _eventBit);
        };
    });
};

void Vision::usbIdleCallback(uint8_t *_buf, uint16_t _len, const EventGroupHandle_t &_event, uint32_t _eventBit)
{
    if (_buf[0] == VISION_UART_CTRL_SOF && _len == VISION_DATA_LEN &&
        Verify_CRC16_Check_Sum(&_buf[0], VISION_DATA_LEN)) {
        memcpy(&rxBuffer_, _buf, VISION_DATA_LEN);
        rxCnt_++;
        xEventGroupSetBitsFromISR(_event, _eventBit, nullptr);
    } else {
        LOG::warn("Vision", "Invalid Vision data received via USB");
    }
};

void Vision::parseData() { memcpy(&visionToEcDatas_, &rxBuffer_, VISION_DATA_LEN); }

void Vision::sendData()
{
    ecToVisionDatas_.header = VISION_UART_REQ_SOF;
    //TODO: send data
    // ecToVisionDatas_.pitch++;
    Append_CRC16_Check_Sum((uint8_t *)&ecToVisionDatas_, sizeof(EcToVision_s));
    Usb::instance().send((uint8_t *)&ecToVisionDatas_, sizeof(EcToVision_s));
};

void Vision::rxFreqCalc()
{
    static uint32_t lastTick = 0;
    if ((xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(1000)) {
        rxFreq_ = static_cast<float>(rxCnt_) / (static_cast<float>(xTaskGetTickCount() - lastTick) / 1000.f);
        rxCnt_ = 0;
        lastTick = xTaskGetTickCount();
    }
}
