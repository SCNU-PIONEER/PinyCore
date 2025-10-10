#include "Nav.hpp"
#include "Bsp_usb.hpp"
#include "Crc.hpp"
#include "Referee.hpp"
#include <cmath>

using namespace NAV;

Nav::Nav()
        : navRxLostCnt_(NAV_RX_LOST_MAX)
#if NAV_COMM_USB == 0 //&& NAV_COMM_UART == 1
        , navUart_(&NAV_UART_HANDLE, &NAV_DMA_HANDLE)
#endif
{
    xTaskCreate(Nav::task, "nav_task", 384, this, 7, nullptr);
    LOG::info("Nav", "task init success");
    memset(&navToEcDatas_, 0, NAV_DATA_LEN);
    memset(&ecToNavDatas_, 0, sizeof(EcToNav_s));
};

void Nav::task(void *_param)
{
    auto instance = static_cast<Nav *>(_param);
    while (true) {
        instance->sendEcToNavData();
        vTaskDelay(1);
    }
}

bool Nav::isNavOffline() { return navRxLostCnt_ >= NAV_RX_LOST_MAX; };
void Nav::sendEcToNavData()
{
    ecToNavDatas_.header = NAV_UART_REQ_SOF;
    ecToNavDatas_.gameProgress =
            refereeRx->getRefereeData().gameStatus.gameProgress;
    ecToNavDatas_.gameTime =
            refereeRx->getRefereeData().gameStatus.stageRemainTime;
    ecToNavDatas_.navHPstate =
            refereeRx->getRefereeData().gameRobotHP.red4RobotHP;
    ecToNavDatas_.pitch--;
    ecToNavDatas_.yaw++;
    Append_CRC16_Check_Sum((uint8_t *)&ecToNavDatas_, sizeof(EcToNav_s));
#if NAV_COMM_USB != 1
    navUart_.transmitDma((uint8_t *)&ecToNavDatas_, sizeof(EcToNav_s));
#else
    navUsb_->send((uint8_t *)&ecToNavDatas_, sizeof(EcToNav_s));
#endif
};
void Nav::incNavRxLost()
{
    if (navRxLostCnt_ < NAV_RX_LOST_MAX) {
        navRxLostCnt_++;
    }
};


#if NAV_COMM_USB == 1 // && NAV_COMM_UART == 0
void Nav::navInit(EventGroupHandle_t _event)
{
    event_ = _event;
    navUsb_ = &Usb::instance();
    navRxBuffer_ = rxBuffer_;
    navUsb_->registerCallback(
            [this](uint8_t *_buf, uint16_t _len) { parseNavData(_buf, _len); });
};
uint8_t Nav::parseNavData(uint8_t *_buf, uint16_t _len)
{
    if (_buf[0] == NAV_UART_CTRL_SOF && _len == NAV_DATA_LEN &&
        Verify_CRC16_Check_Sum(&_buf[0], NAV_DATA_LEN)) {
        memcpy((uint8_t *)&navToEcDatas_, _buf, NAV_DATA_LEN);
        this->navRxLostCnt_ = 0;
        return NAV_DATA_NOERR;
    } else {
        return NAV_DATA_ERR;
    }
};
#else
void Nav::navInit(EventGroupHandle_t _event)
{
    event_ = _event;
    navRxBuffer_ = (uint8_t *)Dma::instance().ram_alloc(2 * NAV_DATA_LEN);
    navUart_.recvDmaMultiBufInit((uint32_t *)&navRxBuffer_[0],
                                 2 * NAV_DATA_LEN);
    navUart_.registerCallback(
            [this](UART_HandleTypeDef *_huart, uint16_t _size) {
                this->navUartIdleHandler(_huart);
            });
}

void Nav::navUartIdleHandler(UART_HandleTypeDef *_huart)
{
    if (_huart->Instance != navUart_.huart_->Instance) {
        return;
    }
    uint16_t dmaRxPos =
            NAV_RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(_huart->hdmarx);
    int32_t lenDif = dmaRxPos - lastPos;
    dataLen = (lenDif >= 0) ? lenDif : (NAV_RX_BUFFER_LEN + lenDif);
    //数据回绕
    if (lenDif < 0)
        memcpy(&navRxBuffer_[NAV_RX_BUFFER_LEN], &navRxBuffer_[0], dmaRxPos);
    if (dmaRxPos == NAV_DATA_LEN) {
        if (parseNavData(&navRxBuffer_[0], NAV_DATA_LEN) == NAV_DATA_NOERR) {
            navRxLostCnt_ = 0;
        }
    }
    xEventGroupSetBitsFromISR(event_, NAV_READY_EVENT, nullptr);
}

uint8_t Nav::parseNavData(uint8_t *_buf, uint16_t _len)
{
    if (_buf[0] == NAV_UART_CTRL_SOF && _len == NAV_DATA_LEN &&
        Verify_CRC16_Check_Sum(&_buf[0], NAV_DATA_LEN)) {
        memcpy((uint8_t *)&navToEcDatas_, _buf, NAV_DATA_LEN);
        this->navRxLostCnt_ = 0;
        return NAV_DATA_NOERR;
    } else {
        return NAV_DATA_ERR;
    }
};
#endif
