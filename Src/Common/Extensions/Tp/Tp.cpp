/**
 * @file teach_pendant.cpp
 * @brief 示教器处理类实现
 *
 * @version Version 1.0.0
 * @author [可补充作者]
 * @date [可补充日期]
 *
 * @copyright [可补充版权信息]
 *
 */

#include "Common/Extensions/Tp/TpMsg.hpp"
#include "Tp.hpp"
#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cstring>
#include "Crc.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"

namespace TP {

TeachPendant::TeachPendant()
{
    memset(&teachJoint_, 0, sizeof(teachJoint_));
    //init parsed data
}

void TeachPendant::init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event)
{
    uart_ = _huart;
    event_ = _event;
    tpRxData = (VtTpFrame_t *)Dma::instance().ram_alloc(sizeof(VtTpFrame_t));
    Uart::instance().RecvDmaInit(_huart, (uint32_t *)tpRxData,
                                 sizeof(VtTpFrame_t));
    Uart::instance().registerCallback(_huart,
                                      &TeachPendant::rawCallBackFromISR);
}

void TeachPendant::rawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos)
{
    TeachPendant::instance().callBackFromISR(_huart, _pos);
}

void TeachPendant::callBackFromISR(UART_HandleTypeDef *_uart, uint16_t _pos)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(event_, TP_READY_EVENT, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
    HAL_UARTEx_ReceiveToIdle_DMA(_uart, (uint8_t *)&tpRxData,
                                 sizeof(VtTpFrame_t));
    __HAL_DMA_DISABLE_IT(_uart->hdmarx, DMA_IT_HT); // NOLINT
}

//handle
void TeachPendant::convert()
{
    //need referee
    // if (Verify_CRC8_Check_Sum((uint8_t *)tpRxData,
    //                           sizeof(frame_header_t)) &&
    //     Verify_CRC16_Check_Sum((uint8_t *)tpRxData, sizeof(VtTpFrame_t))) {
    //     for (uint8_t i = 0; i < 7; i++) {
    //         teachJoint_.joint[i] = tpRxData->tpData.joint[i];
    //     }
    // }
}

} // namespace TP