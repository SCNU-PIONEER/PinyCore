#include "Soc.hpp"
#include <string.h>
#include "Rc.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"
namespace RC {


Rc::Rc() { memset(&data, 0, sizeof(data)); }


void Rc::init(UART_HandleTypeDef *huart, EventGroupHandle_t _event)
{
    uart_ = huart;
    event = _event;
    rc_buffer = (uint8_t *)Dma::instance().ram_alloc(2 * RC_FRAME_LENGTH);
    Uart::instance().RecvDmaMultiBufInit(huart, (uint32_t *)&rc_buffer[0],
                                         2 * RC_FRAME_LENGTH);
    Uart::instance().registerCallback(huart, &Rc::RawCallBackFromISR);
}

void Rc::RawCallBackFromISR(UART_HandleTypeDef *huart, uint16_t Pos)
{
    Rc::instance().callBackFromISR(huart, Pos);
}

void Rc::callBackFromISR(UART_HandleTypeDef *huart, uint16_t Pos)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // if (huart == uart_) {
    uint16_t size = huart->RxXferCount;
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) ==
        RESET) {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 2 * RC_FRAME_LENGTH);
        if (size == RC_FRAME_LENGTH) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            xEventGroupSetBitsFromISR(event, RC_READY_EVENT,
                                      &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
            // dt7_rc_rx_lost = 0; BUG: wait for fix
        } else
            dt7_rc_rxlost = RC_RX_LOST_MAX;

    } else {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 2 * RC_FRAME_LENGTH);
        if (size == RC_FRAME_LENGTH) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            xEventGroupSetBitsFromISR(event, RC_READY_EVENT,
                                      &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}

uint8_t Rc::parseData()
{
    data.rc.ch0 =
            (int16_t)((int16_t)rc_buffer[0] | ((int16_t)rc_buffer[1] << 8)) &
            0x07FF;
    if (data.rc.ch0 > RC_CH_VALUE_MAX || data.rc.ch0 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data.rc.ch0 -= RC_CH_VALUE_OFFSET;

    data.rc.ch1 = (int16_t)(((int16_t)rc_buffer[1] >> 3) |
                            ((int16_t)rc_buffer[2] << 5)) &
                  0x07FF;
    if (data.rc.ch1 > RC_CH_VALUE_MAX || data.rc.ch1 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data.rc.ch1 -= RC_CH_VALUE_OFFSET;

    data.rc.ch2 = (int16_t)(((int16_t)rc_buffer[2] >> 6) |
                            ((int16_t)rc_buffer[3] << 2) |
                            ((int16_t)rc_buffer[4] << 10)) &
                  0x07FF;
    if (data.rc.ch2 > RC_CH_VALUE_MAX || data.rc.ch2 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data.rc.ch2 -= RC_CH_VALUE_OFFSET;

    data.rc.ch3 = (int16_t)(((int16_t)rc_buffer[4] >> 1) |
                            ((int16_t)rc_buffer[5] << 7)) &
                  0x07FF;
    if (data.rc.ch3 > RC_CH_VALUE_MAX || data.rc.ch3 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data.rc.ch3 -= RC_CH_VALUE_OFFSET;

    data.rc.switch_left = (uint8_t)((uint8_t)(rc_buffer[5] >> 4) & 0x0C) >> 2;
    data.rc.switch_right = (uint8_t)((uint8_t)(rc_buffer[5] >> 4) & 0x03);

    data.mouse.x = (int16_t)((int16_t)rc_buffer[6]) |
                   ((int16_t)rc_buffer[7] << 8);
    data.mouse.y = (int16_t)((int16_t)rc_buffer[8]) |
                   ((int16_t)rc_buffer[9] << 8);
    data.mouse.z = (int16_t)((int16_t)rc_buffer[10]) |
                   ((int16_t)rc_buffer[11] << 8);

    data.mouse.press_left = (uint8_t)rc_buffer[12];
    data.mouse.press_right = (uint8_t)rc_buffer[13];

    data.keyboard.keycode =
            (((uint16_t)rc_buffer[14]) | ((uint16_t)rc_buffer[15] << 8));

    data.wheel = (int16_t)((int16_t)rc_buffer[16]) |
                 ((int16_t)rc_buffer[17] << 8);
    return RC_NO_ERROR;
}
}
