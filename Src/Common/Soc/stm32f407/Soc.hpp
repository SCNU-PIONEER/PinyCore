#pragma once

#define HAL_INCLUDE <stm32f4xx_hal.h>

// can
#define SOC_CAN
#define SOC_CAN_NUM (2)
#define HCAN1       hcan1
#define HCAN2       hcan2
#define canHandle   CAN_HandleTypeDef
#define canHeader   CAN_RxHeaderTypeDef
#define RX_FIFO0    CAN_RX_FIFO0
#define RX_FIFO1    CAN_RX_FIFO1

//uart
#define SOC_DMA_RAM
#define SOC_UART_DMA_DR _huart->Instance->DR
enum DmaRam_e { RAM };

#define SET_485_1_DE_UP()
#define SET_485_1_DE_DOWN()
