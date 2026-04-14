#pragma once

#define HAL_INCLUDE <stm32h7xx_hal.h>

/*can*/
#define SOC_FDCAN
#define SOC_CAN_NUM (3)
#define HCAN1       hfdcan1
#define HCAN2       hfdcan2
#define HCAN3       hfdcan3
#define canHandle   FDCAN_HandleTypeDef
#define canHeader   FDCAN_RxHeaderTypeDef
#define RX_FIFO0    FDCAN_RX_FIFO0
#define RX_FIFO1    FDCAN_RX_FIFO1

/*uart*/
#define SOC_UART_DMA_DR      huart_->Instance->RDR

void SET_485_1_DE_UP();
void SET_485_1_DE_DOWN();

/*dma*/
#define SOC_DMA_SRAM        0x24000000
#define SOC_DMA_SRAM_END    0x24020000
#define SOC_DMA_SRAM_D2     0x30000000
#define SOC_DMA_SRAM_D2_END 0x30007FFF
enum DmaRam_e { SRAM, SRAM_D2 };

/*usb*/
#define SOC_USB_HS

/*gpio*/
void Power_OUT1_ON();
void Power_OUT1_OFF();

void Power_OUT2_ON();
void Power_OUT2_OFF();

void Power_5V_ON();
void Power_5V_OFF();
