#pragma once

#define HAL_INCLUDE <stm32f1xx_hal.h>

/*can*/
#define SOC_CAN_NUM (1)
#define HCAN1       hcan
#define canHandle   CAN_HandleTypeDef
#define canHeader   CAN_RxHeaderTypeDef
#define RX_FIFO0    CAN_RX_FIFO0
#define RX_FIFO1    CAN_RX_FIFO1

/*usb*/
#define SOC_USB_FS

#define SOC_CUSTOM_INIT() do { } while (0)
