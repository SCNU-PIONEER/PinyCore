#pragma once
#include "sdkconfig.h"
#include "main.h"

#if BSP_USE_CAN
#include "Bsp_can.hpp"
#endif

#if BSP_USE_DMA
#include "Bsp_dma.hpp"
#endif

#if BSP_USE_PWM
#include "Bsp_pwm.hpp"
#endif

#if BSP_USE_SPI
#include "Bsp_spi.hpp"
#endif

#if BSP_USE_TIM
#include "Bsp_tim.hpp"
#endif

#if BSP_USE_UART
#include "Bsp_uart.hpp"
#endif

#if BSP_USE_USB
#include "Bsp_usb.hpp"
#endif
