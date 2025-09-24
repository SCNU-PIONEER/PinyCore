/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_uart9_tx;
extern DMA_HandleTypeDef hdma_uart9_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_spi6_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi6;
extern SPI_HandleTypeDef hspi2;
extern FDCAN_HandleTypeDef hfdcan3;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_BDMA_Init(void);
void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);
void MX_SPI2_Init(void);
void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_TIM2_Init(void);
void MX_TIM12_Init(void);
void MX_TIM6_Init(void);
void MX_USART10_UART_Init(void);
void MX_TIM13_Init(void);
void MX_TIM7_Init(void);
void MX_TIM3_Init(void);
void MX_ADC1_Init(void);
void MX_SPI6_Init(void);
void MX_UART9_Init(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define power_1_Pin GPIO_PIN_13
#define power_1_GPIO_Port GPIOC
#define power_2_Pin GPIO_PIN_14
#define power_2_GPIO_Port GPIOC
#define power_3_Pin GPIO_PIN_15
#define power_3_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
