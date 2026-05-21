#include "Soc.hpp"
#include "main.h"

#include "sdkconfig.h"

#if SOC_CUSTOM_USE_SPI3

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

namespace {

void errorHandlerIfNotOk(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) {
        Error_Handler();
    }
}

void spi3MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct {};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct {};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    errorHandlerIfNotOk(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct));

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hdma_spi3_tx.Instance = DMA2_Stream7; // Select DMA Channel
    hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    errorHandlerIfNotOk(HAL_DMA_Init(&hdma_spi3_tx));

    __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);

    HAL_NVIC_SetPriority(SPI3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0); // Select DMA Channel
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn); // Select DMA Channel
}

void spi3MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3) {
        return;
    }

    __HAL_RCC_SPI3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
    HAL_DMA_DeInit(hspi->hdmatx);
    HAL_NVIC_DisableIRQ(SPI3_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn); // Select DMA Channel
}

void initSpi3()
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 0x0;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    errorHandlerIfNotOk(HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPINIT_CB_ID, spi3MspInit));
    errorHandlerIfNotOk(HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPDEINIT_CB_ID, spi3MspDeInit));
    errorHandlerIfNotOk(HAL_SPI_Init(&hspi3));
}

} // namespace

extern "C" void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}

 // Select DMA Channel
extern "C" void DMA2_Stream7_IRQHandler(void) 
{
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

#endif

void socCustomInit()
{
#if SOC_CUSTOM_USE_SPI3
    initSpi3();
#endif
}
