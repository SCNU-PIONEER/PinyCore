#include "Bsp_spi.hpp"

void Spi::transmit(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint16_t _dataLength, uint16_t _timeout)
{
    HAL_SPI_Transmit(_spi, _txData, _dataLength, _timeout);
}

void Spi::transmitIT(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint16_t _dataLength)
{
    HAL_SPI_Transmit_IT(_spi, _txData, _dataLength);
}

void Spi::transmitDMA(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint16_t _dataLength)
{
    HAL_SPI_Transmit_DMA(_spi, _txData, _dataLength);
}

void Spi::receive(SPI_HandleTypeDef *_spi, uint8_t *_rxData, uint16_t _dataLength, uint16_t _timeout)
{
    HAL_SPI_Receive(_spi, _rxData, _dataLength, _timeout);
}

void Spi::receiveIT(SPI_HandleTypeDef *_spi, uint8_t *_rxData, uint16_t _dataLength)
{
    HAL_SPI_Receive_IT(_spi, _rxData, _dataLength);
}

void Spi::receiveDMA(SPI_HandleTypeDef *_spi, uint8_t *_rxData, uint16_t _dataLength)
{
    HAL_SPI_Receive_DMA(_spi, _rxData, _dataLength);
}

void Spi::transmitReceive(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint8_t *_rxData, uint16_t _dataLength,
                          uint16_t _timeout)
{
    HAL_SPI_TransmitReceive(_spi, _txData, _rxData, _dataLength, _timeout);
}

void Spi::transmitReceiveIT(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint8_t *_rxData, uint16_t _dataLength)
{
    HAL_SPI_TransmitReceive_IT(_spi, _txData, _rxData, _dataLength);
}

void Spi::transmitReceiveDMA(SPI_HandleTypeDef *_spi, uint8_t *_txData, uint8_t *_rxData, uint16_t _dataLength)
{
    HAL_SPI_TransmitReceive_DMA(_spi, _txData, _rxData, _dataLength);
}
