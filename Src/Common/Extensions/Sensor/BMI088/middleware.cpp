#include "./middleware.hpp"
#include "dwt.h"
#include "Bsp_spi.hpp"


extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;


void Middleware::delay_ms(uint16_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

void Middleware::delay_us(uint16_t us) { dwt_delay_us(us); }

void Middleware::ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void Middleware::ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void Middleware::GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void Middleware::GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t Middleware::read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    Spi::instance().transmitReceive(*BMI088_USING_SPI_UNIT, &txdata, &rx_data,
                                    1, 100);
    return rx_data;
}
