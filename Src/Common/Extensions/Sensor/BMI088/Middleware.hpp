#pragma once

#include "stdint.h"
#include "main.h"

#define BMI088_USE_SPI

class Middleware {
public:
    void init(SPI_HandleTypeDef *_spi) { BMI088_USING_SPI_UNIT = _spi; };

    void delay_ms(uint16_t ms);
    void delay_us(uint16_t us);

    void ACCEL_NS_L(void);
    void ACCEL_NS_H(void);

    void GYRO_NS_L(void);
    void GYRO_NS_H(void);

    uint8_t read_write_byte(uint8_t reg);

protected:
    SPI_HandleTypeDef *BMI088_USING_SPI_UNIT;
};
