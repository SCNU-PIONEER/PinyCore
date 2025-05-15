#pragma once

#include "stdint.h"

#define BMI088_USE_SPI
// #define BMI088_USE_IIC

class Middleware {
  public:
    void GPIO_init(void);
    void com_init(void);
    void delay_ms(uint16_t ms);
    void delay_us(uint16_t us);

    void ACCEL_NS_L(void);
    void ACCEL_NS_H(void);

    void GYRO_NS_L(void);
    void GYRO_NS_H(void);

    uint8_t read_write_byte(uint8_t reg);
};
