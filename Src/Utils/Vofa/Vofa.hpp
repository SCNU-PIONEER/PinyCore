#pragma once

#include <stdio.h>
#include "Soc.hpp"
#include <stdint.h>
#include <utility>
#include <stdarg.h>
#include HAL_INCLUDE
#include "Singleton.hpp"

class Vofa {
public:
    /**
     * @brief vofa init
     */
    void init(UART_HandleTypeDef *_uartHandle, DMA_HandleTypeDef *_dmaTxHandle);

    /*
    * @brief FireWater protocol
    */
    void FireWater(const char *format, ...);

    /**
     * @brief justfloat protocol
     */
    void JustFloat(float *_data, uint8_t _num);

private:
    Vofa() = default;
    friend class Singleton<Vofa>;

    uint8_t *FWBuffer_;
    // uint8_t *JFBuffer_;
    UART_HandleTypeDef *uartHandle_;
    DMA_HandleTypeDef *dmaTxHandle_;
};
