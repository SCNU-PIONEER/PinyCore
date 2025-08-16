#pragma once

#include <cstdio>
#include "Soc.hpp"
#include <cstdint>
#include <cstdarg>
#include HAL_INCLUDE

class Vofa {
public:
    /**
     * @brief vofa init
     */
    void init(UART_HandleTypeDef *_uartHandle, DMA_HandleTypeDef *_dmaTxHandle);

    /*
    * @brief FireWater protocol
    */
    void fireWater(const char *_format, ...);

    /**
     * @brief justfloat protocol
     */
    void justFloat(float *_data, uint8_t _num);

    /**
     * @brief get vofa instance
     */
    static Vofa &instance()
    {
        static Vofa instance;
        return instance;
    }

protected:
    Vofa(const Vofa &);
    Vofa &operator=(const Vofa &);
    Vofa() = default;

private:
    uint8_t *FWBuffer_;
    // uint8_t *JFBuffer_;
    UART_HandleTypeDef *uartHandle_;
    DMA_HandleTypeDef *dmaTxHandle_;
};
