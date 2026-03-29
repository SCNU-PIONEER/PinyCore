#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE

class Pwm {
public:
    /**
     * @brief pwm init
     */
    Pwm(TIM_HandleTypeDef *_htim, uint32_t _channel);

    /**
     * @brief pwm start
     */
    void start();

    /**
     * @brief pwm start with DMA
     */
    void startDMA(uint32_t *_txData, uint16_t _dataLength);

    /**
     * @brief pwm stop
     */
    void stop();

    /**
     * @brief pwm set duty
     */
    void setDutyCycle(uint32_t _dutyCycle);

    /**
     * @brief pwm set auto loader
     */
    void setAutoLoader(uint32_t _arr);

    /**
     * @brief pwm set prescaler
     */
    void setPrescaler(uint32_t _prescaler);

    /**
     * @brief pwm set counter
     */
    void setCounter(uint32_t _counter);

private:
    TIM_HandleTypeDef *htim_;
    uint32_t channel_;
};