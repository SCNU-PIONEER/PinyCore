#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE
#include "BspBase.hpp"

class Pwm : public BspBase<Pwm> {
public:
    /**
     * @brief pwm start
     */
    void start(TIM_HandleTypeDef *_htim, uint32_t _channel);

    /**
     * @brief pwm stop
     */
    void stop(TIM_HandleTypeDef *_htim, uint32_t _channel);

    /**
     * @brief pwm set duty
     */
    void setDutyCycle(TIM_HandleTypeDef *_htim, uint32_t _channel,
                      uint32_t _dutyCycle);

    /**
     * @brief pwm set frequency
     */
    void setFrequency(TIM_HandleTypeDef *_htim, uint32_t _frequency);
};
