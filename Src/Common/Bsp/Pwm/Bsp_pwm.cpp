#include "Bsp_pwm.hpp"

void Pwm::start(TIM_HandleTypeDef *_htim, uint32_t _channel)
{
    HAL_TIM_PWM_Start(_htim, _channel);
}

void Pwm::stop(TIM_HandleTypeDef *_htim, uint32_t _channel)
{
    HAL_TIM_PWM_Stop(_htim, _channel);
}

void Pwm::setDutyCycle(TIM_HandleTypeDef *_htim, uint32_t _channel,
                       uint32_t _dutyCycle)
{
    __HAL_TIM_SET_COMPARE(_htim, _channel, _dutyCycle);
}

void Pwm::setFrequency(TIM_HandleTypeDef *_htim, uint32_t _frequency)
{
    __HAL_TIM_SET_AUTORELOAD(_htim, _frequency);
}
