#include "Bsp_pwm.hpp"

Pwm::Pwm(TIM_HandleTypeDef *_htim, uint32_t _channel)
        : htim_(_htim), channel_(_channel) {};

void Pwm::start() { HAL_TIM_PWM_Start(htim_, channel_); }

void Pwm::stop() { HAL_TIM_PWM_Stop(htim_, channel_); }

void Pwm::setDutyCycle(uint32_t _dutyCycle)
{
    __HAL_TIM_SET_COMPARE(htim_, channel_, _dutyCycle);
}

void Pwm::setFrequency(uint32_t _frequency)
{
    __HAL_TIM_SET_AUTORELOAD(htim_, _frequency);
}
