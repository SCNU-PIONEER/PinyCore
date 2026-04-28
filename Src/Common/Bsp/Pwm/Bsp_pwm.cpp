#include "Bsp_pwm.hpp"

Pwm::Pwm(TIM_HandleTypeDef *_htim, uint32_t _channel) : htim_(_htim), channel_(_channel) {};

void Pwm::start() { HAL_TIM_PWM_Start(htim_, channel_); }

HAL_StatusTypeDef Pwm::startDMA(uint32_t *_txData, uint16_t _dataLength)
{
    return HAL_TIM_PWM_Start_DMA(htim_, channel_, _txData, _dataLength);
}

void Pwm::stop() { HAL_TIM_PWM_Stop(htim_, channel_); }

void Pwm::setDutyCycle(uint32_t _dutyCycle) { __HAL_TIM_SET_COMPARE(htim_, channel_, _dutyCycle); }

void Pwm::setAutoLoader(uint32_t _arr) { __HAL_TIM_SET_AUTORELOAD(htim_, _arr); }

void Pwm::setPrescaler(uint32_t _prescaler) { __HAL_TIM_SET_PRESCALER(htim_, _prescaler); }

void Pwm::setCounter(uint32_t _counter) { __HAL_TIM_SET_COUNTER(htim_, _counter); }

uint32_t Pwm::autoLoader() const { return __HAL_TIM_GET_AUTORELOAD(htim_); }

bool Pwm::isReady() const { return TIM_CHANNEL_STATE_GET(htim_, channel_) == HAL_TIM_CHANNEL_STATE_READY; }
