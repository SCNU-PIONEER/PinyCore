#include "Buzzer.hpp"

#include "BuzzerNote.hpp"
#include <cstdint>
#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

namespace BUZZER {

void Buzzer::init(TIM_HandleTypeDef *_htim, uint32_t _channel)
{
    this->htim_ = _htim;
    this->channel_ = _channel;
    HAL_TIM_PWM_Start(htim_, channel_);
}

void Buzzer::deInit() { HAL_TIM_PWM_Stop(htim_, channel_); }

void Buzzer::set(uint32_t _freq, uint32_t _duration)
{
    freq_ = _freq;
    delay_ = _duration;
}

void Buzzer::play()
{
    __HAL_TIM_SetCompare(htim_, channel_, 0);
    for (uint8_t i = 0; i < sizeof(dji) / sizeof(Note); i++) {
        __HAL_TIM_SetCompare(htim_, channel_, dji[i].frequency);
        vTaskDelay(dji[i].duration / portTICK_PERIOD_MS);
    }
    __HAL_TIM_SetCompare(htim_, channel_, 0);
}

void Buzzer::callBackFromISR(TIM_HandleTypeDef *htim)
{
    Buzzer &_buzzer = Buzzer::getInstance();
    if (_buzzer.delay_)
        __HAL_TIM_SetCompare(_buzzer.htim_, _buzzer.channel_, _buzzer.freq_);
    _buzzer.delay_--;
}

}
