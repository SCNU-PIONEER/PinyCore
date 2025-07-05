#include "Buzzer.hpp"

#include "BuzzerNote.hpp"
#include <cstdint>
#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

namespace BUZZER {

void Buzzer::init(TIM_HandleTypeDef *_htim, uint32_t _channel,
                  uint32_t _timerFreq)
{
    this->htim_ = _htim;
    this->channel_ = _channel;
    this->timerFreq_ = _timerFreq;
    HAL_TIM_PWM_Start(htim_, channel_);
}

void Buzzer::deInit() { HAL_TIM_PWM_Stop(htim_, channel_); }

void Buzzer::set(uint32_t _freq, uint32_t _duration)
{
    freq_ = _freq;
    delay_ = _duration;
}

void Buzzer::playNote(const Note &_note)
{
    if (_note.frequency == NOTE_REST) {
        __HAL_TIM_SetCompare(htim_, channel_, 0);
        vTaskDelay(_note.duration / portTICK_PERIOD_MS);
        return;
    }
    HAL_TIM_PWM_Stop(htim_, channel_);
    freq_ = _note.frequency;
    uint32_t prescaler = 100;
    uint32_t period = (timerFreq_ / (freq_ * prescaler)) - 1;
    __HAL_TIM_SET_PRESCALER(htim_, prescaler - 1);
    __HAL_TIM_SetAutoreload(htim_, period);
    delay_ = _note.duration / portTICK_PERIOD_MS;
    __HAL_TIM_SetCompare(htim_, channel_, period / 2);
    HAL_TIM_PWM_Start(htim_, channel_);
    vTaskDelay(delay_);
    __HAL_TIM_SetCompare(htim_, channel_, 0);
}

void Buzzer::playAllNotes()
{
    __HAL_TIM_SetCompare(htim_, channel_, 0);
    for (uint8_t i = 0; i < sizeof(allNote) / sizeof(Note); i++) {
        playNote(allNote[i]);
        playNote({ NOTE_REST, 500 }); // Rest for 500ms between notes
    }
    __HAL_TIM_SetCompare(htim_, channel_, 0);
}

void Buzzer::playPinyCore()
{
    __HAL_TIM_SetCompare(htim_, channel_, 0);
    for (uint8_t i = 0; i < sizeof(PinyCore) / sizeof(Note); i++) {
        playNote(PinyCore[i]);
    }
    __HAL_TIM_SetCompare(htim_, channel_, 0);
}

void Buzzer::playDJI()
{
    __HAL_TIM_SetCompare(htim_, channel_, 0);
    for (uint8_t i = 0; i < sizeof(dji) / sizeof(Note); i++) {
        playNote(dji[i]);
    }
    __HAL_TIM_SetCompare(htim_, channel_, 0);
}

void Buzzer::callBackFromISR(TIM_HandleTypeDef *_htim)
{
    Buzzer &_buzzer = Buzzer::getInstance();
    if (_buzzer.delay_)
        __HAL_TIM_SetCompare(_buzzer.htim_, _buzzer.channel_, _buzzer.freq_);
    _buzzer.delay_--;
}

}
