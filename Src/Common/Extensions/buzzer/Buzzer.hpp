#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE
#include "BuzzerNote.hpp"
#include "Task.hpp"
#include "Singleton.hpp"

namespace BUZZER {

class Buzzer : public Singleton<Buzzer>, public Task<Buzzer, 128> {
public:
    void init(TIM_HandleTypeDef *_htim, uint32_t _channel, uint32_t _timerFreq);

    void deInit();

    void set(uint32_t _freq, uint32_t _duration);

    void playAllNotes();

    void playPinyCore();

    void playDJI();

    void playNote(const Note &_note);

    static void callBackFromISR();

    void task();

private:
    Buzzer() : Task<Buzzer, 128>("BuzzerTask", TaskPriority_e::LOW1) {}
    friend class Singleton<Buzzer>;

    TIM_HandleTypeDef *htim_;
    uint32_t timerFreq_;
    uint32_t channel_;
    uint32_t delay_;
    uint32_t freq_;
};

}
