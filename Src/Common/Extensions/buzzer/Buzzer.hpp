#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE
#include "BuzzerNote.hpp"

namespace BUZZER {

class Buzzer {
public:
    void init(TIM_HandleTypeDef *_htim, uint32_t _channel, uint32_t _timerFreq);

    void deInit();

    void set(uint32_t _freq, uint32_t _duration);

    void playAllNotes();

    void playPinyCore();

    void playDJI();

    void playNote(const Note &_note);

    static void callBackFromISR(TIM_HandleTypeDef *_htim);

    /**
    * @brief fdcan get Instance
    */
    inline static Buzzer &getInstance()
    {
        static Buzzer instance;
        return instance;
    }

private:
    // static Buzzer *instance;
    TIM_HandleTypeDef *htim_;
    uint32_t timerFreq_;
    uint32_t channel_;
    uint32_t delay_;
    uint32_t freq_;
};

}
