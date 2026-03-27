#pragma once

#include "BuzzerNote.hpp"
#include "Bsp_pwm.hpp"
#include "Task.hpp"
#include "queue.h"
#include "Lazy.hpp"

namespace BUZZER {

class Buzzer : public Task<Buzzer, 128> {
    static constexpr uint32_t PSC = 100;
    struct PlaySequenceParams_s {
        Buzzer *buzzer;
        const Note_s *sequence;
        size_t noteNum;
    };

public:
    Buzzer(TIM_HandleTypeDef *_htim, uint32_t _channel, uint32_t _timerFreq);

    ~Buzzer();

    QueueHandle_t getQueue() const { return queue_; }

    void setPSC(uint32_t _psc);

    void playNote(const Note_s &_note);
    void playNote(Tone_e _tone = Tone_e::REST, uint16_t _dura = 0);
    // predefined sound
    template <size_t N> void playNote(const Note_s (&_sequence)[N]) { playSequence(_sequence, N); }

    void task();

private:
    Pwm pwm_;
    uint32_t timerFreq_;
    uint32_t prescaler_;

    QueueHandle_t queue_ = nullptr;
    Note_s note_;

    void enable();
    void disable();
    void play();

    static void playSequenceTask(void *_params);
    void playSequence(const Note_s *_sequence, size_t _noteNum);
};

#if EXTENSION_BUZZER
inline Lazy<BUZZER::Buzzer> buzz;
#endif

} // namespace BUZZER