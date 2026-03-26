#include "Buzzer.hpp"
#include <memory>

namespace BUZZER {

Buzzer::Buzzer(TIM_HandleTypeDef *_htim, uint32_t _channel, uint32_t _timerFreq)
        : Task("BuzzerTask", TaskPriority_e::MID5)
        , pwm_(_htim, _channel)
        , timerFreq_(_timerFreq)
        , queue_(xQueueCreate(10, sizeof(Note_s)))
{
    setPSC(PSC);
    disable();
};

Buzzer ::~Buzzer()
{
    disable();
    vQueueDelete(queue_);
}

void Buzzer::enable() { pwm_.start(); }
void Buzzer::disable() { pwm_.stop(); }

void Buzzer::setPSC(uint32_t _psc)
{
    prescaler_ = _psc;
    pwm_.setPrescaler(prescaler_ - 1);
}

void Buzzer::play()
{
    if (note_.tone == Tone_e::REST) {
        pwm_.setDutyCycle(0);
        vTaskDelay(note_.duration / portTICK_PERIOD_MS);
        return;
    }

    disable();

    uint16_t freq = static_cast<uint16_t>(note_.tone);
    uint32_t delay = note_.duration / portTICK_PERIOD_MS;
    uint32_t period = (timerFreq_ / (freq * prescaler_)) - 1;
    pwm_.setAutoLoader(period);
    pwm_.setDutyCycle(period / 2);

    enable();

    vTaskDelay(delay);
    pwm_.setDutyCycle(0);
}

void Buzzer::task()
{
    while (true) {
        if (xQueueReceive(queue_, &note_, portMAX_DELAY) == pdPASS) {
            play();
        }
    }
}

void Buzzer::playNote(const Note_s &_note)
{
    // no delay
    xQueueSend(queue_, &_note, 0);
}

void Buzzer::playNote(Tone_e _tone, uint16_t _dura)
{
    Note_s note{ _tone, _dura };
    playNote(note);
}

void Buzzer::playSequenceTask(void *_params)
{
    std::unique_ptr<PlaySequenceParams_s> p(static_cast<PlaySequenceParams_s *>(_params));
    for (size_t i = 0; i < p->noteNum; ++i) {
        xQueueSend(p->buzzer->getQueue(), &(p->sequence[i]), portMAX_DELAY);
        vTaskDelay(1);
    }
    vTaskDelete(nullptr);
}

void Buzzer::playSequence(const Note_s *_sequence, size_t _noteNum)
{
    auto p = std::make_unique<PlaySequenceParams_s>();
    p->buzzer = this;
    p->sequence = _sequence;
    p->noteNum = _noteNum;
    xTaskCreate(Buzzer::playSequenceTask, "PlaySequenceTask", 256, p.release(),
                static_cast<uint8_t>(TaskPriority_e::MID5), nullptr);
}

} // namespace BUZZER
