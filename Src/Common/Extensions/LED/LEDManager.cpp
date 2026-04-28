#include "LEDManager.hpp"

using namespace LED;

LEDDriver *LEDDriver::headDriver_ = nullptr;
LEDDriver *LEDDriver::tailDriver_ = nullptr;

void LEDs::task()
{
    constexpr TickType_t FRAME_PERIOD = pdMS_TO_TICKS(20);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        Cmd_s cmd;
        if (!effectEngine_.needsRefresh() && xQueueReceive(queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            const TickType_t now = xTaskGetTickCount();
            lastWake = now;
            applyCommand(cmd, static_cast<uint32_t>(now) * portTICK_PERIOD_MS);
        }

        const TickType_t now = xTaskGetTickCount();
        const uint32_t nowMs = static_cast<uint32_t>(now) * portTICK_PERIOD_MS;
        drainCommands(nowMs);

        if (effectEngine_.render(nowMs, ledColors_.data(), ledColors_.size())) {
            this->show();
        }

        if (effectEngine_.needsRefresh()) {
            vTaskDelayUntil(&lastWake, FRAME_PERIOD);
        }
    }
}

void LEDs::addLEDs(LEDDriver *_driver)
{
    if (_driver != nullptr) {
        _driver->setIndex(ledColors_.size());
        ledColors_.resize(ledColors_.size() + _driver->num());
        totalLEDs_ += _driver->num();
    }
}

void LEDs::ctrl(CmdType_e _type, uint8_t _index, uint8_t _ctrlNum)
{
    Cmd_s cmd;
    cmd.type = _type;
    cmd.index = _index;
    cmd.ctrlNum = _ctrlNum;
    QueueHandle_t queue = instance().queue_;
    if (xQueueSend(queue, &cmd, 0) != pdTRUE) {
        Cmd_s dropped;
        static_cast<void>(xQueueReceive(queue, &dropped, 0));
        static_cast<void>(xQueueSend(queue, &cmd, 0));
    }
}

void LEDs::off() { ctrl(CmdType_e::OFF, 0, instance().totalLEDs_); }

void LEDs::show()
{
    for (LEDDriver *drv = LEDDriver::head(); drv != nullptr; drv = drv->next()) {
        drv->show(ledColors_);
    }
}

void LEDs::applyCommand(const Cmd_s &_cmd, uint32_t _nowMs)
{
    if (_cmd.ctrlNum == 0 || _cmd.index >= ledColors_.size()) {
        return;
    }
    static_cast<void>(effectEngine_.setEffect(_cmd, _nowMs));
}

void LEDs::drainCommands(uint32_t _nowMs)
{
    Cmd_s cmd;
    while (xQueueReceive(queue_, &cmd, 0) == pdTRUE) {
        applyCommand(cmd, _nowMs);
    }
}
