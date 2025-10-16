#include "LEDManager.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace LED;

LEDDriver *LEDDriver::headDriver_ = nullptr;
LEDDriver *LEDDriver::tailDriver_ = nullptr;

LEDs::LEDs() = default;

void LEDs::task(void *_param)
{
    auto instance = static_cast<LEDs *>(_param);
    instance->queue_ = xQueueCreate(10, sizeof(Cmd_s));
    for (;;) {
        Cmd_s cmd;
        if (xQueueReceive(instance->queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.type) {
            case CmdType_e::OFF:
                instance->handleOff(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::ON_IN_NORMAL:
                instance->handleOnInNormal(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::BLINK_RGB:
                instance->handleBlinkRGB(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::RAINBOW_FLOW:
                instance->handleRainbowFlow(cmd.index, cmd.ctrlNum);
                break;
            default:
                // TODO: other effects
                break;
            }
        }
    }
}

LEDs &LEDs::instance()
{
    static LEDs instance;
    return instance;
}

void LEDs::addLEDs(LEDDriver *_driver, int _numLEDs)
{
    if (_driver != nullptr) {
        _driver->setIndex(ledColors_.size());
        ledColors_.resize(ledColors_.size() + _numLEDs);
    }
    totalLEDs_ += _numLEDs;
}

void LEDs::ctrlLED(CmdType_e _type, uint8_t _index, uint8_t _ctrlNum)
{
    Cmd_s cmd;
    cmd.type = _type;
    cmd.index = _index;
    cmd.ctrlNum = _ctrlNum;
    xQueueSend(instance().queue_, &cmd, portMAX_DELAY);
}

void LEDs::off() { ctrlLED(CmdType_e::OFF, 0, instance().totalLEDs_); }

void LEDs::show()
{
    for (LEDDriver *drv = LEDDriver::head(); drv != nullptr;
         drv = drv->next()) {
        drv->show(ledColors_);
    }
}

void LEDs::handleOff(uint8_t _index, uint8_t _ctrlNum)
{
    for (int i = 0; i < _ctrlNum; ++i) {
        if (_index + i < ledColors_.size())
            ledColors_[_index + i] = { 0, 0, 0 };
    }
    this->show();
}

void LEDs::handleOnInNormal(uint8_t _index, uint8_t _ctrlNum)
{
    for (int i = 0; i < _ctrlNum; ++i) {
        if (_index + i < ledColors_.size())
            ledColors_[_index + i] = { 0, 255, 0 };
    }
    this->show();
}

void LEDs::handleBlinkRGB(uint8_t _index, uint8_t _ctrlNum)
{
    static constexpr LEDDriver::Color RGB[3] = { LEDDriver::Color::Red,
                                                 LEDDriver::Color::Green,
                                                 LEDDriver::Color::Blue };
    uint8_t cnt = 0;
    uint8_t flowingFlag = 0;
    while (cnt++ < 3) {
        vTaskDelay(500 / portTICK_PERIOD_MS); // interval 500ms
        for (int i = 0; i < _ctrlNum; ++i) {
            if (_index + i < ledColors_.size())
                ledColors_[_index + i].setColorCode(RGB[flowingFlag]);
        }
        this->show();
        flowingFlag = (flowingFlag + 1) % 3; // 0, 1, 2
    }
}

void LEDs::handleRainbowFlow(uint8_t _index, uint8_t _ctrlNum)
{
    uint8_t r{}, g{}, b{};
    for (uint8_t i = 0; i < 255; ++i) {
        for (int j = 0; j < _ctrlNum; ++j) {
            if (_index + j < ledColors_.size()) {
                uint8_t p = 255 - ((i + j) & 255);
                if (p < 85) {
                    r = 255 - p * 3;
                    g = 0;
                    b = p * 3;
                } else if (p < 170) {
                    p -= 85;
                    r = 0;
                    g = p * 3;
                    b = 255 - p * 3;
                } else {
                    p -= 170;
                    r = p * 3;
                    g = 255 - p * 3;
                    b = 0;
                }
                ledColors_[_index + j] = { r, g, b };
            }
        }
        this->show();
        vTaskDelay(10 / portTICK_PERIOD_MS); // interval 10ms
    }
}
