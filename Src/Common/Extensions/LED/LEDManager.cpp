#include "LEDManager.hpp"

using namespace LED;

LEDDriver *LEDDriver::headDriver_ = nullptr;
LEDDriver *LEDDriver::tailDriver_ = nullptr;

void LEDs::task()
{
    for (;;) {
        Cmd_s cmd;
        if (xQueueReceive(queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.type) {
            case CmdType_e::OFF:
                handleOff(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::ON_IN_NORMAL:
                handleOnInNormal(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::ON_IN_WARN:
                handleOnInWarn(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::ON_IN_ERROR:
                handleOnInError(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::BLINK_RGB:
                handleBlinkRGB(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::BLINK_RED:
                handleBlinkRed(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::BLINK_GREEN:
                handleBlinkGreen(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::BLINK_BLUE:
                handleBlinkBlue(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::RAINBOW_FLOW:
                handleRainbowFlow(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::RAINBOW_FLOW_REVERSE:
                handleRainbowFlowReverse(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::RAINBOW_FLOW_SNAKE:
                handleRainbowFlowSnake(cmd.index, cmd.ctrlNum);
                break;
            case CmdType_e::RAINBOW_BREATH:
                handleRainbowBreath(cmd.index, cmd.ctrlNum);
                break;
            default:
                // TODO: other effects
                break;
            }
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
    xQueueSend(instance().queue_, &cmd, 0);
}

void LEDs::off() { ctrl(CmdType_e::OFF, 0, instance().totalLEDs_); }

void LEDs::show()
{
    for (LEDDriver *drv = LEDDriver::head(); drv != nullptr; drv = drv->next()) {
        drv->show(ledColors_);
    }
}
