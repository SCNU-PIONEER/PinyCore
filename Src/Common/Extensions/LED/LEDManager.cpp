#include "LEDManager.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace LED;

LEDDriver *LEDDriver::headDriver_ = nullptr;
LEDDriver *LEDDriver::tailDriver_ = nullptr;

LEDs &LEDs::instance()
{
    static LEDs instance;
    return instance;
}

void LEDs::addLEDs(LEDDriver *_driver, int _numLEDs)
{
    if (_driver != nullptr) {
        _driver->createLEDs(_numLEDs);
        _driver->init();
    }
}

void LEDs::show()
{
    for (LEDDriver *drv = LEDDriver::head(); drv != nullptr;
         drv = drv->next()) {
        drv->show();
    }
}

void LEDs::showRGBLoop(uint16_t _interval)
{
    static constexpr LEDDriver::Color RGB[3] = { LEDDriver::Color::Red,
                                                 LEDDriver::Color::Green,
                                                 LEDDriver::Color::Blue };
    static uint8_t flowingFlag = 0;
    if (xTaskGetTickCount() - lastTime_ > _interval) {
        lastTime_ = xTaskGetTickCount();
        for (LEDDriver *drv = LEDDriver::head(); drv != nullptr;
             drv = drv->next()) {
            for (int i = 0; i < drv->numLEDs_; ++i) {
                drv->setColor(RGB[flowingFlag], i);
            }
            drv->show();
        }
        flowingFlag = (flowingFlag + 1) % 3; // 0, 1, 2
    }
}
