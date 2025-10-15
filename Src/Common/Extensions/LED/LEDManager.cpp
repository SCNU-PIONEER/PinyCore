#include "LEDManager.hpp"

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
