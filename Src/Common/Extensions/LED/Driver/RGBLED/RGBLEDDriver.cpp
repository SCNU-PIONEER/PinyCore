#include "RGBLEDDriver.hpp"

using namespace LED;

RGBLEDDriver::RGBLEDDriver(lightTuner _setR(uint8_t), lightTuner _setG(uint8_t),
                           lightTuner _setB(uint8_t))
        : LEDDriver(1), setR_(_setR), setG_(_setG), setB_(_setB)
{
}

void RGBLEDDriver::show(std::vector<RGB_s> &_data)
{
    setR_(_data[vectorIndex_ + 0].r);
    setG_(_data[vectorIndex_ + 0].g);
    setB_(_data[vectorIndex_ + 0].b);
}
