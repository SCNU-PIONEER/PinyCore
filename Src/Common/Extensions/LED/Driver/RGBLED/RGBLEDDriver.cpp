#include "RGBLEDDriver.hpp"

using namespace LED;

RGBLEDDriver::RGBLEDDriver(lightTuner _setR(uint8_t), lightTuner _setG(uint8_t),
                           lightTuner _setB(uint8_t))
        : setR_(_setR), setG_(_setG), setB_(_setB)
{
    numLEDs_ = 1;
}

void RGBLEDDriver::show()
{
    if (colorData_ != nullptr) {
        setR_(colorData_[0].r);
        setG_(colorData_[0].g);
        setB_(colorData_[0].b);
    }
}
