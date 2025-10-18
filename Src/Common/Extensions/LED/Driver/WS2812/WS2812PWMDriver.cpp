#include "WS2812PWMDriver.hpp"
#include "main.h"

using namespace LED;

WS2812PWMDriver::WS2812PWMDriver(Pwm *_pwmHandle, int _num)
        : LEDDriver(_num), pwm_(_pwmHandle)
{
    txbuf = std::unique_ptr<uint32_t[]>(new uint32_t[(numLEDs_ + 1) * 24]);
    memset(txbuf.get(), 0, (numLEDs_ + 1) * 24 * sizeof(uint32_t));
}

void WS2812PWMDriver::show(std::vector<RGB_s> &_data)
{
    for (int id = 0; id < numLEDs_; id++) {
        uint8_t i = 0;
        for (i = 0; i < 8; i++)
            txbuf[(id * 24) + i] =
                    (_data[vectorIndex_ + id].g & (1 << (7 - i))) ? CODE1 :
                                                                    CODE0;
        for (i = 8; i < 16; i++)
            txbuf[(id * 24) + i] =
                    (_data[vectorIndex_ + id].r & (1 << (15 - i))) ? CODE1 :
                                                                     CODE0;
        for (i = 16; i < 24; i++)
            txbuf[(id * 24) + i] =
                    (_data[vectorIndex_ + id].b & (1 << (23 - i))) ? CODE1 :
                                                                     CODE0;
    }
    pwm_->startDMA(txbuf.get(), (numLEDs_ + 1) * 24);
}
