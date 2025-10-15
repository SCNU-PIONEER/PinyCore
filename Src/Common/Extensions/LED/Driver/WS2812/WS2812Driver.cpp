#include "WS2812Driver.hpp"

using namespace LED;

WS2812Driver::WS2812Driver(SPI_HandleTypeDef *_spiHandle)
        : spiHandle_(_spiHandle)
{
}

void WS2812Driver::show()
{
    uint8_t txbuf[24 * numLEDs_];
    for (int id = 0; id < numLEDs_; id++) {
        for (uint8_t i = 0; i < 8; i++) {
            txbuf[(id * 24) + 7 - i] =
                    (((colorData_[id].g >> i) & 0x01) ? CODE1 : CODE0) >> 1;
            txbuf[(id * 24) + 15 - i] =
                    (((colorData_[id].r >> i) & 0x01) ? CODE1 : CODE0) >> 1;
            txbuf[(id * 24) + 23 - i] =
                    (((colorData_[id].b >> i) & 0x01) ? CODE1 : CODE0) >> 1;
        }
    }
    Spi::instance().transmitDMA(*spiHandle_, txbuf, 24 * numLEDs_);
}
