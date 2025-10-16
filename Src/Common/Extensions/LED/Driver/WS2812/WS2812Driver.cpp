#include "WS2812Driver.hpp"

using namespace LED;

WS2812Driver::WS2812Driver(SPI_HandleTypeDef *_spiHandle)
        : spiHandle_(_spiHandle)
{
}

uint8_t txbuf[24 * 1] __attribute__((section(".ram_BDMA"))); // only for test

void WS2812Driver::show(std::vector<RGB_s> &_data)
{
    for (int id = 0; id < numLEDs_; id++) {
        for (uint8_t i = 0; i < 8; i++) {
            txbuf[(id * 24) + 7 - i] =
                    (((_data[vectorIndex_ + id].g >> i) & 0x01) ? CODE1 :
                                                                  CODE0) >>
                    1;
            txbuf[(id * 24) + 15 - i] =
                    (((_data[vectorIndex_ + id].r >> i) & 0x01) ? CODE1 :
                                                                  CODE0) >>
                    1;
            txbuf[(id * 24) + 23 - i] =
                    (((_data[vectorIndex_ + id].b >> i) & 0x01) ? CODE1 :
                                                                  CODE0) >>
                    1;
        }
    }
    while (spiHandle_->State != HAL_SPI_STATE_READY)
        ;
    Spi::instance().transmitDMA(*spiHandle_, txbuf, 24 * numLEDs_);
}
