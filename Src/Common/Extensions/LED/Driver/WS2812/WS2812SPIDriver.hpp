#pragma once

#include "../Driver.hpp"
#include "Bsp_spi.hpp"

namespace LED {

class WS2812SPIDriver : public LEDDriver {
    SPI_HandleTypeDef *spiHandle_ = nullptr;
    uint8_t *txbuf_ = nullptr;
    uint16_t txbufLen_ = 0;

public:
    WS2812SPIDriver(SPI_HandleTypeDef *_spiHandle, int _num);

private:
    void show(std::vector<RGB_s> &_data) final;
};

} // namespace LED
