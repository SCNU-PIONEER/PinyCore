#pragma once

#include "../Driver.hpp"
#include "Bsp_spi.hpp"

namespace LED {

class WS2812Driver : public LEDDriver {
    SPI_HandleTypeDef *spiHandle_ = nullptr;
    static constexpr uint8_t CODE0 = 0xC0; // 0code
    static constexpr uint8_t CODE1 = 0xF0; // 1code
public:
    WS2812Driver(SPI_HandleTypeDef *_spiHandle);

private:
    void show() final;
};

} // namespace LED
