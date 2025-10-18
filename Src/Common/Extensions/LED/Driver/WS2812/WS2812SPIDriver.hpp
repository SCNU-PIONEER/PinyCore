#pragma once

#include "../Driver.hpp"
#include "Bsp_spi.hpp"

namespace LED {

class WS2812SPIDriver : public LEDDriver {
    SPI_HandleTypeDef *spiHandle_ = nullptr;
    static constexpr uint8_t CODE0 = 0xC0; // 0code
    static constexpr uint8_t CODE1 = 0xF0; // 1code
public:
    WS2812SPIDriver(SPI_HandleTypeDef *_spiHandle, int _num);

private:
    void show(std::vector<RGB_s> &_data) final;
};

} // namespace LED
