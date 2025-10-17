#pragma once

#include "../Driver.hpp"
#include "Bsp_pwm.hpp"

#include <memory>

namespace LED {

class WS2812PWMDriver : public LEDDriver {
    TIM_HandleTypeDef *timHandle_ = nullptr;
    static constexpr uint8_t CODE1 = 0x86; // 1code
    static constexpr uint8_t CODE0 = 0x43; // 0code
public:
    WS2812PWMDriver(TIM_HandleTypeDef *_timHandle);

private:
    std::unique_ptr<uint8_t[]> txbuf;
    void show(std::vector<RGB_s> &_data) final;
};

} // namespace LED
