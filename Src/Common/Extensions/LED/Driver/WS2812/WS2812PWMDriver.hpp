#pragma once

#include "../Driver.hpp"
#include "Bsp_pwm.hpp"

#include <array>

namespace LED {

class WS2812PWMDriver : public LEDDriver {
    using EncodedByte = std::array<uint32_t, 8>;

    Pwm pwm_;
    std::array<EncodedByte, 256> encodedByteLut_{};
    uint32_t periodTicks_ = 0;

public:
    WS2812PWMDriver(TIM_HandleTypeDef *_timer, uint32_t _channel, int _num);

private:
    uint32_t *txbuf;
    void updateEncodedByteLut();
    void show(std::vector<RGB_s> &_data) final;
};

} // namespace LED
