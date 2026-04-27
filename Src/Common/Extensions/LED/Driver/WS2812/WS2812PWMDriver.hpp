#pragma once

#include "../Driver.hpp"
#include "Bsp_pwm.hpp"

namespace LED {

class WS2812PWMDriver : public LEDDriver {
    Pwm *pwm_ = nullptr;

public:
    WS2812PWMDriver(Pwm *_pwmHandle, int _num);

private:
    uint32_t *txbuf;
    void show(std::vector<RGB_s> &_data) final;
};

} // namespace LED
