#pragma once

#include "../Driver.hpp"
#include <functional>

namespace LED {

class RGBLEDDriver : public LEDDriver {
public:
    using lightTuner = std::function<void(uint8_t)>;
    RGBLEDDriver(lightTuner _setR(uint8_t), lightTuner _setG(uint8_t),
                 lightTuner _setB(uint8_t));

private:
    void show(std::vector<RGB_s> &_data) final;

    lightTuner setR_;
    lightTuner setG_;
    lightTuner setB_;
};

} // namespace LED
