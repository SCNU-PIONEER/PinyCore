#pragma once
#include "Bsp_pwm.hpp"

namespace ARM {

class VideoCtrl {
public:
    VideoCtrl();
    void init();
    void loop(const float _pitch, const float _yaw);
    void stop();

    static constexpr uint16_t PITCH_RESET_ANG = 1500;
    static constexpr uint16_t YAW_RESET_ANG = 1460;

private:
    Pwm pitch;
    Pwm yaw;
};
} // namespace ARM
