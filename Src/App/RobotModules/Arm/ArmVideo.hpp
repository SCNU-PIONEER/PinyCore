#pragma once
#include "Bsp_pwm.hpp"

namespace ARM {

class VideoCtrl {
public:
    VideoCtrl();
    void init();
    void loop(const float _pitch, const float _yaw);
    void stop();

    static constexpr uint32_t PITCH_RESET_ANG = 1600;
    static constexpr uint32_t YAW_RESET_ANG = 1460;

    static constexpr uint32_t PITCH_ANG_MIN = 1600;
    static constexpr uint32_t PITCH_ANG_MAX = 2000;

    static constexpr uint32_t YAW_ANG_MIN = 1000;
    static constexpr uint32_t YAW_ANG_MAX = 2000;

private:
    Pwm pitch;
    Pwm yaw;

    uint32_t pitchAng = PITCH_RESET_ANG;
    uint32_t yawAng = YAW_RESET_ANG;
};
} // namespace ARM
