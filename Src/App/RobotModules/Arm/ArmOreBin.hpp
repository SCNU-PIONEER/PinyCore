#pragma once
#include "Bsp_pwm.hpp"
#include "MsgImpl.hpp"

namespace ARM {

class OreBin {
public:
    OreBin();

    void init();
    void loop(const ArmMsg_s &_msg);
    void stop();

    static constexpr uint16_t L_RELEASE_ANG = 1120;
    static constexpr uint16_t L_STUCK_ANG = 1200;
    static constexpr uint16_t R_RELEASE_ANG = 830;
    static constexpr uint16_t R_STUCK_ANG = 900;

private:
    Pwm leftOreBin;
    Pwm rightOreBin;
};
} // namespace ARM
