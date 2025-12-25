#pragma once
#include <memory>
#include "Bsp_pwm.hpp"
#include "MsgImpl.hpp"

namespace ARM {
    
class OreBin {
public:
    OreBin() = default;

    void init();
    void loop(ArmMsg_s &_msg);
    void stop();

    uint16_t release_angle = 1000;
    uint16_t stuck_angle = 2000;

private:
    std::unique_ptr<Pwm> pwm1 = nullptr;
    std::unique_ptr<Pwm> pwm2 = nullptr;
};
} // namespace ARM