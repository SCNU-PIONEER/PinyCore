#pragma once

#include "M3508.hpp"
#include "Projdefs.hpp"

namespace ARM {

enum class State_e : uint8_t { MOVING, CLAMP, FAULT, RELEASE };

class Hand {
public:
    Hand(const char _name[16], PINYMOTOR::InitConfig_s _config);
    State_e clamp();
    void release();
    void stop();
    void enable();

private:
    float MOVE_TORQ = 0.083f;
    float RELEASE_TORQ = 0.055f;
    static constexpr float CLAMP_CURRENT = 2.f;
    static constexpr float SAFE_CURRENT = 7.f;

    PINYMOTOR::DJIMOTOR::M3508 motor;

    State_e state = State_e::MOVING;
};
} // namespace ARM
