#pragma once

#include "M3508.hpp"

namespace ARM {

enum class State_e : uint8_t { MOVING, CLAMP, FAULT, RELEASE };

class Hand {
public:
    Hand();
    State_e clamp();
    void release();
    void stop();
    void enable();

private:
    static constexpr float MOVE_CURRENT = 0.05f;
    static constexpr float CLAMP_CURRENT = 6.f;
    static constexpr float SAFE_CURRENT = 8.f;

    PINYMOTOR::DJIMOTOR::M3508 motor;
    State_e state = State_e::MOVING;
};
} // namespace ARM
