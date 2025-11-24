#pragma once

#include "DM4310.hpp"

namespace ARM {

enum class ClampLevel_e : uint8_t { ONE, TWO, THREE };
enum class State_e : uint8_t { MOVING, FINISH, FAULT };

class Hand {
public:
    Hand();
    State_e clamp();
    void release();
    void stop();
    void enable();

private:
    template <typename T, typename U, typename V>
    requires std::floating_point<T> constexpr bool
    inRange(const T _x, const U _m1, const V _m2)
    {
        auto min = (_m1 < _m2) ? _m1 : _m2;
        auto max = (_m1 < _m2) ? _m2 : _m1;
        return (_x >= min && _x <= max);
    }

private:
    PINYMOTOR::DMMOTOR::DM4310 motor;
    ClampLevel_e level;
};
} // namespace ARM
