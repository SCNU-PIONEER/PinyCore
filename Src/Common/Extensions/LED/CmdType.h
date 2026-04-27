#pragma once

#include <cstdint>

namespace LED {

enum class CmdType_e : uint8_t {
    OFF = 0,
    ON_IN_NORMAL,
    ON_IN_WARN,
    ON_IN_ERROR,

    BLINK_RGB,
    BLINK_RED,
    BLINK_GREEN,
    BLINK_BLUE,

    RAINBOW_FLOW,
    RAINBOW_FLOW_REVERSE,
    RAINBOW_FLOW_SNAKE,
    RAINBOW_BREATH,
};

struct Cmd_s {
    CmdType_e type = CmdType_e::OFF;
    uint8_t index = 0;   // index of leds to operate on
    uint8_t ctrlNum = 1; // number of leds to operate on starting from index
};

} // namespace LED
