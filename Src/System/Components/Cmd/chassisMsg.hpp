#pragma once
#include "MsgBase.hpp"

struct chassisMsg : public Msg {
    float vx, vy, wz;
};
