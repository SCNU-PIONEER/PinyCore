#pragma once
#include "msgBase.hpp"

struct chassisMsg : public Msg {
    float vx, vy, wz;
};

struct gimbalMsg : public Msg {
    float pitch, yaw;
};
