#pragma once
#include "MsgBase.hpp"

struct chassisMsg : public Msg {
    float vx, vy, wz;
};

struct gimbalMsg : public Msg {
    float pitch, yaw;
};

struct armMsg : public Msg {
    float j1, j2, j3, j4, j5, j6;
};
