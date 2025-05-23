#pragma once

struct ChassisState_s {
    float v_x; // m/s
    float v_y; // m/s
    float w_z; // rad/s
};

class Wheel {
public:
    virtual void stop() = 0;

    ChassisState_s chassisState;
};
