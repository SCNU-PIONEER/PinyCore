#pragma once

enum class State_e {
    stop = 0,
    run = 1,
};

class Msg {
public:
    State_e state;
};

struct chassisMsg : public Msg {
    float x, y, z;
};
