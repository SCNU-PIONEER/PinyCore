#pragma once

enum class State_e {
    stop = 0,
    run = 1,
};

enum class msgType_e {
    chassis = 0,
};

class Msg {
public:
    msgType_e msgType;
    State_e state;
};
