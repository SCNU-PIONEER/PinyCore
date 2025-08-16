#pragma once

#include <cstdint>
enum class State_e : uint8_t { STOP = 0, NORMAL, TEACH, PLAN };

enum class MsgType_e : uint8_t {
    CHASSIS = 0,
};

class Msg {
public:
    MsgType_e msgType;
    State_e state;
    virtual ~Msg() = default;
};
