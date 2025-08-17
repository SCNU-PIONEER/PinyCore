#pragma once

#include <cstdint>

enum class MsgType_e : uint8_t { CHASSIS = 0, GIMBAL, ARM };

class Msg {
public:
    MsgType_e msgType;
    virtual ~Msg() = default;
};
