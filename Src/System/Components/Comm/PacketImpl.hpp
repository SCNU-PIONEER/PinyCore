#pragma once

#include <cstdint>

namespace COMM {

#pragma pack(push, 1)

struct SamplePacket_s {
    uint8_t state : 8;
    uint16_t vx : 16;
    uint16_t vy : 16;
    uint16_t gimbalYaw : 16;
    uint8_t reversed : 8;
}; // this is a 8 bytes packet

#pragma pack(pop)

} // namespace COMM
