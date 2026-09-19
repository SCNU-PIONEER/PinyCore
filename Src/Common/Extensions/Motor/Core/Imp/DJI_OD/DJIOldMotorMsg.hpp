#pragma once

#include <cstdint>

namespace PINYMOTOR::DJI_ODMOTOR {

#pragma pack(push, 1)
struct Msg_s {
    int16_t cmd[3];
};

struct Feedback_s {
    uint16_t rawAng;
    int16_t rawTorq;
};
#pragma pack(pop)
struct Status_s {
    float voltTxCodeSpan;
    float torqRxCodeSpan;
    float voltMax; // V
    float currMax; // A
    float torqMax; // Nm
    float Kn;      // Nm/A

    Status_s &operator=(const Status_s &_other);
};

} // namespace PINYMOTOR::DJI_ODMOTOR