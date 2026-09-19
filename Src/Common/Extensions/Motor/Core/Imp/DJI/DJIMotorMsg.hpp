#pragma once

#include <cstdint>

namespace PINYMOTOR::DJIMOTOR {

#pragma pack(push, 1)
struct Msg_s {
    int16_t cmd[4];
};
struct Feedback_s {
    uint16_t rawAng;
    int16_t rawRpm;
    int16_t current;
    uint8_t temperature;
};
#pragma pack(pop)
struct Status_s {
    float voltTxCodeSpan;
    float currTxCodeSpan;
    float currRxCodeSpan;
    float voltMax; // V
    float currMax; // A
    float torqMax; // Nm
    float Kn;      // Nm/A

    Status_s &operator=(const Status_s &_other);
};

} // namespace PINYMOTOR::DJIMOTOR