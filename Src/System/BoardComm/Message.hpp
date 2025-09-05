#include <cstdint>
namespace MASTER_MSG {
struct __attribute__((packed)) First_s {
    uint8_t vx : 4;
    uint8_t vy : 4;
    uint8_t yaw : 4;
    uint8_t robot_control_flag : 1;
    uint64_t reserved : 51;
};

struct __attribute__((packed)) Second_s {
    //TODO:: add master message
};
} // namespace MASTER_MSG

namespace SLAVE_MSG {
struct __attribute__((packed)) First_s {
    //TODO:: add slave message
};
} // namespace SLAVE_MSG


struct MsgBase_s {
    uint16_t id;
    uint16_t txFreq;
    uint32_t lastSendTick = 0;
};

struct MsgConfig_s {
    struct {
        MsgBase_s base;
        MASTER_MSG::First_s msg;
    } masterFirMsg;
    struct {
        MsgBase_s base;
        MASTER_MSG::Second_s msg;
    } masterSecMsg;
    struct {
        MsgBase_s base;
        SLAVE_MSG::First_s msg;
    } slaveFirMsg;
};