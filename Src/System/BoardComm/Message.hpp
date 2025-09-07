#include <cstdint>
#include <vector>

#include "sdkconfig.h"
static constexpr uint16_t MASTER_ID = 0x20;
static constexpr uint16_t SLAVE_ID = 0x30;
static constexpr uint16_t TX_BASE_ID = ((BOARD_MASTER) ? MASTER_ID : SLAVE_ID);
static constexpr uint16_t RX_BASE_ID = ((BOARD_MASTER) ? SLAVE_ID : MASTER_ID);

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
    uint64_t reserved : 64;
};
} // namespace MASTER_MSG

namespace SLAVE_MSG {
struct __attribute__((packed)) First_s {
    //TODO:: add slave message
    uint64_t reserved : 64;
};
} // namespace SLAVE_MSG


struct MsgBase_s {
    uint16_t id;
    uint16_t txFreq;
    uint32_t lastSendTick = 0;
};

union MsgData_u {
    MASTER_MSG::First_s masterFirMsg;
    MASTER_MSG::Second_s masterSecMsg;
    SLAVE_MSG::First_s slaveFirMsg;
    uint8_t data[8];
};

struct Msg_s {
    MsgBase_s base;
    MsgData_u msg;
};

struct MsgConfig_s {
    std::vector<Msg_s> masterMsgs;
    std::vector<Msg_s> slaveMsgs;

    MsgConfig_s()
    {
        masterMsgs.push_back(Msg_s{ .base{
                                            .id = MASTER_ID + 0x01,
                                            .txFreq = 1000,
                                    },
                                    .msg{ .masterFirMsg{} } });
        masterMsgs.push_back(Msg_s{ .base{
                                            .id = MASTER_ID + 0x02,
                                            .txFreq = 200,
                                    },
                                    .msg{ .masterSecMsg{} } });
        slaveMsgs.push_back(Msg_s{ .base{
                                           .id = SLAVE_ID + 0x01,
                                           .txFreq = 1000,
                                   },
                                   .msg{ .slaveFirMsg{} } });
    }
};
