#include "Projdefs.hpp"
#include "Message.hpp"
#include "Soc.hpp"
#include "Topic.hpp"
#include <cstdint>
#include "TopicRouter.hpp"
#include "Bsp_can.hpp"

using namespace PINYMOTOR;

static constexpr uint16_t MASTER_ID = 0x20;
static constexpr uint16_t SLAVE_ID = 0x30;

enum class BoradType_e : uint8_t {
    MASTER,
    SLAVE,
};

class BoardMsg {
public:
    BoardMsg(uint8_t _offsetId);
    void registerMsg(uint32_t *_pComHandle, uint16_t _id);
    void send(uint32_t *_pComHandle, uint16_t _id, MsgConfig_s *_msgConfig);
    void praseMessage(uint8_t *_rxBuf, MsgConfig_s *_msgConfig,
                      BoradType_e _boardType);

    bool checkSend(uint16_t _txFreq, uint32_t &_lastSendTick);

    QueueHandle_t &getRxQueue() { return rxQueue_; }
    RxBus_s::CANRxBuf_s<8> &getRxBuf() { return rxBuf_; }

private:
    uint8_t offsetId;

    QueueHandle_t rxQueue_;
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
};

union MsgBus_u {
    struct {
        BoardMsg *first;
        BoardMsg *second;
        BoardMsg *third;
    };
    BoardMsg *_[3];
};


class Board {
public:
    Board(uint32_t *_pComHandle, ComType_e _comType, BoradType_e _boardType);

    void registerBoard();
    void sendMessage();
    void msgPublish();

    BoradType_e getBoardType() { return boardType; };
    MsgBus_u &getMsgBus() { return msgBus; };

    MsgConfig_s msgConfig = {
        .masterFirMsg{ .base{
                .id = MASTER_ID + 0x01,
                .txFreq = 1000,
        } },
        .masterSecMsg{ .base{
                .id = MASTER_ID + 0x02,
                .txFreq = 200,
        } },
        .slaveFirMsg{ .base{
                .id = SLAVE_ID + 0x01,
                .txFreq = 1000,
        } },
    };

private:
    uint16_t id;
    uint32_t *pComHandle;
    ComType_e comType;
    BoradType_e boardType;

    MsgBus_u msgBus;

    ChassisMsg_s chassisMsg_;
    Subscriber<ChassisMsg_s> *chassisSub_;
    Publisher<ChassisMsg_s> *chassisPub_;
};

class BoardManager {
public:
    BoardManager();
    uint8_t update();

    void ctrlTask();

    static BoardManager *instance()
    {
        static BoardManager instance;
        return &instance;
    }

private:
    const float TaskFreq_ = 1000.f;
    Board *board;
};
