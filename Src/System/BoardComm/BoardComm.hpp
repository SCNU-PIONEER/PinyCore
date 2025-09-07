#include "Projdefs.hpp"
#include "Message.hpp"
#include "Soc.hpp"
#include "Topic.hpp"
#include <cstdint>
#include "TopicRouter.hpp"
#include "Bsp_can.hpp"

using namespace PINYMOTOR;

class BoardMsg {
public:
    BoardMsg(uint8_t _offsetId);
    void registerMsg(canHandle *_can);
    void send(canHandle *_can, MsgConfig_s *_msgConfig);
    void praseMessage(uint8_t *_rxBuf, MsgConfig_s *_msgConfig);

    bool checkSend(uint16_t _txFreq, uint32_t &_lastSendTick);

    QueueHandle_t &getRxQueue() { return rxQueue_; }
    RxBus_s::CANRxBuf_s<8> &getRxBuf() { return rxBuf_; }

private:
    uint8_t offsetId;

    QueueHandle_t rxQueue_;
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
};

class Board {
public:
    Board(canHandle *_can);

    void registerBoard();
    void sendMessage();
    void msgPublish();

    std::vector<BoardMsg *> &getMsgList() { return msgList; }
    MsgConfig_s *getMsgConfig() { return &msgConfig; }

private:
    canHandle *can;

    ChassisMsg_s chassisMsg_;
    Subscriber<ChassisMsg_s> *chassisSub_;
    Publisher<ChassisMsg_s> *chassisPub_;

    MsgConfig_s msgConfig;
    std::vector<BoardMsg *> msgList;
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
