#include "BoardComm.hpp"
#include "sdkconfig.h"
#include <cstdint>
#include <cstring>


extern canHandle HCAN1;
BoardMsg::BoardMsg(uint8_t _offsetId)
        : offsetId(_offsetId)
        , rxQueue_(xQueueCreate(3, sizeof(RxBus_s::CANRxBuf_s<8>::data)))
{
}

void BoardMsg::registerMsg(canHandle *_can)
{
    Can::instance().registerCallback(
            _can, RX_BASE_ID + this->offsetId, [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
}

bool BoardMsg::checkSend(uint16_t _txFreq, uint32_t &_lastSendTick)
{
    if ((xTaskGetTickCount() - _lastSendTick) >=
        pdMS_TO_TICKS(1000.f / _txFreq)) {
        _lastSendTick = xTaskGetTickCount();
        return true;
    } else
        return false;
}

void BoardMsg::send(canHandle *_can, MsgConfig_s *_msgConfig)
{
    uint8_t txMsg[8] = {};
    uint16_t msgId = TX_BASE_ID + this->offsetId;
    bool shouldTransmit = false;

    auto &msgs = (BOARD_MASTER) ? _msgConfig->masterMsgs :
                                  _msgConfig->slaveMsgs;

    // 查找匹配的消息
    for (auto &i : msgs) {
        if (msgId == i.base.id) {
            memcpy(txMsg, &i.msg.data, 8);
            shouldTransmit = checkSend(i.base.txFreq, i.base.lastSendTick);
            break;
        }
    }

    if (shouldTransmit)
        Can::instance().transmitData(_can, msgId, txMsg, 8);
}

void BoardMsg::praseMessage(uint8_t *_rxBuf, MsgConfig_s *_msgConfig)
{
    uint16_t msgId = RX_BASE_ID + this->offsetId;

    auto &msgs = (BOARD_MASTER) ? _msgConfig->slaveMsgs :
                                  _msgConfig->masterMsgs;

    for (auto &i : msgs) {
        if (msgId == i.base.id) {
            memcpy(&i.msg.data, _rxBuf, 8);
            break;
        }
    }
}

Board::Board(canHandle *_can) : can(_can)
{
    if constexpr (BOARD_MASTER) {
        chassisSub_ = new Subscriber<ChassisMsg_s>(
                1, &TopicRouter::instance().chassisTopic, &chassisMsg_);
    } else if constexpr (BOARD_SLAVE) {
        chassisPub_ = new Publisher<ChassisMsg_s>(
                &TopicRouter::instance().chassisTopic, &chassisMsg_);
    }

    this->registerBoard();
}

void Board::registerBoard()
{
    msgList.clear();

    uint8_t msgCnt =
            std::max(msgConfig.masterMsgs.size(), msgConfig.slaveMsgs.size());

    for (uint8_t offsetId = 1; offsetId <= msgCnt; offsetId++) {
        BoardMsg *msg = new BoardMsg(offsetId);
        msg->registerMsg(this->can);
        msgList.push_back(msg);
    }
}

void Board::sendMessage()
{
    if constexpr (BOARD_MASTER) {
        // if (chassisSub_->receive()) {
        //     msgConfig.masterMsgs[0].msg.masterFirMsg.vx = chassisMsg_.vx;
        //     msgConfig.masterMsgs[0].msg.masterFirMsg.vy = chassisMsg_.vy;
        // }
    } else if constexpr (BOARD_SLAVE) {
    }
    for (auto &i : msgList) {
        i->send(can, &msgConfig);
    }
}

void Board::msgPublish()
{
    if constexpr (BOARD_MASTER) {
    } else if constexpr (BOARD_SLAVE) {
        //chassisMsg_.vx = msgConfig.masterFirMsg.msg.vx;
        //chassisPub_->publish();
    }
}

BoardManager::BoardManager() : board(new Board(&HCAN1)) {}

uint8_t BoardManager::update()
{
    uint8_t rslt = 0;
    for (auto &i : board->getMsgList()) {
        if (xQueueReceive(i->getRxQueue(), i->getRxBuf().data, 0) == pdTRUE) {
            i->praseMessage(i->getRxBuf().data, board->getMsgConfig());
            board->msgPublish();
        }
    }
    board->sendMessage();
    return rslt;
}

void BoardManager::ctrlTask()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        this->update();
        vTaskDelayUntil(&xLastWakeTime,
                        static_cast<TickType_t>(1000.f / this->TaskFreq_));
    }
}
