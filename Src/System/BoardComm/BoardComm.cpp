#include "BoardComm.hpp"
#include <cstdint>
#include <cstring>
#include "sdkconfig.h"

extern canHandle HCAN1;
BoardMsg::BoardMsg(uint8_t _offsetId)
        : offsetId(_offsetId)
        , rxQueue_(xQueueCreate(3, sizeof(RxBus_s::CANRxBuf_s<8>::data)))
{
}

void BoardMsg::registerMsg(uint32_t *_pComHandle, uint16_t _id)
{
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(_pComHandle), _id + this->offsetId,
            [this](const uint8_t *_rxBuf) {
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

void BoardMsg::send(uint32_t *_pComHandle, uint16_t _id,
                    MsgConfig_s *_msgConfig)
{
    uint8_t txMsg[8] = {};
    uint16_t msgId = _id + this->offsetId;
    bool sendFlag = false;

    if constexpr (BOARD_MASTER) {
        if (msgId == _msgConfig->masterFirMsg.base.id) {
            memcpy(txMsg, &_msgConfig->masterFirMsg.msg, 8);
            sendFlag = checkSend(_msgConfig->masterFirMsg.base.txFreq,
                                 _msgConfig->masterFirMsg.base.lastSendTick);
        } else if (msgId == _msgConfig->masterSecMsg.base.id) {
            memcpy(txMsg, &_msgConfig->masterSecMsg.msg, 8);
            sendFlag = checkSend(_msgConfig->masterSecMsg.base.txFreq,
                                 _msgConfig->masterSecMsg.base.lastSendTick);
        }
    } else if constexpr (BOARD_SLAVE) {
        if (msgId == _msgConfig->slaveFirMsg.base.id) {
            memcpy(txMsg, &_msgConfig->slaveFirMsg.msg, 8);
            sendFlag = checkSend(_msgConfig->slaveFirMsg.base.txFreq,
                                 _msgConfig->slaveFirMsg.base.lastSendTick);
        }
    }

    if (sendFlag)
        Can::instance().transmitData(reinterpret_cast<canHandle *>(_pComHandle),
                                     msgId, txMsg, 8);
}

void BoardMsg::praseMessage(uint8_t *_rxBuf, MsgConfig_s *_msgConfig,
                            BoradType_e _boardType)
{
    uint16_t msgId =
            ((_boardType == BoradType_e::MASTER) ? SLAVE_ID : MASTER_ID) +
            this->offsetId;

    if constexpr (BOARD_MASTER) {
        if (msgId == _msgConfig->slaveFirMsg.base.id) {
            memcpy(&_msgConfig->slaveFirMsg.msg, _rxBuf, 8);
        }
    } else if constexpr (BOARD_SLAVE) {
        if (msgId == _msgConfig->masterFirMsg.base.id) {
            memcpy(&_msgConfig->masterFirMsg.msg, _rxBuf, 8);
        } else if (msgId == _msgConfig->masterSecMsg.base.id) {
            memcpy(&_msgConfig->masterSecMsg.msg, _rxBuf, 8);
        }
    }
}

Board::Board(uint32_t *_pComHandle, ComType_e _comType, BoradType_e _boardType)
        : pComHandle(_pComHandle), comType(_comType), boardType(_boardType)
{
    switch (boardType) {
    case BoradType_e::MASTER:
        chassisSub_ = new Subscriber<ChassisMsg_s>(
                1, &TopicRouter::instance().chassisTopic, &chassisMsg_);
        id = MASTER_ID;
        break;
    case BoradType_e::SLAVE:
        chassisPub_ = new Publisher<ChassisMsg_s>(
                &TopicRouter::instance().chassisTopic, &chassisMsg_);
        id = SLAVE_ID;
        break;
    default:
        break;
    }

    for (uint8_t i = 1; i <= 3; i++) {
        msgBus._[i - 1] = new BoardMsg(i);
    }

    this->registerBoard();
}

void Board::registerBoard()
{
    for (auto &i : msgBus._) {
        i->registerMsg(pComHandle, (boardType == BoradType_e::MASTER) ?
                                           SLAVE_ID :
                                           MASTER_ID);
    }
}

void Board::sendMessage()
{
    if constexpr (BOARD_MASTER) {
        if (chassisSub_->receive()) {
            //msgConfig.masterFirMsg.msg.vx = chassisMsg_.vx;
        }
    } else if constexpr (BOARD_SLAVE) {
    }
    for (auto &i : msgBus._) {
        i->send(pComHandle, id, &msgConfig);
    }
}

void Board::msgPublish()
{
    if constexpr (BOARD_MASTER) {
        //chassisMsg_.vx = msgConfig.masterFirMsg.msg.vx;
        chassisPub_->publish();
    } else if constexpr (BOARD_SLAVE) {
    }
}

BoardManager::BoardManager()
{
    if constexpr (BOARD_MASTER) {
        board = new Board(reinterpret_cast<uint32_t *>(&HCAN1), ComType_e::CAN,
                          BoradType_e::MASTER);
    } else if constexpr (BOARD_SLAVE) {
        board = new Board(reinterpret_cast<uint32_t *>(&HCAN1), ComType_e::CAN,
                          BoradType_e::SLAVE);
    }
}

uint8_t BoardManager::update()
{
    uint8_t rslt = 0;
    for (auto &i : board->getMsgBus()._) {
        if (xQueueReceive(i->getRxQueue(), i->getRxBuf().data, 0) == pdTRUE) {
            i->praseMessage(i->getRxBuf().data, &board->msgConfig,
                            board->getBoardType());
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
        update();
        vTaskDelayUntil(&xLastWakeTime,
                        static_cast<TickType_t>(1000.f / this->TaskFreq_));
    }
}
