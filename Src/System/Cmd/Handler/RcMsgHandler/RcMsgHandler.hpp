#pragma once
#include "Bsp.hpp"
#include "DT7.hpp"
#include "ET08A.hpp"
#include "Handler.hpp"

class RcMsgHandler final : public Handler {
public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    RcMsg_t rcMsg_ = {};

    RcMsg_t rcMsgPrev_ = {};

    MsgBus_s *msgBus_;

    void masterHandle();
};
