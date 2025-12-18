#pragma once
#include "Bsp.hpp"
#include "DT7.hpp"
#include "ET08A.hpp"
#include "Handler.hpp"

class RcMsgHandler : public Handler {
public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

private:
    RcMsg_t rcMsg_ = {};

    RcMsg_t rcMsgPrev_ = {};

    MsgBus_s *msgBus_;

    void chassisHandle();
    void masterHandle();

    ChassisMsg_s cmsg_;

    UART_HandleTypeDef *_uart;
};
