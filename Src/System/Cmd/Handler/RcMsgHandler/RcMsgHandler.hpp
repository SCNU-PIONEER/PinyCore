#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

class RcMsgHandler final : public Handler {
public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;

    void masterHandle();
};
