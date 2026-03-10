#pragma once
#include <cstdint>
#include "Handler.hpp"
#include "MsgImpl.hpp"

#define RTT_NO_ERROR 0
#define RTT_MSG_ERR  0xFE

class RTTMsgHandler final : public Handler {
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

protected:
    static void parse(TimerHandle_t _xTimer);

private:
    uint8_t data_[25] = { 0 };
    MsgBus_s *msgBus_;
};
