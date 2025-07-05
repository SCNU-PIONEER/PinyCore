#pragma once
#include <stdint.h>
#include "Handler.hpp"
#include "MsgImpl.hpp"

#define RTT_NO_ERROR    0
#define RTT_MSG_ERR     0xFE

#define RTT_READY_EVENT (1 << 0)

class RTTMsgHandler : public Handler {
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

protected:
    static void parse(TimerHandle_t xTimer);

private:
    uint8_t data[25] = { 0 };
    MsgBus_s *msgBus;
};
