#pragma once
#include "MsgImpl.hpp"
#include "Referee.hpp"
#include "Handler.hpp"

class RefereeHandler : public Handler {
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

private:
    MsgBus_s *msgBus_;
    RefereeMsg_s msg_;
};
