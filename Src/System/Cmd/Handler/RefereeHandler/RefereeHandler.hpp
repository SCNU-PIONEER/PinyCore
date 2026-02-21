#pragma once
#include "MsgImpl.hpp"
#include "Referee.hpp"
#include "Handler.hpp"

class RefereeHandler final : public Handler {
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;
    RefereeMsg_s msg_;
};
