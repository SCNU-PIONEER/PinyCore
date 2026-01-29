#pragma once

#include "Referee.hpp"
#include "Handler.hpp"

class NavHandler final : public Handler {
public:

    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;
    NavMsg_s navMsg_;
};

class VisionHandler final : public Handler {
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;
    VisionMsg_s visionMsg_;
};
