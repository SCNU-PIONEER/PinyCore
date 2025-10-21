#pragma once
#include "Nav.hpp"
#include "Vision.hpp"
#include "Referee.hpp"
#include "Handler.hpp"

#define MINIPC_READY_EVENT (NAV_READY_EVENT | VISION_READY_EVENT)

class MiniPcHandler : public Handler {
public:
public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

private:
    MsgBus_s *msgBus_;
    NavMsg_s navMsg_;
    VisionMsg_s visionMsg_;
};
