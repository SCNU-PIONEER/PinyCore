#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "RttMsgHandler.hpp"
#include "RcMsgHandler.hpp"
#include "RefereeHandler.hpp"

#define EVENT_MASK (RTT_READY_EVENT | RC_READY_EVENT | REFEREE_READY_EVENT)

class Cmd {
public:
    Cmd();

    static void task(void *_param);
    MsgBus_s *getMsgBus() { return &msgBus_; }

protected:
    void parseMsg();

private:
    MsgBus_s msgBus_;
    EventGroupHandle_t eventGroup_;

    RcMsgHandler rcHandler_;
    RTTMsgHandler rttHandler_;
    RefereeHandler refereeHandler_;
};
