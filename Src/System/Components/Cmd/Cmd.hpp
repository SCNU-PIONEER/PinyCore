#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "RttMsgHandler.hpp"
#include "RcMsgHandler.hpp"

#define EVENT_MASK (RTT_READY_EVENT | RC_READY_EVENT)

class Cmd {
public:
    Cmd();
    void task();
    MsgBus_s *getMsgBus() { return &msgBus; }

protected:
    void parseMsg();

private:
    MsgBus_s msgBus;
    EventGroupHandle_t eventGroup;

    rcMsgHandler rcHandler;
    RTTMsgHandler rttHandler;
};
