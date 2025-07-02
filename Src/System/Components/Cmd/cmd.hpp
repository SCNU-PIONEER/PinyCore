#pragma once

#include <vector>
#include <unordered_map>
#include "msgBase.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "iObserver.hpp"
#include "rttMsgHandler.hpp"
#include "rcMsgHandler.hpp"

#define EVENT_MASK (RTT_READY_EVENT | RC_READY_EVENT)

class Cmd {
public:
    void init();
    void task();
    void addObserver(IObserver *observer);
    void parseMsg();

private:
    EventGroupHandle_t eventGroup;

    rcMsgHandler rcHandler;
    RTTMsgHandler rttHandler;
};
