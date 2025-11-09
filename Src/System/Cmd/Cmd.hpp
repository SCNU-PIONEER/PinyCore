#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "MsgImpl.hpp"

#include "RttMsgHandler.hpp"
#include "RcMsgHandler.hpp"
#include "RefereeHandler.hpp"
#include <bitset>

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
    std::bitset<32> masks_;

    RcMsgHandler rcHandler_;
    RTTMsgHandler rttHandler_;
    RefereeHandler refereeHandler_;
};
