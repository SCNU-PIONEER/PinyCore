#pragma once

#include "MsgImpl.hpp"
#include "Task.hpp"
#include "queue.h"
#include "event_groups.h"

#include "RttMsgHandler.hpp"

#if EXTENSION_RC
#include "RcMsgHandler.hpp"
#endif

#if EXTENSION_REFEREE
#include "RefereeHandler.hpp"
#endif

class Cmd : public Task<Cmd, 256> {
public:
    Cmd();

    void task();
    MsgBus_s *getMsgBus() { return &msgBus_; }

protected:
    void parseMsg();

private:
    MsgBus_s msgBus_;
    EventGroupHandle_t eventGroup_{ xEventGroupCreate() };

#if EXTENSION_DT7
    RcMsgHandler<RCDevType_e::DT7> rcDT7Handler_{ &SBUS_UART, eventGroup_ };
#endif
#if EXTENSION_ET08A
    RcMsgHandler<RCDevType_e::ET08A> rcET08AHandler_{ &SBUS_UART, eventGroup_ };
#endif
#if EXTENSION_VT13
    RcMsgHandler<RCDevType_e::VT13> rcVT13Handler_{ &VT03_UART, eventGroup_ };
#endif

    RTTMsgHandler rttHandler_;

#if EXTENSION_REFEREE
    RefereeHandler refereeHandler_;
#endif
};
