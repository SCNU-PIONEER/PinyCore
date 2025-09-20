#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "MsgBase.hpp"
#include "MsgImpl.hpp"

class Handler {
public:
    Handler() = default;
    virtual ~Handler() = default;
    /* init handler */
    virtual void init(MsgBus_s *_bus, EventGroupHandle_t _event) = 0;

    /* handle data to module msg*/
    virtual void handle() = 0;

    /* notify handler to handle msg */
    virtual void notify(Msg *_msg, QueueHandle_t _queue) = 0;

    EventGroupHandle_t event;
};
