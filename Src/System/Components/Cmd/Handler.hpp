#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "MsgBase.hpp"
#include "MsgImpl.hpp"

class Handler {
public:
    Handler() = default;
    /* init handler */
    virtual void init(MsgBus_s *_bus, EventGroupHandle_t _event);

    /* handle data to module msg*/
    virtual void handle();
    /* notify handler to handle msg */
    virtual void notify(Msg *_msg, QueueHandle_t _queue);

    EventGroupHandle_t event;
};
