#pragma once

#include <cstdint>
#include "msgImpl.hpp"
#include <functional>
#include "FreeRTOS.h"
#include "event_groups.h"

class Handler {
public:
    Handler() = default;

    /* init handler */
    virtual void init(EventGroupHandle_t _event);

    /* handle data to module msg*/
    virtual void handle();

    chassisMsg cmsg;
    gimbalMsg gmsg;
    EventGroupHandle_t event;
};


/* may be we can use factory pattern to create handlers */
class HandlerFactory {};
