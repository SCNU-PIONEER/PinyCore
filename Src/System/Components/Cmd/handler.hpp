#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"
#include "iObserver.hpp"
#include <vector>

class Handler {
public:
    Handler() = default;

    /* init handler */
    virtual void init(EventGroupHandle_t _event) = 0;

    /* add observer */
    virtual void addObserver(IObserver *observer) = 0;

    /* handle data to module msg*/
    virtual void handle() = 0;

    /* notify msg */
    virtual void notify(Msg &_msg) = 0;


    EventGroupHandle_t event;

protected:
    std::vector<IObserver *> observers;
};


/* may be we can use factory pattern to create handlers */
class HandlerFactory {};
