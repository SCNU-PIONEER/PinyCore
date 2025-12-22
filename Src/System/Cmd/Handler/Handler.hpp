#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "MsgBase.hpp"
#include "MsgImpl.hpp"
#include <bitset>
#include <vector>

class Handler {
    struct HandlerItem_s {
        uint32_t bit;
        Handler *handler;
    };

public:
    Handler();

    virtual ~Handler() = default;
    /* init handler */
    virtual void init(MsgBus_s *_bus, EventGroupHandle_t _event) = 0;

    /* handle data to module msg*/
    virtual void handle() = 0;

    /* notify handler to handle msg */
    virtual void notify(Msg *_msg, QueueHandle_t _queue) = 0;

    // Meyer' s Singleton
    static std::vector<HandlerItem_s> &getHandlerList();
    static std::bitset<32> &getMasks();

    EventGroupHandle_t event;

protected:
    uint32_t bit_;

private:
    static uint32_t allocateBit();
};
