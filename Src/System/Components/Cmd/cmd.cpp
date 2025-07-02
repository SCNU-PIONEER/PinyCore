#include "cmd.hpp"
#include "handler.hpp"
#include "stm_log.hpp"

void Cmd::init()
{
    eventGroup = xEventGroupCreate();

    rcHandler.init(eventGroup);
    rttHandler.init(eventGroup);
}

void Cmd::addObserver(IObserver *observer)
{
    if (observer) {
        rcHandler.addObserver(observer);
        rttHandler.addObserver(observer);
    }
}

void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup, EVENT_MASK, pdTRUE,
                                            pdFALSE, portMAX_DELAY);
    if (xBits & RTT_READY_EVENT) {
        rttHandler.handle();
    } else if (xBits & RC_READY_EVENT) {
        rcHandler.handle();
    }
}

void Cmd::task()
{
    while (1) {
        parseMsg();
    }
}
