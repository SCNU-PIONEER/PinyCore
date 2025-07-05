#include "Cmd.hpp"
#include "MsgImpl.hpp"

Cmd::Cmd()
{
    eventGroup = xEventGroupCreate();

    msgBus.chassisQueue = xQueueCreate(30, sizeof(chassisMsg));
    msgBus.gimbalQueue = xQueueCreate(30, sizeof(gimbalMsg));
    msgBus.armQueue = xQueueCreate(30, sizeof(armMsg));

    rttHandler.init(&msgBus, eventGroup);
    rcHandler.init(&msgBus, eventGroup);
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
