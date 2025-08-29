#include "Cmd.hpp"
#include "MsgImpl.hpp"

Cmd::Cmd() {}

void Cmd::init()
{
    eventGroup_ = xEventGroupCreate();

    msgBus_.chassisQueue = xQueueCreate(30, sizeof(chassisMsg));
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(gimbalMsg));
    msgBus_.armQueue = xQueueCreate(30, sizeof(armMsg));

    rttHandler_.init(&msgBus_, eventGroup_);
    rcHandler_.init(&msgBus_, eventGroup_);
    tpHandler_.init(&msgBus_, eventGroup_); //new add
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, EVENT_MASK, pdTRUE,
                                            pdFALSE, portMAX_DELAY);

    if (xBits & RC_READY_EVENT) {
        rcHandler_.handle();
    }
    if (xBits & TP_READY_EVENT) {
        tpHandler_.handle(); //new add
    }
    if (xBits & RTT_READY_EVENT) {
        rttHandler_.handle();
    }
}

void Cmd::task()
{
    while (1) {
        parseMsg();
    }
}
