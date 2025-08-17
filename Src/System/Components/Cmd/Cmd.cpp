#include "Cmd.hpp"
#include "MsgImpl.hpp"


void Cmd::init()
{
    eventGroup_ = xEventGroupCreate();

    msgBus_.chassisQueue = xQueueCreate(30, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(GimbalMsg_s));
    msgBus_.armQueue = xQueueCreate(30, sizeof(ArmMsg_s));

    rttHandler_.init(&msgBus_, eventGroup_);
    rcHandler_.init(&msgBus_, eventGroup_);
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, EVENT_MASK, pdTRUE,
                                            pdFALSE, portMAX_DELAY);
    if (xBits & RTT_READY_EVENT) {
        rttHandler_.handle();
    } else if (xBits & RC_READY_EVENT) {
        rcHandler_.handle();
    }
}

void Cmd::task()
{
    for (;;) {
        parseMsg();
    }
}
